#define _USE_MATH_DEFINES
#include <math.h>
#include <assert.h>

#include "qam_sync.h"
#include "dsp/filter_designer.h"

constexpr float PI = (float)M_PI;

// N level crossing
constexpr float N_levels[4] = {0.5f, 0.0f, -0.5f, -1.0f};
constexpr int total_levels = 4;

QAM_Synchroniser::QAM_Synchroniser(
    QAM_Synchroniser_Specification _spec,
    ConstellationSpecification& _constellation)
: spec(_spec), constellation(_constellation),
  delay_line(5)
{
    // calculate constants
    const float Fsource = spec.f_sample;
    const float Fsymbol = spec.f_symbol;

    const float Fdownsample = Fsource/(float)(spec.downsampling_filter.M);
    const float Fupsample = Fdownsample * (float)(spec.upsampling_filter.L);

    Nsymbol = (int)std::floorf(Fupsample/Fsymbol);

    const float Tsource = 1.0f/Fsource;
    const float Tdownsample = 1.0f/Fdownsample;
    const float Tupsample = 1.0f/Fupsample;
    const float Tsymbol = 1.0f/Fsymbol;

    // downsampling filter is always mandatory
    // This is because it will implement at least one LPF with cutoff Fsymbol
    {
        auto& s = spec.downsampling_filter;
        const float k = Fsymbol/(Fsource/2.0f);
        filter_ds = std::make_unique<PolyphaseDownsampler<std::complex<float>>>(s.M, s.K);
        create_fir_lpf(filter_ds->get_b(), filter_ds->get_K(), k);
    } 

    // ac filter
    {
        auto& s = spec.ac_filter;
        const int N = TOTAL_TAPS_IIR_AC_COUPLE;
        filter_ac = std::make_unique<IIR_Filter<std::complex<float>>>(N);
        create_iir_ac_filter(filter_ac->get_b(), filter_ac->get_a(), s.k);
    }

    // agc
    {
        auto& s = spec.agc;
        filter_agc.beta = s.beta;
        filter_agc.current_gain = s.initial_gain;
        filter_agc.target_power = constellation.GetAveragePower();
    }

    // carrier pll
    {
        auto& s = spec.carrier_pll;
        pll.mixer.integrator.KTs = Tdownsample;
        pll.mixer.fcenter = s.f_center;
        pll.mixer.fgain = -s.f_gain;
        pll.mixer.phase_error_gain = s.phase_error_gain;
    }

    // carrier pll loop filter
    {
        auto& s = spec.carrier_pll_filter;
        pll.prev_error = 0.0f;
        pll.int_error.KTs = s.integrator_gain*Tdownsample;

        const float k = s.butterworth_cutoff/(Fdownsample/2.0f);
        const int N = TOTAL_TAPS_IIR_SINGLE_POLE_LPF;
        auto& filt = pll.filt_iir_lpf_error;
        filt = std::make_unique<IIR_Filter<float>>(N);
        create_iir_single_pole_lpf(filt->get_b(), filt->get_a(), k);
    }

    // upsampling filter
    if (spec.upsampling_filter.L > 1) {
        auto& s = spec.upsampling_filter;
        // const float k = (Fdownsample/2.0f)/(Fupsample/2.0f);
        const float k = Fsymbol/(Fupsample/2.0f);
        const int NN = s.K*s.L;

        auto b = std::vector<float>(NN);
        create_fir_lpf(b.data(), NN, k);
        filter_us = std::make_unique<PolyphaseUpsampler<std::complex<float>>>(b.data(), s.L, s.K);
    } else {
        filter_us = NULL;
    }

    // ted
    {
        auto& s = spec.ted_pll;
        ted.clock.integrator.KTs = Tupsample;
        ted.clock.fcenter = Fsymbol + s.f_offset;
        ted.clock.fgain = -s.f_gain;
        ted.clock.phase_error_gain = s.phase_error_gain;
    }

    // ted pll loop filter
    {
        auto& s = spec.ted_pll_filter;
        ted.prev_error = 0.0f;
        ted.int_error.KTs = s.integrator_gain*Tupsample;

        const float k = s.butterworth_cutoff/(Fupsample/2.0f);
        const int N = TOTAL_TAPS_IIR_SINGLE_POLE_LPF;
        auto& filt = ted.filt_iir_lpf_error;
        filt = std::make_unique<IIR_Filter<float>>(N);
        create_iir_single_pole_lpf(filt->get_b(), filt->get_a(), k);
    }

    I_zcd = std::make_unique<N_Level_Crossing_Detector>(N_levels, total_levels);
    Q_zcd = std::make_unique<N_Level_Crossing_Detector>(N_levels, total_levels);
    zcd_cooldown.N_cooldown = (int)std::floorf(Nsymbol*0.0f);
}

int QAM_Synchroniser::ProcessBlock(QAM_Synchroniser_Buffer& buffers)
{
    float thresh_acquire_error = 0.2f; // max distance allowed for a valid symbol reading
    const bool use_all_points = false;

    const int source_size = buffers.GetInputSize();
    const int ds_size = buffers.GetPLLSize();
    const int us_size = buffers.GetTEDSize();
    const int L = us_size/ds_size;

    int total_symbols = 0;

    for (int i = 0; i < source_size; i++) {
        const auto& IQ = buffers.x_raw[i];
        const float I = static_cast<float>(IQ.real()) - 128.0f;
        const float Q = static_cast<float>(IQ.imag()) - 128.0f;
        buffers.x_in[i] = std::complex<float>(I, Q);
    }

    // per block filtering
    {
        filter_ds->process(buffers.x_in.data(), buffers.x_downsampled.data(), ds_size);
        filter_ac->process(buffers.x_downsampled.data(), buffers.x_ac.data(), ds_size);
        filter_agc.process(buffers.x_ac.data(), buffers.x_agc.data(), ds_size);
    }

    // Our multirate processing loop
    // Outer loop runs at Fdownsample
    // Inner TED loop runs at Fupsample
    for (int i = 0; i < ds_size; i++) {
        const auto IQ_raw = buffers.x_agc[i];
        const auto IQ_mixer_out = pll.mixer.update();
        const auto IQ_pll = IQ_raw * IQ_mixer_out;

        // Run carrier phase estimation for every possible sample
        // {
        //     const auto A = std::abs(IQ_pll);
        //     auto res = estimate_phase_error(IQ_pll, constellation);
        //     if (res.mag_error < thresh_acquire_error) {
        //         pll_error_prev = res.phase_error;
        //     }
        // }

        // pass new pll phase error through first order butterworth filter
        {
            float error_lpf = 0;
            pll.filt_iir_lpf_error->process(&pll.prev_error, &error_lpf, 1);
            pll.int_error.process(error_lpf);
            pll.int_error.yn = dsp::clamp(pll.int_error.yn, -1.0f, 1.0f);
            pll.mixer.phase_error = error_lpf + pll.int_error.yn;
        }

        buffers.x_pll_out[i] = IQ_pll;
        buffers.error_pll[i] = pll.mixer.phase_error;

        // Upsample signal (optional)
        auto rd_buf = buffers.x_pll_out;
        if (filter_us) {
            filter_us->process(&buffers.x_pll_out[i], &buffers.x_upsampled[i*L], 1);
            rd_buf = buffers.x_upsampled;
        }

        for (int j = 0; j < L; j++) {
            const int us_i = i*L + j;

            const auto IQ_us_pll = rd_buf[us_i];
            bool is_zero_crossing = false;
            {
                is_zero_crossing = I_zcd->process(IQ_us_pll.real()) || is_zero_crossing;
                is_zero_crossing = Q_zcd->process(IQ_us_pll.imag()) || is_zero_crossing;
                is_zero_crossing = zcd_cooldown.on_trigger(is_zero_crossing);
            } 

            // if zero crossing detector triggered, update the phase error into the ted clock
            if (is_zero_crossing) {
                ted.prev_error = ted.clock.get_timing_error();
            }             

            // propagate ted error into pll
            {
                float error_lpf = 0.0f;
                ted.filt_iir_lpf_error->process(&ted.prev_error, &error_lpf, 1);
                ted.int_error.process(error_lpf);
                ted.int_error.yn = dsp::clamp(ted.int_error.yn, -1.0f, 1.0f);
                ted.clock.phase_error = error_lpf + ted.int_error.yn;
            }

            const bool is_ted_clock_trigger = ted.clock.update();
            if (is_ted_clock_trigger) {
                delay_line.add(Nsymbol/2);
            }
            const bool is_integrate_dump_trigger = delay_line.process();

            if (is_integrate_dump_trigger) {
                auto IQ_out = IQ_us_pll;
                y_sym_out = IQ_out;
                buffers.y_out[total_symbols++] = IQ_out;

                // Update carrier phase estimate for every sampled symbol
                auto res = estimate_phase_error(IQ_pll, constellation);
                pll.prev_error = res.phase_error;
            } 

            buffers.trig_zero_crossing[us_i] = is_zero_crossing;
            buffers.trig_ted_clock[us_i] = is_ted_clock_trigger;
            buffers.trig_integrator_dump[us_i] = is_integrate_dump_trigger;
            
            // place all of our data into the buffer
            buffers.error_ted[us_i] = ted.clock.phase_error;
            buffers.y_sym_out[us_i] = y_sym_out;
        }
    }

    return total_symbols;
}