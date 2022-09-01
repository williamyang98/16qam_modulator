#define _USE_MATH_DEFINES
#include <math.h>
#include <assert.h>

#include "carrier_dsp.h"
#include "constellation.h"
#include "filter_designer.h"

constexpr float PI = (float)M_PI;

// ac coupling filter
constexpr float AC_FILTER_B[] = {1.0f, -1.0f};

// N level crossing
constexpr float N_levels[4] = {0.5f, 0.0f, -0.5f, -1.0f};
constexpr int total_levels = 4;

CarrierToSymbolDemodulator::CarrierToSymbolDemodulator(
    CarrierDemodulatorSpecification _spec,
    ConstellationSpecification* _constellation)
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

    // downsampling filter
    {
        auto& s = spec.downsampling_filter;
        const float k = Fsymbol/(Fsource/2.0f);
        const int NN = s.K*s.M;
        auto spec = create_fir_lpf(k, NN-1);
        filter_ds = new PolyphaseDownsampler<std::complex<float>>(spec->b, s.M, s.K);
    }

    // ac filter
    {
        auto& s = spec.ac_filter;
        float AC_Filter_A[2] = {1.0f, -s.k};
        filter_ac = new IIR_Filter<std::complex<float>>(AC_FILTER_B, AC_Filter_A, 2);
    }

    // agc
    {
        auto& s = spec.agc;
        filter_agc.beta = s.beta;
        filter_agc.current_gain = s.initial_gain;
        filter_agc.target_power = constellation->GetAveragePower();
    }

    // carrier pll
    {
        auto& s = spec.carrier_pll;
        pll_mixer.integrator.KTs = Tdownsample;
        pll_mixer.fcenter = s.f_center;
        pll_mixer.fgain = -s.f_gain;
        pll_mixer.phase_error_gain = s.phase_error_gain;
    }

    // carrier pll loop filter
    {
        auto& s = spec.carrier_pll_filter;
        pll_error_prev = 0.0f;
        pll_error_int.KTs = s.integrator_gain*Tdownsample;

        const float k = s.butterworth_cutoff/(Fdownsample/2.0f);
        auto res = create_iir_single_pole_lpf(k);
        pll_error_lpf = new IIR_Filter<float>(res->b, res->a, res->N);
        delete res;
    }

    // upsampling filter
    {
        auto& s = spec.upsampling_filter;
        // const float k = (Fdownsample/2.0f)/(Fupsample/2.0f);
        const float k = Fsymbol/(Fupsample/2.0f);
        const int NN = s.K*s.L;
        auto spec = create_fir_lpf(k, NN-1);
        filter_us = new PolyphaseUpsampler<std::complex<float>>(spec->b, s.L, s.K);
    }

    // ted
    {
        auto& s = spec.ted_pll;
        ted_clock.integrator.KTs = Tupsample;
        ted_clock.fcenter = Fsymbol + s.f_offset;
        ted_clock.fgain = -s.f_gain;
        ted_clock.phase_error_gain = s.phase_error_gain;
    }

    // ted pll loop filter
    {
        auto& s = spec.ted_pll_filter;
        ted_error_prev = 0.0f;
        ted_error_int.KTs = s.integrator_gain*Tupsample;

        const float k = s.butterworth_cutoff/(Fupsample/2.0f);
        auto res = create_iir_single_pole_lpf(k);
        ted_error_lpf = new IIR_Filter<float>(res->b, res->a, res->N);
        delete res;
    }

    I_zcd = new N_Level_Crossing_Detector(N_levels, total_levels);
    Q_zcd = new N_Level_Crossing_Detector(N_levels, total_levels);

    zcd_cooldown.N_cooldown = (int)std::floorf(Nsymbol*0.0f);
}

CarrierToSymbolDemodulator::~CarrierToSymbolDemodulator()
{
    delete filter_ds; 
    delete filter_ac;
    delete filter_us;

    delete pll_error_lpf;
    delete ted_error_lpf;

    delete I_zcd;
    delete Q_zcd;
}

int CarrierToSymbolDemodulator::ProcessBlock(CarrierToSymbolDemodulatorBuffers* buffers)
{
    if (buffers == NULL) {
        return -1;
    }

    float thresh_acquire_error = 0.2f; // max distance allowed for a valid symbol reading
    const bool use_all_points = false;

    const int source_size = buffers->GetInputSize();
    const int ds_size = buffers->GetCarrierSize();
    const int us_size = buffers->GetTedSize();

    const int L = us_size/ds_size;

    int total_symbols = 0;

    for (int i = 0; i < source_size; i++) {
        const auto& IQ = buffers->x_raw[i];
        const float I = static_cast<float>(IQ.real()) - 128.0f;
        const float Q = static_cast<float>(IQ.imag()) - 128.0f;
        buffers->x_in[i] = std::complex<float>(I, Q);
    }

    // per block filtering
    {
        filter_ds->process(buffers->x_in, buffers->x_downsampled, ds_size);
        filter_ac->process(buffers->x_downsampled, buffers->x_ac, ds_size);
        filter_agc.process(buffers->x_ac, buffers->x_agc, ds_size);
    }

    // Our multirate processing loop
    // Outer loop runs at Fdownsample
    // Inner TED loop runs at Fupsample
    for (int i = 0; i < ds_size; i++) {
        const auto IQ_raw = buffers->x_agc[i];
        const auto IQ_mixer_out = pll_mixer.update();
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
            float y = 0;
            pll_error_lpf->process(&pll_error_prev, &y, 1);
            pll_error_int.process(y);
            pll_error_int.yn = std::max(std::min(pll_error_int.yn, 1.0f), -1.0f);
            pll_mixer.phase_error = y + pll_error_int.yn;
        }

        buffers->x_pll_out[i] = IQ_pll;
        buffers->error_pll[i] = pll_mixer.phase_error;

        // Upsample signal
        filter_us->process(&buffers->x_pll_out[i], &buffers->x_upsampled[i*L], 1);

        for (int j = 0; j < L; j++) {
            const int us_i = i*L + j;

            const auto IQ_us_pll = buffers->x_upsampled[us_i];
            bool is_zero_crossing = false;
            {
                is_zero_crossing = I_zcd->process(IQ_us_pll.real()) || is_zero_crossing;
                is_zero_crossing = Q_zcd->process(IQ_us_pll.imag()) || is_zero_crossing;
                is_zero_crossing = zcd_cooldown.on_trigger(is_zero_crossing);
            } 

            // if zero crossing detector triggered, update the phase error into the ted clock
            if (is_zero_crossing) {
                ted_error_prev = ted_clock.get_timing_error();
            }             

            // propagate ted error into pll
            {
                float y = 0.0f;
                ted_error_lpf->process(&ted_error_prev, &y, 1);
                ted_error_int.process(y);
                ted_error_int.yn = std::max(std::min(ted_error_int.yn, 1.0f), -1.0f);
                ted_clock.phase_error = y + ted_error_int.yn;
            }

            bool is_ted_clock_trigger = ted_clock.update();
            if (is_ted_clock_trigger) {
                delay_line.add(Nsymbol/2);
            }
            bool is_integrate_dump_trigger = delay_line.process();

            if (is_integrate_dump_trigger) {
                auto IQ_out = IQ_us_pll;
                y_sym_out = IQ_out;
                buffers->y_out[total_symbols++] = IQ_out;

                // Update carrier phase estimate for every sampled symbol
                auto res = estimate_phase_error(IQ_pll, constellation);
                pll_error_prev = res.phase_error;
            } 

            buffers->trig_zero_crossing[us_i] = is_zero_crossing;
            buffers->trig_ted_clock[us_i] = is_ted_clock_trigger;
            buffers->trig_integrator_dump[us_i] = is_integrate_dump_trigger;
            
            // place all of our data into the buffer
            buffers->error_ted[us_i] = ted_clock.phase_error;
            buffers->y_sym_out[us_i] = y_sym_out;
        }
    }

    return total_symbols;
}

// Memory layout of our buffers
CarrierToSymbolDemodulatorBuffers::CarrierToSymbolDemodulatorBuffers(
    const int _block_size, const int M, const int L)
: ds_block_size(_block_size), 
  src_block_size(_block_size*M), 
  us_block_size(_block_size*L)
{
    static auto align_memaddr = [](size_t addr){
        const size_t align_size = 8;
        size_t offset = addr % align_size;
        return addr - offset + (size_t)align_size;
    };

    // calculate size of all members
    const size_t s0  = 0;

    const size_t s1 = s0 + align_memaddr(sizeof(std::complex<uint8_t>) * src_block_size);
    const size_t s2 = s1 + align_memaddr(sizeof(std::complex<float>) * src_block_size);

    const size_t s3 = s2 + align_memaddr(sizeof(std::complex<float>) * ds_block_size);
    const size_t s4 = s3 + align_memaddr(sizeof(std::complex<float>) * ds_block_size);
    const size_t s5 = s4 + align_memaddr(sizeof(std::complex<float>) * ds_block_size);
    const size_t s6 = s5 + align_memaddr(sizeof(std::complex<float>) * ds_block_size);

    const size_t s7 = s6 + align_memaddr(sizeof(std::complex<float>) * us_block_size);
    const size_t s8 = s7 + align_memaddr(sizeof(std::complex<float>) * us_block_size);

    const size_t s9 = s8 + align_memaddr(sizeof(bool) * us_block_size);
    const size_t s10 = s9 + align_memaddr(sizeof(bool) * us_block_size);
    const size_t s11 = s10 + align_memaddr(sizeof(bool) * us_block_size);

    const size_t s12 = s11 + align_memaddr(sizeof(float) * ds_block_size);
    const size_t s13 = s12 + align_memaddr(sizeof(float) * us_block_size);

    const size_t s14 = s13 + align_memaddr(sizeof(std::complex<float>) * us_block_size);

    data_size = s14;
    data_allocate = new uint8_t[data_size];

    // cast with offsets to individual buffers
    x_raw = reinterpret_cast<std::complex<uint8_t>*>(&data_allocate[s0]);
    x_in = reinterpret_cast<std::complex<float>*>(&data_allocate[s1]);

    x_downsampled = reinterpret_cast<std::complex<float>*>(&data_allocate[s2]);
    x_ac = reinterpret_cast<std::complex<float>*>(&data_allocate[s3]);
    x_agc = reinterpret_cast<std::complex<float>*>(&data_allocate[s4]);
    x_pll_out = reinterpret_cast<std::complex<float>*>(&data_allocate[s5]);

    x_upsampled = reinterpret_cast<std::complex<float>*>(&data_allocate[s6]);
    y_sym_out = reinterpret_cast<std::complex<float>*>(&data_allocate[s7]);

    trig_zero_crossing = reinterpret_cast<bool*>(&data_allocate[s8]);
    trig_ted_clock = reinterpret_cast<bool*>(&data_allocate[s9]);
    trig_integrator_dump = reinterpret_cast<bool*>(&data_allocate[s10]);

    error_pll = reinterpret_cast<float*>(&data_allocate[s11]);
    error_ted = reinterpret_cast<float*>(&data_allocate[s12]);

    y_out = reinterpret_cast<std::complex<float>*>(&data_allocate[s13]);
}

CarrierToSymbolDemodulatorBuffers::~CarrierToSymbolDemodulatorBuffers() 
{
    delete [] data_allocate;
    
}


bool CarrierToSymbolDemodulatorBuffers::CopyFrom(CarrierToSymbolDemodulatorBuffers* in)
{
    if (in->Size() != Size()) {
        return false;
    } 

    const size_t total_size = Size();
    memcpy_s(data_allocate, total_size, in->data_allocate, total_size);
    return true;
}
