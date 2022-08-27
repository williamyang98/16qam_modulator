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
    {
        Fs = spec.f_sample;
        Ts = 1.0f/Fs;
        Fsymbol = spec.f_symbol;
        Tsymbol = 1/Fsymbol;
        Nsymbol = (int)std::floorf(Fs/Fsymbol);
    }

    // baseband symbol filter
    {
        auto& s = spec.baseband_filter;
        const float k = s.cutoff/(Fs/2.0f);
        auto res = create_fir_lpf(k, s.M);
        filter_baseband = new FIR_Filter<std::complex<float>>(res->b, res->N);
        delete res;
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
        pll_mixer.integrator.KTs = Ts;
        pll_mixer.fcenter = s.f_center;
        pll_mixer.fgain = -s.f_gain;
        pll_mixer.phase_error_gain = s.phase_error_gain;
    }

    // carrier pll loop filter
    {
        auto& s = spec.carrier_pll_filter;
        pll_error_prev = 0.0f;
        pll_error_int.KTs = s.integrator_gain*Ts;

        const float k = s.butterworth_cutoff/(Fs/2.0f);
        auto res = create_iir_single_pole_lpf(k);
        pll_error_lpf = new IIR_Filter<float>(res->b, res->a, res->N);
        delete res;
    }

    // ted
    {
        auto& s = spec.ted_pll;
        ted_clock.integrator.KTs = Ts;
        ted_clock.fcenter = Fsymbol + s.f_offset;
        ted_clock.fgain = -s.f_gain;
        ted_clock.phase_error_gain = s.phase_error_gain;
    }

    // ted pll loop filter
    {
        auto& s = spec.ted_pll_filter;
        ted_error_prev = 0.0f;
        ted_error_int.KTs = s.integrator_gain*Ts;

        const float k = s.butterworth_cutoff/(Fs/2.0f);
        auto res = create_iir_single_pole_lpf(k);
        ted_error_lpf = new IIR_Filter<float>(res->b, res->a, res->N);
        delete res;
    }

    I_zcd = new N_Level_Crossing_Detector(N_levels, total_levels);
    Q_zcd = new N_Level_Crossing_Detector(N_levels, total_levels);

    zcd_cooldown.N_cooldown = (int)std::floorf(Nsymbol*0.0f);
    // integrate_dump_filter.KTs = Ts/Tsymbol;
}

CarrierToSymbolDemodulator::~CarrierToSymbolDemodulator()
{
    delete filter_baseband;
    delete filter_ac;
    delete pll_error_lpf;
    delete ted_error_lpf;
    delete I_zcd;
    delete Q_zcd;
}

int CarrierToSymbolDemodulator::ProcessBlock(std::complex<float>* x, std::complex<float>* y)
{
    if (buffers == NULL) {
        return -1;
    }

    float thresh_acquire_error = 0.2f; // max distance allowed for a valid symbol reading
    const bool use_all_points = false;

    const int block_size = buffers->block_size;
    int total_symbols = 0;

    // per block filtering
    {
        filter_ac->process(x, buffers->x_in, block_size);
        filter_baseband->process(buffers->x_in, buffers->x_filtered, block_size);
        filter_agc.process(buffers->x_filtered, buffers->x_agc, block_size);
    }

    // carrier pll
    for (int i = 0; i < block_size; i++) {
        // get augmented value from pll mixer
        const auto IQ_raw = buffers->x_agc[i];
        const auto IQ_mixer_out = pll_mixer.update();
        const auto IQ_pll = IQ_raw * IQ_mixer_out;

        float pll_phase_error = pll_error_prev;

        // Run carrier phase estimation for every possible sample
        // {
        //     const auto A = std::abs(IQ_pll);
        //     auto res = estimate_phase_error(IQ_pll, constellation);
        //     if (res.mag_error < thresh_acquire_error) {
        //         pll_phase_error = res.phase_error;
        //     }
        // }

        // pass new pll phase error through first order butterworth filter
        pll_error_prev = pll_phase_error;
        {
            float y = 0;
            pll_error_lpf->process(&pll_error_prev, &y, 1);
            pll_error_int.process(y);
            pll_error_int.yn = std::max(std::min(pll_error_int.yn, 1.0f), -1.0f);
            pll_mixer.phase_error = y + pll_error_int.yn;
        }

        bool is_zero_crossing = false;
        {
            is_zero_crossing = I_zcd->process(IQ_pll.real()) || is_zero_crossing;
            is_zero_crossing = Q_zcd->process(IQ_pll.imag()) || is_zero_crossing;
            is_zero_crossing = zcd_cooldown.on_trigger(is_zero_crossing);
        } 

        // if zero crossing detector triggered, update the phase error into the ted clock
        float ted_timing_error = ted_clock.get_timing_error();
        if (!is_zero_crossing) {
            ted_timing_error = ted_error_prev;
        }             
        ted_error_prev = ted_timing_error;

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
        // bool is_integrate_dump_trigger = is_ted_clock_trigger;
        bool is_integrate_dump_trigger = delay_line.process();

        // get symbols from int+dump filter
        // integrate_dump_filter.process(IQ_pll);
        if (is_integrate_dump_trigger) {
            // auto IQ_out = integrate_dump_filter.yn;
            // integrate_dump_filter.yn = std::complex<float>(0.0f, 0.0f);
            auto IQ_out = IQ_pll;
            y_sym_out = IQ_out;
            y[total_symbols++] = IQ_out;

            // Update carrier phase estimate for every sampled symbol
            auto res = estimate_phase_error(IQ_pll, constellation);
            pll_error_prev = res.phase_error;
        } 
        
        // place all of our data into the buffer
        buffers->x_pll_out[i] = IQ_pll;
        buffers->error_pll[i] = pll_mixer.phase_error;
        buffers->trig_zero_crossing[i] = is_zero_crossing;
        buffers->trig_ted_clock[i] = is_ted_clock_trigger;
        buffers->trig_integrator_dump[i] = is_integrate_dump_trigger;
        buffers->error_ted[i] = ted_clock.phase_error;
        buffers->y_sym_out[i] = y_sym_out;
    }

    return total_symbols;
}

// Memory layout of our buffers
CarrierToSymbolDemodulatorBuffers::CarrierToSymbolDemodulatorBuffers(const int _block_size)
: block_size(_block_size) 
{
    static auto align_memaddr = [](size_t addr){
        const size_t align_size = 8;
        size_t offset = addr % align_size;
        return addr - offset + (size_t)align_size;
    };

    // calculate size of all members
    size_t s0  = 0;
    size_t s1 = s0 + align_memaddr(sizeof(std::complex<float>) * block_size);
    size_t s2 = s1 + align_memaddr(sizeof(std::complex<float>) * block_size);
    size_t s3 = s2 + align_memaddr(sizeof(std::complex<float>) * block_size);
    size_t s4 = s3 + align_memaddr(sizeof(std::complex<float>) * block_size);
    size_t s5 = s4 + align_memaddr(sizeof(std::complex<float>) * block_size);
    size_t s6 = s5 + align_memaddr(sizeof(bool) * block_size);
    size_t s7 = s6 + align_memaddr(sizeof(bool) * block_size);
    size_t s8 = s7 + align_memaddr(sizeof(bool) * block_size);
    size_t s9 = s8 + align_memaddr(sizeof(float) * block_size);
    size_t s10 = s9 + align_memaddr(sizeof(float) * block_size);

    data_size = s10;
    data_allocate = new uint8_t[data_size];

    // cast with offsets to individual buffers
    x_in = reinterpret_cast<std::complex<float>*>(&data_allocate[s0]);
    x_filtered = reinterpret_cast<std::complex<float>*>(&data_allocate[s1]);
    x_agc = reinterpret_cast<std::complex<float>*>(&data_allocate[s2]);
    x_pll_out = reinterpret_cast<std::complex<float>*>(&data_allocate[s3]);
    y_sym_out = reinterpret_cast<std::complex<float>*>(&data_allocate[s4]);

    trig_zero_crossing = reinterpret_cast<bool*>(&data_allocate[s5]);
    trig_ted_clock = reinterpret_cast<bool*>(&data_allocate[s6]);
    trig_integrator_dump = reinterpret_cast<bool*>(&data_allocate[s7]);

    error_pll = reinterpret_cast<float*>(&data_allocate[s8]);
    error_ted = reinterpret_cast<float*>(&data_allocate[s9]);
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
