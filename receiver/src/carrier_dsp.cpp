#include "carrier_dsp.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <assert.h>

constexpr float PI = (float)M_PI;

// all of our demodulator filters

// low pass baseband filter
// 10kHz cutoff
// constexpr float FIR1_TAP_B[] = {0.0115586129393857f,0.0269127184226144f,0.0689577752425094f,0.124811024018249f,0.172306936575946f,0.190905865602590f,0.172306936575946f,0.124811024018249f,0.0689577752425094f,0.0269127184226144f,0.0115586129393857f};
// 50kHz cutoff
// constexpr float FIR1_TAP_B[] = {-1.24137345828279e-18f,-0.0126419757368433f,-0.0246922577087696f,0.0635051299460375f,0.274797751173244f,0.398062704652663f,0.274797751173244f,0.0635051299460375f,-0.0246922577087696f,-0.0126419757368433f,-1.24137345828279e-18f};
// 62.5kHz @ Fs=2MHz cutoff
constexpr float FIR1_TAP_B[] = {0.00701891326561467f,0.0116587959676490f,0.0246187865230115f,0.0451889289532004f,0.0704801267453563f,0.0960007748441594f,0.116717378951263f,0.128316294749745f,0.128316294749745f,0.116717378951263f,0.0960007748441594f,0.0704801267453563f,0.0451889289532004f,0.0246187865230115f,0.0116587959676490f,0.00701891326561467f};
// constexpr int FIR1_TAP_ORDER = sizeof(FIR1_TAP_B) / sizeof(float);
// 100kHz @ Fs=2MHz cutoff
// constexpr float FIR1_TAP_B[] = {0.00253999383501346f,0.00574420105960838f,0.0147083365148430f,0.0314560608717508f,0.0554822508096040f,0.0834419109654486f,0.109888911438294f,0.128859581669095f,0.135757505672686f,0.128859581669095f,0.109888911438294f,0.0834419109654486f,0.0554822508096040f,0.0314560608717508f,0.0147083365148430f,0.00574420105960838f,0.00253999383501346f};
// constexpr int FIR1_TAP_ORDER = sizeof(FIR1_TAP_B) / sizeof(float);
// 200kHz @ Fs=2MHz cutoff
// constexpr float FIR1_TAP_B[] = {-0.01259277478717816f,-0.02704833486706803f,-0.031157016036431583f,-0.003351666747179282f,0.06651710329324828f,0.1635643048779222f,0.249729473226146f,0.2842779082622769f,0.249729473226146f,0.1635643048779222f,0.06651710329324828f,-0.003351666747179282f,-0.031157016036431583f,-0.02704833486706803f,-0.01259277478717816f};
// constexpr int FIR1_TAP_ORDER = sizeof(FIR1_TAP_B) / sizeof(float);
// 400kHz @ Fs=2MHz cutoff
// constexpr float FIR1_TAP_B[] = {1.24593522022721e-18f,0.00557150275358611f,0.00788955841781937f,-0.0165219973844350f,-0.0508056027127646f,1.19921264946869e-17f,0.183871713137048f,0.369994825788746f,0.369994825788746f,0.183871713137048f,1.19921264946869e-17f,-0.0508056027127646f,-0.0165219973844350f,0.00788955841781937f,0.00557150275358611f,1.24593522022721e-18f};
// constexpr int FIR1_TAP_ORDER = sizeof(FIR1_TAP_B) / sizeof(float);
// 50kHz @ Fs=1MHz cutoff
// constexpr float FIR1_TAP_B[] = {0.00253999383501346f,0.00574420105960838f,0.0147083365148430f,0.0314560608717508f,0.0554822508096040f,0.0834419109654486f,0.109888911438294f,0.128859581669095f,0.135757505672686f,0.128859581669095f,0.109888911438294f,0.0834419109654486f,0.0554822508096040f,0.0314560608717508f,0.0147083365148430f,0.00574420105960838f,0.00253999383501346f};
constexpr int FIR1_TAP_ORDER = sizeof(FIR1_TAP_B) / sizeof(float);

// ac coupling filter
constexpr float IIR1_TAP_B[] = {1.0f, -1.0f};
// constexpr float IIR1_TAP_A[] = {1.0f, -0.9995f};
constexpr float IIR1_TAP_A[] = {1.0f, -0.99999f};
constexpr int IIR1_TAP_ORDER = sizeof(IIR1_TAP_A) / sizeof(float);

// pll phase error first order butterworth filter
// 2KHz butterworth 1st order filter
// constexpr float IIR2_TAP_B[] = {0.0245216092494659f,0.0245216092494659f};
// constexpr float IIR2_TAP_A[] = {1.0f,-0.950956781501068f};
// 5kHz butterworth 1st order
// constexpr float IIR2_TAP_B[] = {0.0591907038184055f,0.0591907038184055f};
// constexpr float IIR2_TAP_A[] = {1.0f,-0.881618592363189f};

// 5kHz @ Fs=2MHz Butterworth N=1 lpf
constexpr float IIR2_TAP_B[] = {0.00779293629195155f,0.00779293629195155f};
constexpr float IIR2_TAP_A[] = {1.0f,-0.984414127416097f};
// 2kHz @ Fs=2MHz Butterworth N=1 lpf
// constexpr float IIR2_TAP_B[] = {0.00313176422919270f,0.00313176422919270f};
// constexpr float IIR2_TAP_A[] = {1.0f,-0.993736471541615f};

// 2kHz @ Fs=1MHz Butterworth N=1 lpf
// constexpr float IIR2_TAP_B[] = {0.00624403504634286f,0.00624403504634286f};
// constexpr float IIR2_TAP_A[] = {1.0f,-0.987511929907314f};
const int IIR2_TAP_ORDER = sizeof(IIR2_TAP_A) / sizeof(float);

// ted phase error fir filter
// 50kHz filter @ Fs=2MHz
const float IIR3_TAP_B[] = {0.0729596572682667f,0.0729596572682667f};
const float IIR3_TAP_A[] = {1.0f,-0.854080685463467f};
// 10kHz filter @ Fs=2MHz
// const float IIR3_TAP_B[] = {0.0154662914031034f,0.0154662914031034f};
// const float IIR3_TAP_A[] = {1.0f,-0.969067417193793f};
// 10kHz filter @ Fs=1MHz
// const float IIR3_TAP_B[] = {0.0304687470912538f,0.0304687470912538f};
// const float IIR3_TAP_A[] = {1.0f,-0.939062505817492f};
// 50kHz filter @ Fs=1MHz
// const float IIR3_TAP_B[] = {0.136728735997320f,0.136728735997320f};
// const float IIR3_TAP_A[] = {1.0f,-0.726542528005361f};
const int IIR3_TAP_ORDER = sizeof(IIR3_TAP_B) / sizeof(float);

#include "constellation.h"

constexpr int INTEGRATE_DUMP_DELAY_SAMPLES = 5;

CarrierToSymbolDemodulator::CarrierToSymbolDemodulator(const int _block_size)
:   block_size(_block_size),
    filter_baseband(FIR1_TAP_B, FIR1_TAP_ORDER),
    filter_ac(IIR1_TAP_B, IIR1_TAP_A, IIR1_TAP_ORDER),
    filter_agc(),
    pll_error_lpf(IIR2_TAP_B, IIR2_TAP_A, IIR2_TAP_ORDER),
    ted_error_lpf(IIR3_TAP_B, IIR3_TAP_A, IIR3_TAP_ORDER),
    integrate_dump_trigger_delay_line(INTEGRATE_DUMP_DELAY_SAMPLES) 
{
    // allocate our buffers
    buffers = new CarrierToSymbolDemodulatorBuffers(block_size);

    // calculate constants
    // Fs = 250e3;
    Fs = 2e6;
    Ts = 1.0f/Fs;
    Fsymbol = 25e3;
    Tsymbol = 1/Fsymbol;
    Nsymbol = (int)std::floorf(Fs/Fsymbol);

    // setup our control loop
    // filter_agc.target_power = QAM_Constellation_Average_Power;
    {
        float avg_power = 0.0f;
        for (int i = 0; i < QAM_Constellation_Size; i++) {
            float I = QAM_Constellation[i].real();
            float Q = QAM_Constellation[i].imag();
            avg_power += (I*I + Q*Q);
        }
        avg_power = avg_power / (float)(QAM_Constellation_Size);
        filter_agc.target_power = avg_power;
    }
    filter_agc.beta = 0.05f;
    filter_agc.current_gain = 0.1f;

    pll_mixer.integrator.KTs = Ts;
    pll_mixer.fcenter = 0e3;
    pll_mixer.fgain = -2e3;
    pll_mixer.phase_error_gain = 4.0f/PI;

    ted_clock.integrator.KTs = Ts;
    ted_clock.fcenter = Fsymbol;
    ted_clock.fgain = -10e3;

    zcd_cooldown.N_cooldown = (int)std::floorf(Nsymbol*0.5f);

    integrate_dump_filter.KTs = Ts/Tsymbol;
}

CarrierToSymbolDemodulator::~CarrierToSymbolDemodulator()
{
    delete buffers;
}

int CarrierToSymbolDemodulator::ProcessBlock(std::complex<uint8_t>* x, std::complex<float>* y)
{
    //TODO: test variables
    float thresh_acquire_error = 0.3f; // max distance allowed for a valid symbol reading
    const bool use_all_points = false;

    int total_symbols = 0;

    for (int i = 0, j = 0; i < block_size; i++, j+=2) {
        const uint8_t I = x[i].real();
        const uint8_t Q = x[i].imag();
        buffers->x_in[i].real((float)I - 127.5f);
        buffers->x_in[i].imag((float)Q - 127.5f);
    }

    // per block filtering
    {
        auto filter_out = buffers->x_filtered;
        filter_baseband.process(buffers->x_in, filter_out, block_size);
        filter_ac.process(filter_out, filter_out, block_size);
        filter_agc.process(filter_out, filter_out, block_size);
    }

    // per sample control loop
    for (int i = 0; i < block_size; i++) {
        // get augmented value from pll mixer
        const auto IQ_raw = buffers->x_filtered[i];
        const auto IQ_mixer_out = pll_mixer.update();
        const auto IQ_pll = IQ_raw * IQ_mixer_out;

        // zero crossing detector
        bool is_zero_crossing = false;
        {
            is_zero_crossing = I_zcd.process(IQ_pll.real()) || is_zero_crossing;
            is_zero_crossing = Q_zcd.process(IQ_pll.imag()) || is_zero_crossing;
            is_zero_crossing = zcd_cooldown.on_trigger(is_zero_crossing);
        } 

        // if zero crossing detector triggered, update the phase error into the ted clock
        float ted_timing_error = ted_clock.get_timing_error();
        if (is_zero_crossing) {
            // negative timing error means ramp trigger occurs before half symbol mark
            ted_error_lpf.process(&ted_timing_error, &ted_clock.phase_error, 1);
            // ted_clock.phase_error = ted_timing_error;
        }

        // propagate the trigger pulse into delay line
        bool is_ted_clock_trigger = ted_clock.update();
        if (is_ted_clock_trigger) {
            // convert timing error to sample delay for integrate and dump trigger
            // a negative timing error means that ramp trigger occurs before half symbol point
            int Ndelay = Nsymbol + (int)ceilf(Nsymbol*0.5f*ted_timing_error);
            bool is_added = integrate_dump_trigger_delay_line.add(Ndelay+1);
            // this gets triggered if we our timing error detector is triggering too fast
            assert(is_added);
        }

        float pll_phase_error = pll_mixer.phase_error;

        // integrate and dump filter
        // on the trigger, we dump the filter
        // and estimate the phase error of the constellation and pass it to the carrier pll
        integrate_dump_filter.process(IQ_pll);
        bool is_integrate_dump_trigger = integrate_dump_trigger_delay_line.process();
        if (is_integrate_dump_trigger) {
            auto IQ_out = integrate_dump_filter.yn;
            integrate_dump_filter.yn = std::complex<float>(0.0f, 0.0f);
            y_sym_out = IQ_out;

            // place our demodulated symbol into the output buffer
            y[total_symbols++] = IQ_out;

            // estimate the phase of the IQ output
            auto res = estimate_phase_error(IQ_out, QAM_Constellation, QAM_Constellation_Size);
            if (res.mag_error < thresh_acquire_error) {
                pll_phase_error = res.phase_error;
            }
        // NOTE: if we do not have an integrator+dump output, we will still try to estimate a symbol
        // this is because introducing the zero-order hold filter from the integrate+dump
        // will produce phase noise in the PLL due to the one symbol delay 
        } else if (use_all_points) {
            const auto A = std::abs(IQ_pll);
            if (A > 0.4) {
                auto res = estimate_phase_error(IQ_pll, QAM_Constellation, QAM_Constellation_Size);
                if (res.mag_error < thresh_acquire_error) {
                    pll_phase_error = res.phase_error;
                }
            }
        }

        // pass new pll phase error through first order butterworth filter
        // pll_error_lpf.process(&pll_phase_error, &pll_mixer.phase_error, 1);
        pll_mixer.phase_error = pll_phase_error;

        // place all of our data into the buffer
        buffers->x_pll_out[i] = IQ_pll;
        buffers->y_sym_out[i] = y_sym_out;
        buffers->trig_zero_crossing[i] = is_zero_crossing;
        buffers->trig_ted_clock[i] = is_ted_clock_trigger;
        buffers->trig_integrator_dump[i] = is_integrate_dump_trigger;
        buffers->error_pll[i] = pll_mixer.phase_error;
        buffers->error_ted[i] = ted_clock.phase_error;
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
    size_t s5 = s4 + align_memaddr(sizeof(bool) * block_size);
    size_t s6 = s5 + align_memaddr(sizeof(bool) * block_size);
    size_t s7 = s6 + align_memaddr(sizeof(bool) * block_size);
    size_t s8 = s7 + align_memaddr(sizeof(float) * block_size);
    size_t s9 = s8 + align_memaddr(sizeof(float) * block_size);

    data_size = s9;
    data_allocate = new uint8_t[data_size];

    // cast with offsets to individual buffers
    x_in = reinterpret_cast<std::complex<float>*>(&data_allocate[s0]);
    x_filtered = reinterpret_cast<std::complex<float>*>(&data_allocate[s1]);
    x_pll_out = reinterpret_cast<std::complex<float>*>(&data_allocate[s2]);
    y_sym_out = reinterpret_cast<std::complex<float>*>(&data_allocate[s3]);

    trig_zero_crossing = reinterpret_cast<bool*>(&data_allocate[s4]);
    trig_ted_clock = reinterpret_cast<bool*>(&data_allocate[s5]);
    trig_integrator_dump = reinterpret_cast<bool*>(&data_allocate[s6]);

    error_pll = reinterpret_cast<float*>(&data_allocate[s7]);
    error_ted = reinterpret_cast<float*>(&data_allocate[s8]);
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