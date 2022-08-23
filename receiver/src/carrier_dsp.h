// perform dsp on the raw IQ signal and convert it to symbols
#pragma once
#include <stdint.h>
#include <complex>

#include "filters.h"
#include "carrier_dsp_blocks.h"
#include "carrier_demodulator_spec.h"

class CarrierToSymbolDemodulatorBuffers
{
public:
    uint8_t* data_allocate; // allocate all of the data as a single block
    size_t data_size;
public:
    const int block_size;
    // IQ samples
    std::complex<float>* x_in;
    std::complex<float>* x_filtered;
    // output of phased locked loop
    std::complex<float>* x_pll_out;
    std::complex<float>* y_sym_out;
    // triggers
    bool* trig_zero_crossing;
    bool* trig_ted_clock;
    bool* trig_integrator_dump;
    // error signals
    float* error_pll;
    float* error_ted;
public:
    CarrierToSymbolDemodulatorBuffers(const int _block_size);
    ~CarrierToSymbolDemodulatorBuffers();
    inline size_t Size() { return data_size; }
    bool CopyFrom(CarrierToSymbolDemodulatorBuffers* in);
};

class CarrierToSymbolDemodulator
{
private:
    const CarrierDemodulatorSpecification spec;
    // processing constants
    float Fs;
    float Ts;
    float Fsymbol;
    float Tsymbol;
    int Nsymbol;
public:
    // buffers
    CarrierToSymbolDemodulatorBuffers* buffers = NULL;
private:
    // prefiltering before demodulation
    FIR_Filter<std::complex<float>>* filter_baseband = NULL;
    IIR_Filter<std::complex<float>>* filter_ac = NULL;
    AGC_Filter<std::complex<float>> filter_agc;
    // phase locked loop
    PLL_mixer pll_mixer;
    float pll_error_prev;
    IIR_Filter<float>* pll_error_lpf;
    Integrator_Block<float> pll_error_int;
    // timing error detector
    TED_Clock ted_clock;
    float ted_error_prev;
    IIR_Filter<float>* ted_error_lpf = NULL;
    Integrator_Block<float> ted_error_int;
    // zero crossing detectors
    N_Level_Crossing_Detector* I_zcd = NULL;
    N_Level_Crossing_Detector* Q_zcd = NULL;
    Trigger_Cooldown zcd_cooldown;
    // integrate and dump filter
    Integrator_Block<std::complex<float>> integrate_dump_filter;
private:
    std::complex<float> y_sym_out;
public:
    CarrierToSymbolDemodulator(CarrierDemodulatorSpecification _spec);
    ~CarrierToSymbolDemodulator();
    // return the number of symbols read into the buffer
    // x must be at least block_size large
    int ProcessBlock(std::complex<uint8_t>* x, std::complex<float>* y);
};