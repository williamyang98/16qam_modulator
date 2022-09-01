// perform dsp on the raw IQ signal and convert it to symbols
#pragma once
#include <stdint.h>
#include <complex>

#include "filters.h"
#include "polyphase_filter.h"
#include "carrier_dsp_blocks.h"
#include "carrier_demodulator_spec.h"
#include "constellation.h"

class CarrierToSymbolDemodulatorBuffers
{
public:
    uint8_t* data_allocate; // allocate all of the data as a single block
    size_t data_size;
public:
    const int src_block_size;           // Fs 
    const int ds_block_size;            // Fs/M
    const int us_block_size;            // L/M * Fs
    // IQ samples
    std::complex<uint8_t>* x_raw;       // Fs
    std::complex<float>* x_in;          // Fs
    std::complex<float>* x_downsampled; // Fs/M
    std::complex<float>* x_ac;          // Fs/M 
    std::complex<float>* x_agc;         // Fs/M
    // output of phased locked loop
    std::complex<float>* x_pll_out;     // Fs/M
    std::complex<float>* x_upsampled;   // L/M * Fs
    std::complex<float>* y_sym_out;     // L/M * Fs
    std::complex<float>* y_out;         // Fsymbol
    // triggers
    bool* trig_zero_crossing;           // L/M * Fs
    bool* trig_ted_clock;               // L/M * Fs
    bool* trig_integrator_dump;         // L/M * Fs
    // error signals
    float* error_pll;                   // Fs/M
    float* error_ted;                   // L/M * Fs
public:
    CarrierToSymbolDemodulatorBuffers(const int _block_size, const int M, const int L);
    ~CarrierToSymbolDemodulatorBuffers();
    inline size_t Size() { return data_size; }
    bool CopyFrom(CarrierToSymbolDemodulatorBuffers* in);
    int GetInputSize() const { return src_block_size; }
    int GetCarrierSize() const { return ds_block_size; }
    int GetTedSize() const { return us_block_size; }
};

class CarrierToSymbolDemodulator
{
private:
    const CarrierDemodulatorSpecification spec;
private:
    int Nsymbol;
private:
    // prefiltering before demodulation
    PolyphaseDownsampler<std::complex<float>>* filter_ds = NULL;
    IIR_Filter<std::complex<float>>* filter_ac = NULL;
    AGC_Filter<std::complex<float>> filter_agc;
    PolyphaseUpsampler<std::complex<float>>* filter_us = NULL;
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
    Delay_Line delay_line;
    // Integrator_Block<std::complex<float>> integrate_dump_filter;
    // keep track of last output symbol
    std::complex<float> y_sym_out;
    ConstellationSpecification* constellation = NULL;
public:
    CarrierToSymbolDemodulator(CarrierDemodulatorSpecification _spec, ConstellationSpecification* _constellation);
    ~CarrierToSymbolDemodulator();
    // return the number of symbols read into the buffer
    // x must be at least block_size large
    int ProcessBlock(CarrierToSymbolDemodulatorBuffers* buffers);
};