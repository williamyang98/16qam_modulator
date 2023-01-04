#pragma once

#include <stdint.h>
#include <complex>
#include "utility/aligned_vector.h"
#include "utility/joint_allocate.h"
#include "utility/span.h"

class QAM_Synchroniser_Buffer
{
public:
    AlignedVector<uint8_t> data_allocate;
public:
    const int src_block_size;                    // Fs 
    const int ds_block_size;                     // Fs/M
    const int us_block_size;                     // L/M * Fs
    const int ds_factor;
    const int us_factor;
    // Input 
    tcb::span<std::complex<uint8_t>> x_raw;       // Fs
    tcb::span<std::complex<float>> x_in;          // Fs
    // Downsampled PLL
    tcb::span<std::complex<float>> x_downsampled; // Fs/M
    tcb::span<std::complex<float>> x_ac;          // Fs/M 
    tcb::span<std::complex<float>> x_agc;         // Fs/M
    tcb::span<std::complex<float>> x_pll_out;     // Fs/M
    tcb::span<float> error_pll;                   // Fs/M
    // Upsampled TED
    tcb::span<std::complex<float>> x_upsampled;   // L/M * Fs
    tcb::span<bool> trig_zero_crossing;           // L/M * Fs
    tcb::span<bool> trig_ted_clock;               // L/M * Fs
    tcb::span<bool> trig_integrator_dump;         // L/M * Fs
    tcb::span<float> error_ted;                   // L/M * Fs
    tcb::span<std::complex<float>> y_sym_out;     // L/M * Fs
    // Output symbols
    tcb::span<std::complex<float>> y_out;         // Fsymbol
public:
    QAM_Synchroniser_Buffer(const int _block_size, const int M, const int L);
    size_t Size() { return data_allocate.size(); }
    bool CopyFrom(QAM_Synchroniser_Buffer& in); 
    int GetInputSize() const { return src_block_size; }
    int GetPLLSize() const { return ds_block_size; }
    int GetTEDSize() const { return us_block_size; }
    int GetDownsamplingFactor() const { return ds_factor; }
    int GetUpsamplingFactor() const { return us_factor; }
};