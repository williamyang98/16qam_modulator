#pragma once

// Diagram of our carrier to symbol demodulator
// RX_IN --> 8bit IQ --> [8bit to float] --> Downsample --> AC Filter --> AGC --> X0

// X0 --> IQ Mixer --> Upsample --> [        Sampler          ] --> Y0        
//           ^            |            |                   ^         |
//           |            |            v                   |         |
//           |            |-- ZCD --> TED --> LPF --> PI --|         |
//           |                                                       |
//           |-- PI <-- LPF <-- Phase detector <---------------------|                    

// Specification for the carrier to symbol demodulator 
struct CarrierDemodulatorSpecification 
{
    float f_sample = 1e6;
    float f_symbol = 200e3;

    struct {
        int M = 2;
        int K = 10;
    } downsampling_filter;

    // iir ac filter
    // 0 <= k <= 1.0f
    struct {
        float k = 0.9999f;
    } ac_filter;

    // automatic gain control
    struct {
        float beta = 0.1f;
        float initial_gain = 0.1f;
    } agc;

    // carrier phased lock loop
    struct {
        float f_center = 0e3;
        float f_gain = 5e3;
        float phase_error_gain = 8.0f/3.1415f;
    } carrier_pll;

    struct {
        float proportional_gain = 1.0f;
        float integrator_gain = 1000.0f;
        float butterworth_cutoff = 10e3;
    } carrier_pll_filter;

    // upsampler for timing error detection
    struct {
        int L = 4;
        int K = 3;
    } upsampling_filter;

    // timing error detector
    struct {
        float f_offset = 0e3;
        float f_gain = 10e3;
        float phase_error_gain = 1.0f;
    } ted_pll;

    struct {
        float proportional_gain = 1.0f;
        float integrator_gain = 250.0f;
        float butterworth_cutoff = 10e3;
    } ted_pll_filter;
};