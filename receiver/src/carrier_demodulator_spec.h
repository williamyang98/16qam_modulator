#pragma once

// Specification for the carrier to symbol demodulator 
struct CarrierDemodulatorSpecification 
{
    float f_sample = 2e6;
    float f_symbol = 86e3;

    // fir baseband low pass filter
    struct {
        float cutoff = 200e3;
        int M = 18;
    } baseband_filter;

    // iir ac filter
    // 0 <= k <= 1.0f
    struct {
        float k = 0.99999f;
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