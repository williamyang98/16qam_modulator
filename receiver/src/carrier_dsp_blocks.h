// Classes and functions for demodulating modulated IQ signal into symbols
#pragma once

#include "filters.h"
#include <complex>

// return the errors of the best match in constellation
struct ConstellationErrorResult 
{
    float phase_error;
    float mag_error;
};

// get the phase error from the known constellation
// phase error is defined between [-pi,pi] where error=ref-actual
// a negative phase error means the constellation is rotated counterclockwise
ConstellationErrorResult 
estimate_phase_error(
    const std::complex<float> x, 
    const std::complex<float>* C, const int N);

// phase locked loop mixer for carrier
class PLL_mixer 
{
public:
    Integrator_Block<float> integrator;
    float phase_error;
    float phase_error_gain;
    float fcenter;
    float fgain;
public:
    PLL_mixer();
    std::complex<float> update(void);
};

// timing error detector clock
class TED_Clock 
{
public:
    Integrator_Block<float> integrator; // used as ramp oscillator for symbol timing from 0 to 1
    float phase_error;
    float phase_error_gain;      // the phase error is +-1 from zero crossing detector
    float fcenter;
    float fgain;
public:
    TED_Clock();
    // get current voltage in ramp integrator
    float get_current_timing();
    // get a normalised error if oscillator is out of sync
    float get_timing_error();
    // return true if the oscillator resets this sample
    bool update();
};

class Zero_Crossing_Detector 
{
public:
    float xn = 0.0f;
    // return true if zero crossing detected
    bool process(const float x);
};

class N_Level_Crossing_Detector
{
public:
    float* levels;
    const int N;
private:
    int curr_level;
public:
    N_Level_Crossing_Detector(const float* _levels, const int _N);
    ~N_Level_Crossing_Detector();
    bool process(const float x);
};

// prevent trigger from refiring too quickly
class Trigger_Cooldown 
{
public:
    int N_cooldown = 1;
    int N_remain = 0;
    // only propagate trigger signal if sample cooldown has passed
    bool on_trigger(const bool trig);
};

// delay line for trigger pulses
class Delay_Line 
{
private:
    int curr_count;
public:
    // number of triggers to store in memory
    const int N;
    int *counts;
public:
    Delay_Line(const int _N);
    ~Delay_Line();
    // return false if we couldn't add this to delay line
    bool add(int delay);
    // propagate any delayed trigger signals
    bool process();
};