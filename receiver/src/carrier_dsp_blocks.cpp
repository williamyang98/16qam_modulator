#include "carrier_dsp_blocks.h"

#define _USE_MATH_DEFINES
#include <math.h>

constexpr float PI = (float)M_PI;

PLL_mixer::PLL_mixer() {
    phase_error = 0.0f;
    phase_error_gain = 4.0f/PI;
    fcenter = 0e3;
    fgain = 1e3;
}

std::complex<float> PLL_mixer::update(void) {
    float control = phase_error * phase_error_gain;
    control = std::max(std::min(control, 1.0f), -1.0f);
    float freq = fcenter + control*fgain;
    float t = integrator.process(2.0f*PI*freq);
    t = fmodf(t, 2*PI);
    integrator.yn = t;

    float I = std::cosf(t);
    float Q = std::sinf(t);
    return std::complex<float>(I, Q);
}

// timing error detector clock
TED_Clock::TED_Clock() {
    phase_error = 0.0f;
    phase_error_gain = 1.0f;      // the phase error is +-1 from zero crossing detector
    fcenter = 10e3;
    fgain = 5e3;
}

// get current voltage in ramp integrator
float TED_Clock::get_current_timing() {
    return integrator.yn;
}

// get a normalised error if oscillator is out of sync
float TED_Clock::get_timing_error() {
    float error = get_current_timing();
    error = 2.0f * error;
    // if the zero crossing occurs past the half symbol mark then error is  [-1,0]
    if (error > 1.0f) {
        return error - 2.0f;
    }
    // otherwise if zero crossing occurs before the half symbol mark, then error is [0,1]
    return error;
}

// return true if the oscillator resets this sample
bool TED_Clock::update() {
    float control = phase_error * phase_error_gain;
    control = std::max(std::min(control, 1.0f), -1.0f);
    const float freq = fcenter + control*fgain;
    const float v = integrator.process(freq);
    const float offset = integrator.KTs * freq / 2.0f;

    if (v < (1.0f-offset)) {
        return false;
    }
    // reset integrator otherwise
    integrator.yn = 0.0f;
    return true;
}

bool Zero_Crossing_Detector::process(float x) {
    // crossing occurs if the signals are on opposite sides of the x-axis
    bool is_crossed = (x*xn) < 0.0f;
    xn = x;
    return is_crossed;
}

// only propagate trigger signal if sample cooldown has passed
bool Trigger_Cooldown::on_trigger(bool trig) {
    if (trig && (N_remain == 0)) {
        N_remain = N_cooldown;
        return true;
    }
    if (N_remain > 0) {
        N_remain--;
    }
    return false;
}

// number of triggers to store in memory
Delay_Line::Delay_Line(const int _N) 
: N(_N)
{
    counts = new int[N]{-1};
    curr_count = 0;
}

Delay_Line::~Delay_Line() {
    delete [] counts;
}

// return false if we couldn't add this to delay line
bool Delay_Line::add(int delay) {
    // if the delay line is full, we ignore this pulse
    if (curr_count >= N) {
        return false;
    }

    curr_count++;
    for (int i = 0; i < N; i++) {
        if (counts[i] < 0) {
            counts[i] = delay;
            break;
        }
    }
    return true;
}

bool Delay_Line::process(void) {
    bool trig_out = false;
    for (int i = 0; i < N; i++) {
        if (counts[i] < 0) {
            continue;
        }

        // remove a sample
        if (counts[i] == 0) {
            trig_out = true;
            curr_count--;
        }         
        counts[i]--;
    }
    return trig_out;
}

N_Level_Crossing_Detector::N_Level_Crossing_Detector(const float* _levels, const int _N)
: N(_N)
{
    levels = new float[N];
    curr_level = 0;
    for (int i = 0; i < N; i++) {
        levels[i] = _levels[i];
    }
}

N_Level_Crossing_Detector::~N_Level_Crossing_Detector() {
    delete [] levels;
}

bool N_Level_Crossing_Detector::process(const float x) {
    int new_level = curr_level;
    for (int i = 0; i < N; i++) {
        if (x > levels[i]) {
            new_level = i;
            break;
        }
    }

    bool is_crossed = (new_level != curr_level);
    curr_level = new_level;
    return is_crossed;
}
