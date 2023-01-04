#pragma once

#include "dsp/integrator.h"
#include "dsp/common.h"

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
    TED_Clock() {
        phase_error = 0.0f;
        phase_error_gain = 1.0f; 
        fcenter = 10e3f;
        fgain = 5e3f;
    }

    float get_current_timing() const { return integrator.yn; }

    // normalised error between -1 to 1
    float get_timing_error() const {
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
    bool update() {
        float control = phase_error * phase_error_gain;
        control = dsp::clamp(control, -1.0f, 1.0f);
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
};
