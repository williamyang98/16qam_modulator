#define _USE_MATH_DEFINES
#include <cmath>
#include "pll_mixer.h"
#include "dsp/common.h"

constexpr float PI = (float)M_PI;

PLL_mixer::PLL_mixer() {
    phase_error = 0.0f;
    phase_error_gain = 4.0f/PI;
    fcenter = 0e3;
    fgain = 1e3;
}

std::complex<float> PLL_mixer::update(void) {
    float control = phase_error * phase_error_gain;
    control = dsp::clamp(control, -1.0f, 1.0f);
    float freq = fcenter + control*fgain;
    float t = integrator.process(2.0f*PI*freq);
    t = std::fmod(t, 2.0f*PI);
    integrator.yn = t;

    float I = std::cos(t);
    float Q = std::sin(t);
    return std::complex<float>(I, Q);
}