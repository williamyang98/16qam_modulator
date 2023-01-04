#pragma once

#include <complex>
#include "dsp/integrator.h"

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