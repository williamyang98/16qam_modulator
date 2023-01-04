#pragma once
#include <stdint.h>
#include <complex>
#include <memory>

#include "utility/aligned_vector.h"
#include "utility/span.h"

#include "dsp/integrator.h"
#include "dsp/iir_filter.h"
#include "dsp/polyphase_filter.h"
#include "dsp/agc.h"

#include "pll_mixer.h"
#include "ted_clock.h"
#include "N_level_crossing_detector.h"
#include "trigger_cooldown.h"
#include "delay_line.h"

#include "qam_sync_spec.h"
#include "qam_sync_buffers.h"
#include "constellation/constellation.h"


// perform dsp on the raw IQ signal and convert it to symbols
class QAM_Synchroniser
{
private:
    const QAM_Synchroniser_Specification spec;
private:
    int Nsymbol;
private:
    // prefiltering before demodulation
    std::unique_ptr<PolyphaseDownsampler<std::complex<float>>> filter_ds;
    std::unique_ptr<IIR_Filter<std::complex<float>>> filter_ac;
    AGC_Filter<std::complex<float>> filter_agc;
    std::unique_ptr<PolyphaseUpsampler<std::complex<float>>> filter_us;
    // phase locked loop
    struct {
        PLL_mixer mixer;
        float prev_error;
        Integrator_Block<float> int_error;
        std::unique_ptr<IIR_Filter<float>> filt_iir_lpf_error;
    } pll;
    // timing error detector
    struct {
        TED_Clock clock;
        float prev_error;
        Integrator_Block<float> int_error;
        std::unique_ptr<IIR_Filter<float>> filt_iir_lpf_error;
    } ted;
    // zero crossing detectors
    std::unique_ptr<N_Level_Crossing_Detector> I_zcd;
    std::unique_ptr<N_Level_Crossing_Detector> Q_zcd;
    Trigger_Cooldown zcd_cooldown;
    // integrate and dump filter
    Delay_Line delay_line;
    // Integrator_Block<std::complex<float>> integrate_dump_filter;
    // keep track of last output symbol
    std::complex<float> y_sym_out;
    ConstellationSpecification& constellation;
public:
    QAM_Synchroniser(QAM_Synchroniser_Specification _spec, ConstellationSpecification& _constellation);
    // return the number of symbols read into the buffer
    // x must be at least block_size large
    int ProcessBlock(QAM_Synchroniser_Buffer& buffers);
};