#pragma once
#include <complex>
#include <vector>
#include <memory>

#include "preamble_filter.h"
#include "constellation/constellation.h"

// Consists of a bank of:
// 1. preamble filters
// 2. possible phase shifts
// The purpose of the preamble detector is to find the preamble on the correct phase
// There are multiple potential locked phases for a M-PSK or a QAM signal
// For M-PSK there are M possible phases: k*2*PI/M where k=[1,M]
// For M-QAM there are 4 possible phases: 0, PI/2, PI, -PI/2
class PreambleDetector {
private:
    std::vector<std::unique_ptr<PreambleFilter>> preamble_filters;
    std::vector<std::complex<float>> preamble_phases;
    int bits_since_preamble = 0;
    const int total_phases;
    int selected_phase = 0;
    bool phase_conflict = false;
    int desync_bitcount = 0;
public:
    PreambleDetector(const int32_t _preamble, const int _total_phases);
    bool Process(const std::complex<float> IQ, ConstellationSpecification& constellation);
    bool IsPhaseConflict() { return phase_conflict; }
    std::complex<float> GetPhase() { return preamble_phases[selected_phase]; }
    int GetPhaseIndex() { return selected_phase; }
    int GetDesyncBitcount() { return desync_bitcount; }
};