#include "preamble_detector.h"

PreambleDetector::PreambleDetector(const int32_t _preamble, const int _total_phases) 
: total_phases(_total_phases) {
    const float PI = 3.1415f;

    preamble_filters = new PreambleFilter*[total_phases];
    preamble_phases = new std::complex<float>[total_phases];

    for (int i = 0; i < total_phases; i++) {
        preamble_filters[i] = new VariablePreambleFilter<uint32_t>(_preamble);
        // phase = k*2*PI/M
        const float phase = 2.0f*PI/(float)(total_phases) * (float)(i);
        preamble_phases[i] = std::complex<float>(std::cosf(phase), std::sinf(phase));
    }
}

PreambleDetector::~PreambleDetector() {
    for (int i = 0; i < total_phases; i++) {
        delete preamble_filters[i];
    }

    delete preamble_filters;
    delete preamble_phases;
}

bool PreambleDetector::process(
    const std::complex<float> IQ, 
    ConstellationSpecification* C) 
{
    const int bits_per_symbol = C->GetBitsPerSymbol();
    bits_since_preamble += bits_per_symbol;

    int total_preambles_found = 0;
    for (int i = 0; i < total_phases; i++) {
        auto& filter = preamble_filters[i];
        auto IQ_phi = IQ * preamble_phases[i];
        auto sym = C->GetNearestSymbol(IQ_phi);
        const bool res = filter->process(sym, bits_per_symbol);
        if (!res) {
            continue;
        }

        selected_phase = i;
        total_preambles_found += 1;
        desync_bitcount = bits_since_preamble-filter->get_length();
    }

    if (total_preambles_found > 0) {
        phase_conflict = (total_preambles_found > 1);
        bits_since_preamble = 0;
        return true;
    }

    return false;
}