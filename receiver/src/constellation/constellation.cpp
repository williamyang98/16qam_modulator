#include "constellation.h"

#define _USE_MATH_DEFINES
#include <math.h>
constexpr float PI = (float)M_PI;

SquareConstellation::SquareConstellation(const int _L)
: L(_L), N(_L*_L) 
{
    constellation.resize(N);
    phase_lookup.resize(N);
    gray_code.resize(N);

    const float offset = (L-1)/2.0f;
    const float scale = 1.0f/std::sqrt(2.0f) * 1.0f/offset * 0.5f;

    for (int i = 0; i < L; i++) {
        const float I = 2.0f * ((float)i - offset);
        for (int j = 0; j < L; j++) {
            const float Q = 2.0f * ((float)j - offset);
            const int idx = i*L + j;
            const auto sym = std::complex<float>(I, Q);
            constellation[idx] = sym * scale;
        }
    }

    for (int i = 0; i < N; i++) {
        auto& c = constellation[i];
        phase_lookup[i] = std::atan2(c.real(), c.imag());
    }

    // generate gray code
    // TODO: generalise this?
    uint8_t _gray_code[4] = {0b00, 0b01, 0b11, 0b10};
    for (uint8_t i = 0; i < L; i++) {
        for (uint8_t q = 0; q < L; q++) {
            uint8_t I = _gray_code[i];
            uint8_t Q = _gray_code[q];
            uint8_t idx = (i<<2) | q;
            uint8_t sym = (I<<2) | Q;
            gray_code[idx] = sym;
        }
    }

    m_avg_power = CalculateAveragePower(constellation.data(), N);
}

SquareConstellation::~SquareConstellation() = default;

uint8_t SquareConstellation::GetNearestSymbol(const std::complex<float> x) {
    float min_err = INFINITY;
    uint8_t best_match = 0;

    for (uint8_t i = 0; i < N; i++) {
        auto err_vec = constellation[i]-x;
        auto err = std::abs(err_vec);
        if (err < min_err) {
            best_match = i;
            min_err = err;
        }
    }

    return gray_code[best_match];
}

float SquareConstellation::CalculateAveragePower(const std::complex<float>* C, const int N) {
    float avg_power = 0.0f;
    for (int i = 0; i < N; i++) {
        const auto& x = C[i];
        const float I = x.real();
        const float Q = x.imag();
        avg_power += (I*I + Q*Q);
    }
    avg_power /= (float)(N);
    return avg_power;
}

static float* phase_cache = NULL;

// get the phase error from the known constellation
ConstellationErrorResult estimate_phase_error(const std::complex<float> x, ConstellationSpecification& s) {
    int min_index = 0;
    float best_mag_error = INFINITY;
    const int N = s.GetSize();
    const auto* constellation = s.GetSymbols();
    const auto* phases = s.GetPhaseLookup();

    for (int i = 0; i < N; i++) {
        auto error = x - constellation[i];
        auto I = error.real();
        auto Q = error.imag();
        auto mag_error = I*I + Q*Q;

        if (mag_error < best_mag_error) {
            best_mag_error = mag_error;
            min_index = i;
        }
    }

    const auto closest_point = constellation[min_index];
    // const float angle1 = std::atan2f(closest_point.real(), closest_point.imag());
    const float angle1 = phases[min_index];
    const float angle2 = std::atan2(x.real(), x.imag());

    float phase_error = angle1-angle2;
    phase_error = std::fmod(phase_error + 3*PI, 2*PI);
    phase_error -= PI;

    best_mag_error = std::sqrt(best_mag_error);

    return {phase_error, best_mag_error};
}