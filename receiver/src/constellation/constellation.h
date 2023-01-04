#pragma once

#include <complex>
#include <vector>
#include <stdint.h>

class ConstellationSpecification {
public:
    virtual ~ConstellationSpecification() {}
    // NOTE: We need a phase lookup if we are using the constellation in a decision directed loop
    // This because the atan2 calculation is extremely slow, and having this precomputed is better
    virtual float* GetPhaseLookup() = 0;
    virtual std::complex<float>* GetSymbols() = 0;
    virtual int GetSize() = 0;
    virtual int GetBitsPerSymbol() = 0;
    virtual uint8_t GetNearestSymbol(const std::complex<float> x) = 0;
    virtual float GetAveragePower() = 0; 
};

class SquareConstellation: public ConstellationSpecification {
private:
    const int L;
    const int N;
    std::vector<std::complex<float>> constellation;
    std::vector<float> phase_lookup;
    std::vector<uint8_t> gray_code;
    float m_avg_power;
public:
    // L = number of symbols along one axis
    // Constellation is spaced 2 units apart, center at (0,0)
    SquareConstellation(const int _L);
    virtual ~SquareConstellation();
    virtual std::complex<float>* GetSymbols() { return constellation.data(); }
    virtual float* GetPhaseLookup() { return phase_lookup.data(); }
    virtual int GetSize() { return N; };
    virtual int GetBitsPerSymbol() { return L; };
    virtual float GetAveragePower() { return m_avg_power; }; 
    virtual uint8_t GetNearestSymbol(const std::complex<float> x);
private:
    float CalculateAveragePower(const std::complex<float>* C, const int N);
};

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
    ConstellationSpecification& s);
