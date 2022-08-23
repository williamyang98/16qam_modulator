#pragma once

#include <complex>
#include <stdint.h>

class ConstellationSpecification {
public:
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
    std::complex<float>* C;
    float m_avg_power;
public:
    // L = number of symbols along one axis
    // Constellation is spaced 2 units apart, center at (0,0)
    SquareConstellation(const int _L);
    ~SquareConstellation();
    virtual std::complex<float>* GetSymbols() { return C; }
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
    const std::complex<float>* C, const int N);
