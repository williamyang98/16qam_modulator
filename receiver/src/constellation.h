#pragma once

#include <complex>

// our reference constellation for a 1W signal
// constexpr std::complex<float> QAM_Constellation[] = {{0.707f,0.707f},{0.707f,-0.707f},{-0.707f,0.707f},{-0.707f,-0.707f}};
constexpr float A = 0.707f;
constexpr std::complex<float> QAM_Constellation[] = {
    {-A,-A},{-A,-A/3.0f},{-A,A/3.0f},{-A,A},
    {-A/3.0f,-A},{-A/3.0f,-A/3.0f},{-A/3.0f,A/3.0f},{-A/3.0f,A},
    {A/3.0f,-A},{A/3.0f,-A/3.0f},{A/3.0f,A/3.0f},{A/3.0f,A},
    {A,-A},{A,-A/3.0f},{A,A/3.0f},{A,A},
};
constexpr int QAM_Constellation_Size = sizeof(QAM_Constellation) / sizeof(std::complex<float>);

// constexpr float QAM_Constellation_Average_Power = 1.0f;
// constexpr float QAM_Constellation_Average_Power = 1.0f;