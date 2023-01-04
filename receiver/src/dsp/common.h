#pragma once

namespace dsp 
{

template <typename T>
T clamp(T x, T _min, T _max) {

    T y = x;
    y = (y > _min) ? y : _min;
    y = (y > _max) ? _max : y;
    return y;
}

template <typename T>
T min(T x, T y) {
    return (x > y) ? y : x;
}

template <typename T>
T max(T x, T y) {
    return (x > y) ? x : y;
}

};