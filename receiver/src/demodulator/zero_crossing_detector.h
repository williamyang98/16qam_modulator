#pragma once

class Zero_Crossing_Detector 
{
public:
    float xn = 0.0f;
public:
    // return true if zero crossing detected
    bool process(const float x) {
        // crossing occurs if the signals are on opposite sides of the x-axis
        const bool is_crossed = (x*xn) < 0.0f;
        xn = x;
        return is_crossed;
    }
};

