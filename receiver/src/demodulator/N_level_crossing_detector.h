#pragma once

#include <vector>

class N_Level_Crossing_Detector
{
private:
    std::vector<float> levels;
    const int N;
    int curr_level;
public:
    N_Level_Crossing_Detector(const float* _levels, const int _N)
    : N(_N)
    {
        levels.resize(N);
        curr_level = 0;
        for (int i = 0; i < N; i++) {
            levels[i] = _levels[i];
        }
    }

    bool process(const float x) {
        // Find close level
        int new_level = curr_level;
        for (int i = 0; i < N; i++) {
            if (x > levels[i]) {
                new_level = i;
                break;
            }
        }

        // Check if level changed
        const bool is_crossed = (new_level != curr_level);
        curr_level = new_level;
        return is_crossed;
    }
};

