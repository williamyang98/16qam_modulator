#pragma once

#include <vector>

// delay line for trigger pulses
class Delay_Line 
{
private:
    int curr_count;
    // number of triggers to store in memory
    const int N;
    std::vector<int> counts;
public:
public:
    Delay_Line(const int _N)
    : N(_N)
    {
        counts.resize(N, -1);
        curr_count = 0;
    }
    // return false if we couldn't add this to delay line
    bool add(int delay) {
        // if the delay line is full, we ignore this pulse
        if (curr_count >= N) {
            return false;
        }

        curr_count++;
        for (int i = 0; i < N; i++) {
            if (counts[i] < 0) {
                counts[i] = delay;
                break;
            }
        }
        return true;
    }
    // propagate any delayed trigger signals
    bool process() {
        bool trig_out = false;
        for (int i = 0; i < N; i++) {
            if (counts[i] < 0) {
                continue;
            }

            // remove a sample
            if (counts[i] == 0) {
                trig_out = true;
                curr_count--;
            }         
            counts[i]--;
        }
        return trig_out;
    }
};