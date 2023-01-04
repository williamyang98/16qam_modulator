#pragma once

// prevent trigger from refiring too quickly
class Trigger_Cooldown 
{
public:
    int N_cooldown = 1;
    int N_remain = 0;
    // only propagate trigger signal if sample cooldown has passed
    bool on_trigger(const bool trig) {
        if (trig && (N_remain == 0)) {
            N_remain = N_cooldown;
            return true;
        }
        if (N_remain > 0) {
            N_remain--;
        }
        return false;
    }
};