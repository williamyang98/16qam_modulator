#pragma once

#include <stdint.h>

// Preamble filter interface
class PreambleFilter {
public:
    virtual ~PreambleFilter() {};
    virtual void reset() = 0;
    virtual bool process(const uint8_t sym, const int nb_bits) = 0;
    virtual int get_length() = 0;
};

// shift register which searches for preamble sequence
template <typename T>
class VariablePreambleFilter: public PreambleFilter {
private:
    T reg;
    const T preamble;
public:
    VariablePreambleFilter(const T _preamble) 
    : preamble(_preamble) {
        reg = 0;
    }
    virtual ~VariablePreambleFilter() {};
    virtual void reset() {
        reg = 0;
    }
    virtual bool process(const uint8_t sym, const int nb_bits) {
        const uint8_t mask = (1<<nb_bits)-1;
        reg = reg << nb_bits;
        reg = reg | (sym & mask);

        return (reg ^ preamble) == 0;
    }

    virtual int get_length() {
        return sizeof(T)*8;
    }
};
