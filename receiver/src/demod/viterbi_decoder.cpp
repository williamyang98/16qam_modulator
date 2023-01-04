#include "viterbi_decoder.h"
#include "phil_karn_viterbi_decoder.h"
#include <assert.h>

ViterbiDecoder::ViterbiDecoder(const uint8_t _poly[CODE_RATE], const int _input_bits) 
// Worst case scenario we have equal number of encoded and decoded bits
: max_decoded_bits(_input_bits), 
  max_depunctured_bits(_input_bits*CODE_RATE)
{
    vitdec = create_viterbi(_poly, _input_bits, SOFT_DECISION_VITERBI_HIGH, SOFT_DECISION_VITERBI_LOW);
    depunctured_bits.resize(max_depunctured_bits);
    Reset();
}

ViterbiDecoder::~ViterbiDecoder() {
    delete_viterbi(vitdec);
}

void ViterbiDecoder::Reset() {
    init_viterbi(vitdec, 0);
}

void ViterbiDecoder::Update(tcb::span<const uint8_t> encoded_bytes)
{
    const int nb_encoded_bytes = (int)encoded_bytes.size();
    const int nb_encoded_bits = nb_encoded_bytes*8;
    const int nb_decoded_bits = nb_encoded_bits/CODE_RATE;

    assert(nb_encoded_bits <= max_depunctured_bits);

    for (int i = 0; i < nb_encoded_bytes; i++) {
        const uint8_t b = encoded_bytes[i];
        for (int j = 0; j < 8; j++) {
            const bool v = (b & (1 << (7-j))) != 0;
            const viterbi_bit_t x = v ? SOFT_DECISION_VITERBI_HIGH : SOFT_DECISION_VITERBI_LOW; 
            depunctured_bits[i*8 + j] = x;
        }
    }

    update_viterbi_blk_scalar(vitdec, depunctured_bits.data(), nb_decoded_bits);
}

void ViterbiDecoder::GetTraceback(tcb::span<uint8_t> out_bytes) {
    const int nb_decoded_bits = (int)(out_bytes.size() * 8);
    chainback_viterbi(vitdec, out_bytes.data(), nb_decoded_bits, 0);
}

int16_t ViterbiDecoder::GetPathError(const int state) {
    return get_error_viterbi(vitdec, state);
}