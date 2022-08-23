#include "frame_decoder.h"

FrameDecoder::FrameDecoder(const int _buffer_size)
: buffer_size(_buffer_size) 
{
    state = State::WAIT_BLOCK_SIZE;

    descramble_buffer = new uint8_t[buffer_size];
    encoded_buffer = new uint8_t[buffer_size];
    decoded_buffer = new uint8_t[buffer_size];
}

FrameDecoder::~FrameDecoder() {
    delete [] descramble_buffer;
    delete [] encoded_buffer;
    delete [] decoded_buffer;
}

FrameDecoder::ProcessResult FrameDecoder::process(const uint8_t x, const int nb_bits) {
    switch (state) {
    // fall through when idle to block size processing
    case State::IDLE:
        {
            reset();
            payload.reset();
            state = State::WAIT_BLOCK_SIZE;
        }
    case State::WAIT_BLOCK_SIZE:
        return process_await_block_size(x, nb_bits);
    case State::WAIT_PAYLOAD:
        return process_await_payload(x, nb_bits);
    default:
        return ProcessResult::NONE;
    }
}

FrameDecoder::ProcessResult FrameDecoder::process_await_block_size(const uint8_t x, const int nb_bits) {
    process_decoder_bits(x, nb_bits);

    // Get block size once we have enough bytes decoded
    bool is_done = 
        (encoded_bytes >= nb_bytes_for_block_size) &&
        (encoded_bits == 0);
    
    if (!is_done) {
        return ProcessResult::NONE;
    }

    decoded_bytes += vitdec->process(
        &encoded_buffer[0], 
        nb_bytes_for_block_size, 
        &decoded_buffer[0], 
        buffer_size-decoded_bytes,
        false);

    const uint16_t rx_block_size = *reinterpret_cast<uint16_t*>(&decoded_buffer[0]);
    payload.length = rx_block_size;

    constexpr int frame_overhead = 2+1+1; // 2byte length and crc8 and null terminator

    // our receiving block size must fit into the buffer with length and crc
    const int min_block_size = nb_bytes_for_block_size/2 - 3;
    const int max_block_size = buffer_size/2 - frame_overhead;

    if ((rx_block_size > max_block_size) || (rx_block_size < min_block_size)) {
        payload.buf = NULL; 
        state = State::IDLE; 
        return ProcessResult::BLOCK_SIZE_ERR;
    } else {
        payload.buf = NULL;
        decoded_block_size = rx_block_size;
        encoded_block_size = 2*(decoded_block_size+frame_overhead);
        state = State::WAIT_PAYLOAD; 
        return ProcessResult::BLOCK_SIZE_OK;
    }
}

FrameDecoder::ProcessResult FrameDecoder::process_await_payload(const uint8_t x, const int nb_bits) {
    process_decoder_bits(x, nb_bits);
    bool is_done = 
        (encoded_bytes >= encoded_block_size) &&
        (encoded_bits == 0);

    if (!is_done) {
        return ProcessResult::NONE;
    }

    // Once we have received the known number of encoded bytes, decode the rest of the frame
    // We flush the viterbi decoder to get all of the predicted bits
    // NOTE: We have a known byte of 0x00 as the Trellis terminator
    decoded_bytes += vitdec->process(
        &encoded_buffer[nb_bytes_for_block_size], 
        encoded_block_size-nb_bytes_for_block_size, 
        &decoded_buffer[decoded_bytes], 
        buffer_size-decoded_bytes,
        true);

    // packet structure
    // 0:1 -> uint16_t length
    // 2:2+N -> uint8_t* payload
    // Let K = 2+N+1
    // K:K -> uint8_t crc8
    // K+1:K+1 -> uint8_t trellis null terminator 
    constexpr int frame_length_field_size = 2;
    const int crc8_length = 1;
    const int trellis_terminator_length = 1;

    uint8_t* payload_buf = &decoded_buffer[frame_length_field_size];

    const uint8_t crc8_true = decoded_buffer[decoded_bytes-(crc8_length+trellis_terminator_length)];
    const uint8_t crc8_pred = crc8_calc->process(payload_buf, decoded_block_size);
    const auto dist_err = vitdec->get_curr_error();
    const bool crc8_mismatch = (crc8_true != crc8_pred);

    state = State::IDLE;

    payload.buf = payload_buf;
    payload.crc8_calculated = crc8_pred;
    payload.crc8_received = crc8_true;
    payload.crc8_mismatch = crc8_mismatch;
    payload.decoded_error = dist_err;


    if (crc8_mismatch) {
        return ProcessResult::PAYLOAD_ERR;
    } else {
        return ProcessResult::PAYLOAD_OK;
    }
}

void FrameDecoder::process_decoder_bits(const uint8_t x, const int nb_bits) {
    if (encoded_bits == 0) {
        descramble_buffer[encoded_bytes] = 0;
    }

    const int shift = 8-encoded_bits-nb_bits;
    descramble_buffer[encoded_bytes] |= (x << shift);
    encoded_bits += nb_bits;
    
    // if a byte has been processed, then unscramble and move onto next byte
    if (encoded_bits == 8) {
        encoded_bits = 0;
        encoded_buffer[encoded_bytes] = descrambler->process(descramble_buffer[encoded_bytes]);
        encoded_bytes += 1;
    }
}

void FrameDecoder::reset() {
    // NOTE: we need to clear this since the viterbi decoder ORs in the bits
    for (int i = 0; i < decoded_bytes; i++) {
        decoded_buffer[i] = 0;
    }

    encoded_bits = 0;
    encoded_bytes = 0;
    decoded_bytes = 0;

    decoded_block_size = 0;
    encoded_block_size = 0;

    descrambler->reset();
    vitdec->reset();
}