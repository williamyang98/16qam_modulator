#pragma once

#include <stdint.h>
#include <complex>
#include <memory>
#include <vector>

class ConstellationSpecification;
class PreambleDetector;
class AdditiveScrambler;
class ViterbiDecoder;
class CRC8_Calculator;

// Decodes a encoded payload 
// payload --> FEC --> Scrambler --> Preamble --> TX
// TX --> Preamble detector -> Descrambler --> Viterbi decoder --> payload
// Where payload has the format
// int16_t: length N
// uint8_t[N]: payload data
// uint8_t: crc8 of payload data
// uint8_t: 0x00 trellis null terminator
class FrameDecoder 
{
public:
    enum ProcessResult { 
        NONE, 
        PREAMBLE_FOUND, 
        BLOCK_SIZE_OK, 
        BLOCK_SIZE_ERR, 
        PAYLOAD_OK, 
        PAYLOAD_ERR 
    };
    enum State { 
        WAIT_PREAMBLE,
        WAIT_BLOCK_SIZE, 
        WAIT_PAYLOAD 
    };
    struct Payload {
        uint16_t length;
        uint8_t* buf;
        uint8_t crc8_received;
        uint8_t crc8_calculated;
        bool crc8_mismatch;
        int decoded_error;
        void reset() {
            length = 0;
            buf = NULL;
            crc8_received = 0;
            crc8_calculated = 0;
            crc8_mismatch = false;
            decoded_error = -1;
        }
    };
private:
    ConstellationSpecification& constellation;
    std::unique_ptr<PreambleDetector> preamble_detector;
    std::unique_ptr<AdditiveScrambler> descrambler;
    std::unique_ptr<ViterbiDecoder> vitdec;
    std::unique_ptr<CRC8_Calculator> crc8_calc;
    // internal buffers for decoding
    const int buffer_size;
    std::vector<uint8_t> descramble_buffer;
    std::vector<uint8_t> encoded_buffer;
    std::vector<uint8_t> decoded_buffer;
    // keep track of position in buffers
    int encoded_bits = 0;
    int encoded_bytes = 0;
    int decoded_bytes = 0;
    // keep track of encoded and decoded block size
    const int nb_bytes_for_block_size = 16; // minimum required bytes to decipher block size
    int decoded_block_size = 0;
    int encoded_block_size = 0; 
    State state;
    Payload payload;
public:
    FrameDecoder(
        const int _buffer_size, 
        ConstellationSpecification& _constellation,
        const uint32_t preamble_word,
        const uint16_t scrambler_syncword, 
        const uint8_t conv_poly[2],
        const uint8_t crc8_poly);
    ~FrameDecoder();
    ProcessResult process(const std::complex<float> IQ);
    inline State GetState() { return state; }
    inline Payload GetPayload() { return payload; }
private:
    ProcessResult process_await_preamble(const std::complex<float> IQ);
    // Decode the block size so we can anticipate when to stop decoding
    ProcessResult process_await_block_size(const uint8_t x, const int nb_bits);
    // Decode the rest of the payload after block size is known
    ProcessResult process_await_payload(const uint8_t x, const int nb_bits); 
    // Construct individual bytes for processing from bits
    void process_decoder_bits(const uint8_t x, const int nb_bits); 
    void reset();
};