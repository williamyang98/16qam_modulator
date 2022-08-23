#pragma once

#include <stdint.h>
#include "preamble_detector.h"
#include "frame_decoder.h"
#include "constellation.h"

// A frame synchroniser consists of:
// 1. Preamble detector - Finds the correct carrier phase and symbol offset for the frame
// 2. Frame decoder - Decodes an encoded bitstream into the payload
// The receiver gets physical frames with the following structure
// RX -> [Preamble | Encoded frame] [Preamble | Encoded frame] ...
class FrameSynchroniser {
public:
    enum Result { 
        NONE, PREAMBLE_FOUND, 
        BLOCK_SIZE_ERR, BLOCK_SIZE_OK, 
        PAYLOAD_ERR, PAYLOAD_OK};
public:
    PreambleDetector* preamble_detector = NULL;
    FrameDecoder* frame_decoder = NULL;
    ConstellationSpecification* constellation = NULL;
private:
    bool is_waiting_preamble = true;
public:
    void reset() {
        is_waiting_preamble = true;
    }

    // Logic to switch between the preamble detector and frame decoder whenever a physical frame arrives
    // If we get a PAYLOAD_OK, it means the payload inside the frame decoder is ready
    Result process(const std::complex<float> IQ) {
        if (is_waiting_preamble) {
            bool res = preamble_detector->process(IQ, constellation);
            if (res) {
                is_waiting_preamble = false;
                return Result::PREAMBLE_FOUND;
            } 
            is_waiting_preamble = true;
            return Result::NONE;
        } 

        const auto phase_shift = preamble_detector->GetPhase();
        const auto sym = constellation->GetNearestSymbol(IQ * phase_shift);
        const auto res = frame_decoder->process(sym, constellation->GetBitsPerSymbol());

        using DecoderResult = FrameDecoder::ProcessResult;
        Result rv = Result::NONE;

        switch (res) {
        case DecoderResult::NONE:
            rv = Result::NONE;
            break;
        case DecoderResult::BLOCK_SIZE_OK:
            rv = Result::BLOCK_SIZE_OK;
            is_waiting_preamble = false;
            break;
        case DecoderResult::BLOCK_SIZE_ERR:
            is_waiting_preamble = true;
            rv = Result::BLOCK_SIZE_ERR;
            break;
        case DecoderResult::PAYLOAD_ERR:
            is_waiting_preamble = true;
            rv = Result::PAYLOAD_ERR;
            break;
        case DecoderResult::PAYLOAD_OK:
            is_waiting_preamble = true;
            rv = Result::PAYLOAD_OK;
            break;
        }

        return rv;
    }
};