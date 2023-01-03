#pragma once

#include "carrier_dsp.h"
#include "frame_synchroniser.h"
#include "constellation.h"

// Set configuration options for QAM demodulator
struct QAM_Demodulator_Specification {
    uint32_t preamble_code = 0b11111001101011111100110101101101;
    uint16_t scrambler_syncword = 0b1000010101011001;
    uint8_t crc8_polynomial = 0xD5;
    int buffer_size = 1024;
};

// When the QAM demodulator has received a frame result, send to the callback handler
struct QAM_Demodulator_Callback {
    virtual void OnFrameResult(FrameSynchroniser::Result res, FrameDecoder::Payload payload) = 0;
};  

// A light wrapper that connects all of the blocks into a demodulator circuit
class QAM_Demodulator
{
// Building blocks for QAM demodulator
private:
    // Frame synchroniser blocks
    PreambleDetector* preamble_detector;
    FrameDecoder* frame_decoder;
    AdditiveScrambler* scrambler;
    CRC8_Calculator* crc8_calc;
    ViterbiDecoder<encoder_decoder_type>* vitdec;

    FrameSynchroniser* frame_synchroniser;
    CarrierToSymbolDemodulator* carrier_demodulator;

    CarrierToSymbolDemodulatorBuffers* carrier_demod_buffer;
private:
    const CarrierDemodulatorSpecification carrier_spec; 
    const QAM_Demodulator_Specification qam_spec;
private:
    // externally provided resources (not managed by this class) 
    ConstellationSpecification* constellation;
    QAM_Demodulator_Callback* callback = NULL;
public:
    QAM_Demodulator(
        CarrierDemodulatorSpecification _carrier_spec, 
        QAM_Demodulator_Specification _qam_spec,
        ConstellationSpecification* _constellation);
    ~QAM_Demodulator();
    int Process(CarrierToSymbolDemodulatorBuffers* buffers);
    void SetCallback(QAM_Demodulator_Callback* _callback) {
        callback = _callback;
    }
};