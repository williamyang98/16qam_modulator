#pragma once

#include "carrier_dsp.h"
#include "frame_synchroniser.h"
#include "constellation.h"

// Set configuration options for QAM demodulator
struct QAM_Demodulator_Specification {
    uint32_t preamble_code = 0b11111001101011111100110101101101;
    uint16_t scrambler_syncword = 0b1000010101011001;
    uint8_t crc8_polynomial = 0xD5;
    int block_size = 8192;
    struct {
        int factor = 1;
        int size = 20;
    } downsample_filter;
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
private:
    const CarrierDemodulatorSpecification carrier_spec; 
    const QAM_Demodulator_Specification qam_spec;
private:
    std::complex<float>* IQ_downsampled_input_buffer;
    FIR_Filter<std::complex<float>>* downsample_filter;
    std::complex<float>* IQ_output_buffer;
private:
    // externally provided resources (not managed by this class) 
    CarrierToSymbolDemodulatorBuffers* carrier_demod_buffer;
    ConstellationSpecification* constellation;
    QAM_Demodulator_Callback* callback = NULL;
private:
    const int rx_block_size;
    const int block_size;
public:
    QAM_Demodulator(
        CarrierDemodulatorSpecification _carrier_spec, 
        QAM_Demodulator_Specification _qam_spec,
        ConstellationSpecification* _constellation,
        CarrierToSymbolDemodulatorBuffers* _carrier_demod_buffer);
    ~QAM_Demodulator();
    int GetBlockSize() { return qam_spec.block_size; }
    int Process(std::complex<uint8_t>* x);
    void SetCallback(QAM_Demodulator_Callback* _callback) {
        callback = _callback;
    }
};