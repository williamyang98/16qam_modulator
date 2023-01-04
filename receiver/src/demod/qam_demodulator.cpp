#include "qam_demodulator.h"

QAM_Demodulator::QAM_Demodulator(
    CarrierDemodulatorSpecification _carrier_spec, 
    QAM_Demodulator_Specification _qam_spec,
    ConstellationSpecification* _constellation)
: carrier_spec(_carrier_spec), qam_spec(_qam_spec),
  constellation(_constellation)
{
    const uint8_t POLY[CODE_RATE] = { 0b111, 0b101 };
    vitdec = new ViterbiDecoder(POLY, _qam_spec.buffer_size*8);
    scrambler = new AdditiveScrambler(qam_spec.scrambler_syncword);
    crc8_calc = new CRC8_Calculator(qam_spec.crc8_polynomial);

    frame_decoder = new FrameDecoder(qam_spec.buffer_size);
    frame_decoder->descrambler = scrambler;
    frame_decoder->crc8_calc = crc8_calc;
    frame_decoder->vitdec = vitdec;

    preamble_detector = new PreambleDetector(qam_spec.preamble_code, 4);

    frame_synchroniser = new FrameSynchroniser();
    frame_synchroniser->constellation = constellation;
    frame_synchroniser->frame_decoder = frame_decoder;
    frame_synchroniser->preamble_detector = preamble_detector;

    carrier_demodulator = new CarrierToSymbolDemodulator(carrier_spec, constellation);
}

QAM_Demodulator::~QAM_Demodulator() 
{
    delete preamble_detector;
    delete frame_decoder;
    delete scrambler;
    delete crc8_calc;
    delete vitdec;

    delete frame_synchroniser;
    delete carrier_demodulator;
}

int QAM_Demodulator::Process(CarrierToSymbolDemodulatorBuffers* buffers)
{

    const int total_symbols = carrier_demodulator->ProcessBlock(buffers);

    for (int i = 0; i < total_symbols; i++) {
        const auto& IQ = buffers->y_out[i];
        const auto res = frame_synchroniser->process(IQ);
        const auto p = frame_decoder->GetPayload();
        if (callback != NULL) {
            callback->OnFrameResult(res, p);
        }
    }

    return total_symbols;
}