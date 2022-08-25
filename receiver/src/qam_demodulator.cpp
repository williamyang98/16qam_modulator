#include "qam_demodulator.h"
#include "filter_designer.h"

QAM_Demodulator::QAM_Demodulator(
    CarrierDemodulatorSpecification _carrier_spec, 
    QAM_Demodulator_Specification _qam_spec,
    ConstellationSpecification* _constellation,
    CarrierToSymbolDemodulatorBuffers* _carrier_demod_buffer)
: carrier_spec(_carrier_spec), qam_spec(_qam_spec),
  constellation(_constellation),
  carrier_demod_buffer(_carrier_demod_buffer),
  rx_block_size(qam_spec.block_size*qam_spec.downsample_filter.factor),
  block_size(qam_spec.block_size)
{
    // We have a trackback length of 25 since that is an integer of 8 plus 1
    // N = 8k + 1
    vitdec = new ViterbiDecoder<encoder_decoder_type>(25);
    scrambler = new AdditiveScrambler(qam_spec.scrambler_syncword);
    crc8_calc = new CRC8_Calculator(qam_spec.crc8_polynomial);

    frame_decoder = new FrameDecoder(qam_spec.block_size);
    frame_decoder->descrambler = scrambler;
    frame_decoder->crc8_calc = crc8_calc;
    frame_decoder->vitdec = vitdec;

    preamble_detector = new PreambleDetector(qam_spec.preamble_code, 4);

    frame_synchroniser = new FrameSynchroniser();
    frame_synchroniser->constellation = constellation;
    frame_synchroniser->frame_decoder = frame_decoder;
    frame_synchroniser->preamble_detector = preamble_detector;

    carrier_demodulator = new CarrierToSymbolDemodulator(carrier_spec, constellation);
    carrier_demodulator->buffers = carrier_demod_buffer;

    IQ_downsampled_input_buffer = new std::complex<float>[rx_block_size];
    IQ_output_buffer = new std::complex<float>[block_size];

    const int ds_factor = qam_spec.downsample_filter.factor;
    if (ds_factor == 1) {
        downsample_filter = NULL;
    } else {
        const float Fsample = carrier_spec.f_sample;
        const float RX_Fsample = Fsample*ds_factor;
        const float ds_k = (Fsample/2.0f)/(RX_Fsample/2.0f);
        auto spec = create_fir_lpf(ds_k, qam_spec.downsample_filter.size);
        downsample_filter = new FIR_Filter<std::complex<float>>(spec->b, spec->N);
    }
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

    delete IQ_downsampled_input_buffer;
    delete IQ_output_buffer;

    if (downsample_filter != NULL) {
        delete downsample_filter;
    }
}

int QAM_Demodulator::Process(std::complex<uint8_t>* x)
{
    const int ds_factor = qam_spec.downsample_filter.factor;

    for (int i = 0; i < rx_block_size; i++) {
        const uint8_t I = x[i].real();
        const uint8_t Q = x[i].imag();
        IQ_downsampled_input_buffer[i].real((float)I - 127.5f);
        IQ_downsampled_input_buffer[i].imag((float)Q - 127.5f);
    }

    if (downsample_filter != NULL) {
        downsample_filter->process(
            IQ_downsampled_input_buffer, 
            IQ_downsampled_input_buffer, 
            rx_block_size);

        for (int i = 0; i < block_size; i++) {
            IQ_downsampled_input_buffer[i] = IQ_downsampled_input_buffer[i*ds_factor];
        }
    }

    const int total_symbols = carrier_demodulator->ProcessBlock(
        IQ_downsampled_input_buffer, 
        IQ_output_buffer);

    for (int i = 0; i < total_symbols; i++) {
        const auto& IQ = IQ_output_buffer[i];
        const auto res = frame_synchroniser->process(IQ);
        const auto p = frame_decoder->GetPayload();
        if (callback != NULL) {
            callback->OnFrameResult(res, p);
        }
    }

    return total_symbols;
}