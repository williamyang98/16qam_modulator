#pragma once

#include <stdint.h>
#include <memory>
#include <vector>

// Connect all our code together
#include "demodulator/qam_sync.h"
#include "decoder/frame_decoder.h"
#include "dsp/iir_filter.h"
#include "dsp/filter_designer.h"
#include "audio/frame.h"
#include "utility/span.h"
#include "utility/reconstruction_buffer.h"
#include "utility/observable.h"

#define PRINT_LOG 1
#if PRINT_LOG 
  #include <stdio.h>
  #define LOG_MESSAGE(...) fprintf(stderr, ##__VA_ARGS__)
#else
  #define LOG_MESSAGE(...) (void)0
#endif

// Accepts an audio frame of 8bit data and appends it to an audio buffer
// When that audio buffer has reached the end, it will print to stdout the buffer as a chunk
// Then the index is reset to the beginning, and a new audio chunk is ready
class AudioFilter {
private:
    const float Fs;
    std::vector<Frame<float>> tmp_buffer;
    std::unique_ptr<IIR_Filter<Frame<float>>> ac_filter;
    std::unique_ptr<IIR_Filter<Frame<float>>> notch_filter;
    std::vector<Frame<float>> output_buffer;
    tcb::span<Frame<float>> output_span;
    ReconstructionBuffer<Frame<float>> output_builder;
    Observable<tcb::span<const Frame<float>>> obs_on_output_block;
public:
    AudioFilter(const int _output_length, const float _Fs)
    :   Fs(_Fs),
        output_buffer(_output_length),
        output_span(output_buffer),
        output_builder(output_span)
    {
        {
            const int N_ac = TOTAL_TAPS_IIR_AC_COUPLE;
            ac_filter = std::make_unique<IIR_Filter<Frame<float>>>(N_ac);
            create_iir_ac_filter(ac_filter->get_b(), ac_filter->get_a(), 0.9999f);
        }

        {
            const float k = 50.0f/(Fs/2.0f);
            const float r = 0.9999f;
            const int N_notch = TOTAL_TAPS_IIR_SECOND_ORDER_NOTCH_FILTER;
            notch_filter = std::make_unique<IIR_Filter<Frame<float>>>(N_notch);
            create_iir_notch_filter(notch_filter->get_b(), notch_filter->get_a(), k, r);
        }
    }

    void ProcessFrame(const uint8_t* x, const int N) {
        tmp_buffer.resize(N);
        for (int i = 0; i < N; i++) {
            float v = (float)x[i];
            v = v / 255.0f;
            tmp_buffer[i] = v;
        }

        ac_filter->process(tmp_buffer.data(), tmp_buffer.data(), N);
        // notch_filter->process(tmp_buffer.data(), tmp_buffer.data(), N);

        auto rd_buffer = tcb::span(tmp_buffer);
        while (!rd_buffer.empty()) {
            const auto nb_read = output_builder.ConsumeBuffer(tmp_buffer);
            rd_buffer = rd_buffer.subspan(nb_read);
            if (output_builder.IsFull()) {
                obs_on_output_block.Notify(output_buffer);
                output_builder.Reset();
            }
        }
    }

    int GetOutputBufferSize() { return (int)output_buffer.size(); }
    auto GetOutputBuffer() { return tcb::span(output_buffer); }
    auto& OnOutputBlock() { return obs_on_output_block; }
};

// Handle data frames
class FrameHandler
{
public:
    struct {
        int total = 0;
        int incorrect = 0;
        int correct = 0;
        int corrupted = 0;
        int repaired = 0;
        float GetPacketErrorRate() {
            return  (float)incorrect / (float)total;
        }
        float GetRepairFailureRate() {
            return (float)repaired / (float)correct;
        }
        void reset() {
            total = 0;
            incorrect = 0;
            correct = 0;
            corrupted = 0;
            repaired = 0;
        }
    } stats;
    bool is_output_audio = true;
    bool is_output_data = false;
private:
    AudioFilter& audio;
public: 
    FrameHandler(AudioFilter& _audio)
    : audio(_audio) {}

    void OnFrameResult(
        FrameDecoder::ProcessResult res, 
        FrameDecoder::Payload payload)
    {
        const int AUDIO_PACKET_BLOCK_SIZE = 100;

        using Res = FrameDecoder::ProcessResult;
        switch (res) {
        case Res::PREAMBLE_FOUND:
            break;
        case Res::BLOCK_SIZE_OK:
            break;
        case Res::BLOCK_SIZE_ERR:
            LOG_MESSAGE("Got invalid block size: %d\n", payload.length);
            stats.corrupted++;
            break;
        case Res::PAYLOAD_ERR:
            stats.total++;
            stats.incorrect++;
            break;
        case Res::PAYLOAD_OK:
            stats.total++;
            stats.correct++;
            if (payload.decoded_error > 0) {
                stats.repaired++;
            }
            if (payload.length == AUDIO_PACKET_BLOCK_SIZE) {
                if (is_output_audio) {
                    audio.ProcessFrame(payload.buf, payload.length);
                }
            } else {
                if (is_output_data) {
                    LOG_MESSAGE("Received data[%d]=%.*s\n", 
                        (int)payload.length, 
                        (int)payload.length, reinterpret_cast<char*>(payload.buf));
                }
            }
            break;
        case Res::NONE:
        default:
            break;
        }
    }
};

// Connect our code together into a cohesive 16QAM receiver
class App 
{
public:
    QAM_Synchroniser_Specification qam_sync_spec;
    struct {
        bool rebuild = false;
        bool snapshot = false;
    } controls;
    bool is_read_loop = false;
    bool is_running = true;
private:
    FILE* rx_fp;
    std::unique_ptr<ConstellationSpecification> constellation;
    std::unique_ptr<QAM_Synchroniser_Buffer> active_buffer;
    std::unique_ptr<QAM_Synchroniser_Buffer> snapshot_buffer;

    std::unique_ptr<QAM_Synchroniser> qam_sync;
    std::unique_ptr<FrameDecoder> frame_decoder;
    std::unique_ptr<FrameHandler> audio_frame_handler;
    std::unique_ptr<AudioFilter> audio_filter;
public:
    App(
        FILE* _rx_fp, const int demod_block_size,
        const int decoder_block_size, const int ds_factor, const int us_factor,
        const int audio_block_size, const float F_audio) 
    : rx_fp(_rx_fp)
    {
        constellation = std::make_unique<SquareConstellation>(4);
        active_buffer = std::make_unique<QAM_Synchroniser_Buffer>(demod_block_size, ds_factor, us_factor);
        snapshot_buffer = std::make_unique<QAM_Synchroniser_Buffer>(demod_block_size, ds_factor, us_factor);

        {
            const uint32_t preamble_code = 0b11111001101011111100110101101101;
            const uint16_t scrambler_syncword = 0b1000010101011001;
            const uint8_t conv_poly[2] = { 0b111, 0b101 };
            const uint8_t crc8_polynomial = 0xD5;
            frame_decoder = std::make_unique<FrameDecoder>(
                decoder_block_size,
                *(constellation.get()),
                preamble_code,
                scrambler_syncword,
                conv_poly,
                crc8_polynomial);
        }

        audio_filter = std::make_unique<AudioFilter>(audio_block_size, F_audio);
        audio_frame_handler = std::make_unique<FrameHandler>(*(audio_filter.get()));

        // NOTE: Demodulator has to be built by user
    }
    void Run() {
        is_running = true;
        int rd_total_blocks = 0;
        while (is_running) {
            // read baseband
            auto rx_buffer = active_buffer->x_raw;
            auto rx_length = active_buffer->GetInputSize();
            size_t rd_block_size = fread(rx_buffer.data(), sizeof(std::complex<uint8_t>), rx_length, rx_fp);
            if (rd_block_size != rx_length) {
                LOG_MESSAGE("Got mismatched block size after %d blocks\n", rd_total_blocks);
                if (is_read_loop) {
                    fseek(rx_fp, 0, 0);
                    continue;
                }
                break;
            }
            rd_total_blocks++; 

            // Run decoder chain
            if (qam_sync) {
                const int nb_symbols = qam_sync->ProcessBlock(*(active_buffer.get()));
                auto syms = active_buffer->y_out.first(nb_symbols);
                for (auto& sym: syms) {
                    auto res = frame_decoder->process(sym);
                    auto payload = frame_decoder->GetPayload();
                    audio_frame_handler->OnFrameResult(res, payload);
                }
            }

            if (ReadFlag(controls.snapshot)) {
                snapshot_buffer->CopyFrom(*(active_buffer.get()));
            }

            if (ReadFlag(controls.rebuild)) {
                BuildDemodulator();
            }
        }
    }
    void Stop() {
        is_running = false;
    }
    void BuildDemodulator() {
        qam_sync = std::make_unique<QAM_Synchroniser>(qam_sync_spec, *(constellation.get()));
    }
public:
    auto& GetActiveBuffer() { return *(active_buffer.get()); }
    auto& GetSnapshotBuffer() { return *(snapshot_buffer.get()); }
    auto& GetAudioFilter() { return *(audio_filter.get()); }
    auto& GetFrameHandler() { return *(audio_frame_handler.get()); }
private:
    bool ReadFlag(bool& flag) {
        const bool rv = flag;
        flag = false;
        return rv;
    }
};