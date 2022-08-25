// simulate the rtlsdr output from the transmitter 
// we are outputing a series of symbols with
// 1. preamble
// 2. (scrambler + convolutional code) as encoding
// 3. (length + data + crc8 + trellis-terminator) as payload

#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#include <cmath>
#include <chrono>
#include <thread>

#include "encoding.h"
#include "additive_scrambler.h"
#include "util.h"
#include "crc32.h"
#include "crc8.h"

#include "getopt/getopt.h"

#include <io.h>
#include <fcntl.h>

struct IQ_Symbol {
    uint8_t I;
    uint8_t Q;
};

// pad preamble bits to be byte aligned
// 2x13-barker codes and 1x2-code and 1x4-code
constexpr uint32_t PREAMBLE_CODE = 0b11111001101011111100110101101101;
constexpr uint16_t SCRAMBLER_CODE = 0b1000010101011001;
constexpr uint32_t CRC32_POLY = 0x04C11DB7;
constexpr uint8_t CRC8_POLY = 0xD5;

auto enc = ConvolutionalEncoder<encoder_decoder_type>();
auto scrambler = AdditiveScrambler(SCRAMBLER_CODE);
auto crc32_calc = CRC32_Calculator(CRC32_POLY);
auto crc8_calc = CRC8_Calculator(CRC8_POLY);

void usage() {
    fprintf(stderr, 
        "simulate_transmitter, a simulator for a 4PSK or 16QAM signal\n\n"
        "Usage:\t[-D (dump encoded IQ symbols out to stdout)]\n"
        "\t[-h (show usage)]\n"
        "\t[-R (dont run in real time)]\n"
        "\t[-f sample rate (default: 2MHz)]\n"
        "\t[-s symbol rate (default: 50kHz)]\n"
        "\t[-b block size (default: 4096)]\n"
        "\t[-t recording time (default: 1s)]\n"
        "\t[-m modulation type (default: 16qam)]\n"
        "\t    options: [16QAM, 4QAM]\n"
    );
}

int create_frame(uint8_t* x, const int Nx, uint8_t* y, const int Ny);
int create_test_data(uint8_t** x);
int create_16QAM_symbols(uint8_t x, IQ_Symbol* syms);
int create_4QAM_symbols(uint8_t x, IQ_Symbol* syms);

int main(int argc, char** argv) {
    // check if argument was passed in to dump a byte stream
    int opt;
    bool is_real_time = true;
    bool is_dumping = false;
    float Fs = 2e6;
    float Fsym = 50e3;
    int block_size = 4096;
    float recording_time = 1.0f;

    enum ModulationType { QAM16, QPSK };
    ModulationType modulation_type = ModulationType::QAM16;

    while ((opt = getopt(argc, argv, "s:b:t:m:f:DhR")) != -1) {
        switch (opt) {
        case 'D':
            is_dumping = true;
            break;
        case 'h':
            usage();
            return 0;
        case 'R':
            is_real_time = false;
            break;
        case 'f':
            Fs = static_cast<float>(atof(optarg));
            break;
        case 's':
            Fsym = static_cast<float>(atof(optarg));
            break;
        case 'b':
            block_size = static_cast<int>(atof(optarg));
            break;
        case 't':
            recording_time = static_cast<float>(atof(optarg));
            break;
        case 'm':
            if (strncmp(optarg, "16qam", 5) == 0) {
                modulation_type = ModulationType::QAM16;
            } else if (strncmp(optarg, "4qam", 4) == 0) {
                modulation_type = ModulationType::QPSK;
            } else if (strncmp(optarg, "4psk", 4) == 0) {
                modulation_type = ModulationType::QPSK;
            } else {
                fprintf(stderr, "Unknown modulation type: %s\n", optarg);
                return 1;
            }
            break;
        case '?':
            usage();
            return 0;
        }
    }

    // NOTE: Windows does extra translation stuff that messes up the file if this isn't done
    // https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/setmode?view=msvc-170
    _setmode(_fileno(stdout), _O_BINARY);

    uint8_t* test_data = NULL;
    int total_encoded = create_test_data(&test_data);
    fprintf(stderr, "Generated %dB\n", total_encoded);

    if (is_dumping) {
        printf("{");
        for (int i = 0; i < total_encoded; i++) {
            printf("0x%02X,", test_data[i]);
        }
        printf("};\n");
        delete [] test_data;
        return 0;
    }


    const float F_block = Fs/(float)(block_size);
    const float T_block = 1.0f/F_block;
    const int N_blocks = static_cast<int>(std::ceil(recording_time/T_block));
    int curr_block = 0;
    
    // number of samples per symbol
    const int Nsamples = (int)std::round(Fs/Fsym);
    const int T_block_microseconds = static_cast<int>(std::ceil(T_block * 1e6));

    uint8_t* tx_block = new uint8_t[block_size*2];
    int curr_tx_block_idx = 0;

    IQ_Symbol sym_out[8];

    // convert to symbols
    auto dt_prev_tx = std::chrono::high_resolution_clock::now();
    while (1) {
        for (int i = 0; i < total_encoded; i++) {
            const auto x = test_data[i];
            int total_symbols = 0;
            switch (modulation_type) {
            case ModulationType::QAM16:
                total_symbols = create_16QAM_symbols(x, sym_out);
                break;
            case ModulationType::QPSK:
            default:
                total_symbols = create_4QAM_symbols(x, sym_out);
                break;
            }

            for (int j = 0; j < total_symbols; j++) {
                auto sym = sym_out[j];

                for (int k = 0; k < Nsamples; k++) {
                    tx_block[curr_tx_block_idx  ] = sym.I;
                    tx_block[curr_tx_block_idx+1] = sym.Q;
                    curr_tx_block_idx += 2;

                    // transmit the block
                    if (curr_tx_block_idx >= (block_size*2)) {
                        fwrite(tx_block, 2, block_size, stdout);
                        curr_tx_block_idx = 0;
                        curr_block++;

                        // if it is real time, add a delay
                        if (is_real_time) {
                            auto dt_curr_tx = std::chrono::high_resolution_clock::now();
                            const auto dt = std::chrono::duration_cast<std::chrono::microseconds>(dt_curr_tx-dt_prev_tx);
                            int T_offset =  static_cast<int>(dt.count());
                            int delay = T_offset - T_block_microseconds;
                            dt_prev_tx = dt_curr_tx;
                            std::this_thread::sleep_for(std::chrono::microseconds((T_block_microseconds - delay)/2));
                        // if this isn't real time, we will impose a time recording limit
                        } else {
                            if (curr_block >= N_blocks) {
                                return 0;
                            }
                        }
                   }
                }
            }
        }
    }

    delete [] tx_block;
    delete [] test_data;

    return 0;
}

int create_16QAM_symbols(uint8_t x, IQ_Symbol* syms) {
    static const uint8_t _gray_code[4] = {0b00, 0b01, 0b11, 0b10};

    uint8_t I1 = (x & 0b11000000) >> 6;
    uint8_t Q1 = (x & 0b00110000) >> 4;
    uint8_t I2 = (x & 0b00001100) >> 2;
    uint8_t Q2 = (x & 0b00000011);

    I1 = _gray_code[I1]*8 + 128;
    Q1 = _gray_code[Q1]*8 + 128;
    I2 = _gray_code[I2]*8 + 128;
    Q2 = _gray_code[Q2]*8 + 128;

    syms[0].I = I1;
    syms[0].Q = Q1;
    syms[1].I = I2;
    syms[1].Q = Q2;

    return 2;
}

int create_4QAM_symbols(uint8_t x, IQ_Symbol* syms) {
    // NOTE: 4QAM is already gray code by itself
    int j = 0;
    for (int i = 0; i < 8; i+=2) {
        const int shift = 7-i;
        uint8_t I = (x >> shift    ) & 0x1;
        uint8_t Q = (x >> (shift-1)) & 0x1;
        syms[j].I = I*8 + 128;
        syms[j].Q = Q*8 + 128;
        j++;
    }
    return 4;
}

int create_frame(uint8_t* x, const int Nx, uint8_t* y, const int Ny) {
    // encoding size is given as the following
    // 4: preamble
    // additive scrambler + fec of 1/2 K=3 [7,5] code
    // 2: length of payload
    // N: payload
    // 1: CRC8 
    // 1: NULL trellis terminator

    // T = 4 + 2*(2+N+1+1)
    // T = 2N + 12

    assert(sizeof(encoder_decoder_type) == G_SYM_SIZE);

    int offset = 0;
    offset += push_big_endian_byte(&y[offset], PREAMBLE_CODE);

    const int scrambler_offset = offset;

    enc.reset();
    scrambler.reset();

    uint16_t Nx_copy = static_cast<uint16_t>(Nx);
    auto Nx_addr = reinterpret_cast<uint8_t*>(&Nx_copy);
    for (int i = 0; i < sizeof(Nx_copy); i++) {
        auto enc_out = enc.consume_byte(Nx_addr[i]);
        offset += push_big_endian_byte(&y[offset], enc_out);
    }

    for (int i = 0; i < Nx; i++) {
        auto enc_out = enc.consume_byte(x[i]);
        offset += push_big_endian_byte(&y[offset], enc_out);
    }

    uint8_t crc8 = crc8_calc.process(x, Nx);
    auto crc8_addr = reinterpret_cast<uint8_t*>(&crc8);
    for (int i = 0; i < sizeof(crc8); i++) {
        auto enc_out = enc.consume_byte(crc8_addr[i]);
        offset += push_big_endian_byte(&y[offset], enc_out);
    }

    {
        uint8_t trellis_terminator = 0x00;
        auto enc_out = enc.consume_byte(trellis_terminator);
        offset += push_big_endian_byte(&y[offset], enc_out);
    }

    for (int i = scrambler_offset; i < offset; i++) {
        y[i] = scrambler.process(y[i]);
    }


    return offset;
}

int create_test_data(uint8_t** x) {
    uint8_t data1[] = "Test string 1";
    const int data1_size = sizeof(data1) / sizeof(uint8_t);

    uint8_t data2[] = "Test string 2";
    const int data2_size = sizeof(data2) / sizeof(uint8_t);

    uint8_t data3[] = "Trust in the LORD with all your heart, and do not lean on your own understanding. In all your ways acknowledge Him, and He will make straight your paths. May the God of hope fill you with all joy and peace as you trust in Him, so that you may overflow with hope by the power of the Holy Spirit.";
    const int data3_size = sizeof(data3) / sizeof(uint8_t);

    uint8_t data4[] = "Test string 4";
    const int data4_size = sizeof(data4) / sizeof(uint8_t);

    int total_data = 0;
    total_data += data1_size;
    total_data += data2_size;
    total_data += data3_size;
    total_data += data4_size;

    int total_encoded = 0;
    total_encoded += (2*data1_size + 12);
    total_encoded += (2*data2_size + 12);
    total_encoded += (2*data3_size + 12);
    total_encoded += (2*data4_size + 12);

    // encoding size is given as the following
    // 4: preamble
    // additive scrambler + fec of 1/2 K=3 [7,5] code
    // 2: length of payload
    // N: payload
    // 1: CRC8 
    // 1: Trellis terminator 

    // T = 4 + 2*(2+N+1)
    // T = 2N + 12

    uint8_t* frame = new uint8_t[total_encoded];

    int total_written = 0;
    total_written += create_frame(data4, data4_size, &frame[total_written], total_encoded-total_written);
    total_written += create_frame(data3, data3_size, &frame[total_written], total_encoded-total_written);
    total_written += create_frame(data2, data2_size, &frame[total_written], total_encoded-total_written);
    total_written += create_frame(data1, data1_size, &frame[total_written], total_encoded-total_written);

    *x = frame;
    return total_encoded;
}
