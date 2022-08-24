/*
 * transmitter.c
 *
 * Created: 12/08/2022 3:26:55 PM
 * Author : acidi
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "crc8.h"
#include "crc32.h"
#include "conv_enc.h"
#include "scrambler.h"

#define DEBUG_SIGNALS 1
#define DEBUG_PORT PORTB

#define DEBUG_PIN_ENCODING	PORTB0 // pin 8		Pulse to keep track of time spent encoding
#define DEBUG_PIN_PACKET	PORTB5 // pin 13	Toggled every time a packet has been created
#define DEBUG_PIN_TRANSMIT	PORTB3 // pin 11	Pulse to keep track of time spent on transmit interrupt
#define DEBUG_PIN_ADC		PORTB4 // pin 12	Pulse to keep track of time spent on ADC interrupt

// Constants used for our encoding pipeline
const uint32_t PREAMBLE_CODE = 0b11111001101011111100110101101101;
const uint16_t SCRAMBLER_CODE = 0b1000010101011001;
const uint8_t CRC8_POLY = 0xD5;

// Generate gray coding for better FEC
uint8_t GRAY_CODE[16] = {0};

// Formula for encoded frame size
// N (payload), 2 (length), 1 (crc8), 1 (zero byte for trellis terminator)
// M = 2*(N+2+1+1) + 4
// M = 2*N+12

#define ADC_BUFFER_SIZE 100
// ADC frame is 212B
#define ADC_ENC_FRAME_SIZE ((ADC_BUFFER_SIZE+2+1+1)*2 + 4)

// Message frame is 38B
#define SAMPLE_MESSAGE_LENGTH 13
#define MESSAGE_ENC_FRAME_SIZE ((SAMPLE_MESSAGE_LENGTH+2+1+1)*2 + 4)

// Each frame transmitted at 250B blocks
// ADC frame is 212B
// Message frame is 38B
// We want to fit 1 audio frame and some extra data
// NOTE: We are unpacking the data into 4 bit chunks so the transmit ISR will run faster
#define TX_BUFFER_SIZE 250*2

// Our extra data frame
uint8_t SAMPLE_MESSAGE[SAMPLE_MESSAGE_LENGTH] = {0};

// ADC buffers
uint8_t ADC_BUFFER_0[ADC_BUFFER_SIZE] = {0};
uint8_t ADC_BUFFER_1[ADC_BUFFER_SIZE] = {0};

// Offset of 2 bytes for the uint16_t length field
volatile uint8_t* ADC_BUFFER_WR = ADC_BUFFER_0;
volatile uint8_t* ADC_BUFFER_RD = ADC_BUFFER_1;
volatile int curr_adc_wr_byte = 0;
volatile uint8_t adc_buffer_rd_ready = 1;

// Encoding and transmit buffers
uint8_t TX_BUFFER_0[TX_BUFFER_SIZE] = {0};
uint8_t TX_BUFFER_1[TX_BUFFER_SIZE] = {0};
uint8_t TX_BUFFER_PACKED[TX_BUFFER_SIZE/2] = {0};

volatile uint8_t* TX_BUFFER_WR = TX_BUFFER_0;
volatile uint8_t* TX_BUFFER_RD = TX_BUFFER_1;
volatile uint32_t curr_tx_rd_byte = 0;
volatile uint8_t tx_buffer_wr_ready = 1;

// Unpack preamble code into 4bit chunks for 16QAM 
#define PREAMBLE_UNPACKED_SIZE 8
uint8_t PREAMBLE_CODE_UNPACKED[PREAMBLE_UNPACKED_SIZE] = {0};

// Do as much preencoding as possible to reduce overhead
void generate_preencodings();

int generate_packet(uint8_t* x, const int N, uint8_t* y);
static inline void write_adc_buffer();
static inline void read_transmit_buffer();

void init(void) {
	// Timer 1 channels A and B for quadrature 4MHz carrier
	// Port B is pins 8 to 13
	// Pin 8 = DDRB0
	// Pin 9 = DDRB1, etc...
	DDRB = (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4) | (1 << DDB5);
	// Use pins 7 to 4 for QAM control
	DDRD = 0xF0;
	
	// Timer 1 is used to generate quadrature 4MHz clock as carrier
	TCCR1A = (1 << COM1A0) | (1 << COM1B0);
	TCCR1B = (1 << WGM12) | (1 << CS10);
	TIMSK1 = 0;
	OCR1A = 1;	// sets the TOP value for CTC oscillator for both channels A and B (f = fclk/(2*(1+TOP)) = 4MHz
	OCR1B = 0;  // sets the value when it toggles (Setting it to 0 gives a 90' phase shift)
	
	
	// Setup Timer 0 channel A for symbol transmitting @ 50kHz ===> 100kb/s
	// NOTE: We use timer 0 for the symbol transmit since it has a higher interrupt priority than timer 0
	// This is because interrupt vectors with lower addresses have higher priority
	// NOTE: Page 49 - Interrupt Vectors in ATmega328p
	//       Page 15 - Reset and Interrupt Handling - "The lower the address the higher is the priority level"
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01); // Prescaler = 8
	//OCR0A = 39;			  // F = 50kHz
	//OCR0A = 27;			  // F = 71.43kHz
	//OCR0A = 23;			  // F = 83.33kHz
	OCR0A = 22;			  // F = 86.956kHz
	//OCR0A = 21;			  // F = 90.90kHz
	//OCR0A = 20;			  // F = 95.23kHz
	//OCR0A = 19;			  // F = 100kHz
	
	
	TIMSK0 = (1 << OCIE0A);
	
	// setup Timer 2 channel A for ADC sampling @ 5kHz ===> 5kB/s
	// If we are using 16QAM we have double the bit rate
	// Therefore we can double the ADC sampling rate
	TCCR2A = (1 << WGM21);
	TCCR2B = (1 << CS21) | (1 << CS20); // Prescaler = 32
	//OCR2A = 49;							// F = 10kHz
	OCR2A = 34;							// F = 14.285kHz
	//TIMSK2 = (1 << OCIE2A);
	
	// ADC setup
	// ADLAR = 8bit mode
	// REFS0 = AVcc
	// MUX3:MUX0 = 0 for ADC0
	ADMUX = (1 << ADLAR) | (1 << REFS0);
	// ADC_CLOCK = FCPU / PRESCALER
	// For Fs = 20kHz (worst case)
	// Takes 13 clock cycles normally, but up to 25 cycles worst case if ADEN is started at unfortuante timing
	// ADC_CLOCK = Fs * 25 = 500kHz
	// Therefore the prescaler upper bound is FCPU/ADC_CLOCK = 32
	ADCSRA = (1 << ADPS2) | (1 << ADPS0); // prescaler 32
	ADCSRA |= (1 << ADEN);	// enable adc
}

int main(void)
{
	cli();
	
	// Setup IO, timers, clocks, interrupts, peripherals, etc...
	init();
	
	// Initialise our encoding pipeline	
	crc8_generate_table(CRC8_POLY);
	conv_enc_calculate_table(0b111, 0b101);
	scrambler_init(SCRAMBLER_CODE);
	
	// Reset counter and flags for ADC and transmit buffers
	curr_tx_rd_byte = 0;
	tx_buffer_wr_ready = 1;
	curr_adc_wr_byte = 0;
	adc_buffer_rd_ready = 1;
	
	// Do all our pre-encoded calculations
	generate_preencodings();
	
	sei();
	
	// Real time encoding 
	// This is triggered by flags from ISR for symbol transmit and ADC sampling
    while (1) 
    {
		// Busy wait until ADC and transmit buffers are ready via flags
		while (!(tx_buffer_wr_ready && adc_buffer_rd_ready));
		#if DEBUG_SIGNALS
		DEBUG_PORT ^= (1 << DEBUG_PIN_PACKET);
		#endif
		
		#if DEBUG_SIGNALS
		DEBUG_PORT |= (1 << DEBUG_PIN_ENCODING);
		#endif
		
		tx_buffer_wr_ready = 0;
		adc_buffer_rd_ready = 0;
		
		// Create our packets
		// NOTE: This cannot take too long otherwise we have a half-written packet used for transmit
		uint8_t* tx_buf = TX_BUFFER_WR;
		generate_packet(ADC_BUFFER_RD, ADC_BUFFER_SIZE, &tx_buf[0]);
		//generate_packet(SAMPLE_MESSAGE, SAMPLE_MESSAGE_LENGTH, &tx_buf[ADC_ENC_FRAME_SIZE*2]);
		
		#if DEBUG_SIGNALS
		DEBUG_PORT &= ~(1 << DEBUG_PIN_ENCODING);
		#endif
    }
}

// Pre-encode as much as possible prior to real-time encoding
void generate_preencodings() {
	// generate gray code for 2D
	uint8_t _gray_code[4] = {0b00, 0b01, 0b11, 0b10}; // 1D gray code
	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t q = 0; q < 4; q++) {
			uint8_t I = _gray_code[i];
			uint8_t Q = _gray_code[q];
			uint8_t idx = (i<<2) | q;
			uint8_t sym = (I<<2) | Q;
			// shift by 4 bits to align with output pins
			GRAY_CODE[idx] = sym << 4;
		}
	}
	
	
	// Unpack preamble into 4bit chunks for 16QAM
	for (int i = 0; i < PREAMBLE_UNPACKED_SIZE; i++) {
		const int shift = (PREAMBLE_UNPACKED_SIZE-1-i)*4;
		const uint8_t bits = (PREAMBLE_CODE >> shift) & 0x0F;
		PREAMBLE_CODE_UNPACKED[i] = GRAY_CODE[bits];
	}
	
	// Copy preamble code into known locations
	{
		const int audio_offset = 0;
		const int metadata_offset = ADC_ENC_FRAME_SIZE*2;
		for (int i = 0; i < PREAMBLE_UNPACKED_SIZE; i++) {
			TX_BUFFER_WR[audio_offset+i] = PREAMBLE_CODE_UNPACKED[i];
			TX_BUFFER_RD[audio_offset+i] = PREAMBLE_CODE_UNPACKED[i];
			TX_BUFFER_WR[metadata_offset+i] = PREAMBLE_CODE_UNPACKED[i];
			TX_BUFFER_RD[metadata_offset+i] = PREAMBLE_CODE_UNPACKED[i];
		}
	}
	
	// Store a sample message in our misc data buffer
	{
		for (int i = 0; i < SAMPLE_MESSAGE_LENGTH; i++) {
			SAMPLE_MESSAGE[i] = 'A' + i;
		}
		SAMPLE_MESSAGE[SAMPLE_MESSAGE_LENGTH-1] = 0;
	}
	
	// NOTE: pre-encoded metadata packet
	{
		generate_packet(SAMPLE_MESSAGE, SAMPLE_MESSAGE_LENGTH, &TX_BUFFER_WR[ADC_ENC_FRAME_SIZE*2]);
		generate_packet(SAMPLE_MESSAGE, SAMPLE_MESSAGE_LENGTH, &TX_BUFFER_RD[ADC_ENC_FRAME_SIZE*2]);
	}
}

// Our encoding pipeline
int generate_packet(uint8_t* x, const int N, uint8_t* y0) {
	conv_enc_reset();
	scrambler_reset();
	
	// We cast this to uint16_t so we can write the trellis terminator along with it
	// NOTE: this uses a big endian notation so the null byte is located correctly without a shift
	uint16_t crc8 = (uint16_t)crc8_calculate(x, N);
	uint16_t len = (uint16_t)(N);
	uint8_t* y = TX_BUFFER_PACKED;
	
	
	int offset = 0;
	// NOTE: We are generating this before hand to save a few instructions
	//y[0] = (PREAMBLE_CODE >> 24) & 0xFF;
	//y[1] = (PREAMBLE_CODE >> 16) & 0xFF;
	//y[2] = (PREAMBLE_CODE >> 8 ) & 0xFF;
	//y[3] = (PREAMBLE_CODE      ) & 0xFF;
	//offset += 4;
	
	// Encode then scramble the output
	offset += conv_enc_process((uint8_t*)(&len),	&y[offset], 2);
	offset += conv_enc_process(x,					&y[offset], N);
	offset += conv_enc_process(&crc8,				&y[offset], 2);
	//offset += conv_enc_process(&trellis_terminator,	&y[offset], 1);
	
	// Scramble and unpack into 4bit symbol chunks
	int k = PREAMBLE_UNPACKED_SIZE;
	for (int i = 0; i < offset; i++) {
		uint8_t b = scrambler_process(y[i]);
		y0[k] = GRAY_CODE[b >> 4];
		k = k+1;
		y0[k] = GRAY_CODE[b & 0x0F];
		k = k+1;
	}
	
	return PREAMBLE_UNPACKED_SIZE + offset*2;
}

void write_adc_buffer() {
	#if DEBUG_SIGNALS
	DEBUG_PORT |= (1 << DEBUG_PIN_ADC);
	#endif
	
	ADC_BUFFER_WR[curr_adc_wr_byte++] = ADCH;
	ADCSRA |= (1 << ADSC); // take a single conversion
	
	if (curr_adc_wr_byte == ADC_BUFFER_SIZE) {
		curr_adc_wr_byte = 0;
		uint8_t* tmp = ADC_BUFFER_RD;
		ADC_BUFFER_RD = ADC_BUFFER_WR;
		ADC_BUFFER_WR = tmp;
		adc_buffer_rd_ready = 1;
	}
	
	#if DEBUG_SIGNALS
	DEBUG_PORT &= ~(1 << DEBUG_PIN_ADC);
	#endif
}

void read_transmit_buffer() {
	#if DEBUG_SIGNALS
	DEBUG_PORT |= (1 << DEBUG_PIN_TRANSMIT);
	#endif
	
	PORTD = TX_BUFFER_RD[curr_tx_rd_byte++];
	
	// swap buffers when done
	// raise flag for main loop to begin writing
	if (curr_tx_rd_byte == TX_BUFFER_SIZE) {
		curr_tx_rd_byte = 0;
		uint8_t* tmp = TX_BUFFER_RD;
		TX_BUFFER_RD = TX_BUFFER_WR;
		TX_BUFFER_WR = tmp;
		tx_buffer_wr_ready = 1;
	}
	
	#if DEBUG_SIGNALS
	DEBUG_PORT &= ~(1 << DEBUG_PIN_TRANSMIT);
	#endif
}

uint8_t adc_repeat = 5;
// symbol transmitter
ISR(TIMER0_COMPA_vect) {
	read_transmit_buffer();

	// fit ADC read routine here since ISR overhead is too much at higher speeds
	if (adc_repeat == 5) {
		adc_repeat = 0;
		write_adc_buffer();	
	}
	adc_repeat++;
}

// ADC sampling interrupt
ISR(TIMER2_COMPA_vect) {
	write_adc_buffer();	
}

