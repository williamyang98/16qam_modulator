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

const uint32_t PREAMBLE_CODE = 0b11111001101011111100110101101101;
const uint16_t SCRAMBLER_CODE = 0b1000010101011001;
const uint8_t CRC8_POLY = 0xD5;

// Formula for encoded frame size
// M = 2*(N+2+1) + 4
// M = 2*N+10

#define ADC_BUFFER_SIZE 100

// ADC frame is 210B
#define ADC_ENC_FRAME_SIZE ((ADC_BUFFER_SIZE+2+1)*2 + 4)

// Message frame is 40B
#define SAMPLE_MESSAGE_LENGTH 15
uint8_t SAMPLE_MESSAGE[SAMPLE_MESSAGE_LENGTH] = {0};
#define MESSAGE_ENC_FRAME_SIZE ((SAMPLE_MESSAGE_LENGTH+2+1)*2 + 4)

// Each frame transmitted at 250B blocks
// ADC frame is 210B
// Message frame is 40B

// We want to fit 1 audio frame and some extra data
#define TX_BUFFER_SIZE 250

const uint8_t qam_data[4] = {0b00000000, 0b11000000, 0b00110000, 0b11110000};

// ADC buffers
uint8_t ADC_BUFFER_0[ADC_BUFFER_SIZE] = {0};
uint8_t ADC_BUFFER_1[ADC_BUFFER_SIZE] = {0};

volatile uint8_t* ADC_BUFFER_WR = ADC_BUFFER_0;
volatile uint8_t* ADC_BUFFER_RD = ADC_BUFFER_1;
volatile int curr_adc_wr_byte = 0;
volatile uint8_t adc_buffer_rd_ready = 1;

// Encoding and transmit buffers
uint8_t TX_BUFFER_0[TX_BUFFER_SIZE] = {0};
uint8_t TX_BUFFER_1[TX_BUFFER_SIZE] = {0};
	
volatile uint8_t* TX_BUFFER_WR = TX_BUFFER_0;
volatile uint8_t* TX_BUFFER_RD = TX_BUFFER_1;
volatile uint32_t curr_tx_rd_byte = 0;
volatile uint8_t tx_buffer_wr_ready = 1;

int generate_packet(uint8_t* x, const int N, uint8_t* y);

int main(void)
{
	cli();
	
	// setup buffers
	TX_BUFFER_WR = TX_BUFFER_0;
	TX_BUFFER_RD = TX_BUFFER_1;
	ADC_BUFFER_WR = ADC_BUFFER_0;
	ADC_BUFFER_RD = ADC_BUFFER_1;
	
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
	
	// setup Timer 0 channel A for ADC sampling
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01); // Prescaler = 8
	OCR0A = 39;			  // F = 50kHz
	// NOTE: Disabling second ISR to check if it causes glitches in symbol tx interrupt
	TIMSK0 = (1 << OCIE0A);
	
	// NOTE: We use timer 2 for the symbol transmit since it has a higher interrupt priority than timer 0
	// This is because interrupt vectors with lower addresses have higher priority
	// NOTE: Page 49 - Interrupt Vectors in ATmega328p
	//       Page 15 - Reset and Interrupt Handling - "The lower the address the higher is the priority level"
	TCCR2A = (1 << WGM21);
	TCCR2B = (1 << CS21) | (1 << CS20); // Prescaler = 32
	OCR2A = 99;			  // F = 5kHz
	TIMSK2 = (1 << OCIE2A);
	
	// ADC setup
	// ADLAR = 8bit mode
	// REFS0 = AVcc
	// MUX3:MUX0 = 0 for ADC0
	ADMUX = (1 << ADLAR) | (1 << REFS0);	
	// Set ADC clock prescaler to 64
	ADCSRA = (1 << ADPS2) | (1 << ADPS1);
	ADCSRA |= (1 << ADEN);	// enable adc
	
	
    /* Replace with your application code */
	
	crc8_generate_table(CRC8_POLY);
	conv_enc_calculate_table(0b111, 0b101);
	scrambler_init(SCRAMBLER_CODE);
	
	
	curr_tx_rd_byte = 0;
	tx_buffer_wr_ready = 1;
	curr_adc_wr_byte = 0;
	adc_buffer_rd_ready = 1;
	
	// initialise our adc buffers with some test data
	for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
		ADC_BUFFER_0[i] = '0'+i;
		ADC_BUFFER_1[i] = '0'+i;
	}
	ADC_BUFFER_0[ADC_BUFFER_SIZE-1] = 0;
	ADC_BUFFER_1[ADC_BUFFER_SIZE-1] = 0;
	
	for (int i = 0; i < SAMPLE_MESSAGE_LENGTH; i++) {
		SAMPLE_MESSAGE[i] = 'A' + i;
	}
	SAMPLE_MESSAGE[SAMPLE_MESSAGE_LENGTH-1] = 0;
	
	sei();
	
    while (1) 
    {
		// busy wait until ready to write to buffer
		
		while (!(tx_buffer_wr_ready && adc_buffer_rd_ready));
		#if DEBUG_SIGNALS
		PORTB ^= (1 << PORTB5);
		#endif
		
		#if DEBUG_SIGNALS
		PORTB |= (1 << PORTB0);
		#endif
		
		tx_buffer_wr_ready = 0;
		adc_buffer_rd_ready = 0;
		
		
		uint8_t* tx_buf = TX_BUFFER_WR;
		int tx_buf_byte = 0;
		tx_buf_byte += generate_packet(ADC_BUFFER_RD, ADC_BUFFER_SIZE, &tx_buf[tx_buf_byte]);
		tx_buf_byte += generate_packet(SAMPLE_MESSAGE, SAMPLE_MESSAGE_LENGTH, &tx_buf[tx_buf_byte]);
		
		#if DEBUG_SIGNALS
		PORTB &= ~(1 << PORTB0);
		#endif
    }
}

int generate_packet(uint8_t* x, const int N, uint8_t* y) {
	
	conv_enc_reset();
	scrambler_reset();
	uint8_t crc8 = crc8_calculate(x, N);
	uint16_t len = (uint16_t)(N);
	
	int offset = 0;
	y[0] = (PREAMBLE_CODE >> 24) & 0xFF;
	y[1] = (PREAMBLE_CODE >> 16) & 0xFF;
	y[2] = (PREAMBLE_CODE >> 8 ) & 0xFF;
	y[3] = (PREAMBLE_CODE      ) & 0xFF;
	offset += 4;
	
	const int scrambler_start = offset;
	offset += conv_enc_process((uint8_t*)(&len),	&y[offset], 2);
	offset += conv_enc_process(x,					&y[offset], N);
	offset += conv_enc_process(&crc8,				&y[offset], 1);

	const int scrambler_end = offset;
	for (int i = scrambler_start; i < scrambler_end; i++) {
		y[i] = scrambler_process(y[i]);
	}
	
	return offset;
}

volatile uint8_t* tx_buffer_swap_tmp;

int tx_buffer_byte_shift = 6;

// symbol transmitter
ISR(TIMER0_COMPA_vect) {
	#if DEBUG_SIGNALS
	PORTB |= (1 << PORTB3);
	#endif
	
	const uint8_t b = TX_BUFFER_RD[curr_tx_rd_byte];
	const uint8_t i = (b >> tx_buffer_byte_shift) & 0b11;
	PORTD = qam_data[i];
	tx_buffer_byte_shift -= 2;
	
	if (tx_buffer_byte_shift == -2) {
		tx_buffer_byte_shift = 6;
		curr_tx_rd_byte++;
	}
	
	// swap buffers when done
	// raise flag for main loop to begin writing
	if (curr_tx_rd_byte == TX_BUFFER_SIZE) {
		curr_tx_rd_byte = 0;
		tx_buffer_swap_tmp = TX_BUFFER_RD;
		TX_BUFFER_RD = TX_BUFFER_WR;
		TX_BUFFER_WR = tx_buffer_swap_tmp;
		tx_buffer_wr_ready = 1;
	}
	
	#if DEBUG_SIGNALS
	PORTB &= ~(1 << PORTB3);
	#endif
}

volatile uint8_t* adc_buffer_swap_tmp;

// ADC fetching at 6.25kHz
ISR(TIMER2_COMPA_vect) {
	#if DEBUG_SIGNALS
	PORTB |= (1 << PORTB4);
	#endif
	
	ADC_BUFFER_WR[curr_adc_wr_byte++] = ADCH;
	ADCSRA |= (1 << ADIF);
	ADCSRA |= (1 << ADSC); // take a single conversion
	if (curr_adc_wr_byte == ADC_BUFFER_SIZE) {
		curr_adc_wr_byte = 0;
		adc_buffer_swap_tmp = ADC_BUFFER_RD;
		ADC_BUFFER_RD = ADC_BUFFER_WR;
		ADC_BUFFER_WR = adc_buffer_swap_tmp;
		adc_buffer_rd_ready = 1;
	}
	
	#if DEBUG_SIGNALS
	PORTB &= ~(1 << PORTB4);
	#endif
}
