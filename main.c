/**
 * main.c - EGb_play40 Digital Voice Recorder Skeleton Code
 *
 * This code provides a skeleton implementation of a digital voice
 * recorder using the Teensy microcontroller and QUT TensyBOBv2
 * development boards. This skeleton code demonstrates usage of
 * the EGb_play40DVR library, which provides functions for recording
 * audio samples from the ADC, storing samples temporarily in a
 * circular buffer, and reading/writing samples to/from flash
 * memory on an SD card (using the FAT file system and WAVE file
 * format.
 *
 * This skeleton code provides a recording implementation which
 * samples CH0 of the ADC at 8-bit, 15.625kHz. Samples are stored
 * in flash memory on an SD card in the WAVE file format. The
 * filename is set to "EGb_play40.WAV". The SD card must be formatted
 * with the FAT file system. Recorded WAVE files are playable on
 * a computer.
 *
 * LED4 on the TeensyBOBv2 is configured to flash as an
 * indicator that the programme is running; a 1 Hz, 50 % duty
 * cycle flash should be observed under normal operation.
 *
 * A serial USB interface is provided as a secondary control and
 * debugging interface. Errors will be printed to this interface.
 *
 * Version: v1.0
 *    Date: 10/04/2016
 *  Author: Mark Broadmeadow
 *  E-mail: mark.broadmeadow@qut.edu.au
 */

 /************************************************************************/
 /* INCLUDED LIBRARIES/HEADER FILES                                      */
 /************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdio.h>

#include "serial.h"
#include "timer.h"
#include "wave.h"
#include "buffer.h"
#include "adc.h"

#define LED1 PIND4
#define LED2 PIND5
#define LED3 PIND6
#define LED4 PIND7

#define B_PLAY PINF4
#define B_RECORD PINF5
#define B_STOP PINF6
#define B_SKIP PINF7

/************************************************************************/
/* ENUM DEFINITIONS                                                     */
/************************************************************************/
enum {
	DVR_STOPPED,
	DVR_RECORDING,
	DVR_PLAYING,
};

/************************************************************************/
/* GLOBAL VARIABLES                                                     */
/************************************************************************/
uint16_t pageCount = 0;	// Page counter - used to terminate recording
volatile uint16_t newPage = 0;	// Flag that indicates a new page is available for read/write
uint8_t stop = 0;		// Flag that indicates playback/recording is complete

/************************************************************************/
/* FUNCTION PROTOTYPES                                                  */
/************************************************************************/
void pageFull();
void pageEmpty();

/************************************************************************/
/* INITIALISATION FUNCTIONS                                             */
/************************************************************************/

// Initialise PLL (required by USB serial interface, PWM)
void pll_init() {
	PLLFRQ = 0x6A; // PLL = 96 MHz, USB = 48 MHz, TIM4 = 64 MHz
}

// Configure system clock for 16 MHz
void clock_init() {
	CLKPR = 0x80;	// Prescaler change enable
	CLKPR = 0x00;	// Prescaler /1, 16 MHz
}

void pwm_ex1_init() {
	OCR1A = 511; 				// TOP, 15.625kHz, T = 64 us
	OCR1B = 255; 				// 50% duty cycle

	TCCR1A = 0b00100011; 		// fast-pwm mode (top = OCR1A) set OC1B on TOP, reset on CMP
	TCCR1B = 0b00011001; 		// fast pwm (TOP = OCR1A), /1 prescaler

	DDRB |= (1 << PINB6); 		// Set pinb6 as output for pwm
}

void led_init() {
	DDRD |= 0b11110000;			// Set four led pins as output
	PORTD &= ~(0b11110000);		// Force all leds to be off
	PORTD |= (1 << B_STOP);		// Set LED 3 on (stopped)
}

// Initialise DVR subsystems and enable interrupts
void init() {
	cli();			// Disable interrupts
	clock_init();	// Configure clocks
	pwm_ex1_init(); // Configure PWM
	pll_init();     // Configure PLL (used by Timer4 and USB serial)
	serial_init();	// Initialise USB serial interface (debug)
	led_init();
	timer_init();	// Initialise timer (used by FatFs library)
	buffer_init(pageFull, pageEmpty);  // Initialise circular buffer (must specify callback functions)
	adc_init();		// Initialise ADC
	sei();			// Enable interrupts

	// Must be called after interrupts are enabled
	wave_init();	// Initialise WAVE file interface
}

/************************************************************************/
/* CALLBACK FUNCTIONS FOR CIRCULAR BUFFER                               */
/************************************************************************/

// CALLED FROM BUFFER MODULE WHEN A PAGE IS FILLED WITH RECORDED SAMPLES
void pageFull() {
	if(!(--pageCount)) {
		// If all pages have been read
		adc_stop();		// Stop recording (disable new ADC conversions)
		stop = 1;		// Flag recording complete
	} else {
		newPage = 1;	// Flag new page is ready to write to SD card
	}
}

// CALLED FROM BUFFER MODULE WHEN A NEW PAGE HAS BEEN EMPTIED
void pageEmpty() {
	// TODO: Implement code to handle "page empty" callback
	if (!(--pageCount)) {
		stop = 1;
	} else {
		newPage = 1;
	}
}

/************************************************************************/
/* RECORD/PLAYBACK ROUTINES                                             */
/************************************************************************/

// Initiates a record cycle
void dvr_record() {
	buffer_reset();		// Reset buffer state

	pageCount = 305;	// Maximum record time of 10 sec
	newPage = 0;		// Clear new page flag

	wave_create();		// Create new wave file on the SD card
	adc_start();		// Begin sampling

	// TODO: Add code to handle LEDs
	PORTD &= ~(1 << B_STOP);
	PORTD |= (1 << B_RECORD);
}

// TODO: Implement code to initiate playback and to stop recording/playback.
void dvr_playback() {
	buffer_reset();

	uint32_t samples;
	samples = wave_open();
	pageCount = samples / 512;
	newPage = 0;

	printf("Samples: %ld pageCount: %d\n\r", samples, pageCount);

	wave_read(buffer_writePage(), 512);
	wave_read(buffer_writePage(), 512);

	TIMSK1 = 0b00000001;		// Enable overflow inturrupt
	PORTD &= ~(1 << B_STOP);
	PORTD |= (1 << B_PLAY);
}

volatile uint8_t ovflowcount = 0;

ISR(TIMER1_OVF_vect) {
	ovflowcount++;
	if (ovflowcount % 2 == 0) {
		int16_t result = buffer_dequeue();
		OCR1B = result * 2;
		ovflowcount = 0;
	}

}

/************************************************************************/
/* MAIN LOOP (CODE ENTRY)                                               */
/************************************************************************/
int main(void) {
	uint8_t pb;
	uint8_t state = DVR_STOPPED;	// Start DVR in stopped state

	uint16_t b_play = 0;
	uint16_t b_record = 0;
	uint16_t b_stop = 0;
	uint16_t b_skip = 0;

	uint8_t pressed = 0;

	// Initialisation
	init();

	// Loop forever (state machine)
  	for(;;) {
		pb = ~(PINF | 0b00001111);

		b_play = b_play << 1;
		b_record = b_record << 1;
		b_stop = b_stop << 1;
		b_skip = b_skip << 1;

		switch (pb) {
			case (1 << B_STOP):
				// PB S1 pressed
				b_stop |= 1;
				break;
			case (1 << B_RECORD):
				// PB S2 pressed
				b_record |= 1;
				break;
			case (1 << B_PLAY):
				// PB S3 pressed
				b_play |= 1;
				break;
			case (1 << B_SKIP):
				b_skip |= 1;
				break;
			default:
				pressed = 0;
				break;
		}

		// Switch depending on state
		switch (state) {
			case DVR_STOPPED:
				// TODO: Implement button/LED handling for record/playback/stop
				if (b_record == 0xFFFF) {
					printf("Recording...\n\r");
					dvr_record();
					state = DVR_RECORDING;
				}

				if (b_play == 0xFFFF) {
					printf("Playing...\n\r");
					dvr_playback();
					state = DVR_PLAYING;
				}

				if (b_skip == 0xFFFF && !pressed) {
					pressed = 1;
					printf("Skipping track...\n\r");
					wave_skip();
				}

				break;
			case DVR_RECORDING:
				// TODO: Implement stop functionality
				if (b_stop == 0xFFFF && !pressed) {
					pressed = 1;
					pageCount = 1;
					printf("Stopping...\n\r");
				}

				// Write samples to SD card when buffer page is full
				if (newPage) {
					newPage = 0;	// Acknowledge new page flag
					wave_write(buffer_readPage(), 512);
				} else if (stop) {
					// Stop is flagged when the last page has been recorded
					stop = 0;							// Acknowledge stop flag
					wave_write(buffer_readPage(), 512);	// Write final page
					wave_close();						// Finalise WAVE file
					printf("DONE!\n\r");					// Print status to console
					state = DVR_STOPPED;				// Transition to stopped state
					PORTD &= ~(1 << B_RECORD);
					PORTD |= (1 << B_STOP);
				}
				break;
			case DVR_PLAYING:
				// TODO: Implement playback functionality
				// printf("newPage: %d\n\r", newPage);
				if (newPage) {
					newPage = 0;
					wave_read(buffer_writePage(), 512);
				} else if (stop) {
					stop = 0;
					wave_close();
					printf("Done!\n\r");
					state = DVR_STOPPED;
					TIMSK1 = 0b00000000;		// Enable overflow inturrupt
					PORTD &= ~(1 << B_PLAY);
					PORTD |= (1 << B_STOP);
				}

				break;
			default:
				// Invalid state, return to valid idle state (stopped)
				printf("ERROR: State machine in main entered invalid state!\n");
				state = DVR_STOPPED;
				break;

		} // END switch(state)
	} // END for(;;)
}
