/**
 * adc.c - EGB240DVR Library, ADC module
 *
 * Configures the ADC to sample on CH0 and store conversion
 * results into a circular buffer. Conversions are triggered
 * from the Timer0 CMPA signal.
 *
 * Requires:
 *   timer	- Configures Timer0 to trigger ADC conversions. 
 *   buffer - Circular buffer (queue) used to store audio samples.
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

#include "buffer.h"

/************************************************************************/
/* PUBLIC/USER FUNCTIONS                                                */
/************************************************************************/
void adc_init() {
	ADMUX = 0x60;	// Left adjust result, AREF = AVCC
	ADCSRB = 0x03;	// Select Timer0 CMPA as trigger	
}

void adc_start() {
	ADCSRA = 0xAE;	// /64 prescaler (250 kHz clock), enable interrupts, ADC enable
}

void adc_stop() {
	ADCSRA = 0x00;
}

/************************************************************************/
/* INTERRUPT SERVICE ROUTINES                                           */
/************************************************************************/

/**
 * ISR: ADC conversion complete
 * 
 * Interrupt service routine which executes on completion of ADC conversion.
 */
ISR(ADC_vect) {
	uint8_t result = ADCH;	//Read result
	buffer_queue(result);	//Store result into buffer
}