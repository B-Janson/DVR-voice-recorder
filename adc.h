/**
 * adc.h - EGB240DVR Library, ADC module header
 *
 * Version: v1.0
 *    Date: 10/04/2016
 *  Author: Mark Broadmeadow
 *  E-mail: mark.broadmeadow@qut.edu.au
 */ 

#ifndef ADC_H_
#define ADC_H_

void adc_init();	// Initialises ADC
void adc_start();	// Enables ADC to start conversions (triggered by Timer0 CMPA)
void adc_stop();	// Disables ADC conversions

#endif /* ADC_H_ */