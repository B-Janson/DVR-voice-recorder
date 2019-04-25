/**
 * timer.h - EGB240DVR Library, Timer module header
 *
 * Configures Timer0 to generate regular interrupts for sampling 
 * and other timing purposes.
 *
 * Version: v1.0
 *    Date: 10/04/2016
 *  Author: Mark Broadmeadow
 *  E-mail: mark.broadmeadow@qut.edu.au
 */ 

#ifndef TIMER_H_
#define TIMER_H_

// Defines for timer intervals
#define TIMER_INTERVAL_FATFS	156		// 10 ms interval
#define TIMER_INTERVAL_LED		7813	// 500 ms interval

void timer_init();	// Initialise and start Timer0

#endif /* TIMER_H_ */