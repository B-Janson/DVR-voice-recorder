/**
 * serial.h - EGB240DVR Library, Serial interface header
 *
 * Provides a serial interface via the USB port.
 *
 * Version: v1.0
 *    Date: 10/04/2016
 *  Author: Mark Broadmeadow
 *  E-mail: mark.broadmeadow@qut.edu.au
 */ 

#ifndef SERIAL_H_
#define SERIAL_H_

void serial_init();			// Initialises the serial module for use.
uint8_t serial_ready();		// Returns true if the serial interface is ready for use.
uint8_t serial_available(); // Returns true if characters are available on the serial interface.

#endif /* SERIAL_H_ */