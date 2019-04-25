/**
 * buffer.h - EGB240DVR Library, Circular buffer module header
 *
 * Implements a circular buffer in RAM to temporarily store audio samples
 * when reading/writing to flash memory (SD card).
 *
 * Version: v1.0
 *    Date: 10/04/2016
 *  Author: Mark Broadmeadow
 *  E-mail: mark.broadmeadow@qut.edu.au
 */ 

#ifndef BUFFER_H_
#define BUFFER_H_

// Initialises the buffer for first use. 
// Users must supply pointers to callback function implementation.
void buffer_init(void (*pFuncPageFull)(void), void (*pFuncPageEmpty)(void));	

void buffer_reset();				// Resets read/write pointers to top of buffer
void buffer_queue(uint8_t word);	// Writes a sample to the buffer and advances the write pointer
uint8_t buffer_dequeue();			// Reads a sample from the buffer and advances the read pointer
uint8_t* buffer_readPage();			// Allows user code to read a full page from the buffer
uint8_t* buffer_writePage();		// Allows user code to write a full page to the buffer

#endif /* BUFFER_H_ */