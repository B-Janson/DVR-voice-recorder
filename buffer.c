/**
 * buffer.c - EGB240DVR Library, Circular buffer module
 *
 * Implements a circular buffer in RAM to temporarily store audio samples
 * when reading/writing to flash memory (SD card).
 *
 * The buffer is implemented as two 512 byte pages (contiguous 1024 byte
 * block of memory). Samples can be queued/dequeued a byte or a page at
 * a time. The buffer module provides callback functionality to signal
 * application code when a page is full (when writing samples bytewise)
 * or empty (when reading samples bytewise). No overflow or underflow
 * protection is implemented.
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

/************************************************************************/
/* GLOBAL VARIABLES                                                     */
/************************************************************************/
uint8_t samples[1024];	// Buffer: 2 x 512 byte pages (1024 bytes total)

uint8_t* pPage0	= samples;			// Pointer to top of Page 0
uint8_t* pPage1	= samples + 512;	// Pointer to top of Page 1
uint8_t* pEnd	= samples + 1024;	// Pointer to bottom of buffer

volatile uint8_t* pHead;	// Pointer to head of queue (write pointer)
volatile uint8_t* pTail;	// Pointer to tail of queue (read pointer)

/************************************************************************/
/* FUNCTION POINTERS                                                    */
/************************************************************************/
void (*callbackPageFull)(void);		// Pointer to "page full" function
void (*callbackPageEmpty)(void);	// Pointer to "page empty" function

/************************************************************************/
/* PUBLIC/USER FUNCTIONS                                                */
/************************************************************************/

/**
 * Function: buffer_init
 *
 * Initialises the circular buffer for first use. Read/write pointers are
 * reset to the top of Page 0 and the user supplied callback functions
 * are assigned.
 *
 * Parameters:
 *    pFuncPageFull - Pointer to function to execute on "page full"
 *    pFuncPageEmpty - Pointer to function to execute on "page empty"
 */
void buffer_init(void (*pFuncPageFull)(void), void (*pFuncPageEmpty)(void)) {
	// Reset read/write pointers
	pHead = pPage0;
	pTail = pPage0;

	// Assign user supplier callback functions
	callbackPageFull = pFuncPageFull;
	callbackPageEmpty = pFuncPageEmpty;
}

/**
 * Function: buffer_reset
 *
 * Resets the read/write pointers of the buffer to the top of Page 0.
 */
void buffer_reset() {
	// Reset pointers to top of buffer
	pHead = pPage0;
	pTail = pPage0;
}

/**
 * Function: buffer_queue
 *
 * Adds a sample to the head of the queue (buffer). The sample is
 * placed at the memory location pointed to by pHead. The write
 * pointer is automatically incremented (with wraparound where
 * necessary). A "page full" callback is generated when the write
 * pointer overflows to a new page.
 *
 * Parameters:
 *    word - sample (unsigned 8-bit integer) to add to queue (buffer)
 */
void buffer_queue(uint8_t word) {
	*(pHead++) = word;

	if (pHead == pPage1) {
		callbackPageFull();
	} else if (pHead == pEnd) {
		pHead = pPage0;
		callbackPageFull();
	}
}

/**
 * Function: buffer_dequeue
 *
 * Removes and returns a sample from the head of the queue (buffer).
 * The sample is loaded from the memory location pointed to by pTail.
 * The read pointer is automatically incremented (with wraparound
 * where necessary). A "page empty" callback is generated when the
 * read pointer overflows to a new page.
 *
 * Returns: The sample read from the buffer (unsigned 8-bit integer)
 */
uint8_t buffer_dequeue() {
	uint8_t word = *(pTail++);

	if (pTail == pPage1) {
		callbackPageEmpty();
	} else if (pTail == pEnd) {
		pTail = pPage0;
		callbackPageEmpty();
	}

	return word;
}

/**
 * Function: buffer_readPage
 *
 * Allows application code to read a full page from the buffer.
 * Returns a pointer to the top of the current page (assumes that
 * the read pointer is always page aligned). The read pointer is
 * advanced to the next page boundary immediately. Callbacks
 * are never generated from this function call.
 *
 * Returns: Pointer to the top of the current page (>= pTail)
 */
uint8_t* buffer_readPage() {
	uint8_t* page;

	// Advance tail to next page boundary
	if (pTail > pPage0) {
		page = pPage1;
		pTail = pPage0;
	} else {
		page = pPage0;
		pTail = pPage1;
	}

	return page;
}

/**
 * Function: buffer_writePage
 *
 * Allows application code to write a full page to the buffer.
 * Returns a pointer to the top of the current page (assumes that
 * the write pointer is always page aligned). The write pointer is
 * advanced to the next page boundary immediately. Callbacks are
 * never generated from this function call.
 *
 * Returns: Pointer to the top of the current page (>= pHead)
 */
uint8_t* buffer_writePage() {
	uint8_t* page;

	// Advance head to next page boundary
	if (pHead > pPage0) {
		page = pPage1;
		pHead = pPage0;
	} else {
		page = pPage0;
		pHead = pPage1;
	}

	return page;
}
