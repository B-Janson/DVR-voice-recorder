/**
 * wave.c - EGB240DVR Library, WAVE file interface
 *
 * Provides an interface to read and write WAVE files to an SD card via
 * the FATFS library. This implementation hardcodes the WAVE filename 
 * to "EGB240.WAV" in the root directory of the SD card.
 *
 * Requires:
 *   lib/fatfs - FatFs FAT file system library published by ChaN
 *   timer - Timer module, used to service the FatFs library
 *   serial - USB serial interface to provide debugging information
 *
 * Hardware resources:
 *   The WAVE file modules accesses an SD card via the SPI interface.
 *   This has been pre-configured in the FatFs library. Use of the SPI
 *   module is reserved for SD card access, and is not available for
 *   use by other application code. Pins B1, B2, B3 and B7 are used
 *   for the SD card interface and should not be accessed directly.
 *   Pin B0 (SS) must be set as an output for the SPI interface to
 *   function correctly.
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

#include <string.h>
#include <stdio.h>

#include "lib/fatfs/ff.h"
#include "lib/fatfs/diskio.h"

#include "wave.h"

/************************************************************************/
/* GLOBAL VARIABLES                                                     */
/************************************************************************/
FATFS fs;	// File system structure for SD card access
FIL file;	// File structure for WAVE file access

WAVE_HEADER waveHeader;	// WAVE file header structure for read/write of WAVE file proerties

volatile uint32_t sampleCount = 0;	// Sample counter (used to finalise WAVE header)

uint8_t finaliseHeader = 0;			// Flag to indicate header must be updated/finalised

/************************************************************************/
/* FUNCTION PROTOTYPES                                                  */
/************************************************************************/
void write_wave_header();
uint32_t read_wave_header();
void finalise_wave_header();
void initialise_header(uint32_t samplerate, uint8_t bps, uint8_t channels);

/************************************************************************/
/* PRIVATE/UTILLITY FUNCTIONS                                           */
/************************************************************************/

/**
 * Function: set_char_array
 * 
 * Utility function. Copies first four characters of a null terminated string 
 * into a destination character array. Used to operate on WAVE file headers.
 * 
 * Parameters:
 *   array - Destination array.
 *   string - Source string.
 */
void set_char_array(char* array, char* string) {
	for (int i = 0; i < 4; i++) {
		array[i] = string[i];
	}
}

/**
 * Function: initialise_header
 * 
 * Initialiases the header of a WAVE file using supplied parameters.
 * Header must be finalised using "finalise_header" once all audio
 * samples are written to the file.
 * 
 * Parameters:
 *   samplerate - Sample rate of the WAVE file.
 *   bps - Bits per sample.
 *   channels - Number of audio channels (1 = mono, 2 = stereo, ...).
 */
void initialise_header(uint32_t samplerate, uint8_t bps, uint8_t channels) {
	set_char_array(waveHeader.fields.ChunkID, "RIFF");
	waveHeader.fields.ChunkSize = 0;	// placeholder, update when number of samples is known (36 + dataSize)
	set_char_array(waveHeader.fields.Format, "WAVE");
	
	set_char_array(waveHeader.fields.fmtID, "fmt ");	
	waveHeader.fields.fmtSize = 16;		// for PCM
	waveHeader.fields.AudioFormat = 1;	// PCM
	waveHeader.fields.NumChannels = channels;
	waveHeader.fields.SampleRate = samplerate;
	waveHeader.fields.ByteRate = samplerate*channels*(bps>>3);
	waveHeader.fields.BlockAlign = channels*(bps>>3);
	waveHeader.fields.BitsPerSample = bps;
	
	set_char_array(waveHeader.fields.dataID, "data");
	waveHeader.fields.dataSize = 0;		// placeholder, update with NumSamples * BlockAlign
}

/**
 * Function: write_wave_header
 * 
 * Writes a WAVE header structure into an open file.
 * Wave configuration is hardcoded to 15625 samples per second, 8 bits per sample, mono.
 */
void write_wave_header() {
	FRESULT result;
	uint16_t bw;
	
	initialise_header(15625, 8, 1);	// Create header for 15.625 kHz, 8-bit per sample, mono WAVE file
	result = f_write(&file, &(waveHeader.bytes), 44, &bw); // Write header to file

	// If error has occurred, write status to console
	if (result) printf("f_write returned error code: %d\n", result);
	if (bw != 44) printf("f_write wrote %d of 44 bytes to file.", bw);
	
	// Flag that header requires finalisation
	finaliseHeader = 1;
}

/**
 * Function: read_wave_header
 * 
 * Reads a WAVE header from an open file into a structure.
 * 
 * Returns: The number of samples in the opened wave file (as reported in the header)
 */
uint32_t read_wave_header() {
	FRESULT result;
	uint16_t br;
	
	// Read header from WAVE file into structure
	result = f_read(&file, &(waveHeader.bytes), 44, &br);

	// If error has occurred, write status to console
	if (result) printf("f_read returned error code: %d\n", result);
	if (br != 44) printf("f_read read %d of 44 bytes from file.", br);
	
	
	if (result | (br != 44)) {
		// Return "empty" wave file if read is unsuccessful
		return 0;
	} else {
		return waveHeader.fields.dataSize;
	}
}

/**
 * Function: finalise_wave_header
 * 
 * Finalises the header of an open WAVE file on the basis of the number of samples written to the file.
 */
void finalise_wave_header() {
	FRESULT result;
	uint16_t bw;
	
	// Calculate header fields to update
	uint32_t dataSize = sampleCount;
	uint32_t chunkSize = 36 + dataSize;
	
	// Finalise wave file header
	// Where errors occur, print to console
	result = f_lseek(&file, 4);						// Seek to dataSize location
	if (result) printf("f_lseek returned error code: %d\n", result);
	result = f_write(&file, &chunkSize, 4, &bw);	// Write dataSize field to file
	if (result) printf("f_write returned error code: %d\n", result);
	if (bw != 4) printf("f_write wrote %d of 4 bytes to file.", bw);
	
	result = f_lseek(&file, 40);					// Seek to chunkSize location
	if (result) printf("f_lseek returned error code: %d\n", result);
	result = f_write(&file, &dataSize, 4, &bw);		// Write chuckSize field to file
	if (result) printf("f_write returned error code: %d\n", result);
	if (bw != 4) printf("f_write wrote %d of 4 bytes to file.", bw);
}

/************************************************************************/
/* PUBLIC/USER FUNCTIONS                                                */
/************************************************************************/

/**
 * Function: wave_init
 * 
 * Initialises the WAVE module for use. Mounts the SD card for filesystem access.
 * Must be called prior to calling any other function in the WAVE module.
 */
void wave_init() {
	FRESULT result;
	
	result = f_mount(&fs, "/", 1);	// force mount SD card root directory

	// If error occurs, write status to console
	if (result) printf("f_mount returned error code: %d\n", result);
}

/**
 * Function: wave_create
 * 
 * Creates a and initialises a WAVE file for read/write access.
 * The WAVE filename is hardcoded to "EGB240.WAV"
 * If a file with the same name exists it is overwritten and cleared.
 * The created WAVE file is initialised with an empty header.
 *
 * Postcondition:
 *    Creating a wave file resets the sample counter.
 */
void wave_create() {
	FRESULT result;
	
	// Create new WAVE file with read/write access (force overwrite if file exists)
	result = f_open(&file, "EGB240.WAV", FA_CREATE_ALWAYS | FA_READ | FA_WRITE);

	// If error occurs, write status to console
	if (result) printf("f_open returned error code: %d\n", result);
	
	// Write WAVE file header to file
	write_wave_header();
	
	// Reset sample counter
	sampleCount = 0;
}

/**
 * Function: wave_open
 * 
 * Opens an existing WAVE file for read only access.
 * The WAVE filename is hardcoded to "EGB240.WAV"
 *
 * Returns: The number of samples in the opened WAVE file.
 */
uint32_t wave_open() {
	FRESULT result;
	
	// Open an existing WAVE file with read only access
	result = f_open(&file, "EGB240.WAV", FA_READ);

	// If error occurs, write status to console
	if (result) printf("f_open returned error code: %d\n", result);
	
	// Read the WAVE file header and return the number of samples reported
	return read_wave_header();
}

/**
 * Function: wave_close
 * 
 * Closes an open WAVE file. If required, the WAVE file header is finalised prior to closing.
 */
void wave_close() {
	FRESULT result;
	
	if (finaliseHeader) {
		// Only finalise header where WAVE file is newly created 
		finaliseHeader = 0;
		finalise_wave_header();
	}
	
	// Close WAVE file
	result = f_close(&file);

	// If error occurs, write status to console
	if (result) printf("f_close returned error code: %d\n", result);
}

/**
 * Function: wave_write
 * 
 * Writes a number of audio samples into a open WAVE file.
 * This function expects 8-bit audio samples.
 *
 * Parameters:
 *    pSamples - Pointer to array of 8-bit audio samples to write to WAVE file.
 *    count - Number of samples to write from array into WAVE file.
 */
void wave_write(uint8_t* pSamples, uint16_t count) {
	FRESULT result;
	uint16_t bw;
	
	result = f_write(&file, pSamples, count, &bw); // Write samples to file

	// If error occurs, write status to console
	if (result) printf("f_write returned error code: %d\n", result);
	if (bw != count) printf("f_write wrote %d of %d bytes to file.", bw, count);

	// Increment sample count by number of samples written to file
	sampleCount += bw;
}

/**
 * Function: wave_read
 * 
 * Reads a number of audio samples from an open WAVE file.
 * This function expects 8-bit audio samples.
 *
 * Parameters:
 *    pSamples - Pointer to array of 8-bit audio samples into which samples will be read.
 *    count - Number of samples to read into array from WAVE file.
 */
void wave_read(uint8_t* pSamples, uint16_t count) {
	FRESULT result;
	uint16_t br;
	
	result = f_read(&file, pSamples, count, &br); // Read samples from file

	// If error occurs, write status to console
	if (result) printf("f_write returned error code: %d\n", result);
	if (br != count) printf("f_write wrote %d of %d bytes to file.", br, count);
}