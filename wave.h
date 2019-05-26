/**
 * wave.h - EGB240DVR Library, WAVE file interface header
 *
 * Provides an interface to read and write WAVE files to an SD card via
 * the FATFS library. This implementation hardcodes the WAVE filename
 * to "EGB240.WAV" in the root directory of the SD card.
 *
 * Version: v1.0
 *    Date: 10/04/2016
 *  Author: Mark Broadmeadow
 *  E-mail: mark.broadmeadow@qut.edu.au
 */

#ifndef WAVE_H_
#define WAVE_H_

// WAVE file header structure
typedef struct {
	char		ChunkID[4];	// Contains "RIFF" in ASCII
	uint32_t	ChunkSize;	// Size of file minus 8 bytes
	char		Format[4];	// Contains "WAVE" in ASCII

	char		fmtID[4];		// Contains "fmt " in ASCII
	uint32_t	fmtSize;		// 16 for PCM (size of fmt subchunk minus 8)
	uint16_t	AudioFormat;	// 1 for PCM
	uint16_t	NumChannels;	// 1 for mono, 2 for stereo
	uint32_t	SampleRate;		// Audio sample rate, e.g. 41000
	uint32_t	ByteRate;		// = SampleRate * NumChannels * BitsPerSample/8
	uint16_t	BlockAlign;		// = NumChannels * BitsPerSample/8
	uint16_t	BitsPerSample;	// Bits per sample, e.g. 6, 18 ..

	char		dataID[4];		// Contains "data" in ASCII
	uint32_t	dataSize;		// = NumSamples * NumChannels * BitsPerSample/8
} WAVE_HEADER_FIELDS;

// Union to provide byte-wise access to WAVE file header structure
// Used for serialisation of WAVE file header (for read/write to/from memory)
typedef union {
	WAVE_HEADER_FIELDS fields;
	uint8_t bytes[44];
} WAVE_HEADER;

void wave_init();		// Initialise WAVE file interface
void wave_create();		// Create and open new WAVE file (read/write)
uint32_t wave_open();	// Open existing wave file (read only)
void wave_skip();
void wave_write(uint8_t* pSamples, uint16_t count);	// Write samples to a WAVE file
void wave_read(uint8_t* pSamples, uint16_t count);	// Read samples from WAVE file
void wave_close();		// Close wave file opened with wave_create or wave_open

#endif /* WAVE_H_ */
