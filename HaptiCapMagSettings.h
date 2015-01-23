/** @file
Library for reading and writing HaptiCap settings (header)

This code is released under a Creative Commons Attribution 4.0 International license
CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)).

@author Paul J. Ganssle
@version 0.1
@date 2015-01-18
*/

#ifndef HAPTICAPSETTINGS_H
#define HAPTICAPSETTINGS_H

#include <Arduino.h>

#define HC_SETTINGS_VERSION 0x00

// Allocate 8 bytes for various settings 0x00 -> 0x08 (5 bytes unused)
#define HC_VERSION_ADDR 0x00
#define HC_CHECKSUM_ADDR 0x01

// Allocate another 32 bytes 0x09 -> 0x29 (12 bytes reserved)
#define HC_NMOTORS_ADDR 0x09            // 1 byte  0x09
#define HC_DECLINATION_ADDR 0x0a        // 4 bytes 0x0a -> 0x0e
#define HC_INCLINATION_ADDR 0x0f        // 4 bytes 0x0f -> 0x12
#define HC_SAMPLERATE_ADDR 0x13         // 4 bytes 0x13 -> 0x16
#define HC_PULSEWIDTH_ADDR 0x17         // 2 bytes 0x17 -> 0x18
#define HC_OFFSET_ADDR 0x19             // 4 bytes 0x19 -> 0x1c
#define HC_USE_CALIBRATION 0x1d         // 1 byte  0x1d

// Array settings - 16 bytes each, allocating a total of 32 bytes 0x2a -> 0x4a
#define HC_PINLOCS_ADDR 0x20        // Allocate 16 bytes 0x20 -> 0x2f
#define HC_MOTORCAL_ADDR 0x30       // Allocate 16 bytes 0x30 -> 0x3f

class HaptiCapMagSettings {
    /** Class for access to the HaptiCap Magnetometer settings */
public:
    HaptiCapMagSettings(uint32_t base_location=0x00);

    // Getters and setters
    float getDeclination(void);
    float getInclination(void);
    float getSampleRate(void);
    uint16_t getPulseWidth(void);
    uint8_t getNMotors(void);
    float getPhi(void);

    uint8_t setDeclination(float declination);
    uint8_t setInclination(float inclination);
    uint8_t setSampleRate(float sample_rate);
    uint8_t setPulseWidth(uint16_t pulse_width);
    uint8_t setNMotors(uint8_t n_motors);
    uint8_t setPhaseOffset(float phi);

    // Read / write functions
    uint8_t writeAll();
    uint8_t readAll();
    
private:
    uint16_t calculateChecksum(uint8_t settings_version=HC_SETTINGS_VERSION);

    void writeDeclination(void);
    void writeInclination(void);
    void writeSampleRate(void);
    void writePulseWidth(void);
    void writeNMotors(void);
    void writePhaseOffset(void);

    void readDeclination(void);
    void readInclination(void);
    void readSampleRate(void);
    void readPulseWidth(void);
    void readNMotors(void);
    void readPhaseOffset(void);

    uint32_t base_addr;

    bool useCalibration;

    uint8_t NMotors;

    float declination;
    float inclination;
    float sampleRate;
    unsigned short pulseWidth;
    float phaseOffset;

    int8_t pinLocs[16];
    uint8_t motorCal[16];
};

uint8_t update_CRC8_float(uint8_t crc, float f);
uint8_t update_CRC8_uint16(uint8_t crc, uint16_t sh);

#endif