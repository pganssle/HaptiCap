/** @file
Arduino sketch for general HaptiCap calibration via serial port.

This code is released under a Creative Commons Attribution 4.0 International license
CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)).

@author Paul J. Ganssle
@version 0.1
@date 2015-01-18
*/

#include <Arduino.h>

// Allocate 32 bytes for various settings
#define HC_DECLINATION_ADDR 0x01
#define HC_INCLINATION_ADDR 0x02
#define HC_SAMPLERATE_ADDR 0x03
#define HC_PULSEWIDTH_ADDR 0x04
#define HC_NMOTORS_ADDR 0x05
#define HC_OFFSET_ADDR 0x06

// Array settings
#define HC_PINLOCS_ADDR 0x20        // Allocate 16 bytes
#define HC_MOTORCAL_ADDR 0x30       // Allocate 16 bytes

void setup() {
    
}

void loop() {
    
}

void save_settings_EEPROM() {
    /** Save all current settings to the EEPROM */

}