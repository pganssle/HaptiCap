/** @file
Library for reading and writing HaptiCap settings

This code is released under a Creative Commons Attribution 4.0 International license
CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)).

@author Paul J. Ganssle
@version 0.1
@date 2015-01-18
*/

#include <Arduino.h>
#include <EEPROMex.h>
#include <HaptiCapMagSettings.h>
#include <tgmath.h>
#include <util/crc16.h>

HaptiCapMagSettings::HaptiCapMagSettings(uint32_t base_location) {
    /** Constructor for the HaptiCapMagSettings class

    @param[in] base_location The location in EEPROM memory to read and write the settings.
                             The settings takes 72 bytes, so ensure that the the base location is
                             no less than 72 bytes from the end of the memory location.
    */

    base_addr = base_location;

    // Set up the default values
    declination = 0.0;          // Degrees
    inclination = 0.0;          // Degrees
    sampleRate = 15.0;          // Hz
    phaseOffset = 0.0;          // Degrees
    pulseWidth = 20;            // ms
    NMotors = 8;

    for(int i = 0; i < 16; i++) {
        pinLocs[i] = -1;                // Invalid motor
        motorCal[i] = 0xff;             // Frac = 1.0
    }
}

float HaptiCapMagSettings::getDeclination() {
    /** Get the declination from the current settings object */

    return declination;
}

float HaptiCapMagSettings::getInclination() {
    /** Get the inclination from the current settings object */

    return inclination;
}

float HaptiCapMagSettins::getSampleRate() {
    /** Get the sample rate from the current settings object */

    return sampleRate;
}

unsigned short HaptiCapMagSettins::getPulseWidth() {
    /** Get the pulse width from the current settings object */

    return pulseWidth;
}

uint8_t HaptiCapMagSettins::getNMotors() {
    /** Get the number of motors from the current settings object */

    return NMotors;
}

uint8_t HaptiCapMagSettings::setDeclination(float declination) {
    /** Set the declination offset for the current settings object, in degrees

    Declination is the offset angle between magnetic north and the direction which you'd
    like to indicate - generally true north. This depends on your geographic location.
    You can use the [NOAA declination calculator](http://www.ngdc.noaa.gov/geomag-web/#declination)
    to calculate the declination in heading from magnetic north to true north. Use negative numbers
    for east and positive numbers for west. Range is +/- 180.0, numbers outside that range will
    wrap around.

    If you'd like to indicate some other direction, you can then calculate the declination of your
    chosen waypoint vs. true north at your location. Keep in mind that the distance between you and
    your waypoint can have a profound effect on the declination error introduced by moving around;
    as such, this setting is not useful for indicating nearby waypoints.

    @param[in] declination The declination in degrees, which wraps at +/- 180.

    @return Returns 0 on no error. Currently this can raise no errors.
    */

    declination = fmod(declination, 180.0);                                 // Wrap overflows
    return 0;
}

uint8_t HaptiCapMagSettings::setInclination(float inclination) {
    /** Set the inclination offset for the current settings object, in degrees
    
    Inclination is the offset angle between the x-y plane and the z-axis of the magnetic field
    vector. For simple directional headings, you generally don't need to take the inclination
    into account. It is included however in case you want to indicate a three-dimensional magnetic
    field vector.

    @param[in] inclination The inclination in degrees, which wraps at +/- 180.

    @return Returns 0 on no error. Currently this can raise no errors.
    */

    inclination = fmod(inclination, 180.0);
    return 0;
}

uint8_t HaptiCapMagSettinsg::setSampleRate(float sample_rate) {
    /** Set the sample rate for the current settings object, in Hz.
    
    In continuous read mode, the HMC5883L takes samples at one of the 7 set data output rates,
    ranging from 0.75 Hz to 75 Hz. In single shot mode, the data output rate is limited by the rate
    at which data are polled, and can be up to 160 Hz. This controls the rate at which the data
    are requested.

    @param[in] sample_rate The sample rate at which the magnetometer will be polled, in Hz. Limited
                           to between 0.75 and 160.

    @return Returns 0 on no error. Returns `HC_EC_BAD_SAMPLERATE` if an invalid sample rate is
            specified.
    */

    if (sample_rate < 0.75 or sample_rate > 160) {
        return HC_EC_BAD_SAMPLERATE;
    }

    sampleRate = sample_rate;
    return 0;
}

uint8_t HaptiCapMagSettings::setPulseWidth(uint16_t pulse_width);
uint8_t HaptiCapMagSettings::setNMotors(uint8_t n_motors);
uint8_t HaptiCapMagSettings::setPhaseOffset(float phi);

uint8_t HaptiCapMagSettings::writeAll();
uint8_t HaptiCapMagSettings::readAll();


void writeDeclination() {
    /** Write the declination to the EEPROM memory.
    
    Writes the declination stored in the settings object (which can be user specified by a call to
    `setDeclination()`) to the EEPROM memory. Declination is stored as a single precision (32-bit)
    float, in location `HC_DECLINATION_ADDR`.
    */

    float cDeclination = readDeclination();
    if (cDeclination != declination) {
       EEPROM.writeFloat(HC_DECLINATION_ADDR + base_addr, declination);        // Write to EEPROM
   }
}

void writeInclination() {
    /** Write the declination to the EEPROM memory.

    Writes the inclination stored in the settings object (which can be user specified by a call to
    `setInclination()`) to the EEPROM memory. Inclination is stored as a single precision (32-bit)
    float, in location `HC_INCLINATION_ADDR`.
    */

    float cInclination = readInclination();
    if (inclination != cInclination) {
        EEPROM.writeFloat(HC_INCLINATION_ADDR + base_addr, inclination);
    }
}

void writeSampleRate() {
    /** Write the sample rate to the EEPROM memory

    Writes the sample rate stored in the settings object (which can be user specified by a call to
    `setSampleRate()`) to the EEPROM memory. Sample rate is stored as a single precision (32-bit)
    float, in location `HC_SAMPLERATE_ADDR`.
    */

    float cSampleRate = readSampleRate();
    if (sampleRate != cSampleRate) {
        EEPROM.writeFloat(HC_SAMPLERATE_ADDR + base_addr, sampleRate);
    }
}

void writePulseWidth(uint16_t pulse_width) {
    /** Write the pulse width to the EEPROM memory

    Writes the pulse width stored in the settings object (which can be user specified by a call to
    `setPulseWidth()`) to the EEPROM memory. Pulse width is stored as an unsigned 32-bit integer
    (long), in location `HC_PULSEWIDTH_ADDR`. 

    */

    uint16_t cPulseWidth = readPulseWidth();
    if (pulseWidth != cPulseWidth) {
        EEPROM.writeInt(HC_PULSEWIDTH_ADDR + base_addr, pulseWidth);
    }
}

void writeNMotors(uint8_t n_motors) {
    /** Write the number of motors to the EEPROM memory
    
    Writes the number of motors stored in the settings object (which can be user specified by a call
    to `setNMotors()`) to the EEPROM memory. Number of motors is stored as an unsigned 8-bit integer
    (byte), in location `HC_NMOTORS_ADDR`.

    */

    uint8_t cNMotors = readNMotors();
    if (nMotors != cNMotors) {
        EEPROM.writeByte(HC_NMOTORS_ADDR + base_addr, nMotors);
    }
}

void writePhaseOffset() {
    /** Write the phase offset to the EEPROM memory

    Writes the phase offset stored in the settings object (which can be user specified by a call to
    `setPhaseOffset()`) to the EEPROM memory. Phase offset is stored as a single precision (32-bit)
    float, in location `HC_OFFSET_ADDR`
    */

    float cPhaseOffset = readPhaseOffset();
    if (phaseOffset != cPhaseOffset) {
        EEPROM.writeByte(HC_OFFSET_ADDR + base_addr, phaseOffset);
    }

}

float readDeclination(void) {
    /** Read the declination offset from the EEPROM memory, in degrees
    
    The inverse operation from `write_declination()`. Reads the stored declination from the
    EEPROM memory. If set from `write_declination()`, the declination should never be outside
    the range +/- 180.0, so this can be used as a sanity check against invalid values.

    @return Returns the declination in degrees as a single precision (32 bit) float. Positive
            numbers indicate a west declination, negative numbers indicate an east declination.
    */

    return EEPROM.readFloat(HC_DECLINATION_ADDR + base_addr);
}

float readInclination(void) {
    /** Read the inclination offset from the EEPROM memory, in degrees
    
    The inverse operation from `write_inclination()`. Reads the stored inclination from the
    EEPROM memory. If set from `write_inclination()`, the inclination should never be outside
    the range +/- 180.0, so this can be used as a sanity check against invalid values.l

    @return Returns the inclination in degrees as a single precision (32 bit) float.
    */

    return EEPROM.readFloat(HC_INCLINATION_ADDR + base_addr);
}

float readSampleRate(void) {
    /** Read the sample rate from the EEPROM memory, in Hz

    The inverse operation from `writeSampleRate()`. Reads the sample rate from the EEPROM, which
    is stored as a single-precision float at location `HC_SAMPLERATE_ADDR`.

    @return Returns the sample rate in Hz.
    */

    return EEPROM.readFloat(HC_SAMPLERATE_ADDR + base_addr);
}
uint16_t readPulseWidth(void) {
    /** Read the pulse width from the EEPROM memory, in ms

    The inverse operation from `writePulseWidth()`. Reads the pulse width from the EEPROM, which
    is stored as an unsigned long (32-bit) integer, at location `HC_PULSEWIDTH_ADDR`.

    @return Returns the pulse width read from the EEPROM, in ms.
    */

    return EEPROM.readInt(HC_PULSEWIDTH_ADDR + base_addr);
}

uint8_t readNMotors(void) {
    /** Read the number of motors from the EEPROM memory

    The inverse operation from `writeNMotors()`. Reads the number of motors from the EEPROM, which
    is stored as an unsigned 8-bit integer (byte), at location `HC_NMOTORS_ADDR`.

    @return Returns the number of motors.
    */

    return EEPROM.readByte(HC_NMOTORS_ADDR + base_addr);
}

float readPhaseOffset(void) {
    /** Read the phase offset from the EEPROM memory

    The inverse operation from `writePhaseOffset()`. Reads the phase offset from the EEPROM, which
    is stored as a single-precision (32-bit) float at location `HC_OFFSET_ADDR`.

    @return Returns the phase offset, in degrees.
    */

    return EEPROM.readFloat(HC_OFFSET_ADDR + base_addr);
}

uint16_t HaptiCapMagSettings::calculateChecksum(uint8_t settings_version) {
    /** Calculates a checksum of all the settings

    In order to validate that the settings stored in the EEPROM were validly set by this library
    or a previous incarnation, a checksum of all the settings is calculated using the `crc16()`
    function. The value stored at `HC_CHECKSUM_ADDR` should match the output of this function
    after a call to `readAll()` but before any calls to setter functions. When checking the
    validity of the stored settings, pass the output of `readSettingsVersion()` to this function,
    to ensure backwards compatibility.

    @param[in] settings_version The version of the settings file used when storing the data. This
                                parameter is set by default to the current settings version, but
                                should be used when validating settings as read to ensure backward
                                compatibility.
    */

    uint8_t crc = 0x00;         // Initial value

    // Ordered by place in memory, starting with number of motors.
    crc = update_CRC8_byte(crc, nMotors);
    crc = update_CRC8_float(crc, declination);
    crc = update_CRC8_float(crc, inclination);
    crc = update_CRC8_float(crc, sampleRate);
    crc = update_CRC8_uint16_t(crc, pulseWidth);
    crc = update_CRC8_float(crc, phaseOffset);
    crc = update_CRC8_byte(crc, useCalibration);

    for (int i = 0; i < 16; i++) {
        crc = update_CRC8_float(pinLocs[i]);
    }

    for (int i = 0; i < 16; i++) {
        crc = update_CRC8_float(motorCal[i]);
    }

    return crc;
}

uint8_t update_CRC8_byte(uint8_t crc, uint8_t b) {
    /** Method for updating 8-bit cyclic redundancy checksum using a byte.
    
    This is essentially a wrapper for `_crc8_ccitt_update()`, for symmetry with the other data types
    and to centralize all calls to the library function to make it easier to change the checksum
    algorithm later if necessary.

    @param[in] crc The existing CRC. Use `0x00` to initialize for simple CRC.
    @param[in] b The byte with which to update the CRC.

    @return Returns the updated checksum.
    */

    return _crc8_ccitt_update(crc, b);
}

uint8_t update_CRC8_float(uint8_t crc, float f) {
    /** Method for updating 8-bit cyclic redundancy checksum using a float.
    
    A single precision float consists of 4 bytes, so this updates the CRC8 4 times using
    a byte-wise representation of the float.

    @param[in] crc The initial CRC value. Use `0x00` to initialize for simple CRC.
    @param[in] f The float whose value is being used to update the CRC.

    @return Returns the updated checksum.
    */
    union f2byte {
        float f;
        uint8_t b[4];
    } f2b;

    f2b.f = f;

    for(int i = 0; i < 4; i++) {
        crc = update_CRC8_byte(crc, f2b.b[i]);
    }

    return crc;
}

uint8_t update_CRC8_uint16_t(uint8_t crc, uint16_t sh) {
    /** Method for updating 8-bit cyclic redundancy checksum using an unsigned short integer.
    
    Unsigned short integers consist of 2 bytes, so this updates the CRC8 2 times using a bytewise
    representation of the short.

    @param[in] crc The initial CRC value. Use `0x00` to initialize for simple CRC.
    @param[in] sh The unsigned short whose value is being used to update the CRC.

    @return Returns the updated checksum.
    */

    union s2byte {
        uint16_t s;
        uint8_t b[2];
    } s2b;

    s2b.s = sh;

    for (int i = 0; i < 2; i++) {
        crc = update_CRC8_byte(crc, s2b.b[i]);
    }

    return crc;
}