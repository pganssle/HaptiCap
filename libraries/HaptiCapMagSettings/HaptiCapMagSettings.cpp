/** @file
Library for reading and writing HaptiCap settings.

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
    nMotors = 8;
    useCalibration = true;
    gain = 1;

    for(int i = 0; i < HC_MAX_NMOTORS; i++) {
        pinLocs[i] = -1;                // Invalid motor
        motorCals[i] = 0xff;             // Frac = 1.0
    }
}

float HaptiCapMagSettings::getDeclination() {
    /** Get the declination from the current settings object.
    
    See `setDeclination()` for more details.

    @return Returns the declination.
    */

    return declination;
}

float HaptiCapMagSettings::getInclination() {
    /** Get the inclination from the current settings object.

    See `setInclination()` for more details.

    @return Returns the inclination.
    */

    return inclination;
}

float HaptiCapMagSettings::getSampleRate() {
    /** Get the sample rate from the current settings object.
    
    See `setSampleRate()` for more details.

    @return Returns the sample rate.
    */

    return sampleRate;
}

unsigned short HaptiCapMagSettings::getPulseWidth() {
    /** Get the pulse width from the current settings object.
    
    See `setPulseWidth()` for more details.

    @return Returns the pulse width.
    */

    return pulseWidth;
}

uint8_t HaptiCapMagSettings::getNMotors() {
    /** Get the number of motors from the current settings object.
    
    See `setNMotors()` for more details.

    @return Returns the number of motors.
    */

    return NMotors;
}

float HaptiCapMagSettings::getPhaseOffset() {
    /** Get the phase offset from the current settings object.

    See `setPhaseOffset()` for more details.

    @return Returns the phase offset.
    */

    return phaseOffsets;
}

bool HaptiCapMagSettings::getUseCalibration() {
    /** Get the `useCalibration` flag from the current settings object. 

    See `setUseCalibration()` for more details.

    @return Returns the `useCalibration` flag.
    */

    return useCalibration;
}

uint8_t HaptiCapMagSettings::getGain() {
    /** Get the gain value from the current settings object.

    See `setGain()` for details.

    @return Returns the gain value.
    */

    return gain;
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

    @param[in] declination The declination in degrees, which wraps at ±180.

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

    @param[in] inclination The inclination in degrees, which wraps at ±180.

    @return Returns 0 on no error. Currently this can raise no errors.
    */

    inclination = fmod(inclination, 180.0);
    return 0;
}

uint8_t HaptiCapMagSettings::setSampleRate(float sample_rate) {
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

uint8_t HaptiCapMagSettings::setPulseWidth(uint16_t pulse_width) {
    /** Set the pulse width for the current settings object, in milliseconds
    
    Motor calibrations are expressed as fractional duty cycles - the vibrator motors are pulsed on
    and off over the period `pulse_width`. For sufficiently short pulse widths, this is experienced
    as continuous vibration with the strength scaled by the duty cycle. For longer pulse widths,
    it is experienced as intermittent vibration.

    @param[in] pulse_width The pulse width, in milliseconds

    @return Returns 0 on no error. There are currently no exceptional conditions which would raise
            an error.
    */

    pulseWidth = pulse_width;
    return 0;
}

uint8_t HaptiCapMagSettings::setNMotors(uint8_t n_motors) {
    /** Sets the number of motors for the current settings object.
    
    If the number of motors built into the HaptiCap is changed, update this setting.
    
    @param[in] n_motors The number of motors in the HaptiCap. Minimum is 1,
                        maximum is `HC_MAX_NMOTORS`.

    @return Returns 0 on no error. Raises `EC_INVALID_N_MOTORS` if an invalid number of motors is
            specified.
    */

    if (n_motors < 1 || n_motors > HC_MAX_NMOTORS) {
        return EC_INVALID_N_MOTORS;
    }

    nMotors = n_motors;
    return 0;
}

uint8_t HaptiCapMagSettings::setPhaseOffset(float phi) {
    /** Sets the phase offset for the current settings object, in degrees. Wraps at ±180.
    
    The default assumption is that the motors are evenly arrayed around the band of the cap, with
    the `0` motor located at 0°. This offset can be used to adjust the assumed location of the `0`
    motor.

    @param[in] phi The phase offset of the `0` motor. Wraps at ±180

    @return Returns 0 on no error.
    */

    phaseOffset = fmod(phi, 180);
    return 0;
}
uint8_t HaptiCapMagSettings::setUseCalibration(bool use) {
    /** Set the `useCalibration` flag for the current settings object.
    
    The HMC5883L magnetometer has a self-test procedure wherein known fields are applied along all
    three axes for calibration, using built-in coils. These can be used to calibrate the response
    along the three axes, potentially providing a more accurate heading value. The primary danger
    in using the calibration is that, because the single calibration value is used for all
    measurements, errors in calibration may result in systematic errors.

    @param[in] use The new value for the `useCalibration` flag.

    @return Returns 0 on no error.
    */

    useCalibration = use;
    return 0;
}

uint8_t HaptiCapMagSettings::setGain(uint8_t gain_value) {
    /** Set the gain value for the HMC5883L magnetometer.
    
    @param[in] gain The gain value. If using the provided `HMC5883L` library, aliases are defined
                    for the valid values:

    |     Alias     | `gain_value` | Gain (LSB/G) | Range (G) | Resolution (mG / LSB) |
    | :------------ | :----------: | :----------: | :-------: | :-------------------: |
    | `HMC_GAIN088` |      0       |    1370      |   ±0.88   |        0.73           |
    | `HMC_GAIN130` |      1       |    1090      |   ±1.30   |        0.92           |
    | `HMC_GAIN190` |      2       |     820      |   ±1.90   |        1.22           |
    | `HMC_GAIN250` |      3       |     660      |   ±2.50   |        1.52           |
    | `HMC_GAIN400` |      4       |     440      |   ±4.00   |        2.27           |
    | `HMC_GAIN470` |      5       |     390      |   ±4.70   |        2.56           |
    | `HMC_GAIN560` |      6       |     330      |   ±5.60   |        3.03           |
    | `HMC_GAIN810` |      7       |     230      |   ±8.10   |        4.35           |

    Future versions may include an auto-gain setting, gain is determined adaptively.

    @return Returns 0 on no error. Returns `EC_INVALID_GAIN_SETTING` if an invalid gain value is
            specified.
    */

    if (gain_value > 7) {
        return (err_code = EC_INVALID_GAIN_SETTING);
    }

    gain = gain_value;
    return 0;
}

uint8_t HaptiCapMagSettings::setPinLoc(uint8_t motor, int8_t pin_loc) {
    /** Set the pin location for the specified motor.

    Each motor is actuated by a specific pin (either the output pin of a multiplexer or one of
    the digital IO pins of the microcontroller). This setting defines the mapping between motor
    number and pin location.

    @param[in] motor The motor whose pin location you are specifying. Must be less than
                     `HC_MAX_NMOTORS`.
    @param[in] pin_loc The pin location which actuates the specified motor. Specify a negative value
                      if the motor is not used. Invalid pin locations are stored as `-1`.

    @return Returns 0 on no error. Returns `EC_INVALID_MOTOR` if `motor >= HC_MAX_NMOTORS`.
    */

    if (motor >= HC_MAX_NMOTORS) {
        err_code = EC_INVALID_MOTOR;
        return err_code;
    }

    if (pin_loc < 0) {
        pin_loc = -1;        // Invalid pin locations are always -1.
    }

    pinLocs[motor] = pin_loc;
    return 0;
}

uint8_t HaptiCapMagSettings::setMotorCal(uint8_t motor, uint8_t motor_cal) {
    /** Set the motor calibration for motor `motor` in the current settings object.

    Based on the position and strength of each motor, the perceived intensity of the motors may
    differ (e.g. motors located close to the ear tend to feel stronger than those located in the
    back of the head). The `motorCals` array specifies the duty cycle for each motor, which, for
    a sufficiently short `pulseWidth`, should be subjectively equivalent to a reduction in the
    motor intensity. 
    
    The duty cycle is expressed in units of `1/255`, so a 50% duty cycle is represented as `128`,
    etc. A convenience method for specifying duty cycles as floats is provided as
    `setMotorCal(uint8_t, float)`. 
    
    @param[in] motor The motor whose calibration is to be set. This value must not exceed
                     `HC_MAX_NMOTORS`.
    @param[in] motor_cal The motor calibration duty cycle, specified as a number between 0 and 255.
                        The duty cycle will be interpreted as `motorCal`/255.0.

    @return Returns 0 on no error. Returns `EC_INVALID_MOTOR` if an invalid motor is specified.
    */

    if (motor >= HC_MAX_NMOTORS) {
        err_code = EC_INVALID_MOTOR;
        return err_code;
    }

    motorCals[motor] = motor_cal;
    return 0;
}

uint8_t HaptiCapMagSettings::setMotorCal(uint8_t motor, float motor_cal) {
    /** Set the motor calibration for motor `motor` in the current settings object from duty cycle.
    
    Alias for `setMotorCal(uint8_t, uint8_t)` which takes a float as input.

     @param[in] motor_cal A float in the range [0.0, 1.0] representing the duty cycle. This will be
                        represented as a fraction of 255, rounded to the nearest integer (as
                        opposed to the floor, which is default behavior when casting from float to
                        int).

    @return Returns 0 on no error. Returns `EC_INVALID_DUTY_CYCLE` if an invalid `motor_cal` is
            specified. Errors also can arise from an internal call to
            `setMotorCal(uint8_t, uint8_t)`.
    */

    return setMotorCal(uint8_t motor, uint8_t((motor_cal*255) + 0.5));
}

uint8_t HaptiCapMagSettings::writeAll() {
    /** Write the values from the settings object to the EEPROM memory
    
    @param[in] motorCal A float in the range [0.0, 1.0] representing the duty cycle. This will be
                        represented as a fraction of 255, rounded to the nearest integer (as
                        opposed to the floor, which is default behavior when casting from float to
                        int).
    @return Returns 0 on no error.
    */
    
    // This is a wrapper for all-in one calls to the individual reads and writes.
    writeChecksum();
    writeSettingsVersion();

    writeDeclination();
    writeInclination();
    writeSampleRate();
    writePulseWidth();
    writeNMotors();
    writePhaseOffset();
    writeUseCalibration();
    writeGain();

    for (int i = 0; i < HC_MAX_NMOTORS; i++) {
        writePinLoc(i);
        writeMotorCal(i);
    }

    return 0;
}

uint8_t HaptiCapMagSettings::readAll() {
    /** Read the values from the EEPROM memory into this settings object

    @return Returns 0 on no error. Returns `EC_BAD_CHECKSUM` on invalid checksum.
    */

    // Scalar settings
    declination = readDeclination();
    inclination = readInclination();
    sampleRate = readSampleRate();
    pulseWidth = readPulseWidth();
    nMotors = readNMotors();
    phaseOffset = readPhaseOffset();
    useCalibration = readUseCalibration();
    gain = readGain();

    // Array settings
    for (int i = 0; i < HC_MAX_NMOTORS; i++) {
        pinLocs[i] = readPinLoc(i);
        motorCals[i] = readMotorCal(i);
    }

    // Validate checksum
    uint8_t sv = readSettingsVersion();
    uint8_t ochecksum = readChecksum();
    uint8_t checksum = calculateChecksum(sv);

    if (checksum != ochecksum) {
        err_code = EC_BAD_CHECKSUM;
    }

    return err_code;
}

uint8_t HaptiCapMagSettings::get_err_code() {
    /** Get the error code from the current settings object.

    @return Returns the error code from the current settings object.  The details for each code can
            be found in \ref ErrorCodes; 0 for no error. 
    */

    return err_code;
}

void HaptiCapMagSettings::writeChecksum() {
    /** Writes a checksum of the current settings object to the EEPROM */

    // Calculate the checksum
    uint8_t checksum = calculateChecksum(HC_SETTINGS_VERSION);

    // Write the checksum to the EEPROM if it's not already written there.
    uint8_t cChecksum = readChecksum();
    if (checksum != cChecksum) {
        EEPROM.writeByte(HC_CHECKSUM_ADDR + base_addr, checksum);
    }
}

void HaptiCapMagSettings::writeDeclination() {
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

void HaptiCapMagSettings::writeInclination() {
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

void HaptiCapMagSettings::writeSampleRate() {
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

void HaptiCapMagSettings::writePulseWidth() {
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

void HaptiCapMagSettings::writeNMotors() {
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

void HaptiCapMagSettings::writePhaseOffset() {
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

void HaptiCapMagSettings::writeUseCalibration() {
    /** Write the `useCalibration` flag to the EEPROM memory

    Write the `useCalibration` flag stored in the settings object (which can be user specified by
    a call to `setUseCalibration()`) to the EEPROM memory. The flag is stored as a byte at location
    `HC_USE_CALIBRATION_ADDR`.
    */

    bool cUseCalibration = readUseCalibration();

    if (useCalibration != cUseCalibration) {
        EEPROM.writeByte(HC_USE_CALIBRATION_ADDR + base_addr, uint8_t(useCalibration));
    }
}

void HaptiCapMagSettings::writePinLoc(uint8_t motor) {
    /** Write the pin location for the specified motor to the EEPROM memory.

    @param[in] motor The motor whose pin location is to be written to EEPROM. Maximum value is
                     `HC_MAX_NMOTORS`.
    */

    int8_t cPinLoc = readPinLoc(motor);

    if (pinLoc[motor] != cPinLoc) {
        EEPROM.writeByte(HC_PINLOCS_ADDR + motor + base_addr, pinLoc[motor]);
    }
}

void HaptiCapMagSettings::writeMotorCal(uint8_t motor) {
    /** Write the motor calibration for the specified motor to the EEPROM memory. 

    @param[in] motor The motor whose calibration you want to write to EEPROM. Maximum value is
                     `HC_MAX_NMOTORS`.
    */

    uint8_t cMotorCal = readMotorCal(motor);

    if (motorCals[motor] != cMotorCal) {
        EEPROM.writeByte(HC_MOTORCAL_ADDR + motor + base_addr, motorCals[motor]);
    }
}

float HaptiCapMagSettings::readDeclination() {
    /** Read the declination offset from the EEPROM memory, in degrees
    
    The inverse operation from `write_declination()`. Reads the stored declination from the
    EEPROM memory. If set from `write_declination()`, the declination should never be outside
    the range +/- 180.0, so this can be used as a sanity check against invalid values.

    @return Returns the declination in degrees as a single precision (32 bit) float. Positive
            numbers indicate a west declination, negative numbers indicate an east declination.
    */

    return EEPROM.readFloat(HC_DECLINATION_ADDR + base_addr);
}

float HaptiCapMagSettings::readInclination() {
    /** Read the inclination offset from the EEPROM memory, in degrees
    
    The inverse operation from `write_inclination()`. Reads the stored inclination from the
    EEPROM memory. If set from `write_inclination()`, the inclination should never be outside
    the range +/- 180.0, so this can be used as a sanity check against invalid values.l

    @return Returns the inclination in degrees as a single precision (32 bit) float.
    */

    return EEPROM.readFloat(HC_INCLINATION_ADDR + base_addr);
}

float HaptiCapMagSettings::readSampleRate() {
    /** Read the sample rate from the EEPROM memory, in Hz

    The inverse operation from `writeSampleRate()`. Reads the sample rate from the EEPROM, which
    is stored as a single-precision float at location `HC_SAMPLERATE_ADDR`.

    @return Returns the sample rate in Hz.
    */

    return EEPROM.readFloat(HC_SAMPLERATE_ADDR + base_addr);
}
uint16_t HaptiCapMagSettings::readPulseWidth() {
    /** Read the pulse width from the EEPROM memory, in ms

    The inverse operation from `writePulseWidth()`. Reads the pulse width from the EEPROM, which
    is stored as an unsigned long (32-bit) integer, at location `HC_PULSEWIDTH_ADDR`.

    @return Returns the pulse width read from the EEPROM, in ms.
    */

    return EEPROM.readInt(HC_PULSEWIDTH_ADDR + base_addr);
}

uint8_t HaptiCapMagSettings::readNMotors() {
    /** Read the number of motors from the EEPROM memory

    The inverse operation from `writeNMotors()`. Reads the number of motors from the EEPROM, which
    is stored as an unsigned 8-bit integer (byte), at location `HC_NMOTORS_ADDR`.

    @return Returns the number of motors.
    */

    return EEPROM.readByte(HC_NMOTORS_ADDR + base_addr);
}

float HaptiCapMagSettings::readPhaseOffset() {
    /** Read the phase offset from the EEPROM memory

    The inverse operation from `writePhaseOffset()`. Reads the phase offset from the EEPROM, which
    is stored as a single-precision (32-bit) float at location `HC_OFFSET_ADDR`.

    @return Returns the phase offset, in degrees.
    */

    return EEPROM.readFloat(HC_OFFSET_ADDR + base_addr);
}

bool HaptiCapMagSettings::readUseCalibration() {
    /** Read the `useCalibration` flag from the EEPROM memory
    
    The inverse operation from `writeUseCalibration()`. Reads the `useCalibration` flag from the
    EEPROM, which is stored as a byte at location `HC_USE_CALIBRATION_ADDR`.

    @return Returns the flag.
    */

    return bool(EEPROM.readByte(HC_USE_CALIBRATION_ADDR + base_addr));
}

uint8_t HaptiCapMagSettings::readGain() {
    /** Reads the gain setting from the EEPROM memory

    The inverse operation from `writeGain()`. Reads the gain setting from the EEPROM memory, which
    is stored as a `uint8_t` at location `HC_GAIN_ADDR`.

    @return Returns the gain setting.
    */

    return EEPROM.readByte(HC_GAIN_ADDR + base_addr);
}

int8_t HaptiCapMagSettings::readPinLoc(uint8_t motor) {
    /** Reads a single pin location from the EEPROM memory
    
    @param[in] motor The motor whose pin location you want to read out. 0-based index. Maximum
                     value is 15.

    @return Returns the pin location. Returns 0 and sets the error code on error.
    */

    if (motor >= HC_MAX_NMOTORS) {
        err_code = EC_INVALID_MOTOR;
        return 0;
    }

    return EEPROM.readByte(HC_PINLOCS_ADDR + motor + base_addr);
}

uint8_t HaptiCapMagSettings::readMotorCal(uint8_t motor) {
    /** Reads a single motor calibration from the EEPROM memory

    @param[in] motor The motor whose calibration you'd want to read out. 0-based index. Maximum
                     value is 15.

    @return Returns the motor calibration, as a numeric setting from 0 to 255. On error, returns
            0 and sets the error code.
    */

    if (motor >= HC_MAX_NMOTORS) {
        err_code = EC_INVALID_MOTOR;
        return 0;
    }

    return EEPROM.readByte(HC_MOTORCAL_ADDR + motor + base_addr);
}

uint8_t HaptiCapMagSettings::calculateChecksum(uint8_t settings_version) {
    /** Calculates an 8-bit checksum of all the settings

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
    crc = update_CRC8_byte(crc, gain);

    // Update the array settings - these are done in order, even though doing it in parallel would
    // only require a single loop.
    for (int i = 0; i < HC_MAX_NMOTORS; i++) {
        crc = update_CRC8_float(pinLocs[i]);
    }

    for (int i = 0; i < HC_MAX_NMOTORS; i++) {
        crc = update_CRC8_float(motorCals[i]);
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