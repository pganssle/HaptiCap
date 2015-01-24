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

#define HC_SETTINGS_VERSION 0x00    /*!< Version of the settings file - needs to be stored along
                                         with the checksum for later backwards compatibilty. */
#define HC_MAX_NMOTORS 16           /*!< Maximum number of motors */

/** @defgroup EEPROMAddrs EEPROM addresses
Addresses of the HaptiCap settings in the EEPROM memory. The object can be considered a virtual
"file", located at `HaptiCapMagSettings::base_addr`. These are the offsets from that EEPROM
address at which each setting is stored.
@{
*/
// Allocate 8 bytes for various settings 0x00 -> 0x08 (5 bytes unused)
/** @defgroup MetadataAddrs Metadata addresses
@ingroup EEPROMAddrs
Addresses of metadata settings. 8 bytes are reserved (`0x00` to `0x08`).
@{ */
#define HC_VERSION_ADDR 0x00
#define HC_CHECKSUM_ADDR 0x01
/** @} */

/** @defgroup ScalarSettings Scalar settings
@ingroup EEPROMAddrs
Single value settings for the magnetometer HaptiCap. 32 bytes are allocated, from
`0x09` to `0x29`, 20 of which are currently used.
@{
*/
#define HC_NMOTORS_ADDR 0x09            /*!< Number of motors in the HaptiCap (max: 16)
                                         See `HaptiCapMagSettings::setNMotors()` for details.

                                         1 byte, type `uint8_t`, `0x09` */
#define HC_DECLINATION_ADDR 0x0a        /*!< Declination offset in degrees. (Range ±180)
                                         See `HaptiCapMagSettings::setDeclination()` for details.

                                         4 bytes, type `float`,  `0x0a` -> `0x0e` */
#define HC_INCLINATION_ADDR 0x0f        /*!< Inclination offset in degrees. (Range ±180)
                                         See `HaptiCapMagSettings::setInclination()` for details.

                                        4 bytes, type `float`, `0x0f` -> `0x12` */
#define HC_SAMPLERATE_ADDR 0x13         /*!< Sample rate, in Hz.
                                         See `HaptiCapMagSettings::setSampleRate()` for details.

                                         4 bytes, type `float`, `0x13` -> `0x16` */
#define HC_PULSEWIDTH_ADDR 0x17         /*!< Pulse width, in milliseconds
                                         See `HaptiCapMagSettings::setPulseWidth()` for details.
                                         2 bytes, type `uint16_t`, `0x17` -> `0x18` */
#define HC_OFFSET_ADDR 0x19             /*!< Phase offset, in degrees.
                                         See `HaptiCapMagSettings::setPhaseOffset()` for details.

                                         4 bytes, type `float`, `0x19` -> `0x1c` */
#define HC_USE_CALIBRATION_ADDR 0x1d    /*!< Whether or not to use calibration.
                                         See `HaptiCapMagSettings::setUseCalibration()` for details.

                                         1 byte, type `uint8_t`,  `0x1d` */
#define HC_GAIN_ADDR 0x1e               /*!< The gain setting to be used.
                                         See `HaptiCapMagSettinsg::setGain()` for details.

                                         1 byte, type `uint8_t`, `0x1e`
                                        */
/** @} */

/** @defgroup ArraySettings Array settings
@ingroup EEPROMAddrs
Array settings for the magnetometer HaptiCap. These are at the end of the file, assuming a maximum
of 16 items.
@{ */
#define HC_PINLOCS_ADDR 0x20            /*!< Pin locations - the pin locations for each motor.
                                         See `HaptiCapMagSettings::setPinLocation()` for details. 

                                         16 items, 1 byte each, `uint8_t`, `0x20` -> `0x2f` */
#define HC_MOTORCAL_ADDR HC_PINLOCS_ADDR + HC_MAX_NMOTORS  /*!< Motor calibrations per motor. 
                                         Fractional duty cycle as a fraction of 255. 
                                         See `HaptiCapMagSettings::setPinLocation()` for details. 

                                         16 items, 1 byte each, `uint8_t`, `0x30` -> `0x3f` */
/** @} */
/** @} */

/** @defgroup ErrorCodes Error Codes
Error codes for the HaptiCapMagSettings
@{ */
#define EC_INVALID_CHECKSUM 32      /*!< Checksum stored in EEPROM does not match object checksum.*/
#define EC_INVALID_GAIN_SETTING 33  /*!< Specified gain setting is invalid. */
#define EC_INVALID_MOTOR 34         /*!< Specified motor is invalid. */
#define EC_INVALID_DUTY_CYCLE 35    /*!< Specified duty cycle is outside the valid range [0, 1]. */
/** @} */

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
    float getPhaseOffset(void);
    bool getUseCalibration(void);
    uint8_t getGain(void);

    uint8_t getPinLoc(uint8_t motor);
    uint8_t getMotorCal(uint8_t motor);

    uint8_t setDeclination(float declination);
    uint8_t setInclination(float inclination);
    uint8_t setSampleRate(float sample_rate);
    uint8_t setPulseWidth(uint16_t pulse_width);
    uint8_t setNMotors(uint8_t n_motors);
    uint8_t setPhaseOffset(float phi);
    uint8_t setUseCalibration(bool use);
    uint8_t setGain(uint8_t gain);
    uint8_t setPinLoc(uint8_t motor, uint8_t pin_loc);
    
    uint8_t setMotorCal(uint8_t motor, uint8_t motor_cal);
    uint8_t setMotorCal(uint8_t motor, float motor_cal);

    // Read / write functions
    uint8_t writeAll(void);
    uint8_t readAll(void);

    // Error handling
    uint8_t get_err_code(void);
    
private:
    uint16_t calculateChecksum(uint8_t settings_version=HC_SETTINGS_VERSION);

    void writeChecksum(void);
    void writeSettingsVersion(void);
    void writeDeclination(void);
    void writeInclination(void);
    void writeSampleRate(void);
    void writePulseWidth(void);
    void writeNMotors(void);
    void writePhaseOffset(void);
    void writeUseCalibration(void);
    void writeGain(void);

    void writePinLoc(uint8_t motor);
    void writeMotorCal(uint8_t motor);

    uint8_t readChecksum(void);
    uint8_t readSettingsVersion(void);
    float readDeclination(void);
    float readInclination(void);
    float readSampleRate(void);
    uint16_t readPulseWidth(void);
    uint8_t readNMotors(void);
    float readPhaseOffset(void);
    bool readUseCalibration(void);
    uint8_t readGain(void);

    int8_t readPinLoc(uint8_t motor);
    uint8_t readMotorCal(uint8_t motor);

    uint32_t base_addr;

    bool useCalibration;

    uint8_t nMotors;
    uint8_t gain;

    float declination;
    float inclination;
    float sampleRate;
    unsigned short pulseWidth;
    float phaseOffset;

    int8_t pinLocs[HC_MAX_NMOTORS];
    uint8_t motorCals[HC_MAX_NMOTORS];

    uint8_t err_code;
};

uint8_t update_CRC8_byte(uint8_t crc, uint8_t b);
uint8_t update_CRC8_float(uint8_t crc, float f);
uint8_t update_CRC8_uint16_t(uint8_t crc, uint16_t sh);

#endif