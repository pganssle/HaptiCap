/** @file
Arduino sketch for HaptiCap haptic compass, using only the HMC5883L digital magnetometer and
the microcontroller.

In this version, serial input and output provides access to a debugging mode and allows the user
to update their general settings.

This code is released under a Creative Commons Attribution 4.0 International License
([CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)).

@author Paul J. Ganssle
@version 0.1
@date 2015-01-18
*/

#include <Arduino.h>
#include <EEPROMex.h>
#include <Wire.h>
#include <Vec3.h>
#include <HMC5883L.h>
#include <HaptiCapMagSettings.h>
#include "HaptiCapCalibration.h"
#include <tgmath>
#include <stdlib.h>

#define SETTINGS_LOC 0x00       /*!< The location of the settings file in EEPROM memory */
 
#define SERIAL_BAUD_RATE 9600   /*!< The baud rate for the serial interface */
#define STRING_BUFFER 128

#define N_FUNCS 19
#define FNAME_BUFF 25
#define FARGS_BUFF 25

/** @defgroup RunningModes Running modes
These are flags for a set of partially non-exclusive running modes.
@{ */
#define HCM_RUN_MODE 0
#define HCM_DEBUG_MODE 1
#define HCM_OUT_MODE 2
#define HCM_CART_MODE 3

/** @} */

// Declare global variables
HMC5883L compass;               /*!< Interface to the HMC5883L magnetometer */
HaptiCapMagSettings settings;   /*!< Object which contains the settings for the run */

float motor_fracs[HC_MAX_NMOTORS];

uint8_t cmotor = HC_MAX_NMOTORS+1;               // Current active motor
uint8_t cdmotor = HC_MAX_NMOTORS+1;

unsigned long cDelay = 100;

boolean stringComplete = false;
uint8_t nCharsFound = 0;
char serialBuffer[STRING_BUFFER];

boolean verbose = false;
uint8_t mode = HCM_RUN_MODE;   /*!< The current running mode - defaults to running the compass. */

void setup() {
    settings = HaptiCapMagSettings(SETTINGS_LOC);       // Create the settings object

    // Initialize the compass
    compass = HMC5883L();
    compass.initialize();

    if (reloadSettings()) {
        mode = HCM_DEBUG_MODE;
    }

    // Start the serial port
    Serial.begin(SERIAL_BAUD_RATE);
}

void loop() {
    if (mode & HCM_RUN_MODE) {
        update_compass();
    }

    if (stringComplete) {
        uint8_t ev = processFunction(serialBuffer);
        stringComplete = false;
    }
}

void serialEvent() {
    while(!stringComplete && Serial.available()) {
        char inChar = 0;
        
        if (nCharsFound >= STRING_BUFFER) {
            Serial.println("Serial buffer exceeded!");
            inChar = '\n';  // Complete the string
        } else {
            inChar = (char)Serial.read();
            // Get the new byte
            serialBuffer[nCharsFound++] = inChar;
            serialBuffer[nCharsFound] = '\0';        // Null terminate
        }

        // If the incoming line is a newline or ';', set a flag.
        if(inChar == '\n' || inChar == ';') {
            stringComplete = true;
            nCharsFound = 0;
        }
    }
}

uint8_t processFunction(char * funcString) {
    /** Process function strings passed to the Arduino via the serial input
    
    @param[in] funcString The function string, including arguments. Valid commands are:
        | Function          | Description                                      |
        | :--------------:  | :----------------------------------------------- |
        | `ToggleOutput()`  | Toggles whether or not to output the values      |
        | `ToggleCompass()` | Toggles whether or not to run the haptic compass |
        | `ToggleDebugMode()`| Toggles whether or not to run in debug mode     |
        | `ToggleCartesianOutput()` | Toggles between cartesian and spherical coordinates in outputs |
        | `ToggleVerboseMode()` | Toggles whether or not to run in verbose mode |
        | `SetMotor(uint8_t motor)` | Set the selected motor running (with the current settings for its duty cycle |
        | `SetDeclination(float declination)` | Set the current declination (East is negative) |
        | `SetInclination(float inclination)` | Set the current inclination |
        | `SetPulseWidth(uint16_t pulse_width)` | Set the pulse width for motor calibrations |
        | `SetSampleRate(float sample_rate)` | Set the sample rate for the current settings object |
        | `SetNMotors(uint8_t nMotors)` | Set the number of motors in the HaptiCap |
        | `SetPhaseOffset(float phaseOffset)` | Set the phase offset for the current settings object |
        | `SetGain(uint8_t gain)` | Set the gain for the settings object |
        | `SetAvg(uint8_t rate) | Set the averaging rate for the settings object |
        | `ToggleCalibration()` | Toggles whether or not to use the calibration. |
        | `SetPinLoc(uint8_t motor, int8_t pin)` | Sets the digital output pin (`pin`) for a given motor (`motor`) |
        | `SetMotorCal(uint8_t motor, uint8_t frac)` | Set motor calibration as a fraction of 255 |
        | `SetMotorCalF(uint8_t motor, float frac)` | Set the motor calibration by fractional duty cycle [0, 1] |
        | `WriteSettings()` | Write the current settings to the EEPROM |
        | `ReloadSettings()` | Reload the settings from the EEPROM (or the default, if no settings are saved) |
        | `OutputSettings()` | Prints to the serial port a breakdown of the current settings object | 
    */
    // Break this apart into functions and arguments
    char func_match[FNAME_BUFF] = "";
    char func_args[FARGS_BUFF] = "";

    int i;      // Declare outside of loop because its final value is used when finding args
    boolean func_arg_found = false;
    for (i = 0; i < strlen(funcString); i++) {
        if(funcString[i] == '(') {
            func_match[i++] = '\0';     // Null terminate and increment i
            func_arg_found = true;
            break;
        }
        func_match[i] = funcString[i];   // Read into the buffer
    }

    // If an invalid function was passed, return an error.
    if (!func_arg_found) {
        return HCC_EC_MALFORMED_FUNC;
    }

    boolean func_arg_end_found = false;
    uint8_t strPos = 0;
    for (int j = 0; j < strlen(funcString)-i; j++) {
        if (funcString[i+j] == ')') {
            func_args[strPos] = '\0';
            func_arg_end_found = true;
            break;
        } else if (funcString[i + j] == ' ' || funcString[i + j] == '\n' ||
                   funcString[i + j] == '\t') {
            continue;           // Skip over any whitespace.
        }
        func_args[strPos++] = funcString[i+j];  // Read into the buffer and update position
    }

    if (!func_arg_end_found) {
        return HCC_EC_MALFORMED_ARGS;
    }

    // Determine which function it is so we can parse the arguments intelligently
    char * func_names[N_FUNCS];
    func_names[HCC_FD_TOG_OUTPUT] = "ToggleOutput";
    func_names[HCC_FD_TOG_COMPASS] = "ToggleCompass";
    func_names[HCC_FD_TOG_CART] = "ToggleCartesianOutput";
    func_names[HCC_FD_TOG_VERBOSE] = "ToggleVerboseMode";
    func_names[HCC_FD_TOG_DEBUG] = "ToggleDebugMode";
    func_names[HCC_FD_SET_MOTOR] = "SetMotor";
    func_names[HCC_FD_SET_DECL] = "SetDeclination";
    func_names[HCC_FD_SET_INCL] = "SetInclination";
    func_names[HCC_FD_SET_PW] = "SetPulseWidth";
    func_names[HCC_FD_SET_SR] = "SetSampleRate";
    func_names[HCC_FD_SET_NMOTORS] = "SetNMotors";
    func_names[HCC_FD_SET_PHASE_OFF] = "SetPhaseOffset";
    func_names[HCC_FD_SET_GAIN] = "SetGain";
    func_names[HCC_FD_SET_AVG] = "SetAvg";
    func_names[HCC_FD_TOG_CAL] = "ToggleCalibration";
    func_names[HCC_FD_SET_PIN_LOC] = "SetPinLoc";
    func_names[HCC_FD_SET_MCAL] = "SetMotorCal";
    func_names[HCC_FD_SET_MCALF] = "SetMotorCalF";
    func_names[HCC_FD_WRITE_SET] = "WriteSettings";
    func_names[HCC_FD_RELOAD_SET] = "ReloadSettings";
    func_names[HCC_FD_OUTPUT_SET] = "OutputSettings";

    // Test for a valid function
    int cFunc = -1;
    for (i = 0; i < N_FUNCS; i++) {
        if (strcmp(func_match, func_names[i]) == 0) {
            cFunc = i;
            break;
        }
    }

    if (cFunc < 0) {
        return HCC_EC_UNKNOWN_FUNCTION;
    }

    // Parse arguments
    switch (cFunc) {
        // First the cases with no arguments to parse - just run them.
        case HCC_FD_TOG_OUTPUT:
            return toggleOutput();
        case HCC_FD_TOG_COMPASS:
            return toggleCompass();
        case HCC_FD_TOG_DEBUG:
            return toggleDebugMode();
        case HCC_FD_TOG_CAL:
            return toggleCalibration();
        case HCC_FD_WRITE_SET:
            return writeSettings();
        case HCC_FD_RELOAD_SET:
            return reloadSettings();
        case HCC_FD_OUTPUT_SET:
            return outputSettings();
        case HCC_FD_TOG_CART:
            return toggleCartesianOutput();
        case HCC_FD_TOG_VERBOSE:
            return toggleVerbose();

        // Now the oddball cases with unique arguments
        case HCC_FD_SET_PIN_LOC:
        case HCC_FD_SET_MCAL:
            int8_t int1, int2;
            if (sscanf(func_args, "%d,%d", &int1, &int2) < 2) {
                return HCC_EC_INVALID_FARG;
            }

            if (int1 < 0) {
                return HCC_EC_INVALID_UINT;
            }

            if (cFunc == HCC_FD_SET_PIN_LOC) {
                return setPinLoc(int1, int2);
            } else if (cFunc == HCC_FD_SET_MCAL) {
                if (int2 < 0) {
                    return HCC_EC_INVALID_UINT;
                }
                return setMotorCal(int1, int2);
            }

        case HCC_FD_SET_MCALF:
            int8_t motor;
            float dc;
            if (sscanf(func_args, "%d,%f", &motor, &dc) < 2) {
                return HCC_EC_INVALID_FARG;
            }

            if (motor < 0) {
                return HCC_EC_INVALID_UINT;
            }

            return setMotorCalFloat(motor, dc);


        // Next the cases with one uint argument
        case HCC_FD_SET_MOTOR:
        case HCC_FD_SET_PW:
        case HCC_FD_SET_NMOTORS:
        case HCC_FD_SET_GAIN:
        case HCC_FD_SET_AVG:
            int int_arg;
            if (!sscanf(func_args, "%d", &int_arg)) {
                return HCC_EC_INVALID_FARG;
            }

            if (int_arg < 0) {
                return HCC_EC_INVALID_UINT;
            }

            if (cFunc == HCC_FD_SET_MOTOR) {
                return setDebugMotor(int_arg);
            } else if (cFunc == HCC_FD_SET_PW) {
                return setPulseWidth(int_arg);
            } else if (cFunc == HCC_FD_SET_NMOTORS) {
                return setNMotors(int_arg);
            } else if (cFunc == HCC_FD_SET_GAIN) {
                return setGain(int_arg);
            } else if (cFunc == HCC_FD_SET_AVG) {
                return setAvg(int_arg);
            }

        // Now the cases that take a single float
        case HCC_FD_SET_DECL:
        case HCC_FD_SET_INCL:
        case HCC_FD_SET_SR:
        case HCC_FD_SET_PHASE_OFF:
            float float_arg;
            if (!sscanf(func_args, "%f", &float_arg)) {
                return HCC_EC_INVALID_FARG;
            }

            if (cFunc == HCC_FD_SET_DECL) {
                return setDeclination(float_arg);
            } else if (cFunc == HCC_FD_SET_INCL) {
                return setInclination(float_arg);
            } else if (cFunc == HCC_FD_SET_SR) {
                return setSampleRate(float_arg);
            } else if (cFunc == HCC_FD_SET_PHASE_OFF) {
                return setPhaseOffset(float_arg);
            }
    }



}

uint8_t update_compass() {
    /** The main loop function run when in compass run mode. */

    // Set the compass to the task of measuring. Must wait at least 6.25 ms.
    compass.setMeasurementMode(HMC_MeasurementSingle);

    // If we're on a motor with a fractional duty cycle, set up a PWM cycle.
    // Try to use a sufficiently short fractional subunit to avoid perceptible
    // pulsation.
    if (cmotor < 8 && motor_fracs[cmotor] < 1.0) {
        // Calculate the number of pulseWidth units make up the sampling period.
        unsigned long delay_decimation = cDelay/settings.getPulseWidth();

        float off_delay = cDelay*(1-motor_fracs[cmotor]) / delay_decimation;
        float on_delay = cDelay*motor_fracs[cmotor] / delay_decimation;

        for(int i = 0; i < delay_decimation; i++) {
            set_motor(cmotor, LOW);
            delay(off_delay);

            set_motor(cmotor, HIGH);
            delay(on_delay);
        }
    } else {
        delay(cDelay);              // Wait the sampling period before reading the values.
    }

    // Read the calibrated sensor values from the compass
    uint8_t saturated;
    Vec3<float> values = compass.readCalibratedValues(NULL);

    // The sensor is upside-down, so invert the x and z axes.
    values.x = -values.x;
    values.z = -values.z;           // Not strictly necessary, as this is currently unused.

    // Figure out which motor is facing north.
    float north = calc_north(values.x, values.y, settings.getDeclination());
    uint8_t motor = select_motor(north);

    // If the motor is different, set the new one buzzing.
    if (motor != cmotor) {
        change_motor(motor);
    }

    if (mode & HCM_OUT_MODE) {
        output_vector_cart_serial(values);
    }

    return 0;
}

//
// Various outputs
//
void output_vector_cart_serial(Vec3<float> value) {
    /** Output a specified value over the serial port as an ASCII line.
    
    Outputs the x, y and z axes in mG on a single line in the format:
    `##±XXXX.XX, ±YYYY.YY, ±ZZZZ.ZZ`

    @param[in] value The value to output.
    */

    char strBuff[10];

    Serial.print("##");       // Starts an output line, just my personal convention

    // x value
    dtostrf(value.x, 4, 2, strBuff);
    if (value.x >= 0.0) {
        Serial.print("+");
    }
    Serial.print(strBuff);
    Serial.print(", ");

    // y value
    dtostrf(value.y, 4, 2, strBuff);
    if (value.y >= 0.0) {
        Serial.print("+");
    }
    Serial.print(strBuff);
    Serial.print(", ");

     // z value
    dtostrf(value.z, 4, 2, strBuff);
    if (value.z >= 0.0) {
        Serial.print("+");
    }
    Serial.print(strBuff);
    Serial.print("\n");
}

void output_vector_spherical_serial(Vec3<float> value) {
    /** Output a specified value over the serial port as an ASCII line.
    
    Outputs the r, theta and phi axes in mG and degrees on a single line in the format:
    `##±RRRR.RR, ±TTT.TT, ±PPP.PP`

    @param[in] value The value to output.
    */

    // Convert to r, theta, phi.
    float r, theta, phi;
    r = sqrt(pow(value.x, 2) + pow(value.y, 2) + pow(value.z, 2));
    theta = atan2(value.y, value.x) * 180 / M_PI;
    phi = acos(value.z/r) * 180 / M_PI;


    char strBuff[10];

    Serial.print("##");       // Starts an output line, just my personal convention

    // r value
    dtostrf(r, 4, 2, strBuff);
    if (r >= 0.0) {
        Serial.print("+");
    }
    Serial.print(strBuff);
    Serial.print(", ");

    // theta value
    dtostrf(theta, 3, 2, strBuff);
    if (theta >= 0.0) {
        Serial.print("+");
    }
    Serial.print(strBuff);
    Serial.print(", ");

     // phi value
    dtostrf(phi, 3, 2, strBuff);
    if (phi >= 0.0) {
        Serial.print("+");
    }
    Serial.print(strBuff);
    Serial.print("\n");
}

//
// Motor handling
//
uint8_t select_motor(float phi) {
    /** Determine the motor closest to the angle phi

    @param[in] phi The heading angle you want to indicate.

    @return Returns the motor closest to the angle phi.
    */
    
    float phi_sep = 360/settings.getNMotors();
    float adjusted_heading = fmod(phi-phi_sep*0.5 + settings.getPhaseOffset(), 360.0);

    if(adjusted_heading < 0) {
        adjusted_heading = 360+adjusted_heading;
    }

    return (uint8_t)(adjusted_heading/phi_sep);
}

void change_motor(uint8_t motor) {
    /** Turn off the current (`cmotor`) and turn on the new motor (`motor`)

    @param[in] motor The new selected motor
    */
    
    if (cmotor < settings.getNMotors()) {
        set_motor(cmotor, LOW);
    }

    if(motor < settings.getNMotors()) {
        set_motor(motor, HIGH);
    }

    cmotor = motor;
}

void set_motor(uint8_t motor, bool h_low) {
    /** Turn motor `motor` to on or off, as specified by `h_low`

    @param[in] motor The motor whose state you want to update.
    @param[in] h_low The new state of the motor
    */

    if (motor > settings.getNMotors()) {
        return;
    }

    digitalWrite(settings.getPinLoc(motor), h_low);
}

float calc_north(float x, float y, float decl) {
    /** Calculate the magnetometer heading

    Calculates the magnetometer heading, i.e. the angle between the x axis of the
    device and north.

    @param[in] x x component of the magnetic field vector
    @param[in] y y component of the magnetic field vector
    @param[in] decl Declination to north

    @param Returns the heading to north.
    */

    float north = fmod(atan2(y, x) * 180.0 / M_PI, 360.0);
    if (north < 0) {
        north += 360.0;
    }

    return north + decl;
}

// Functions run by the user over serial ports
uint8_t toggleOutput() {
    mode ^= HCM_OUT_MODE;       // Exclusive OR on the output mode bit.

    return 0;
}

uint8_t toggleCompass() {
    mode ^= HCM_RUN_MODE;               // Toggle running mode
    
    // Turn off debugging mode if running mode is on
    if (mode & HCM_RUN_MODE) {
        mode &= 0xff - HCM_DEBUG_MODE;
        if (verbose) {
            Serial.println("Running mode enabled.");
        }
    } else if (verbose) {
        Serial.println("Running mode disabled.");
    }

    return 0;
}

uint8_t toggleDebugMode() {
    mode ^= HCM_DEBUG_MODE;             // Toggle debug mode

    // Turn off running mode if debug mode is on
    if (mode & HCM_DEBUG_MODE) {
        mode &= 0xff - HCM_RUN_MODE;
    }

    if (verbose) {
        Serial.print("Debug mode ");
        Serial.print((mode & HCM_DEBUG_MODE)?"enabled":"disabled");
        Serial.println(".");
    }
    return 0;
}

uint8_t toggleCalibration() {
    boolean useCalibration = !settings.getUseCalibration();
    if (verbose) {
        Serial.print("Calibration ");
        Serial.print(useCalibration?"will":"will not");
        Serial.println(" be used.");
    }

    if (useCalibration) {
        compass.getCalibration(true);   // Make sure the calibration is valid.
    }

    return settings.setUseCalibration(useCalibration);
}

uint8_t toggleVerbose() {
    verbose = !verbose;
    if (verbose) {
        Serial.println("Setting mode to verbose.");
    }
    return 0;
}

uint8_t toggleCartesianOutput() {
    mode ^= HCM_CART_MODE;      // Toggle cartesian mode

    if (verbose) {
        Serial.print("Cartesian output mode ");
        Serial.println((mode & HCM_CART_MODE)?"enabled":"disabled");
    }
}

uint8_t writeSettings() {
    if (verbose) {
        Serial.println("Writing settings to EEPROM");
    }

    return settings.writeAll();
}

uint8_t reloadSettings() {
    uint8_t read_all_err = settings.readAll();

    if (read_all_err) {
        settings = HaptiCapMagSettings(SETTINGS_LOC);
    }
    // Set up the initial averaging rate and sensor gain
    compass.setAveragingRate(settings.getAveraging());     // 8 Averages per measurement
    compass.setGain(settings.getGain());

    if (settings.getUseCalibration()) {
        compass.getCalibration(true);               // Calibrate the compass with the self test
    }

    // Set the measurement mode to Idle (no measurements)
    compass.setMeasurementMode(HMC_MeasurementIdle);

    // Initialize the digital pin outs and store the motor pin locations
    for (int i = 0; i < settings.getNMotors(); i++) {
        int8_t pin = settings.getPinLoc(i);
        motor_fracs[i] = settings.getMotorCal(i) / 255.0;   // Get motor calibrations as duty cycles
        
        // Skip unused motors.
        if (pin < 0) {
            continue;
        }

        // Set up the pins and initialize low
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    cDelay = (unsigned long)(1.0/settings.getSampleRate() + 1); // Get sample delay, round up

    return read_all_err;
}

uint8_t outputSettings() {
    /** Output the values of the settings object over the serial port

    @return Returns 0 on no error.
    */

    char strBuff[10];       // For floats

    Serial.print("Declination: ");
    dtostrf(settings.getDeclination(), 3, 2, strBuff);
    strcat(strBuff, "°");
    Serial.println(strBuff);

    Serial.print("Inclination: ");
    dtostrf(settings.getInclination(), 3, 2, strBuff);
    strcat(strBuff, "°");
    Serial.println(strBuff);

    Serial.print("Sample rate: ");
    dtostrf(settings.getSampleRate(), 3, 2, strBuff);
    Serial.print(strBuff);
    Serial.println(" Hz");

    Serial.print("Pulse width: ");
    Serial.print(int(settings.getPulseWidth()));
    Serial.println(" ms");

    Serial.print("Num. Motors: ");
    Serial.println(int(settings.getNMotors()));

    Serial.print("Phase Offset: ");
    dtostrf(settings.getPhaseOffset(), 3, 2, strBuff);
    strcat(strBuff, "°");;
    Serial.println(strBuff);

    Serial.print("Use Calibration: ");
    Serial.println(settings.getUseCalibration()?"True":"False");

    Serial.print("Gain: ");
    Serial.print(int(settings.getGain()));
    Serial.print("; Range: ±");
    dtostrf(compass.gainRanges[settings.getGain()], 1, 2, strBuff);
    Serial.print(strBuff);
    Serial.println(" mG");

    Serial.print("Averages per measurement: ");
    Serial.print(int(settings.getAveraging()));
    Serial.print("(");
    Serial.print(1<<int(settings.getAveraging()));
    Serial.println(" averages)");

    Serial.println("Motor\tPin\tFrac");
    for (int motor = 0; motor < settings.getNMotors(); motor++) {
        Serial.print(motor<10?"    ":"   ");
        Serial.print(motor);
        Serial.print("\t");
        Serial.print(" ");
        Serial.print(settings.getPinLoc(motor));
        Serial.print("\t");
        dtostrf(settings.getMotorCal(motor) / 255.0, 1, 3, strBuff);
        Serial.println(strBuff);
    }


    return 0;
}

uint8_t setPinLoc(uint8_t motor, int8_t pin_loc) {
    /** Set the pin location for a specific motor. 

    @return Returns 0 on no error.
    */

    return settings.setPinLoc(motor, pin_loc);
}

uint8_t setMotorCal(uint8_t motor, uint8_t motor_cal) {
    /** Set the motor calibration as a fraction of 255.

    @return Returns 0 on no error.
    */

    return settings.setMotorCal(motor, motor_cal);
}

uint8_t setMotorCalFloat(uint8_t motor, float motor_cal) {
    /** Set the motor calibration as a floating point duty cycle.

    @return Returns 0 on no error.
    */

    return settings.setMotorCal(motor, motor_cal);
}

uint8_t setDebugMotor(uint8_t motor) {
    /** In debug mode, set which motor is active.

    @return Returns 0 on no error.
    */

    cdmotor = motor;
}

uint8_t setPulseWidth(uint16_t pulse_width) {
    /** Sets the pulse width in milliseconds

    @return Returns 0 on no error.
    */

    return settings.setPulseWidth(pulse_width);
}

uint8_t setNMotors(uint8_t n_motors) {
    /** Sets the number of motors in the cap.

    @return Returns 0 on no error.
    */

    return settings.setNMotors(n_motors);
}

uint8_t setGain(uint8_t gain) {
    /** Sets the gain values.

    @return Returns errors from `HaptiCapMagSettings::setGain()`
    */

    return settings.setGain(gain);
}

uint8_t setAvg(uint8_t avg) {
    /** Sets the averaging rate

    @return Returns errors from `HaptiCapMagSettings::setAveraging()`
    */

    return settings.setAveraging(avg);
}

uint8_t setDeclination(float decl) {
    /** Sets the declination for the current settings object

    @return Returns errors from `HaptiCapMagSettings::setDeclination()`
    */

    return settings.setDeclination(decl);
}

uint8_t setInclination(float incl) {
    /** Sets the inclination for the current settings object

    @return Returns errors from `HaptiCapMagSettings::setInclination()`
    */

    return settings.setInclination(incl);
}

uint8_t setSampleRate(float sr) {
    /** Sets the sample rate for the current settings object

    @return Returns errors from `HaptiCapMagSettings::setSampleRate()`
    */

    return settings.setSampleRate(sr);
}

uint8_t setPhaseOffset(float phi) {
    /** Sets ths phase offset for the current settings object

    @return Returns errors from `HaptiCapMagSettinsg::setPhaseOffset()`
    */

    return settings.setPhaseOffset(phi);
}