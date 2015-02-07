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

#define N_FUNCS 19
#define FNAME_BUFF 32
#define FARGS_BUFF 32
#define STRING_BUFFER FNAME_BUFF+FARGS_BUFF+4

// Declare global variables
HMC5883L compass;               /*!< Interface to the HMC5883L magnetometer */
HaptiCapMagSettings settings;   /*!< Object which contains the settings for the run */

float motor_fracs[HC_MAX_NMOTORS];

uint8_t cmotor = HC_MAX_NMOTORS+1;               // Current active motor
uint8_t cdmotor = HC_MAX_NMOTORS+1;

unsigned long cDelay = 100;
unsigned long debugOscillation = 1000;

boolean stringComplete = false;
uint8_t nCharsFound = 0;
char serialBuffer[STRING_BUFFER];

// Mode flags
/** @defgroup RunningModes Running modes
These are flags for a set of partially non-exclusive running modes.
@{ */
boolean running = true;
boolean debugging = false;
boolean debug_oscillates = false;
boolean verbose = false;
boolean output_mag = false;
boolean output_cart = true;
/** @} */

void setup() {
    settings = HaptiCapMagSettings(SETTINGS_LOC);       // Create the settings object

    // Initialize the compass
    compass = HMC5883L();
    compass.initialize();

    uint8_t read_err = reloadSettings();

    if (read_err) {
        settings = HaptiCapMagSettings(SETTINGS_LOC);
    }

    // Start the serial port
    Serial.begin(SERIAL_BAUD_RATE);
    
    Serial.println("Program initialized.");
}

void loop() {
    if (running) {
        update_compass();
    } else if (debugging) {
        update_debug();
    }

    if (stringComplete) {
        Serial.print("Processing function: ");
        Serial.println(serialBuffer);
        uint8_t ev = processFunction(serialBuffer);

        if (ev) {
            Serial.print("Error, Code ");
            Serial.println(ev);
        }
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

        // If the incoming line is ';', set a flag.
        if(inChar == ';') {
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
        | `SetAllPinLocs(uint8_t pin_start, int8_t pin_increment)` | Set all pin locations as an increasing or decreasing function |
        | `SetAllMotorCals(uint8_t frac)` | Set all motor calibrations to the same thing |
        | `SetAllMotorCalsF(float frac)` | Set all motor calibrations to the same thing |
        | `SetOscillation(uint32_t delay_time)` | Oscillate between the motors with delay delay_time |

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
    // The Arduino Nano cannot hold all these in memory at once, so I'm going to try to compare them one by one.
    int cFunc = -1;
    if (strncmp(func_match, "Toggle", 6) == 0) {
        if (strcmp(func_match, "ToggleOutput") == 0) { cFunc = HCC_FD_TOG_OUTPUT; }
        else if (strcmp(func_match, "ToggleCompass") == 0) { cFunc = HCC_FD_TOG_COMPASS; }
        else if (strcmp(func_match, "ToggleCartesianOutput") == 0) { cFunc = HCC_FD_TOG_CART; }
        else if (strcmp(func_match, "ToggleVerboseMode") == 0) { cFunc = HCC_FD_TOG_VERBOSE; }
        else if (strcmp(func_match, "ToggleDebugMode") == 0) { cFunc = HCC_FD_TOG_DEBUG; }
        else if (strcmp(func_match, "ToggleCalibration") == 0) { cFunc = HCC_FD_TOG_CAL; }
    } else if (strncmp(func_match, "Set", 3) == 0) {
        if (strcmp(func_match, "SetMotor") == 0) { cFunc = HCC_FD_SET_MOTOR; }
        else if (strcmp(func_match, "SetDeclination") == 0) { cFunc = HCC_FD_SET_DECL; }  
        else if (strcmp(func_match, "SetInclination") == 0) { cFunc = HCC_FD_SET_INCL; }  
        else if (strcmp(func_match, "SetPulseWidth") == 0) { cFunc = HCC_FD_SET_PW; }  
        else if (strcmp(func_match, "SetSampleRate") == 0) { cFunc = HCC_FD_SET_SR; }  
        else if (strcmp(func_match, "SetNMotors") == 0) { cFunc = HCC_FD_SET_NMOTORS; }  
        else if (strcmp(func_match, "SetPhaseOffset") == 0) { cFunc = HCC_FD_SET_PHASE_OFF; }  
        else if (strcmp(func_match, "SetGain") == 0) { cFunc = HCC_FD_SET_GAIN; }  
        else if (strcmp(func_match, "SetAvg") == 0) { cFunc = HCC_FD_SET_AVG; }  
        else if (strcmp(func_match, "SetPinLoc") == 0) { cFunc = HCC_FD_SET_PIN_LOC; }  
        else if (strcmp(func_match, "SetMotorCal") == 0) { cFunc = HCC_FD_SET_MCAL; }  
        else if (strcmp(func_match, "SetMotorCalF") == 0) { cFunc = HCC_FD_SET_MCALF; }  
        else if (strcmp(func_match, "SetOscillation") == 0) { cFunc = HCC_FD_SET_D_OSC; }  
        else if (strcmp(func_match, "SetAllPinLocs") == 0) { cFunc = HCC_FD_SET_ALL_PL; }  
        else if (strcmp(func_match, "SetAllMotorCals") == 0) { cFunc = HCC_FD_SET_ALL_MCAL; }  
        else if (strcmp(func_match, "SetAllMotorCalsF") == 0) { cFunc = HCC_FD_SET_ALL_MCALF; }
    } else {
        if (strcmp(func_match, "WriteSettings") == 0) { cFunc = HCC_FD_WRITE_SET; }
        else if (strcmp(func_match, "ReloadSettings") == 0) { cFunc = HCC_FD_RELOAD_SET; }
        else if (strcmp(func_match, "OutputSettings") == 0) { cFunc = HCC_FD_OUTPUT_SET; }
    }

    if (cFunc < 0) {
        return HCC_EC_UNKNOWN_FUNCTION;
    }

    // Parse arguments
    int int_arg;
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
        case HCC_FD_SET_ALL_PL:
        case HCC_FD_SET_MCAL:
            int16_t int1, int2;
            if (sscanf(func_args, "%d,%d", &int1, &int2) < 2) {
                return HCC_EC_INVALID_FARG;
            }

            if (int1 < 0 || int1 > 255) {
                return HCC_EC_INVALID_UINT;
            }

            if (cFunc == HCC_FD_SET_PIN_LOC) {
                return setPinLoc(int1, int2);
            } else if(cFunc == HCC_FD_SET_ALL_PL) {
                return setAllPinLocs(int1, int2);
            }else if (cFunc == HCC_FD_SET_MCAL) {
                if (int2 < 0 || int2 > 255) {
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

            return setMotorCal(motor, dc);


        // Next the cases with one uint argument
        case HCC_FD_SET_MOTOR:
        case HCC_FD_SET_PW:
        case HCC_FD_SET_NMOTORS:
        case HCC_FD_SET_GAIN:
        case HCC_FD_SET_AVG:
        case HCC_FD_SET_ALL_MCAL:
        case HCC_FD_SET_D_OSC:
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
            } else if (cFunc == HCC_FD_SET_ALL_MCAL) {
                return setAllMotorCals(uint8_t(int_arg));
            } else if (cFunc == HCC_FD_SET_D_OSC) {
                return setDebugOscillation(int_arg);
            }

        // Now the cases that take a single float
        case HCC_FD_SET_DECL:
        case HCC_FD_SET_INCL:
        case HCC_FD_SET_SR:
        case HCC_FD_SET_PHASE_OFF:
        case HCC_FD_SET_ALL_MCALF:
            float float_arg;
            if (!sscanf(func_args, "%d", &int_arg)) {
                return HCC_EC_INVALID_FARG;
            }

            float_arg = float(int_arg)/1000;

            if (cFunc == HCC_FD_SET_DECL) {
                return setDeclination(float_arg);
            } else if (cFunc == HCC_FD_SET_INCL) {
                return setInclination(float_arg);
            } else if (cFunc == HCC_FD_SET_SR) {
                return setSampleRate(float_arg);
            } else if (cFunc == HCC_FD_SET_PHASE_OFF) {
                return setPhaseOffset(float_arg);
            } else if (cFunc == HCC_FD_SET_ALL_MCALF) {
                return setAllMotorCals(float_arg);
            }
    }



}

uint8_t update_compass() {
    /** The main loop function run when in compass run mode. */

    // Set the compass to the task of measuring. Must wait at least 6.25 ms.
    compass.setMeasurementMode(HMC_MeasurementSingle);

    run_motor(cmotor, cDelay, settings.getPulseWidth(), motor_fracs[cmotor]);

    // Read the calibrated sensor values from the compass
    uint8_t saturated;
    Vec3<float> values;
    if (settings.getUseCalibration()) {
        values = compass.readCalibratedValues(&saturated);
    } else {
        values = compass.readScaledValues(&saturated);
    }

    // The sensor is upside-down, so invert the x and z axes.
    values.x = -values.x;
    values.z = -values.z;           // Not strictly necessary, as this is currently unused.

    // Figure out which motor is facing north.
    float north = calc_north(values.x, values.y, settings.getDeclination());
    uint8_t motor = select_motor(north);

    // If the motor is different, set the new one buzzing.
    if (motor != cmotor) {
        change_motor(motor, &cmotor);
    }

    if (output_mag) {
        if (output_cart) {
            output_vector_cart_serial(values);
        } else {
            output_vector_spherical_serial(values);
        }
    }

    return 0;
}

void update_debug() {
    /** The main loop when running in debug mode */
    if (debug_oscillates) {
        run_motor(cdmotor, debugOscillation, settings.getPulseWidth(), motor_fracs[cdmotor]);
        change_motor(cdmotor+1 % settings.getNMotors(), &cdmotor);
    } else {
        run_motor(cdmotor, cDelay, settings.getPulseWidth(), motor_fracs[cdmotor]);
    }
}

void run_motor(uint8_t motor, float time_delay, uint16_t pulse_width, float frac) {
    /** Runs the selected motor with a fractional duty cycle.

    */
    if (motor < 8 && frac < 1.0) {
        // Calculate the number of pulseWidth units make up the sampling period.
        unsigned long delay_decimation = time_delay/pulse_width;

        float off_delay = time_delay*(1-frac) / delay_decimation;
        float on_delay = time_delay*frac / delay_decimation;

        for(int i = 0; i < delay_decimation; i++) {
            set_motor(motor, LOW);
            delay(off_delay);

            set_motor(motor, HIGH);
            delay(on_delay);
        }
    } else {
        delay(time_delay);              // Wait the sampling period before reading the values.
    }
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
    
    phi -= 90;                          // Change to angle off of the Y-axis, not angle off of X-axis
    phi += settings.getPhaseOffset();   // Include phase offset.
    phi += phi_sep * 0.5;               // Make the change-over point equidistant between two motors.

    float adjusted_heading = fmod(phi, 360.0);  // Wrap at +/- 360.

    // Ensure that it's between 0 and 360, then invert the logic to select the motor.
    if(adjusted_heading < 0) {
        adjusted_heading = -adjusted_heading;
    } else {
        adjusted_heading = 360-adjusted_heading;
    }

    return (uint8_t)(adjusted_heading/phi_sep);
}

void change_motor(uint8_t motor, uint8_t * old_motor) {
    /** Turn off the current (`cmotor`) and turn on the new motor (`motor`)

    @param[in] motor The new selected motor
    */
    
    if (*old_motor < settings.getNMotors()) {
        set_motor(*old_motor, LOW);
    }

    if(motor < settings.getNMotors()) {
        set_motor(motor, HIGH);
    }

    *old_motor = motor;
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
    output_mag = !output_mag;

    if (verbose) {
        Serial.print("Output mode: ");
        Serial.print(output_mag?"enabled":"disabled");
        Serial.println(".");
    }

    return 0;
}

uint8_t toggleCompass() {
    running = !running;

    if (running && debugging) {
        set_motor(cdmotor, LOW);
        debugging = false;
    }
    

    if (verbose) {
        Serial.print("Running mode: ");
        Serial.print(running?"enabled":"disabled");
        Serial.println(".");
    }

    return 0;
}

uint8_t toggleDebugMode() {
    debugging = !debugging;

    if (debugging && running) {
        set_motor(cmotor, LOW);
        running = false;
    }

    if (verbose) {
        Serial.print("Debugging mode: ");
        Serial.print(debugging?"enabled":"disabled");
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
        compass.getCalibration(true, NULL, 0, 200);   // Make sure the calibration is valid.
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
    output_cart = !output_cart;

    if (verbose) {
        Serial.print("Cartesian output mode ");
        Serial.println(output_cart?"enabled":"disabled");
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

    // Set up the initial averaging rate and sensor gain
    compass.setAveragingRate(settings.getAveraging());     // 8 Averages per measurement
    compass.setGain(settings.getGain());

    if (settings.getUseCalibration()) {
        compass.getCalibration(true, NULL, 0, 200);               // Calibrate the compass with the self test
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

    cDelay = (unsigned long)(1000.0/settings.getSampleRate() + 1); // Get sample delay, round up

    return read_all_err;
}

uint8_t outputSettings() {
    /** Output the values of the settings object over the serial port

    @return Returns 0 on no error.
    */

    char strBuff[10];       // For floats
    Serial.print("Checksum: ");
    Serial.println(int(settings.calculateChecksum()));

    Serial.print("Checksum (EEPROM):");
    Serial.println(int(settings.readChecksum()));

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

    uint8_t rv = settings.setPinLoc(motor, pin_loc);
    if (!rv) {
        pinMode(pin_loc, OUTPUT);
    }
}

uint8_t setMotorCal(uint8_t motor, uint8_t motor_cal) {
    /** Set the motor calibration as a fraction of 255.

    @return Returns 0 on no error.
    */

    uint8_t rv = settings.setMotorCal(motor, motor_cal);
    if (!rv) {
        motor_fracs[motor] = settings.getMotorCal(motor) / 255.0;
    }
    return rv;
}

uint8_t setDebugMotor(uint8_t motor) {
    /** In debug mode, set which motor is active.

    @return Returns 0 on no error.
    */

    // Turn off oscillation if you're setting the motor manually
    debug_oscillates = false;

    change_motor(motor, &cdmotor);

    if (verbose) {
        Serial.print("Setting debugging motor to: ");
        Serial.println(int(motor));
    }
    return 0;
}

uint8_t setDebugOscillation(uint32_t delay_time) {
    /** Oscillate between the different motors in debug mode. */
    debugOscillation = delay_time;

    debug_oscillates = true;
    return 0;
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

uint8_t setAllPinLocs(uint8_t pin_start, int8_t pin_increment) {
    /** Set all the pin locations as a linearly increasing or decreasing function
    */

    uint8_t rv, orv = 0;
    for (int motor = 0; motor < settings.getNMotors(); motor++) {
        if(rv = setPinLoc(motor, pin_start + motor*pin_increment)) {
            orv = rv;
        }
    }

    return orv;
}

uint8_t setAllMotorCals(uint8_t motor_cal) {
    /** Set all the motor fractions to the same value
    */

    uint8_t orv = 0;
    uint8_t rv;
    for (int motor = 0; motor < settings.getNMotors(); motor++) {
        if(rv = setMotorCal(motor, motor_cal)) {
            orv = rv;
        }
    }

    return orv;
}

uint8_t setAllMotorCals(float motor_cal) {
    /** Set all the motor fractions to the same value (float version) */
    uint8_t orv = 0;
    uint8_t rv;
    for (int motor = 0; motor < settings.getNMotors(); motor++) {
        if (rv = setMotorCal(motor, motor_cal)) {
            orv = rv;
        }
    }

    return orv;
}