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
#include <Wire.h>
#include <Vec3.h>
#include <HMC5883L.h>
#include <HaptiCapMagSettings.h>
#include <tgmath>
#include <stdlib.h>

#define SETTINGS_LOC 0x00       /*!< The location of the settings file in EEPROM memory */

#define SERIAL_BAUD_RATE 9600   /*!< The baud rate for the serial interface */
#define STRING_BUFFER 128

/** @defgroup RunningModes Running modes
These are flags for a set of partially non-exclusive running modes.
@{ */
#define HCM_RUN_MODE 0
#define HCM_OUT_MODE 1
/** @} */

// Declare global variables
HMC5883L compass;               /*!< Interface to the HMC5883L magnetometer */
HaptiCapMagSettings settings;   /*!< Object which contains the settings for the run */

float motor_fracs[HC_MAX_NMOTORS];

uint8_t cmotor = 8;               // Current active motor
int8_t nCharsFound = 0;
char serialBuffer[STRING_BUFFER];

uint8_t mode = HCM_RUN_MODE;   /*!< The current running mode - defaults to running the compass. */

void setup() {
    settings = HaptiCapMagSettings(SETTINGS_LOC);       // Create the settings object
    uint8_t read_all_err = settings.readAll();

    // If the checksum is invalid, initialize to the default value and set the mode to debug mode.
    if (read_all_err) {
        settings = HaptiCapMagSettings(SETTINGS_LOC);   // Generate a new object to get the defaults

        mode = HCM_DEBUG_MODE;
    }

    // Initialize the compass
    compass = HMC5883L();
    compass.initialize();

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

    // Start the serial port
    Serial.begin(SERIAL_BAUD_RATE);
}

void loop() {
    if (mode & HCM_RUN_MODE) {
        update_compass();
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
            inputString[nCharsFound++] = inChar;
            inputString[nCharsFound] = '\0';        // Null terminate
        }

        // If the incoming line is a newline or ';', set a flag.
        if(inChar == '\n' || inChar == ';') {
            stringComplete = true;
            nCharsFound = 0;
        }
    }
}

void update_compass() {
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
    Vec3<float> values = compass.readCalibratedValues();

    // The sensor is upside-down, so invert the x and z axes.
    values.x = -values.x;
    values.z = -values.z;           // Not strictly necessary, as this is currently unused.

    // Figure out which motor is facing north.
    float north = calc_north(values.x, values.y, values.z);
    uint8_t motor = select_motor(north);

    // If the motor is different, set the new one buzzing.
    if (motor != cmotor) {
        change_motor(motor);
    }

    if (mode & HCM_OUT_MODE) {
        output_vector_cart_serial(values);
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

    char[10] strBuff;

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
    Serial.print("\n")
}

void output_vector_spherical_serial(Vec3<float> value) {
    /** Output a specified value over the serial port as an ASCII line.
    
    Outputs the r, theta and phi axes in mG and degrees on a single line in the format:
    `##±RRRR.RR, ±TTT.TT, ±PPP.PP`

    @param[in] value The value to output.
    */

    // Convert to r, theta, phi.
    float r, theta phi;
    r = sqrt(pow(value.x, 2) + pow(value.y, 2) + pow(value.z, 2));
    theta = atan2(value.y, value.x) * 180 / M_PI;
    phi = acos(z/r) * 180 / M_PI;


    char[10] strBuff;

    Serial.print("##");       // Starts an output line, just my personal convention

    // x value
    dtostrf(r, 4, 2, strBuff);
    if (r >= 0.0) {
        Serial.print("+");
    }
    Serial.print(strBuff);
    Serial.print(", ");

    // y value
    dtostrf(theta, 3, 2, strBuff);
    if (theta >= 0.0) {
        Serial.print("+");
    }
    Serial.print(strBuff);
    Serial.print(", ");

     // z value
    dtostrf(phi, 3, 2, strBuff);
    if (phi >= 0.0) {
        Serial.print("+");
    }
    Serial.print(strBuff);
    Serial.print("\n")
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
    
    if (cmotor < N_MOTORS) {
        set_motor(cmotor, LOW);
    }

    if(motor < N_MOTORS) {
        set_motor(motor, HIGH);
    }

    cmotor = motor; ,
}

void set_motor(uint8_t motor, bool h_low) {
    /** Turn motor `motor` to on or off, as specified by `h_low`

    @param[in] motor The motor whose state you want to update.
    @param[in] h_low The new state of the motor
    */

    if (motor > N_MOTORS) {
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

    return north + declination;
}