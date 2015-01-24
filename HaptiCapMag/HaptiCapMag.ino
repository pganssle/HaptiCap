/** @file
Arduino sketch for HaptiCap haptic compass, using only the HMC5883L digital magnetometer and
the microcontroller. In this version, the device parameters and motor calibrations are hard
coded into the sketch.

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
#include <tgmath>

// Declare global variables
HMC5883L compass;

uint8_t motor_pins[8];
float motor_fracs[8];

float sampling_rate = 15.0;
long fractional_subunit = 20;      // Clock fractional subunit for motor duty cycle
long cDelay = 1000/sampling_rate; // Delay between samples

float frac = 0.5;                 // Base fractional on-time

uint8_t pin_start = 5;            // Pin 1 location
uint8_t pin_interval = 1;         // Pins between 

uint8_t cmotor = 8;               // Current active motor
float declination = -2.8;         // Declination off north. E = negative, W = positive
                                  // Find yours at http://www.ngdc.noaa.gov/geomag-web/#declination

void setup() {
    // Initialize the compass
    compass = HMC5883L();
    compass.initialize();

    // Round cDelay up
    if (1000.0 / sampling_rate > cDelay) {
        cDelay++;
    }

    // Set up the initial averaging rate and sensor gain
    compass.setAveragingRate(HMC_AVG8);     // 8 Averages per measurement
    compass.setGain(HMC_GAIN400);           // +/- 4 G sensor range
    compass.getCalibration();               // Calibrate the compass with the self test

    // Set the measurement mode to Idle (no measurements)
    compass.setMeasurementMode(HMC_MeasurementIdle);

    // Initialize the digital pin outs and store the motor pin locations
    for(int i = 0; i < 8; i++) {
        uint8_t pin = pin_start + pin_interval*i;

        motor_pins[i] = pin;
        motor_fracs[i] = frac;

        // Set up the pins and initialize low
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    /* Replace these with your own calibrations if you want */
    //motor_fracs[0] = 1.0;
    //motor_fracs[1] = 1.0;
    //motor_fracs[2] = 1.0;
    //motor_fracs[3] = 1.0;
    //motor_fracs[4] = 1.0;
    //motor_fracs[5] = 1.0;
    //motor_fracs[6] = 1.0;
    //motor_fracs[7] = 1.0;
}

void loop() {
    // Set the compass to the task of measuring. Must wait at least 6.25 ms.
    compass.setMeasurementMode(HMC_MeasurementSingle);

    // If we're on a motor with a fractional duty cycle, set up a PWM cycle.
    // Try to use a sufficiently short fractional subunit to avoid perceptible
    // pulsation.
    if(cmotor < 8 && motor_fracs[cmotor] < 1.0) {
        unsigned long delay_decimation = cDelay/fractional_subunit;

        float off_delay = cDelay*(1-motor_fracs[cmotor])/delay_decimation;
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
}

uint8_t select_motor(float phi) {
    /** Determine the motor closest to the angle phi

    @param[in] phi The heading angle you want to indicate.

    @return Returns the motor closest to the angle phi.
    */
    
    float phi_sep = 45.0;
    float adjusted_heading = fmod(phi-phi_sep*1.5, 360.0);

    if(adjusted_heading < 0) {
        adjusted_heading = 360+adjusted_heading;
    }

    int motor = (int)(adjusted_heading/phi_sep);

    return (uint8_t) 7-motor;
}

void change_motor(uint8_t motor) {
    /** Turn off the current (`cmotor`) and turn on the new motor (`motor`)

    @param[in] motor The new selected motor
    */
    
    if (cmotor < 8) {
        set_motor(cmotor, LOW);
    }

    if(motor < 8) {
        set_motor(motor, HIGH);
    }

    cmotor = motor; 
}

void set_motor(uint8_t motor, bool h_low) {
    /** Turn motor `motor` to on or off, as specified by `h_low`

    @param[in] motor The motor whose state you want to update.
    @param[in] h_low The new state of the motor
    */

    if (motor > 8) {
        return;
    }

    digitalWrite(motor_pins[motor], h_low);
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

void setup() {
    
}

void loop() {
    
}

    return north + declination;
}