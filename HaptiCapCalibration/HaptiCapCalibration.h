#ifndef HAPTICAPCALIBRATION_H
#define HAPTICAPCALIBRATION_H

#include <Vec3.h>

#define N_FUNCS 21
#define HCC_FD_TOG_OUTPUT 0
#define HCC_FD_TOG_COMPASS 1
#define HCC_FD_TOG_DEBUG 2
#define HCC_FD_SET_MOTOR 3
#define HCC_FD_SET_DECL 4
#define HCC_FD_SET_INCL 5
#define HCC_FD_SET_PW 6
#define HCC_FD_SET_SR 7
#define HCC_FD_SET_NMOTORS 8
#define HCC_FD_SET_PHASE_OFF 9
#define HCC_FD_SET_GAIN 10
#define HCC_FD_SET_AVG 11
#define HCC_FD_TOG_CAL 12
#define HCC_FD_SET_PIN_LOC 13
#define HCC_FD_SET_MCAL 14
#define HCC_FD_SET_MCALF 15
#define HCC_FD_WRITE_SET 16
#define HCC_FD_RELOAD_SET 17
#define HCC_FD_OUTPUT_SET 18
#define HCC_FD_TOG_CART 19
#define HCC_FD_TOG_VERBOSE 20

#define HCC_EC_MALFORMED_FUNC 64
#define HCC_EC_MALFORMED_ARGS 65
#define HCC_EC_UNKNOWN_FUNCTION 66
#define HCC_EC_INVALID_FARG 67
#define HCC_EC_INVALID_UINT 68

void setup(void);
void loop(void);

void serialEvent(void);

uint8_t processFunction(char * funcString);

uint8_t update_compass(void);

void output_vector_cart_serial(Vec3<float> value);
void output_vector_spherical_serial(Vec3<float> value);

uint8_t select_motor(float phi);
void change_motor(uint8_t motor);
void set_motor(uint8_t motor, bool h_low);
float calc_north(float x, float y, float decl);

uint8_t toggleCompass();
uint8_t toggleDebugMode();
uint8_t toggleCalibration();
uint8_t toggleVerbose();
uint8_t writeSettings();
uint8_t reloadSettings();
uint8_t outputSettings();
uint8_t toggleCartesianOutput();
uint8_t toggleVerbose();

uint8_t setPinLoc(uint8_t motor, int8_t pin);
uint8_t setMotorCal(uint8_t motor, uint8_t frac);
uint8_t setMotorCalFloat(uint8_t motor, float frac);

uint8_t setDebugMotor(uint8_t motor);
uint8_t setPulseWidth(uint16_t pulse_width);
uint8_t setNMotors(uint8_t nMotors);
uint8_t setGain(uint8_t gain);
uint8_t setAvg(uint8_t avg);

uint8_t setDeclination(float decl);
uint8_t setInclination(float incl);
uint8_t setSampleRate(float sr);
uint8_t setPhaseOffset(float phi);

#endif