#ifndef HAPTICAPMAG_H
#define HAPTICAPMAG_H

void setup(void);
void loop(void);

uint8_t select_motor(float phi);
uint8_t change_motor(uint8_t motor);
uint8_t set_motor(uint8_t motor, bool h_low);
float calc_north(float x, float y, float decl);

#endif