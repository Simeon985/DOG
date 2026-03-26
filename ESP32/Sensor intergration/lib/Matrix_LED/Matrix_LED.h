
#ifndef __MATRIX_LED_H__
#define __MATRIX_LED_H__

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES	2
#define CLK_PIN_LED   25  // or SCK
#define DATA_PIN_LED  32  // or MOSI
#define CS_PIN_LED    26  // or SS


#include <Arduino.h>
#include <stdint.h>
#include <MD_MAX72xx.h>

class Matrix_LED {
public:
Matrix_LED(void);
void render(void);
void update(unsigned long delta_time,float distance);
private:
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN_LED, CLK_PIN_LED, CS_PIN_LED, MAX_DEVICES);

unsigned long delta_time;
unsigned long previous_timestamp;
enum class LedState : uint8_t {S0, S1, S2, S3, S4, S5, S6, S7, S8};
LedState state_led;


const u_int8_t heart[8] = {
  0b00000000,  // Row 0: ........
  0b01100110,  // Row 1: .XX..XX.
  0b11111111,  // Row 2: XXXXXXXX
  0b11111111,  // Row 3: XXXXXXXX
  0b01111110,  // Row 4: .XXXXXX.
  0b00111100,  // Row 5: ..XXXX..
  0b00011000,  // Row 6: ...XX...
  0b00000000   // Row 7: ........
};
const u_int8_t heart_mirror[8] = {
  0b00000000,  // Row 0: ........
  0b01100110,  // Row 1: .XX..XX.
  0b11111111,  // Row 2: XXXXXXXX
  0b11111111,  // Row 3: XXXXXXXX
  0b01111110,  // Row 4: .XXXXXX.
  0b00111100,  // Row 5: ..XXXX..
  0b00011000,  // Row 6: ...XX...
  0b00000000   // Row 7: ........
};



const u_int8_t sad_klein_links[8] = {
  0b00000000,
  0b00000100,
  0b00000110,
  0b00001111,
  0b00011011,
  0b01111111,
  0b00111110,
  0b00011100
};
const u_int8_t sad_klein_rechts[8] = {
  0b00000000,
  0b00100000,
  0b01100000,
  0b11110000,
  0b11011000,
  0b11111110,
  0b01111100,
  0b00111000
};


const u_int8_t sad_groot_links[8] = {
  0b00000100,
  0b00000110,
  0b00001111,
  0b00011111,
  0b00110111,
  0b11111111,
  0b01111110,
  0b00111100
};
const u_int8_t sad_groot_rechts[8] = {
  0b00100000,
  0b01100000,
  0b11110000,
  0b11111000,
  0b11101100,
  0b11111111,
  0b01111110,
  0b00111100
};



const u_int8_t boos_klein_links[8] = {
  0b00000000,
  0b00010000,
  0b00110000,
  0b01111000,
  0b01111100,
  0b01110111,
  0b00111110,
  0b00011100
};
const u_int8_t boos_klein_rechts[8] = {
  0b00000000,
  0b00001000,
  0b00001100,
  0b00011110,
  0b00111110,
  0b11101110,
  0b01111100,
  0b00111000
};


const u_int8_t boos_groot_links[8] = {
  0b00100000,
  0b01100000,
  0b11110000,
  0b11111000,
  0b11101100,
  0b11111111,
  0b01111110,
  0b00111100
};
const u_int8_t boos_groot_rechts[8] = {
  0b00000100,
  0b00000110,
  0b00001111,
  0b00011111,
  0b00110111,
  0b11111111,
  0b01111110,
  0b00111100
};





const u_int8_t neutraal_klein_links[8] = {
  0b00000000,
  0b00011100,
  0b00111110,
  0b01111111,
  0b01111011,
  0b01111111,
  0b00111110,
  0b00011100
};
const u_int8_t neutraal_klein_rechts[8] = {
  0b00000000,
  0b00011100,
  0b00111110,
  0b01111111,
  0b01111011,
  0b01111111,
  0b00111110,
  0b00011100
};


const u_int8_t neutraal_groot_links[8] = {
  0b00111100,
  0b01111110,
  0b11111111,
  0b11111111,
  0b11111011,
  0b11110111,
  0b01111110,
  0b00111100
};
const u_int8_t neutraal_groot_rechts[8] = {
  0b00111100,
  0b01111110,
  0b11111111,
  0b11111111,
  0b11111011,
  0b11110111,
  0b01111110,
  0b00111100
};



const u_int8_t hartoog_klein_links[8] = {
  0b00000000,
  0b00011100,
  0b00111110,
  0b01101011,
  0b01000001,
  0b01100011,
  0b00110110,
  0b00011100
};
const u_int8_t hartoog_klein_rechts[8] = {
  0b00000000,
  0b00111000,
  0b01111100,
  0b11010110,
  0b10000010,
  0b11000110,
  0b01101100,
  0b00111000
};


const u_int8_t hartoog_groot_links[8] = {
  0b00111100,
  0b01111110,
  0b11101011,
  0b11000001,
  0b11100011,
  0b11110111,
  0b01111110,
  0b00111100
};
const u_int8_t hartoog_groot_rechts[8] = {
  0b00111100,
  0b01111110,
  0b11010111,
  0b10000011,
  0b11000111,
  0b11101111,
  0b01111110,
  0b00111100
};






};















#endif