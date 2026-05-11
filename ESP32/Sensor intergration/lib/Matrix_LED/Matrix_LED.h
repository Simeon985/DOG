
#ifndef __MATRIX_LED_H__
#define __MATRIX_LED_H__
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 2
#define CLK_PIN_LED   25  // or SCK
#define DATA_PIN_LED  33  // or MOSI
#define CS_PIN_LED    26  // or SS

#include <Arduino.h>
#include <stdint.h>
#include <MD_MAX72xx.h>
#include <random>
class Matrix_LED {
public:
Matrix_LED(void);
void render(void);
void render_old(void);
void update(unsigned long delta_time,float distance);
private:
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN_LED, CLK_PIN_LED, CS_PIN_LED, MAX_DEVICES);
unsigned long delta_time;
unsigned long previous_timestamp;
unsigned long delta_time_animation;
unsigned long previous_timestamp_animation;
unsigned long delta_time_blink;
unsigned long previous_timestamp_blink;
enum class LedState : uint8_t {S0, S1, S2, S3, S4, S5, S6, S7, S8};
enum class EmotionalState : uint8_t {boos, sad, hart, neutraal};
LedState state_led;
EmotionalState emotional_state;
int blink_state;
bool blinking;
bool unblinking;
bool is_scared = false;
int vertical_offset_pupil = 0;
int horizontal_offset_pupil = 0;
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
0b00111000,
0b01111100,
0b11111110,
0b11011000,
0b11110000,
0b01100000,
0b00100000,
0b00000000,
};
const u_int8_t sad_klein_rechts[8] = {
0b00011100,
0b00111110,
0b01111111,
0b00011011,
0b00001111,
0b00000110,
0b00000100,
0b00000000,
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
0b00111000,
0b01111100,
0b11111110,
0b11011110,
0b11111110,
0b01111100,
0b00111000,
0b00000000,
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
0b00111000,
0b01101100,
0b11000110,
0b10001110,
0b11000110,
0b01101100,
0b00111000,
};
const u_int8_t hartoog_klein_rechts[8] = {
0b00000000,
0b00011100,
0b00110110,
0b01100011,
0b01110001,
0b01100011,
0b00110110,
0b00011100,
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

















// nieuwe oogjes, nu overzichtelijker hopelijk:
// bij hartexpressie animatie die snel wisselt tussen hart_naar_links en hart_naar_rechts. dus ofwel linkeroog & rechteroog beide hart_naar_links, ofwel beide hart_naar_rechts
const u_int8_t hart_naar_links[8] = {
0b00000000,
0b00001100,
0b00011110,
0b00111110,
0b01111100,
0b00111110,
0b00011110,
0b00001100,
};
const u_int8_t hart_naar_rechts[8] = {
0b00001100,
0b00011110,
0b00111110,
0b01111100,
0b00111110,
0b00011110,
0b00001100,
0b00000000,
};



// bij boos expressie & sad expressie zijn er aparte pupillen. linkeroog is altijd _links en rechteroog _rechts, en dan pupillen ofwel beide _naar_links ofwel beide _naar_rechts
// bij emotionele states wil ik vlugge willekeurige bewegingen heen en weer as opposed to het rustig rondbewegen van de pupil bij neutraal => mss links vs rechts gwn op moment bepalen met random()
const u_int8_t boos_links[8] = {
0b00111100,
0b01111110,
0b11111111,
0b11111100,
0b11111000,
0b11110000,
0b01100000,
0b00100000,
};
const u_int8_t boos_rechts[8] = {
0b00100000,
0b01100000,
0b11110000,
0b11111000,
0b11111100,
0b11111111,
0b01111110,
0b00111100,
};
const u_int8_t sad_links[8] = {
0b00100000,
0b01100000,
0b11110000,
0b11111000,
0b11111100,
0b11111111,
0b01111110,
0b00111100,
};
const u_int8_t sad_rechts[8] = {
0b00111100,
0b01111110,
0b11111111,
0b11111100,
0b11111000,
0b11110000,
0b01100000,
0b00100000,
};
const u_int8_t sad_pupil_naar_rechts[8] = {
0b11111111,
0b11111111,
0b11111111,
0b11111111,
0b10111111,
0b11111111,
0b11111111,
0b11111111,
};
const u_int8_t sad_pupil_naar_links[8] = {
0b11111111,
0b11111111,
0b11111111,
0b10111111,
0b11111111,
0b11111111,
0b11111111,
0b11111111,
};
const u_int8_t boos_pupil_naar_rechts[8] = {
0b11111111,
0b11111111,
0b11111111,
0b11111111,
0b11101111,
0b11111111,
0b11111111,
0b11111111,
};
const u_int8_t boos_pupil_naar_links[8] = {
0b11111111,
0b11111111,
0b11111111,
0b11101111,
0b11111111,
0b11111111,
0b11111111,
0b11111111,
};
const u_int8_t neutraal[8] = {
  0b00111100,
  0b01111110,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b01111110,
  0b00111100
};

const u_int8_t blink_1[8] = {
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
};
const u_int8_t blink_2[8] = {
0b11111110,
0b01111111,
0b01111111,
0b01111111,
0b01111111,
0b01111111,
0b01111111,
0b11111110,
};
const u_int8_t blink_3[8] = {
0b01111100,
0b00111110,
0b00111110,
0b00111110,
0b00111110,
0b00111110,
0b00111110,
0b01111100,
};
const u_int8_t blink_4[8] = {
0b00111000,
0b00011100,
0b00011100,
0b00011100,
0b00011100,
0b00011100,
0b00011100,
0b00111000,
};
const u_int8_t blink_5[8] = {
0b00010000,
0b00001000,
0b00001000,
0b00001000,
0b00001000,
0b00001000,
0b00001000,
0b00010000,
};




const uint8_t normal_pupil[4] = {
    0b0000,
    0b0110,
    0b0110,
    0b0000,
};
const uint8_t scared_pupil[4] = {
    0b0110,
    0b1111,
    0b1111,
    0b0110,
};








#endif
