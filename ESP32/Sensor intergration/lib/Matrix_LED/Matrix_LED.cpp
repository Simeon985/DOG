#include "Matrix_LED.h"

Matrix_LED::Matrix_LED(void) {
    delta_time=0;
    previous_timestamp=0;
    delta_time_animation=0;                  // restored
    previous_timestamp_animation=0;          // restored
    state_led = LedState::S0;
    emotional_state = EmotionalState::neutraal;
    if (!mx.begin()){
        Serial.println("\nMD_MAX72XX initialization failed");
    }
    Serial.println("\nMD_MAX72XX initialization succeeded");
    Serial.println("Timing flow\n");
    render();
}

void Matrix_LED::update(unsigned long current_time, float distance){
    delta_time = current_time - previous_timestamp;
    delta_time_animation = current_time - previous_timestamp_animation;  // restored

    if(2<distance && distance<10){
        if(delta_time>1000000){
            previous_timestamp=current_time;
            state_led = static_cast<LedState>((static_cast<uint8_t>(state_led) + 1) % 8);
            emotional_state = static_cast<EmotionalState>((static_cast<uint8_t>(emotional_state) + 1) % 4);
            render();
        }
    }

    if (delta_time_animation>50000){        // restored: throttled animation
        previous_timestamp_animation=current_time;
        render();
    }

    delta_time_blink = current_time - previous_timestamp_blink;
    if(delta_time_blink > 5000000){
        previous_timestamp_blink = current_time;
        blinking = true;
        unblinking = false;                  // restored
        blink_state = 1;                     // restored
    }
}

#include <cstdint>
void rotate180(const uint8_t in[8], uint8_t out[8]) {
    for (int i = 0; i < 8; i++) {
        uint8_t b = in[7 - i];
        b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);
        b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);
        b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);
        out[i] = b;
    }
}
void mirrorVertical(const uint8_t in[8], uint8_t out[8]) {
    for (int i = 0; i < 8; i++) {
        uint8_t b = in[i];
        b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);
        b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);
        b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);
        out[i] = b;
    }
}
void generatePupil(const int vertical_offset, const int horizontal_offset, const bool scared, uint8_t out[8]){
    for (int i = 0; i < 8; i++) {
        out[i] = 0x00;
    }
    int start_row = horizontal_offset + 1; // door de rotatie die hierna gebeurt is dit de horizontal offset
    int bit_shift = 3 - vertical_offset;
    for (int i = 0; i < 4; i++) {
        if (scared){
            out[start_row + i] = scared_pupil[i] << bit_shift;
        } else {
            out[start_row + i] = normal_pupil[i] << bit_shift;
        }
    }
    for (int i = 0; i < 8; i++) {
        out[i] = ~out[i];
    }
}

bool random_bool() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dist(0, 1);
    return dist(gen) == 1;
}

void Matrix_LED::render(void) {
    static uint8_t linkeroog[8];
    static uint8_t rechteroog_unrotated[8];
    static uint8_t rechteroog[8];
    switch (emotional_state) {
        case EmotionalState::boos:
            if (random_bool()){
                for (int i = 0; i < 8; i++) {
                    linkeroog[i] = boos_links[i] & sad_pupil_naar_links[i];
                    rechteroog_unrotated[i] = boos_rechts[i] & sad_pupil_naar_links[i];
                }
            } else {
                for (int i = 0; i < 8; i++) {
                    linkeroog[i] = boos_links[i] & sad_pupil_naar_rechts[i];
                    rechteroog_unrotated[i] = boos_rechts[i] & sad_pupil_naar_rechts[i];
                }
            }
            break;
        case EmotionalState::sad:
            if (random_bool()){
                for (int i = 0; i < 8; i++) {
                    linkeroog[i] = sad_links[i] & sad_pupil_naar_links[i];
                    rechteroog_unrotated[i] = sad_rechts[i] & sad_pupil_naar_links[i];
                }
            } else {
                for (int i = 0; i < 8; i++) {
                    linkeroog[i] = sad_links[i] & sad_pupil_naar_rechts[i];
                    rechteroog_unrotated[i] = sad_rechts[i] & sad_pupil_naar_rechts[i];
                }
            }
            break;
        case EmotionalState::hart:
            if (random_bool()){
                for (int i = 0; i < 8; i++) {
                    linkeroog[i] = hart_naar_links[i];
                    rechteroog_unrotated[i] = hart_naar_links[i];
                }
            } else {
                for (int i = 0; i < 8; i++) {
                    linkeroog[i] = hart_naar_rechts[i];
                    rechteroog_unrotated[i] = hart_naar_rechts[i];
                }
            }
            break;
        case EmotionalState::neutraal:
            static uint8_t pupil_mask[8];
            static uint8_t blink_mask[8];

            if (rand() < RAND_MAX / 10){
                if (random_bool()){
                    horizontal_offset_pupil = max(0,horizontal_offset_pupil-1);
                } else {
                    horizontal_offset_pupil = min(2,horizontal_offset_pupil+1);
                }
            } else if (rand() < RAND_MAX / 30){
                if (random_bool()){
                    vertical_offset_pupil = max(0,vertical_offset_pupil-1);
                } else {
                    vertical_offset_pupil = min(2,vertical_offset_pupil+1);
                }
            }

            generatePupil(vertical_offset_pupil, horizontal_offset_pupil, is_scared, pupil_mask);

            if (blinking){
                if (blink_state == 5){
                    unblinking = true;
                    blinking = false;
                } else {
                    blink_state += 1;
                }
            }
            if (unblinking){
                if (blink_state == 1){
                    unblinking = false;
                } else {
                    blink_state -= 1;
                }
            }
            switch(blink_state){
                case 1: memcpy(blink_mask,blink_1,8); break;
                case 2: memcpy(blink_mask,blink_2,8); break;
                case 3: memcpy(blink_mask,blink_3,8); break;
                case 4: memcpy(blink_mask,blink_4,8); break;
                case 5: memcpy(blink_mask,blink_5,8); break;
            }

            for (int i = 0; i < 8; i++) {
                linkeroog[i] = neutraal[i] & pupil_mask[i] & blink_mask[i];
                rechteroog_unrotated[i] = neutraal[i] & pupil_mask[i] & blink_mask[i];
            }
            break;
    }
    rotate180(rechteroog_unrotated,rechteroog);
    for (u_int8_t row = 0; row < 8; row++) {
        mx.setColumn(row, linkeroog[row]);
        mx.setColumn(row+8, rechteroog[row]);
    }
}

void Matrix_LED::render_old(void) {
    for (u_int8_t row = 0; row < 8; row++) {
        switch (state_led) {
            case LedState::S0:
                mx.setColumn(row, hartoog_klein_links[row]);
                mx.setColumn(row+8, hartoog_klein_rechts[row]);
                break;
            case LedState::S1:
                mx.setColumn(row, sad_klein_links[row]);
                mx.setColumn(row+8, sad_klein_rechts[row]);
                break;
            case LedState::S2:
                mx.setColumn(row, boos_klein_links[row]);
                mx.setColumn(row+8, boos_klein_rechts[row]);
                break;
            case LedState::S3:
                mx.setColumn(row, neutraal_klein_links[row]);
                mx.setColumn(row+8, neutraal_klein_rechts[row]);
                break;
            case LedState::S4:
                mx.setColumn(row, sad_groot_links[row]);
                mx.setColumn(row+8, sad_groot_rechts[row]);
                break;
            case LedState::S5:
                mx.setColumn(row, boos_groot_links[row]);
                mx.setColumn(row+8, boos_groot_rechts[row]);
                break;
            case LedState::S6:
                mx.setColumn(row, neutraal_groot_links[row]);
                mx.setColumn(row+8, neutraal_groot_rechts[row]);
                break;
            case LedState::S7:
                mx.setColumn(row, hartoog_groot_links[row]);
                mx.setColumn(row+8, hartoog_groot_rechts[row]);
                break;
        }
    }
}