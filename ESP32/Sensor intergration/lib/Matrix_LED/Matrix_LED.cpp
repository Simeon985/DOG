#include "Matrix_LED.h"

Matrix_LED::Matrix_LED(void) {

    //initialize the LED displays
    delta_time=0;
    previous_timestamp=0;
    delta_time_animation=0;
    previous_timestamp_animation=0;
    state_led = LedState::S0;
    emotional_state = EmotionalState::boos;
    if (!mx.begin()){
    Serial.println("\nMD_MAX72XX initialization failed");
    }
    Serial.println("\nMD_MAX72XX initialization succeeded");
    Serial.println("Timing flow\n");
    render();

}
void Matrix_LED::update(unsigned long current_time,float distance){
    delta_time = current_time - previous_timestamp;
    delta_time_animation = current_time - previous_timestamp_animation;
    if(2<distance && distance<10){
        if(delta_time>1000000){
            previous_timestamp=current_time;
            state_led = static_cast<LedState>((static_cast<uint8_t>(state_led) + 1) % 8);
            emotional_state = static_cast<EmotionalState>((static_cast<uint8_t>(emotional_state) + 1) % 4);       
            render();     
        }
    }
    if (delta_time_animation>100000){
        previous_timestamp_animation=current_time;
        render();
    }
    render();
    
}


void rotate180(const uint8_t in[8], uint8_t out[8]) {
    for (int i = 0; i < 8; i++) {
        uint8_t b = in[7 - i];
        // Reverse the bits in the byte
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

bool random_bool() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dist(0, 1);
    
    return dist(gen) == 1;
}


void Matrix_LED::render(void) {
    uint8_t linkeroog[8];
    uint8_t rechteroog_unrotated[8];
    uint8_t rechteroog[8];
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
            for (int i = 0; i < 8; i++) {
                linkeroog[i] = neutraal[i];
                rechteroog_unrotated[i] = neutraal[i];
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