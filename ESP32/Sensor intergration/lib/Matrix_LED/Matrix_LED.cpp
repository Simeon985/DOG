#include "Matrix_LED.h"

Matrix_LED::Matrix_LED(void) {

    //initialize the LED displays
    delta_time=0;
    previous_timestamp=0;
    state_led = LedState::S0;
    if (!mx.begin()){
    Serial.println("\nMD_MAX72XX initialization failed");
    }
    Serial.println("\nMD_MAX72XX initialization succeeded");
    Serial.println("Timing flow\n");
    render();

}
void Matrix_LED::update(unsigned long current_time,float distance){
    delta_time = current_time - previous_timestamp;
    if(2<distance && distance<10){
        if(delta_time>1000000){
            previous_timestamp=current_time;
            state_led = static_cast<LedState>((static_cast<uint8_t>(state_led) + 1) % 8);
            render();
        }
    }

}
void Matrix_LED::render(void) {

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