#include <stdint.h>

class Ultrasone_sensor{
    public:
        Ultrasone_sensor(uint8_t trigPin,uint8_t echoPin)
         : trig{trigPin}, echo{echoPin}{}
        void read_distance(float &distance);
        bool begin();
        void IRAM_ATTR echoISR();  // called by static wrapper
        void trigger();
        void update();
        float get_distance();      // non-blocking read
        unsigned long lastTrigger = 0;
    private:
        uint8_t trig;
        uint8_t echo;
        float duration;
        float distance;

        volatile unsigned long _echoStart = 0;
        volatile unsigned long _echoEnd   = 0;
        volatile bool          _echoReady = false;
};