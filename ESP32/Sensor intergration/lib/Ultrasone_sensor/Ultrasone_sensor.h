#include <stdint.h>

class Ultrasone_sensor{
    public:
        Ultrasone_sensor(uint8_t trigPin,uint8_t echoPin)
         : trig{trigPin}, echo{echoPin}{}
        void read_distance(float &distance);
        bool begin();
    private:
        uint8_t trig;
        uint8_t echo;
        float duration;
};