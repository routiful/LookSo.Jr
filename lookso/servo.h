#include <Arduino.h>

#define RESOLUTION  10

class Servo
{
 public:
  Servo();
  ~Servo();

  uint32_t pwm_pin_;

  void attach(uint32_t pwm_pin, uint32_t freq = 50);
  void write(uint32_t duty);
};