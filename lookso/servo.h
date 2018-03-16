#include <Arduino.h>

typedef struct 
{
  int16_t max;
  int16_t min;
} angle_range_t;

typedef struct
{
  int32_t for_min_pulse;
  int32_t for_max_pulse;
} offset_t;

class Servo
{
 private:
  uint8_t  pin_;
  uint32_t freq_;
  uint32_t res_;

  angle_range_t angle_range_;
  offset_t offset_;

 public:
  Servo();
  ~Servo();

  void begin(uint16_t range = 120, uint32_t freq = 50, uint32_t res = 10);
  void offset(uint16_t min_pulse_offset, uint16_t max_pulse_offset);
  void attach(uint8_t pin);
  void write(uint32_t angle);
};