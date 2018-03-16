#include "servo.h"

Servo::Servo()
{
  pin_ = 3;
  freq_ = 50;
  res_ = 10;

  angle_range_.max = 180;
  angle_range_.min = -180;

  offset_.for_min_pulse = 0;
  offset_.for_max_pulse = 0;
}

Servo::~Servo(){}

void Servo::begin(uint16_t range, uint32_t freq, uint32_t res)
{
  angle_range_.max = range/2;
  angle_range_.min = (-1) * (range/2);

  freq_ = freq;
  res_  = res;
}

void Servo::attach(uint8_t pin)
{
  pin_  = pin;

  drv_pwm_set_freq(pin_, freq_);
  drv_pwm_setup(pin_);
}

void Servo::offset(uint16_t min_pulse_offset, uint16_t max_pulse_offset)
{
  offset_.for_min_pulse = min_pulse_offset;
  offset_.for_max_pulse = max_pulse_offset;
}

void Servo::write(uint32_t angle)
{
  uint32_t get_angle = angle;
  uint16_t duty = 0;

  uint32_t min_pulse = (uint32_t)(pow((double)(2), (double)(res_))*0.05) + offset_.for_min_pulse;
  uint32_t max_pulse = (uint32_t)(pow((double)(2), (double)(res_))*0.1)  + offset_.for_max_pulse;

  // angle = constrain(angle, angle_range_.min, angle_range_.max);    
  duty = map(angle, angle_range_.min, angle_range_.max, min_pulse, max_pulse);

  drv_pwm_set_duty(pin_, res_, duty);
}