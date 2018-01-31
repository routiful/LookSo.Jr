
uint8_t pwm_pin = 3;
uint32_t freq = 50;

void setup()
{
  drv_pwm_set_freq(pwm_pin, freq);
  drv_pwm_setup(pwm_pin);
}

void loop()
{
  uint32_t res = 8;
  uint32_t duty = 0;
  uint32_t deg = 0;

  deg = 45;
  duty = map(deg, 0, 180, 0, 50);

  drv_pwm_set_duty(pwm_pin, res, duty);
  delay(1000);

  deg = 135;
  duty = map(deg, 0, 180, 0, 50);

  drv_pwm_set_duty(pwm_pin, res, duty);
  delay(1000);
}