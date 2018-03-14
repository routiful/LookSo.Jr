uint8_t pwm_pin = 3;
uint32_t freq = 50;

void setup()
{
  drv_pwm_set_freq(pwm_pin, freq);
  drv_pwm_setup(pwm_pin);
}

void loop()
{
  uint32_t res = 10;
  uint32_t duty = 0;
  uint32_t deg = 0;

  deg = 0;
  duty = map(deg, 0, 180, 52, 102);

  drv_pwm_set_duty(pwm_pin, res, 80);
  // delay(1000);

  // deg = 90;
  // duty = map(deg, 0, 180, 50, 100);

  // drv_pwm_set_duty(pwm_pin, res, duty);
  // delay(1000);

  // deg = 180;
  // duty = map(deg, 0, 180, 50, 100);

  // drv_pwm_set_duty(pwm_pin, res, duty);
  // delay(1000);
}

// 70
