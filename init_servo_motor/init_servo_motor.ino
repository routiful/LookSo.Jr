#define PWM_PIN        3
#define PWM_FREQ       50 //hz
#define PWM_RESOLUTION 10

#define MAX_DEGREE  60
#define MIN_DEGREE -60

int deg = 0;
int duty = 0;

void setup()
{
  Serial.begin(9600);

  drv_pwm_set_freq(PWM_PIN, PWM_FREQ);
  drv_pwm_setup(PWM_PIN);
}

void loop()
{
  if (Serial.available()) 
  {
    String read_string = Serial.readStringUntil('\n');

    deg = read_string.toInt();

    Serial.print("deg: ");
    Serial.println(deg);
  }

  deg = constrain(deg, MIN_DEGREE, MAX_DEGREE);    
  duty = map(deg, MIN_DEGREE, MAX_DEGREE, 52, 122);

  drv_pwm_set_duty(PWM_PIN, PWM_RESOLUTION, duty);
}