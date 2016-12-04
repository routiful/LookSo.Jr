#ifndef LUXO_CORE_INO
#define LUXO_CORE_INO

#include "luxo_jr_core_config.h"

HardwareTimer Timer(TIMER_CH1);
RC100 remote_controller;
WheelDriver LuxoJrWheel;
LuxoJrController LuxoJrJoint;

int64_t luxo_jr_pos[4] = {330, 800, 700, 512};
float luxo_jr_linear_x = 0.0, luxo_jr_angular_z = 0.0,  const_cmd_vel = 0.0;

void setup()
{
  Serial.begin(115200);
  remote_controller.begin(1);

  LuxoJrWheel.init();
  LuxoJrJoint.init();
  LuxoJrJoint.positionControl(luxo_jr_pos);
  timerInit();
}

void loop()
{
  receive_remote_control();

  Serial.print("luxo_jr_linear_x : ");
  Serial.print(luxo_jr_linear_x);
  Serial.print(" luxo_jr_angular_z : ");
  Serial.println(luxo_jr_angular_z);
}

void timerInit()
{
  Timer.pause();
  Timer.setPeriod(CONTROL_PEROID);           // in microseconds
  Timer.attachInterrupt(handler_control);
  Timer.refresh();
  Timer.resume();
}

void handler_control(void)
{
  control_motor_speed();
}

void receive_remote_control(void)
{
  int received_data = 0;

  if (remote_controller.available())
  {
    received_data = remote_controller.readData();

    if(received_data & RC100_BTN_U)
    {
      luxo_jr_linear_x  += VELOCITY_LINEAR_X;
    }
    else if(received_data & RC100_BTN_D)
    {
      luxo_jr_linear_x  -= VELOCITY_LINEAR_X;
    }
    else if(received_data & RC100_BTN_L)
    {
      luxo_jr_angular_z += VELOCITY_ANGULAR_Z;
    }
    else if(received_data & RC100_BTN_R)
    {
      luxo_jr_angular_z -= VELOCITY_ANGULAR_Z;
    }
    else if(received_data & RC100_BTN_1)
    {
      const_cmd_vel += VELOCITY_STEP;
    }
    else if(received_data & RC100_BTN_3)
    {
      const_cmd_vel -= VELOCITY_STEP;
    }
    else if(received_data & RC100_BTN_6)
    {
      luxo_jr_linear_x  = const_cmd_vel;
      luxo_jr_angular_z = 0.0;
    }
    else if(received_data & RC100_BTN_5)
    {
      luxo_jr_linear_x  = 0.0;
      luxo_jr_angular_z = 0.0;
    }
  }
}

void control_motor_speed(void)
{
  double wheel_speed_cmd[2];
  double lin_vel1;
  double lin_vel2;

  wheel_speed_cmd[LEFT]  = luxo_jr_linear_x - (luxo_jr_angular_z * WHEEL_SEPARATION / 2);
  wheel_speed_cmd[RIGHT] = luxo_jr_linear_x + (luxo_jr_angular_z * WHEEL_SEPARATION / 2);

  lin_vel1 = wheel_speed_cmd[LEFT] * VELOCITY_CONSTANT_VAULE;

  if (lin_vel1 > LIMIT_XM_MAX_VELOCITY)       lin_vel1 =  LIMIT_XM_MAX_VELOCITY;
  else if (lin_vel1 < -LIMIT_XM_MAX_VELOCITY) lin_vel1 = -LIMIT_XM_MAX_VELOCITY;

  lin_vel2 = wheel_speed_cmd[RIGHT] * VELOCITY_CONSTANT_VAULE;

  if (lin_vel2 > LIMIT_XM_MAX_VELOCITY)       lin_vel2 =  LIMIT_XM_MAX_VELOCITY;
  else if (lin_vel2 < -LIMIT_XM_MAX_VELOCITY) lin_vel2 = -LIMIT_XM_MAX_VELOCITY;

  LuxoJrWheel.speedControl((int64_t)lin_vel1, (int64_t)lin_vel2);
}

#endif // LUXO_CORE_INO
