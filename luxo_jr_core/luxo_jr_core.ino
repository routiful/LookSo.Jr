#ifndef LUXO_CORE_INO
#define LUXO_CORE_INO

#include "luxo_jr_core_config.h"

HardwareTimer Timer(TIMER_CH1);
RC100 remote_controller;
WheelDriver LuxoJrWheel;
LuxoJrController LuxoJrJoint;

int luxo_jr_present_pos[4] = {0, 0, 0, 0};
int luxo_jr_goal_pos[4] = {400, 700, 730, 512};
float luxo_jr_linear_x = 0.0, luxo_jr_angular_z = 0.0,  const_cmd_vel = 0.0;

void setup()
{
  Serial.begin(115200);
  remote_controller.begin(1);

  LuxoJrWheel.init();
  LuxoJrJoint.init();
  LuxoJrJoint.positionControl(luxo_jr_goal_pos);
  timerInit();
}

void loop()
{
  receiveRemoteControl();

  Serial.print("luxo_jr_linear_x : ");
  Serial.print(luxo_jr_linear_x);
  Serial.print(" luxo_jr_angular_z : ");
  Serial.print(luxo_jr_angular_z);

  for (int id = 1; id < 5; id++)
  {
    LuxoJrJoint.readPosition(id, &luxo_jr_present_pos[id-1]);
    Serial.print(" luxo_jr_present_pos : ");
    Serial.print(luxo_jr_present_pos[0]);
    Serial.print(" ");
    Serial.print(luxo_jr_present_pos[1]);
    Serial.print(" ");
    Serial.print(luxo_jr_present_pos[2]);
    Serial.print(" ");
    Serial.println(luxo_jr_present_pos[3]);
  }
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
  controlMotorSpeed();
  //trapezoidalProfile();
}

void receiveRemoteControl(void)
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

void controlMotorSpeed(void)
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

void trapezoidalProfile(void)
{
  float vel = 0.0;
  float acc = 12.0;
  float maxVel = 12.0;

  for (int id = 1; id < 5; id++)
  {
    LuxoJrJoint.readPosition(id, &luxo_jr_present_pos[id-1]);

    if (luxo_jr_goal_pos[id-1] > luxo_jr_present_pos[id-1])
    {
      vel = min(acc * 0.008, maxVel);
      vel = min(vel, sqrt(2*acc*abs(luxo_jr_goal_pos[id-1] - luxo_jr_present_pos[id-1])));
      luxo_jr_present_pos[id-1] = vel * 0.008;
      LuxoJrJoint.positionControl(luxo_jr_present_pos);
    }
    else if (luxo_jr_goal_pos[id-1] < luxo_jr_present_pos[id-1])
    {
      vel = max(-acc * 0.008, -maxVel);
      vel = max(vel, -sqrt(2*acc*abs(luxo_jr_goal_pos[id-1] - luxo_jr_present_pos[id-1])));
      luxo_jr_present_pos[id-1] = vel * 0.008;
      LuxoJrJoint.positionControl(luxo_jr_present_pos);
    }
  }
}

#endif // LUXO_CORE_INO
