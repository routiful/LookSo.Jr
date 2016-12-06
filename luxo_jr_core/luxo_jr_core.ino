#ifndef LUXO_CORE_INO
#define LUXO_CORE_INO

#include "luxo_jr_core_config.h"

HardwareTimer Timer(TIMER_CH1);
RC100 remote_controller;
WheelDriver LuxoJrWheel;
LuxoJrController LuxoJrJoint;

int luxo_jr_dxl_present_pos[4] = {0, 0, 0, 0};
int luxo_jr_dxl_goal_pos[4] = {0, 0, 0, 0}; //degree

float luxo_jr_dxl_present_rad[4] = {0.0, 0.0, 0.0, 0.0};
float luxo_jr_dxl_goal_rad[4] = {0.0, 0.0, 0.0, 0.0};

float joint_vel[4]= {0.0, 0.0, 0.0, 0.0};

float luxo_jr_linear_x = 0.0, luxo_jr_angular_z = 0.0,  const_cmd_vel = 0.0;
float mov_time = 1.5, ts = 0.008, luxo_jr_acc = 0.0, luxo_jr_max_vel = 0.0;
int mov_cnt = 0, scene = 1;

void setup()
{
  Serial.begin(115200);
  //while(!Serial);

  remote_controller.begin(1);

  LuxoJrWheel.init();
  LuxoJrJoint.init();

  blink(1500, 3);
  timerInit();
}

void loop()
{
  receiveRemoteControl();

#ifdef DEBUG
  // Serial.print("luxo_jr_linear_x : ");
  // Serial.print(luxo_jr_linear_x);
  // Serial.print(" luxo_jr_angular_z : ");
  // Serial.print(luxo_jr_angular_z);
  //
  // for (int id = 1; id < 2; id++)
  // {
  //   LuxoJrJoint.readPosition(id, &luxo_jr_present_pos[id-1]);
  //   Serial.print(" luxo_jr_present_pos : ");
  //   Serial.print(luxo_jr_present_pos[0]);
  //   Serial.print(" ");
  //   Serial.print(luxo_jr_present_pos[1]);
  //   Serial.print(" ");
  //   Serial.print(luxo_jr_present_pos[2]);
  //   Serial.print(" ");
  //   Serial.println(luxo_jr_present_pos[3]);
  // }
#endif
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

  trailor(scene);
}

void trailor(int scene)
{
  switch (scene)
  {
    case 1:
      luxo_motion(1.5, 0, 90, 0, 0, 4.0, 4.0, 500, 0.0, 0.0);
      break;
    case 2:
      luxo_motion(1.5, 0, 90, 0, -90, 3.0, 3.0, 800, 0.0, 0.0);
      break;
    case 3:
      luxo_motion(2.0, 0, 90, 0, 90, 3.0, 3.0, 800, 0.0, 0.0);
      break;
    case 4:
      luxo_motion(1.5, -45, 85, 50, 0, 3.0, 3.0, 800, 0.0, 0.0);
      break;
    case 5:
      luxo_motion(1.5, 50, -50, -30, 0, 3.0, 3.0, 2000, 0.0, 0.0);
      break;
    case 6:
      luxo_motion(1.5, 30, -50, -30, 0, 3.0, 3.0, 1500, 0.0, 0.0);
      break;
    case 7:
      luxo_motion(1.5, 10, -50, -30, 0, 3.0, 3.0, 2000, -0.02, 0.0);
      break;
    case 8:
      luxo_motion(3.0, -45, 85, 50, 0, 3.0, 3.0, 100, 0.0, 0.0);
      break;
    case 9:
      luxo_motion(0.1, -45, 85, 50, 0, 3.0, 3.0, 100, 0.0, -1.0);
      scene++;
      break;
    case 10:
      luxo_motion(0.1, -45, 85, 50, 0, 3.0, 3.0, 100, 0.0, 1.0);
      scene++;
      break;
    case 11:
      luxo_motion(0.0, -45, 85, 50, 0, 3.0, 3.0, 100, 0.0, 0.0);
      break;
    default:
      break;
  }
}

void luxo_motion(float mov, int a, int b, int c, int d, float acc, float vel, int ms, float linear, float angular)
{
  Serial.println(scene);
  luxo_jr_dxl_goal_pos[0] = a;
  luxo_jr_dxl_goal_pos[1] = b;
  luxo_jr_dxl_goal_pos[2] = c;
  luxo_jr_dxl_goal_pos[3] = d;

  luxo_jr_linear_x  = linear;
  luxo_jr_angular_z = angular;

  if (mov > 0.0)
  {
    if (mov_cnt < (mov_time/ts))
    {
      trapezoidalProfile(acc, vel, luxo_jr_dxl_goal_pos);
      mov_cnt++;
    }
    else
    {
      delay(ms);
      scene++;
      mov_cnt = 0;
    }
  }
  else
  {
    delay(ms);
    scene++;
  }
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

void trapezoidalProfile(float acc, float max_vel, int goal_pos[4])
{
  // Serial.println(joint_vel[2]);
  // Serial.print(" ");

  if (mov_cnt == 1)
  {
    for (int id = 1; id < MOTOR_NUM+1; id++)
    {
      LuxoJrJoint.readPosition(id, &luxo_jr_dxl_present_pos[id-1]);
    }
  }

  for (int id = 1; id < MOTOR_NUM+1; id++)
  {
    luxo_jr_dxl_present_rad[id-1] = LuxoJrJoint.convertValue2Radian(luxo_jr_dxl_present_pos[id-1]);
    luxo_jr_dxl_goal_rad[id-1] = goal_pos[id-1]*DEGREE2RADIAN;

    if (luxo_jr_dxl_goal_rad[id-1] > luxo_jr_dxl_present_rad[id-1])
    {
      joint_vel[id-1] = min(joint_vel[id-1] + (acc * ts), max_vel);
      joint_vel[id-1] = min(joint_vel[id-1], sqrt(2*acc*abs(luxo_jr_dxl_goal_rad[id-1] - luxo_jr_dxl_present_rad[id-1])));
      luxo_jr_dxl_present_rad[id-1] = luxo_jr_dxl_present_rad[id-1] + joint_vel[id-1] * ts;
      luxo_jr_dxl_present_pos[id-1] = LuxoJrJoint.convertRadian2Value(luxo_jr_dxl_present_rad[id-1]);

      LuxoJrJoint.positionControl(luxo_jr_dxl_present_pos);
    }
    else if (luxo_jr_dxl_goal_rad[id-1] < luxo_jr_dxl_present_rad[id-1])
    {
      joint_vel[id-1] = max(joint_vel[id-1] - (acc * ts), -max_vel);
      joint_vel[id-1] = max(joint_vel[id-1], -sqrt(2*acc*abs(luxo_jr_dxl_goal_rad[id-1] - luxo_jr_dxl_present_rad[id-1])));
      luxo_jr_dxl_present_rad[id-1] = luxo_jr_dxl_present_rad[id-1] + joint_vel[id-1] * ts;
      luxo_jr_dxl_present_pos[id-1] = LuxoJrJoint.convertRadian2Value(luxo_jr_dxl_present_rad[id-1]);

      LuxoJrJoint.positionControl(luxo_jr_dxl_present_pos);
    }
  }
}

void blink(int ms, int repeat)
{
  for (int num = 0; num < repeat; num++)
  {
    LuxoJrJoint.setLED(true);
    delay(ms);
    LuxoJrJoint.setLED(false);
    delay(ms);
  }
}
#endif // LUXO_CORE_INO
