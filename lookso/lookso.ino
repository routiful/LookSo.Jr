#include "servo.h"

#define JOINT_NUM 5
#define GRIPPER   1

Servo joint[JOINT_NUM];
Servo gripper;

void setup()
{
  joint[0].begin();
  joint[1].begin();
  joint[2].begin();
  joint[3].begin();
  joint[4].begin();

  joint[0].attach(3);
  joint[1].attach(5);
  joint[2].attach(6);
  joint[3].attach(9);
  joint[4].attach(10);

  joint[0].offset(1, 20);
  joint[1].offset(1, 20);
  joint[2].offset(1, 20);
  joint[3].offset(1, 20);
  joint[4].offset(1, 20);

  gripper.begin();
  gripper.attach(11);
  gripper.offset(1, 20);
}

void loop()
{
  joint[0].write(0);
  joint[1].write(0);
  joint[2].write(0);
  joint[3].write(0);
  joint[4].write(0);

  gripper.write(10);

  delay(1000);

  gripper.write(30);

  delay(1000);
}