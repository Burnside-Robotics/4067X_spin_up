/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Alex Cutforth                                             */
/*    Created:      Wednesday 17 August 2022                                  */
/*    Description:  Program for team 4067X's robot for the spin up season     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

competition Competition;
controller Controller;

motor left_drive_front = motor(PORT1, ratio18_1, true);
motor left_drive_back = motor(PORT11, ratio18_1);

motor right_drive_front = motor(PORT10, ratio18_1);
motor right_drive_back = motor(PORT20, ratio18_1, true);

motor_group left_drive = motor_group(left_drive_front, left_drive_back);
motor_group right_drive = motor_group(right_drive_front, right_drive_back);

const float ACCELERATION_LIMIT = 20;

float current_left_speed = 0.0;
float current_right_speed = 0.0;

int sign(float x) {
  if(x > 0) {
    return 1;
  } else if(x < 0) {
    return -1;
  } else {
    return 0;
  }
}

void drive_tank(int left_target, int right_target) {
  current_left_speed += ACCELERATION_LIMIT * sign(left_target - current_left_speed);
  current_right_speed += ACCELERATION_LIMIT * sign(right_target - current_right_speed);

  left_drive.spin(fwd, current_left_speed, pct);
  right_drive.spin(fwd, current_right_speed, pct);
}
 
void pre_auton(void) {
  vexcodeInit();
}

void autonomous(void) {
  
}

void user_control(void) {
  while (true) {
    drive_tank(Controller.Axis3.position(), Controller.Axis2.position());
    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(user_control);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
