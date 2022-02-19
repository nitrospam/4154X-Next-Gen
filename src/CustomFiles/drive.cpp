#include "main.h"

//Helper Functions
void setDrive(int left, int right) {

  frontLeft = left;
  middleLeft = left;
  backLeft = left;
  frontRight = right;
  middleRight = right;
  backRight = right;

}

//Drive Functions
void setDriveMotors() {

  int leftJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int rightJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  setDrive(leftJoystick,rightJoystick);

}
