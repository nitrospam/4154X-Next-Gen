#include "main.h"
#include "autoSelect/selection.h"

void initialize(){
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  liftPot.calibrate();
  if (pros::competition::is_connected()){
    selector::init();
  }
  else {
    pros::lcd::initialize();
  }
}

void disabled() {
  while(true){
    if (pros::competition::is_connected()){
    if(selector::auton == 1)
      controller.print(0, 0, "Middle Goal Selected");
    if(selector::auton == -1)
      controller.print(0, 0, "Middle Goal Selected");
    if(selector::auton == 2)
      controller.print(0, 0, "Right Goal Selected");
    if (selector::auton == -2)
      controller.print(0, 0, "Right Goal Selected");
    if(selector::auton == -3)
      controller.print(0, 0, "Left Goal Selected");
    if(selector::auton == 3)
      controller.print(0, 0, "Left Goal Selected");
    if(selector::auton == 4)
      controller.print(0, 0, "Middle + Right Selected");
    if(selector::auton == -4)
      controller.print(0, 0, "Middle + Right Selected");
    if(selector::auton == 0)
      controller.print(0, 0, "Skills Selected");
    }
  }
}

void autonomous() {
  frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  middleRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  middleLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  backRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  /*if (pros::competition::is_connected()){
  if(selector::auton == 1)
    midGoalAuton();
  if(selector::auton == -1)
    midGoalAuton();
  if(selector::auton == 2)
    rightGoalAuton();
  if (selector::auton == -2)
    rightGoalAuton();
  if(selector::auton == -3)
    leftGoalAuton();
  if(selector::auton == 3)
    leftGoalAuton();
  if(selector::auton == 4)
    midRightAuton();
  if(selector::auton == -4)
    midRightAuton();
  if(selector::auton == 0)
    skills();
  }*/
  testSkills();
}


void opcontrol() {

  frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  middleRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  middleLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  backRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  while(true){

  setDriveMotors();
  setLiftMotor();
  setClawPistons();
  ringToggle();
  

  if (pros::lcd::is_initialized())
    pros::lcd::print(1, "Lift Potentiometer: %i",liftPot.get_value());

  pros::delay(10);

  }

}
