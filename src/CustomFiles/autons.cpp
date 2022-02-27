#include "main.h"
#include "autoSelect/selection.h"
using namespace okapi;

const int platformLiftPosition = 2650;
const int drivingLiftPosition = 1220;
const int holdingLiftPosition = 1470;

std::shared_ptr<OdomChassisController> chassis =
  ChassisControllerBuilder()
    .withMotors({9,8,19},{10,7,20})
    .withDimensions({AbstractMotor::gearset::blue, (60.0 / 36.0)}, {{3.25_in, 13_in}, imev5BlueTPR})
    .withOdometry()
    .buildOdometry();

std::shared_ptr<AsyncMotionProfileController> profileController = 
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      5.0, // Maximum linear velocity of the Chassis in m/s
      3.0, // Maximum linear acceleration of the Chassis in m/s^2
      3.0 // Maximum linear jerk of the Chassis in m/s^3
    })
    .withOutput(chassis)
    .buildMotionProfileController();

std::shared_ptr<AsyncPositionController<double, double>> liftOkapi = 
  AsyncPosControllerBuilder()
    .withMotor(18) // lift motor port 3
    .withSensor(std::make_shared<okapi::Potentiometer>(3)) // Potentiometer: ADI Port 'C'
    .build();


void goForwardNonPID(int distance){

  middleLeft.tare_position();

  while(middleLeft.get_position() < distance){
    frontLeft = 127;
    middleLeft = 127;
    backLeft = 127;
    frontRight = 127;
    middleRight = 127;
    backRight = 127;
  }
  frontLeft.move_velocity(0);
  middleLeft.move_velocity(0);
  backLeft.move_velocity(0);
  frontRight.move_velocity(0);
  middleRight.move_velocity(0);
  backRight.move_velocity(0);
}

void goBackwardNonPID(int distance){

  middleLeft.tare_position();

  while(abs(middleLeft.get_position()) < distance){
    frontLeft = -100;
    middleLeft = -100;
    backLeft = -100;
    frontRight = -100;
    middleRight = -100;
    backRight = -100;
  }
  frontLeft.move_velocity(0);
  middleLeft.move_velocity(0);
  backLeft.move_velocity(0);
  frontRight.move_velocity(0);
  middleRight.move_velocity(0);
  backRight.move_velocity(0);
}

void goForward(int distance){

    double wheelCircumference = 10.2101761242;
    double ticksPerRevolution = 500.0;
    double constant = (wheelCircumference/ticksPerRevolution);

    int target = (distance/constant);

    middleLeft.tare_position();

    float kP = 0.3;
    float kD = 0.1;
    float prevError = 0.0;

  while(fabs(middleLeft.get_position() < target)){

    double error = distance - middleLeft.get_position();
    double derivative = error - prevError;
    prevError = error;

    int power = (error*kP) + (derivative*kD);

    frontLeft = power;
    frontRight = -power;
    backLeft = power;
    backRight = -power;

  }

  frontLeft = 0;
  frontRight = 0;
  backLeft = 0;
  backRight = 0;

}


void goForwardSlow(int distance){

}

void goBackward(int distance){

}

void goBackwardSlow(int distance){

}

void turnRight(int degrees){

  inertial.tare_rotation();

  float kP = 0.03;
  float kD = 0.1;
  float prevError = 0.0;

  while(inertial.get_rotation() < degrees){

    double error = degrees - inertial.get_rotation();
    double derivative = error - prevError;
    prevError = error;

    int power = (error*kP) + (derivative*kD);

    frontLeft = power;
    frontRight = -power;
    middleLeft = power;
    middleRight = -power;
    backLeft = power;
    backRight = -power;

  }

  frontLeft = 0;
  frontRight = 0;
  middleLeft = 0;
  middleRight = 0;
  backLeft = 0;
  backRight = 0;

}

void turnLeft(int degrees){

  inertial.tare_rotation();

  float kP = 0.3;
  float kD = 0.1;
  float prevError = 0.0;

  while(fabs(inertial.get_rotation()) < degrees){

    double error = degrees - inertial.get_rotation();
    double derivative = error - prevError;
    prevError = error;

    int power = (error*kP) + (derivative*kD);

    frontLeft = -power;
    frontRight = power;
    middleLeft = -power;
    middleRight = power;
    backLeft = -power;
    backRight = power;

  }

  frontLeft = 0;
  frontRight = 0;
  middleLeft = 0;
  middleRight = 0;
  backLeft = 0;
  backRight = 0;

}

void clampBackpack(){
  dumpTruckLeft.set_value(true);
  dumpTruckRight.set_value(true);
  pros::delay(300);
}

void dropBackpack(){
  dumpTruckLeft.set_value(false);
  dumpTruckRight.set_value(false);
}

void liftForPlatform(){
  if(liftPot.get_value() < platformLiftPosition){
    while(liftPot.get_value() < platformLiftPosition){
      lift = 127;
    }
  }
  else{
    while(liftPot.get_value() > platformLiftPosition){
      lift = -127;
    }
  }
  lift.move_velocity(0);
}

void liftForDriving(){
  if(liftPot.get_value() < drivingLiftPosition){
    while(liftPot.get_value() < drivingLiftPosition){
      lift = 127;
    }
  }
  else{
    while(liftPot.get_value() > drivingLiftPosition){
      lift = -127;
    }
  }
  lift = 0;
}

void clearEncoders(){
  frontLeft.tare_position();
  middleLeft.tare_position();
  backLeft.tare_position();
  frontRight.tare_position();
  middleRight.tare_position();
  backRight.tare_position();
}

void rightGoalAuton(){
  /*//Pseudocode - write after comment line by line
  chassis->setMaxVelocity(400);
  
  //Dash forward to right neutral mogo
  goForwardNonPID(1800);

  //Clamp onto right neutral mogo
  frontClaw.set_value(true);
  pros::delay(200);

  //Lift up to holding position async
  liftOkapi->setTarget(holdingLiftPosition);

  //Drive backwards to align with red awp line amogo
  goBackwardNonPID(1500);

  //Wait until lift is fully raised to turn
  liftOkapi->waitUntilSettled();
  clearEncoders();

  //Turn left to align backpack with red awp line amogo
  chassis->setMaxVelocity(250);
  chassis->turnAngle(-220_deg);

  //Put lift down to driving height
  liftForDriving();

  //Unclamp front neutral mogo
  frontClaw.set_value(false);

  //Back up into red awp line amogo
  chassis->moveDistance(-2_ft);

  //Clamp backpack onto red awp line amogo
  chassis->setMaxVelocity(400);
  chassis->turnAngle(-207_deg);

  //Drive forward to align with row of rings
  chassis->setMaxVelocity(200);
  chassis->moveDistance(1.05_ft);

  //Turn right to face row of rings
  frontClaw.set_value(true);
  pros::delay(200);

  //Raise lift up to platform height
  chassis->setMaxVelocity(400);
  chassis->moveDistance(-4_ft);
  */
  //Pseudocode - write after comment line by line
  chassis->setMaxVelocity(300);
  
  //Dash forward to right neutral mogo
  goForwardNonPID(1800);

  //Clamp onto right neutral mogo
  frontClaw.set_value(true);
  pros::delay(200);

  //Lift up to holding position async
  liftOkapi->setTarget(holdingLiftPosition);

  //Drive backwards to align with red awp line amogo
  goBackwardNonPID(1280);

  //Wait until lift is fully raised to turn
  liftOkapi->waitUntilSettled();
  clearEncoders();

  pros::delay(500);
  chassis->setMaxVelocity(250);
  chassis->turnAngle(-99_deg);

  chassis->setMaxVelocity(150);
  chassis->moveDistance(-1.4_ft);

  clampBackpack();

  chassis->moveDistance(0.4_ft);

  chassis->turnAngle(-45_deg);

  intake.move_velocity(550);

  chassis->setMaxVelocity(100);

  chassis->moveDistance(1.7_ft);

  intake = 0;

}

void leftGoalAuton(){
  //Pseudocode - write after comment line by line

  goForwardNonPID(2000);

  //Clamp onto right neutral mogo
  frontClaw.set_value(true);
  pros::delay(200);

  //Lift up to holding position async
  liftOkapi->setTarget(holdingLiftPosition);

  //Drive backwards to align with red awp line amogo
  goBackwardNonPID(1700);

  //Wait until lift is fully raised to turn
  liftOkapi->waitUntilSettled();
  clearEncoders();

  //Turn left to align backpack with red platform amogo

  
  //Drive back into red platform amogo


  //Clamp backpack onto red amogo


  //Turn on ring mech


  //Wait 3 seconds to fix alignment of match loads


  //Drive forward at 40% speed into row of match loads


  //Turn off ring mech


  //Lower lift to driving height


}

void midGoalAuton(){
  //Pseudocode - write after comment line by line (Start on right)


  //Drive forward to align center of robot in line with tall neutral mogo


  //Turn left to face tall neutral mogo


  //Drive forward into tall neutral mogo


  //Clamp onto tall neutral mogo


  //Drive backwards into home zone

  
}

void midRightAuton(){
  //Pseudocode - write after comment line by line
  chassis->setMaxVelocity(400);
  
  //Dash forward to right neutral mogo
  goForwardNonPID(1800);

  //Clamp onto right neutral mogo
  frontClaw.set_value(true);
  pros::delay(200);

  //Lift up to holding position async
  liftOkapi->setTarget(holdingLiftPosition);

  //Drive backwards to clear home zone
  goBackwardNonPID(1500);

  //Wait until lift is fully raised to turn and clear encoders for accurate okapi turns
  liftOkapi->waitUntilSettled();
  clearEncoders();

  //Turn left to align backpack with tall neutral mogo
  chassis->setMaxVelocity(250);
  chassis->turnAngle(-220_deg);

  //Put lift down to driving height
  liftForDriving();

  //Unclamp front neutral mogo
  frontClaw.set_value(false);

  //Back up to turn towards tall mogo
  chassis->moveDistance(-2_ft);

  //Turn towards tall mogo
  chassis->setMaxVelocity(400);
  chassis->turnAngle(-207_deg);

  //Drive forward towards tall mogo
  chassis->setMaxVelocity(200);
  chassis->moveDistance(1.05_ft);

  //Clamp onto tall mog
  frontClaw.set_value(true);
  pros::delay(200);

  //Back up to clear home zone
  chassis->setMaxVelocity(400);
  chassis->moveDistance(-4_ft);
}

void skills(){
  //Psuedocode - Write under comments line by line
  chassis->setMaxVelocity(400);
  
  //Align with flat side of blue amogo near platform
  chassis->moveDistance(1.4_ft);

  //Back up into blue amogo
  chassis->setMaxVelocity(250);
  chassis->turnAngle(-99_deg);
  
  //Clamp amogo into backpack
  chassis->setMaxVelocity(150);
  chassis->moveDistance(-1.45_ft);
  chassis->setMaxVelocity(500);

  //Drive forward to align with right neutral mogo
  clampBackpack();
  
  //Turn right to face right neutral mogo
  chassis->moveDistance(1.4_ft);

  //Drive forward into right neutral mogo
  chassis->setMaxVelocity(250);
  chassis->turnAngle(96_deg);

  //Clamp right neutral mogo
  chassis->setMaxVelocity(140);
  chassis->moveDistance(1.82_ft);
  
  //Raise lift with goal
  frontClaw.set_value(true);
  pros::delay(200);

  //Turn to face center of blue platform
  liftForPlatform();

  //Turn on ring mech
  chassis->turnAngle(-32_deg);

  intake.move_velocity(475);

  chassis->setMaxVelocity(550);
  chassis->moveDistance(5.1_ft);

  lift = -127;
  pros::delay(800);
  lift = 0;
  //liftForPlatform();

  frontClaw.set_value(false);
  pros::delay(800);
  
  //Drive at half speed towards platform
  chassis->setMaxVelocity(250);
  chassis->moveDistance(-1.5_ft);

  //Turn off ring mech

  intake = 0;
  liftForDriving();
  clearEncoders();
  chassis->setMaxVelocity(350);
  chassis->turnAngle(-125_deg);
  
  //Unclamp neutral mogo (40)
  chassis->setMaxVelocity(200);
  chassis->moveDistance(1_ft);

  frontClaw.set_value(true);
  
  //Back up to align with line between 2nd and 3rd row of tiles
  chassis->setMaxVelocity(450);
  chassis->moveDistance(4.5_ft);

  frontClaw.set_value(false);

  chassis->moveDistance(-1_ft);

  //Turn left to align with line between 2nd and 3rd row of tiles
  clearEncoders();

  //Turn on ring mech
  chassis->turnAngle(98_deg);
  
  //Drive forward at normal speed to align with far neutral mogo
  chassis->setMaxVelocity(200);

  //Turn off ring mech
  chassis->moveDistance(2_ft);
  
  //Turn left to face far neutral mogo
  frontClaw.set_value(true);

  //Put lift down to driving height
  liftForPlatform();

  //Drive forward into far neutral mogo
  chassis->turnAngle(30_deg);
  
  //Clamp onto far neutral mogo
  chassis->setMaxVelocity(500);
  chassis->moveDistance(5_ft);

  //Lift for platform height
  frontClaw.set_value(false);

  //Turn right to face blue platform
  chassis->moveDistance(-0.5_ft);

  //Drive forward at half speed towards platform


  //Unclamp neutral mogo (80)


  //Back up enough distance to drop the amogo but still go back and make a turn


  //Unclamp the amogo


  //Drive forward enough distance to make a 180 degree turn towards the amogo


  //Put lift down to driving height

  
  //Turn 180 degrees towards the amogo


  //Drive into the amogo


  //Clamp the amogo


  //Lift the amogo to platform height


  //Turn 180 degrees towards the platform


  //Drive at half speed towards the platform


  //Unclamp the amogo (120 + rings)


  //Back up to align with red amogo on awp line


  //Turn to align backpack with red awp line amogo


  //Drive backwards into red amogo


  //Clamp red amogo with backpack

  
  //Drive forward to align with row of rings


  //Turn to face row of rings


  //Turn ring mech on
  
  
  //Drive forward at 75% speed into row of rings until alligned with blue amogo on red platform

  
  //Turn off ring mech


  //Turn to face towards red platform


  //Put lift down to driving height


  //Drive forward until parked (190 + rings)

  
  //Unclamp backpack amogo


  //Two goals with rings, 3 goals on platform, robot on platform
  
}

void testerAuton(){
  /*//Pseudocode - write after comment line by line
  chassis->setMaxVelocity(400);
  
  //Dash forward to right neutral mogo
  goForwardNonPID(1800);

  //Clamp onto right neutral mogo
  frontClaw.set_value(true);
  pros::delay(200);

  //Lift up to holding position async
  liftOkapi->setTarget(holdingLiftPosition);
  liftOkapi->waitUntilSettled();

  clearEncoders();

  chassis->turnAngle(-30_deg);

  liftForPlatform();

  chassis->moveDistance(4.65_ft);

  frontClaw.set_value(false);

  chassis->moveDistance(-2_ft);

  liftForDriving();

  chassis->turnAngle(-60_deg)

  chassis->moveDistance(7_ft);

  chassis->turnAngle(-90_deg);

  chassis->moveDistance(1.5_ft);

  frontClaw.set_value(true);
  pros::delay(200);

  liftOkapi->setTarget(holdingLiftPosition);

  chassis->turnAngle(-120_deg);

  liftForPlatform();

  chassis->moveDistance(4_ft);

  frontClaw.set_value(false);
  
  chassis->moveDistance(-2_ft);
  */
  //Pseudocode - write after comment line by line
  chassis->setMaxVelocity(400);
  
  //Dash forward to right neutral mogo
  goForwardNonPID(1800);

  //Clamp onto right neutral mogo
  frontClaw.set_value(true);
  pros::delay(200);

  //Lift up to holding position async
  liftForPlatform();

  //Drive backwards to align with red awp line amogo
  clearEncoders();

  chassis->turnAngle(-35_deg);

  chassis->moveDistance(5.1_ft);

  lift = -127;
  pros::delay(800);
  lift = 0;
  liftForPlatform();

  frontClaw.set_value(false);
  pros::delay(800);
  
  chassis->moveDistance(-1.7_ft);

  clearEncoders();
  chassis->turnAngle(-132_deg);
  
  chassis->moveDistance(5.1_ft);

  chassis->turnAngle(110_deg);

  chassis->moveDistance(9.1_ft);
}