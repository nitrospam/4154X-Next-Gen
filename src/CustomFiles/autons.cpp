#include "main.h"
#include "autoSelect/selection.h"
using namespace okapi;

const int platformLiftPosition = 2600;
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
      20.0, // Maximum linear velocity of the Chassis in m/s
      8.0, // Maximum linear acceleration of the Chassis in m/s^2
      14.0 // Maximum linear jerk of the Chassis in m/s^3
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

void liftForHolding(){
  if(liftPot.get_value() < holdingLiftPosition){
    while(liftPot.get_value() < holdingLiftPosition){
      lift = 127;
    }
  }
  else{
    while(liftPot.get_value() > holdingLiftPosition){
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
  //Pseudocode - write after comment line by line
  chassis->setMaxVelocity(400);
  
  //Dash forward to right neutral mogo
  goForwardNonPID(1820);

  //Clamp onto right neutral mogo
  frontClaw.set_value(true);
  pros::delay(200);

  //Lift up to holding position async
  liftOkapi->setTarget(holdingLiftPosition);

  //Drive backwards to clear home zone
  goBackwardNonPID(1400);

  //Wait until lift is fully raised to turn and clear encoders for accurate okapi turns
  liftOkapi->waitUntilSettled();
  clearEncoders();

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
  goForwardNonPID(1820);

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
  chassis->turnAngle(-208_deg);

  //Drive forward towards tall mogo
  chassis->setMaxVelocity(450);
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {1.4_ft, 0_ft, 0_deg}}, "midGoalGrab");
  //Have the robot follow the path above in reverse
  profileController->setTarget("midGoalGrab");
  profileController->waitUntilSettled();

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

  frontClaw.set_value(false);
  pros::delay(800);
  
  chassis->moveDistance(-1.7_ft);

  clearEncoders();
  chassis->turnAngle(-132_deg);
  
  chassis->moveDistance(5.1_ft);

  chassis->turnAngle(110_deg);

  chassis->moveDistance(9.1_ft);*/

  chassis->setMaxVelocity(450);

  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {2.1_ft, 0_ft, 0_deg}}, "blue_right_amogo_backpack");
  profileController->setTarget("blue_right_amogo_backpack", true);
  profileController->waitUntilSettled();

  clampBackpack();

  clearEncoders();

  chassis->moveDistance(1.55_ft);

  chassis->turnAngle(136_deg);

  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {4.8_ft, 0.4_ft, 0_deg}}, "right_neutral_mogo_clamp");
  profileController->setTarget("right_neutral_mogo_clamp");
  profileController->waitUntilSettled();

}

void rightHalfWinPoint(){
  
  //Pseudocode - write after comment line by line
  chassis->setMaxVelocity(400);
  
  //Dash forward to right neutral mogo
  goForwardNonPID(1820);

  //Clamp onto right neutral mogo
  frontClaw.set_value(true);
  pros::delay(200);

  //Lift up to holding position async
  liftOkapi->setTarget(holdingLiftPosition);

  //Drive backwards to clear home zone
  goBackwardNonPID(1250);

  //Wait until lift is fully raised to turn and clear encoders for accurate okapi turns
  liftOkapi->waitUntilSettled();
  clearEncoders();

  pros::delay(500);

  chassis->setMaxVelocity(225);
  chassis->turnAngle(-82_deg);
  chassis->setMaxVelocity(600);

  pros::delay(200);

  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {1.08_ft, 0_ft, 0_deg}}, "rightAmogoGrab");
  //Set robot to follow the above path
  profileController->setTarget("rightAmogoGrab", true);
  profileController->waitUntilSettled();

  clampBackpack();

  liftForPlatform();

  chassis->setMaxVelocity(325);
  chassis->turnAngle(-65_deg);
  chassis->setMaxVelocity(600);

  intake.move_velocity(525);

  pros::delay(500);

  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {2.5_ft, 0_ft, 0_deg}}, "rightMatchLoads");
  //Have the robot follow the path above
  profileController->setTarget("rightMatchLoads");
  profileController->waitUntilSettled();

  pros::delay(400);

  dropBackpack();

  intake = 0;
}

void leftHalfWinPoint(){
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

  pros::delay(500);

  chassis->setMaxVelocity(325);
  chassis->turnAngle(-60_deg);
  chassis->setMaxVelocity(600);

  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {1.1_ft, 0_ft, 0_deg}}, "leftAmogoBack");
  //Have the robot follow the path above in reverse
  profileController->setTarget("leftAmogoBack", true);
  profileController->waitUntilSettled();

  clampBackpack();

  chassis->setMaxVelocity(325);
  chassis->turnAngle(-60_deg);
  chassis->setMaxVelocity(600);

  pros::delay(200);
  
  intake.move_velocity(500);

  chassis->setMaxVelocity(425);

  liftForPlatform();

  //Generate a path for the robot to drive towards the alliance station wall while picking up match loads
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {2.5_ft, 0_ft, 0_deg}}, "leftMatchLoads");
  //Have the robot follow the path above
  profileController->setTarget("leftMatchLoads");
  profileController->waitUntilSettled();

  pros::delay(400);

  profileController->setTarget("leftMatchLoads", true);
  profileController->waitUntilSettled();

  profileController->setTarget("leftMatchLoads");
  profileController->waitUntilSettled();

  pros::delay(400);

  dropBackpack();

  intake = 0;

}

void fullWinPoint(){
  
  //Set the limit for chassis linear velocity
  chassis->setMaxVelocity(600);

  //Generate path to have robot be perpidicularly aligned with the right side alliance mobile goal
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {1.65_ft, 0_ft, 0_deg}}, "rightAmogoAlign");
  //Set robot to follow the above path
  profileController->setTarget("rightAmogoAlign");
  profileController->waitUntilSettled();

  //Set the limit for chassis turn velocity 
  chassis->setMaxVelocity(325);

  //Turn the chassis left to align the backpack with the right side alliance mobile goal
  chassis->turnAngle(-98.5_deg);

  //Set the limit for chassis linear velocity (Special Case: Backpacking mobile goal)
  chassis->setMaxVelocity(450);

  //Generate path to have the robot back into the right side alliance mobile goal
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {1.4_ft, 0_ft, 0_deg}}, "rightAmogoSlam");
  //Have the robot follow the path in reverse
  profileController->setTarget("rightAmogoSlam", true);
  profileController->waitUntilSettled();

  //Activate the backpack pistons to backpack the right side alliance mobile goal
  clampBackpack();

  //Asynchronously lift to allow the ring mech to run without obstruction
  liftOkapi->setTarget(holdingLiftPosition);

  //Turn slightly to the left to straighten the robot
  chassis->turnAngle(-2_deg);

  //Set the limit for chassis linear velocity
  chassis->setMaxVelocity(600);

  //Start the ring intake
  intake.move_velocity(525);

  //Generate a path to cross to the left side of the field
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {11_ft, 0_ft, 0_deg}}, "leftAmogoDash");
  //Have the robot follow the path above
  profileController->setTarget("leftAmogoDash");
  profileController->waitUntilSettled();

  //Pause for half a second
  pros::delay(100);

  //Kill the intake
  intake = 0;

  //Deactivate the backpack pistons
  dropBackpack();

  //Generate a path to diagonally align with the left alliance mobile goal
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {2_ft, 0_ft, 0_deg}}, "leftAmogoSmallDash");
  //Have the robot follow the path above
  profileController->setTarget("leftAmogoSmallDash");
  profileController->waitUntilSettled();

  //Set the limit for chassis turn velocity
  chassis->setMaxVelocity(325);

  //Turn the robot to the right to align the backpack with the left alliance mobile goal
  chassis->turnAngle(48_deg);

  //Set the limit for chassis linear velocity (Special case: Alliance mogo backpack)
  chassis->setMaxVelocity(450);

  //Generate a path for the robot to back into the left alliance mobile goal
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {2.42_ft, 0_ft, 0_deg}}, "leftAmogoBack");
  //Have the robot follow the path above in reverse
  profileController->setTarget("leftAmogoBack", true);
  profileController->waitUntilSettled();

  //Activate the backpack pistons
  clampBackpack();

  //Pause for a quarter of a second
  pros::delay(250);

  //Follow the previously generated path forward
  /*profileController->setTarget("leftAmogoBack");
  profileController->waitUntilSettled();
  */

  //Set the limit for chassis turning velocity
  chassis->setMaxVelocity(325);

  //Turn left to face towards the alliance station wall
  chassis->turnAngle(-65_deg);

  //Start the intake
  intake.move_velocity(550);

  //Pause for a little under half a second to give drive team members time to place rings that alligned with the robot
  pros::delay(200);

  //Set limit for chassis linear velocity (Special Case: Match Load Rings)
  chassis->setMaxVelocity(425);

  //Generate a path for the robot to drive towards the alliance station wall while picking up match loads
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {2.5_ft, 0_ft, 0_deg}}, "leftMatchLoads");
  //Have the robot follow the path above
  profileController->setTarget("leftMatchLoads");
  profileController->waitUntilSettled();

  dropBackpack();
}

void testSkills(){
  
  //Set the limit for chassis linear velocity
  chassis->setMaxVelocity(600);

  //Generate path to have robot be perpidicularly aligned with the right side alliance mobile goal
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {4.1_ft, 0_ft, 0_deg}}, "rightNeutralGrab");
  //Set robot to follow the above path
  profileController->setTarget("rightNeutralGrab");
  profileController->waitUntilSettled();

  frontClaw.set_value(true);

  //liftOkapi->setTarget(holdingLiftPosition);
  //liftOkapi->waitUntilSettled();
  liftForHolding();

  //Funny turn 
  chassis->setMaxVelocity(325);
  chassis->turnAngle(-137_deg);
  chassis->setMaxVelocity(600);

  liftForPlatform();

  //Stack right neutral goal on blue platform
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {5.15_ft, 0_ft, 0_deg}}, "rightNeutralPlatform");
  //Set robot to follow the above path
  profileController->setTarget("rightNeutralPlatform");
  profileController->waitUntilSettled();

  lift = -127;
  pros::delay(500);
  lift = 0;

  frontClaw.set_value(false);

  pros::delay(400);

  clearEncoders();

  lift = 127;
  pros::delay(300);
  lift = 0;
  
  //back up from platform
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {0.95_ft, 0_ft, 0_deg}}, "backUpFirstMogoPlatform");
  //Set robot to follow the above path
  profileController->setTarget("backUpFirstMogoPlatform", true);
  profileController->waitUntilSettled();

  //Turn to face red goal
  chassis->setMaxVelocity(325);
  chassis->turnAngle(-135_deg);
  chassis->setMaxVelocity(600);

  liftForDriving();

  //Approach red goal on our side
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {3_ft, 0_ft, 0_deg}}, "redMogoGrab");
  //Set robot to follow the above path
  profileController->setTarget("redMogoGrab");
  profileController->waitUntilSettled();

  //clamp onto red goal
  frontClaw.set_value(true);

  //back up a tad bit, just a itty bit
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {.5_ft, 0_ft, 0_deg}}, "backUpRedMogoPlat");
  //Set robot to follow the above path
  profileController->setTarget("backUpRedMogoPlat", true);
  profileController->waitUntilSettled();

  //lift goal up for driving
  liftForHolding();

  chassis->setMaxVelocity(325);
  chassis->turnAngle(-120_deg);
  chassis->setMaxVelocity(600);

  liftForPlatform();

  intake.move_velocity(-550);

  //Drive to red platform
  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {15.2_ft, 0_ft, 0_deg}}, "redMogoPlatform");
  //Set robot to follow the above path
  profileController->setTarget("redMogoPlatform");
  profileController->waitUntilSettled();

  intake = 0;

  lift = -127;
  pros::delay(500);
  lift = 0;

  frontClaw.set_value(false);

  pros::delay(400);

  clearEncoders();

  lift = 127;
  pros::delay(300);
  lift = 0;

  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {0.3_ft, 0_ft, 0_deg}}, "redMogoPlatformBack");
  //Set robot to follow the above path
  profileController->setTarget("redMogoPlatformBack", true);
  profileController->waitUntilSettled();

  clearEncoders();

  chassis->setMaxVelocity(325);
  chassis->turnAngle(-60_deg);
  chassis->setMaxVelocity(600);

  liftForDriving();

  profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {6.5_ft, 0_ft, 0_deg}}, "blueMogo");
  //Set robot to follow the above path
  profileController->setTarget("blueMogo", true);
  profileController->waitUntilSettled();

  frontClaw.set_value(true);

  liftForHolding();

  chassis->setMaxVelocity(325);
  chassis->turnAngle(-100_deg);
  chassis->setMaxVelocity(600);


  
}