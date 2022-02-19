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
  //Pseudocode - write after comment line by line
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
  chassis->turnAngle(-208_deg);

  //Drive forward to align with row of rings
  chassis->setMaxVelocity(200);
  chassis->moveDistance(1_ft);

  //Turn right to face row of rings
  frontClaw.set_value(true);
  pros::delay(200);

  //Raise lift up to platform height
  chassis->setMaxVelocity(400);
  chassis->moveDistance(-4_ft);
  
  //Turn on ring mech


  //Drive forward at half speed into row of rings until you hit perpindicular row of rings


  //Turn off ring mech


  //Back up into home zone at full speed


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

}

void skills(){
  //Psuedocode - Write under comments line by line
  chassis->setMaxVelocity(600);
  
  //Align with flat side of blue amogo near platform

  //Back up into blue amogo

  
  //Clamp amogo into backpack


  //Drive forward to align with right neutral mogo

  
  //Turn right to face right neutral mogo


  //Drive forward into right neutral mogo


  //Clamp right neutral mogo

  
  //Raise lift with goal


  //Turn to face center of blue platform


  //Turn on ring mech
  
  
  //Drive at half speed towards platform


  //Turn off ring mech
  
  
  //Unclamp neutral mogo (40)

  
  //Back up to align with line between 2nd and 3rd row of tiles


  //Turn left to align with line between 2nd and 3rd row of tiles


  //Turn on ring mech
  
  
  //Drive forward at normal speed to align with far neutral mogo


  //Turn off ring mech

  
  //Turn left to face far neutral mogo


  //Put lift down to driving height


  //Drive forward into far neutral mogo

  
  //Clamp onto far neutral mogo


  //Lift for platform height


  //Turn right to face blue platform


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

  chassis->setMaxVelocity(600);
  
  chassis->turnAngle(90_deg);

}