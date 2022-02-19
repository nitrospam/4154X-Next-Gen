#include "main.h"

//Motors
pros::Motor frontLeft(7, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor frontRight(10, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor middleLeft(8, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor middleRight(9, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor backLeft(19, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor backRight(20, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor lift(7, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intake(8, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);

//Sensors & Pistons
pros::ADIAnalogIn liftPot('C');
pros::Imu inertial(13);
pros::ADIEncoder trackingWheel ('A', 'B', false);
pros::ADIDigitalOut frontClaw('D', false);
pros::ADIDigitalOut dumpTruck('E', false);

//Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Helper Variables
bool clawToggle = false;
bool dumpToggle = false;