#include "main.h"
//Motors
extern pros::Motor frontLeft;
extern pros::Motor frontRight;
extern pros::Motor middleLeft;
extern pros::Motor middleRight;
extern pros::Motor backLeft;
extern pros::Motor backRight;
extern pros::Motor lift;
extern pros::Motor intake;

//Sensors
extern pros::ADIAnalogIn liftPot;
extern pros::Imu inertial;
extern pros::ADIDigitalOut frontClaw;
extern pros::ADIDigitalOut dumpTruck;
extern pros::ADIEncoder trackingWheel;

//Controller
extern pros::Controller controller;

//Helper Variables
extern bool clawToggle;
extern bool dumpToggle;
