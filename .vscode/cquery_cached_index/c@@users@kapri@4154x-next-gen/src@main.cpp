#include "main.h"
#include "autoSelect/selection.h"

void initialize(){
    liftPot.calibrate();
    if (pros::competition::is_connected()){
      selector::init();
    }
    else {
      pros::lcd::initialize();
    }
}

void disabled() {}

void autonomous() {

}


void opcontrol() {
  setDriveMotors();
  setLiftMotor();
}
