#include "main.h"

void ringToggle(){
if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        intake.move_velocity(400);
    }
else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        intake.move_velocity(-400);
    }
else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        intake = 0;
    }
}