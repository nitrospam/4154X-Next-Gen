#include "main.h"

//Helpers
void clawActuate(){
    if(clawToggle == false){
        frontClaw.set_value(true);
        clawToggle = true;
    }
    else{
        frontClaw.set_value(false);
        clawToggle = false;
    }

}

void dumpActuate(){
    if(dumpToggle == false){
        dumpTruck.set_value(true);
        dumpToggle = true;
    }
    else{
        dumpTruck.set_value(false);
        dumpToggle = false;
    }

}

//Toggles

void setClawPistons(){

    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            pros::delay(1);
        }
        dumpActuate();
    }
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            pros::delay(1);
        }
        clawActuate();
    }

}