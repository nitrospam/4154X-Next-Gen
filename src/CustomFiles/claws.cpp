#include "main.h"

//vars
bool goalInBackpack = false;

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
        dumpTruckLeft.set_value(true);
        dumpTruckRight.set_value(true);
        dumpToggle = true;
    }
    else{
        dumpTruckLeft.set_value(false);
        dumpTruckRight.set_value(true);
        dumpToggle = false;
    }

}

//Toggles

void setClawPistons(){

    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        frontClaw.set_value(true);
    }
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        frontClaw.set_value(false);
    }
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        dumpTruckLeft.set_value(true);
        dumpTruckRight.set_value(true);
        goalInBackpack = true;
    }
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
        dumpTruckLeft.set_value(false);
        dumpTruckRight.set_value(false);
        goalInBackpack = false;
        intake = 0;
    }
}