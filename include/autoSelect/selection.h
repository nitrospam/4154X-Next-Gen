#pragma once

#include <string>

//selector configuration
#define HUE 180
#define DEFAULT 1
#define AUTONS "Middle Goal", "Right Goal", "Left Goal", "RightMiddle"

namespace selector{

extern int auton;
const char *b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
