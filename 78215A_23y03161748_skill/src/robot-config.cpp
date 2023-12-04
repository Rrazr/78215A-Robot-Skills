#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
encoder encoderLeft = encoder(Brain.ThreeWirePort.C);
encoder encoderRight = encoder(Brain.ThreeWirePort.A);
encoder encoderBack = encoder(Brain.ThreeWirePort.E);
inertial imu = inertial(PORT9);
controller Controller1 = controller(primary);
motor leftFront = motor(PORT11, ratio18_1, false);
motor leftBack = motor(PORT2, ratio18_1, false);
motor rightFront = motor(PORT12, ratio18_1, true);
motor rightBack = motor(PORT4, ratio18_1, true);
motor roller = motor(PORT8, ratio36_1, false);
digital_out shoot_air = digital_out(Brain.ThreeWirePort.H);
motor R_shoot = motor(PORT7, ratio36_1, true);
motor L_shoot = motor(PORT6, ratio6_1, true);
digital_out expand = digital_out(Brain.ThreeWirePort.G);
/*vex-vision-config:begin*/
signature Vision__BLUEB = signature (1, -3025, -1193, -2110, 3189, 8705, 5948, 1.4, 0);
signature Vision__SIG_2 = signature (2, 0, 0, 0, 0, 0, 0, 3, 0);
vision Vision = vision (PORT15, 59, Vision__BLUEB, Vision__SIG_2);
/*vex-vision-config:end*/
inertial imuFort = inertial(PORT1);
motor fort = motor(PORT5, ratio36_1, false);
distance DistanceFront = distance(PORT19);
distance DistanceBack = distance(PORT20);
distance DistanceLeft = distance(PORT18);
distance DistanceRight = distance(PORT17);
motor Intake = motor(PORT3, ratio18_1, false);
optical Color = optical(PORT13);
distance discNumber = distance(PORT10);
optical IntakeRight = optical(PORT16);
optical IntakeLeft = optical(PORT14);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}