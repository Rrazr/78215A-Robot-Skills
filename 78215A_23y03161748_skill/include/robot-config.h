using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern encoder encoderLeft;
extern encoder encoderRight;
extern encoder encoderBack;
extern inertial imu;
extern controller Controller1;
extern motor leftFront;
extern motor leftBack;
extern motor rightFront;
extern motor rightBack;
extern motor roller;
extern digital_out shoot_air;
extern motor R_shoot;
extern motor L_shoot;
extern digital_out expand;
extern signature Vision__BLUEB;
extern signature Vision__SIG_2;
extern signature Vision__SIG_3;
extern signature Vision__SIG_4;
extern signature Vision__SIG_5;
extern signature Vision__SIG_6;
extern signature Vision__SIG_7;
extern vision Vision;
extern inertial imuFort;
extern motor fort;
extern distance DistanceFront;
extern distance DistanceBack;
extern distance DistanceLeft;
extern distance DistanceRight;
extern motor Intake;
extern optical Color;
extern distance discNumber;
extern optical IntakeRight;
extern optical IntakeLeft;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );