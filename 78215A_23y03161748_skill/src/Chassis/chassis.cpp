#include "vex.h"

using namespace vex;

//DRIVE HELPER FUNCTIONS
void Chassis::setVel(double LF, double LB, double RF, double RB){//rpm 操縱底盤
  leftFront.spin(fwd, LF, rpm);
  leftBack.spin(fwd, LB, rpm);
  rightFront.spin(fwd, RF, rpm);
  rightBack.spin(fwd, RB, rpm);
}

void Chassis::setVolt(double LF, double LB, double RF, double RB){//伏特 操縱底盤
  leftFront.spin(fwd, LF, volt);
  leftBack.spin(fwd, LB, volt);
  rightFront.spin(fwd, RF, volt);
  rightBack.spin(fwd, RB, volt);
}

void Chassis::setStop(brakeType type){// 剎車型態設定
  leftFront.setStopping(type);
  leftBack.setStopping(type);
  rightFront.setStopping(type);
  rightBack.setStopping(type);
}

void Chassis::groupStop(void){// 底盤停止
  leftFront.stop();
  leftBack.stop();
  rightFront.stop();
  rightBack.stop();

}

//DRIVER CONTROL FUNCTIONS
//相對底盤遙控程式
void Chassis::robotCentric(){
  int controllerX = Controller1.Axis4.position() * 2;
  int controllerY = Controller1.Axis3.position() * 2;
  int turning = Controller1.Axis1.position() <= 40 ? Controller1.Axis1.position()
  * 2 / 3 * 2 : Controller1.Axis1.position() * 2;

  //Deadzone
  if(abs(Controller1.Axis4.position()) < 5) {controllerX = 0;}
  if(abs(Controller1.Axis3.position()) < 5) {controllerY = 0;}
  if(abs(Controller1.Axis1.position()) < 5) {turning = 0;}

  int leftF = controllerY + controllerX + turning;
  int leftB = controllerY - controllerX + turning;
  int rightF = controllerY - controllerX - turning;
  int rightB = controllerY + controllerX - turning;

  // fix::fix_motor(leftF,leftB,rightF,rightB);
  // setVel(fix::LF_fix_V, fix::LB_fix_V, fix::RF_fix_V, fix::RB_fix_V);
  setVel(leftF,leftB,rightF,rightB);
}

//絕對角度底盤遙控程式
void Chassis::fieldCentric(){
  //Controller variables
  int controllerX = Controller1.Axis4.position() * 2;
  int controllerY = Controller1.Axis3.position() * 2;
   int turning = Controller1.Axis1.position() <= 40 ? Controller1.Axis1.position()
  * 2 / 3 * 2 : Controller1.Axis1.position() * 2;


  //Deadzone
  if(abs(Controller1.Axis4.position()) < 5) {controllerX = 0;}
  if(abs(Controller1.Axis3.position()) < 5) {controllerY = 0;}
  if(abs(Controller1.Axis1.position()) < 5) {turning = 0;}


  //Quicc maths
  double magnitude = sqrt(pow(controllerX, 2) + pow(controllerY, 2));
  double theta = atan2(controllerY, controllerX);
  double theta2 = theta + (imu.heading(degrees)*M_PI/180);
  double x2 = magnitude * cos(theta2);
  double y2 = magnitude * sin(theta2);

  //Imu reset
  //if(Controller1.ButtonA.pressing()) {imu.resetRotation();}
  /*if (Controller1.ButtonY.pressing()){
    //L_shoot.spin(forward, 61*120, voltageUnits::mV);
    //R_shoot.spin(forward, 61*120, voltageUnits::mV);
    noOdom::No_odometry_move_linear(70,215.68,488);
    Shoot::auto_shoot_air();
    wait(300, msec);
    Shoot::auto_shoot_air();*/

  //Motor output
  double leftF = (y2 + x2)*1.5 + turning;
  double leftB = (y2 - x2)*1.5 + turning;
  double rightF = (y2 - x2)*1.5 - turning;
  double rightB = (y2 + x2)*1.5 - turning;

  setVel(leftF, leftB, rightF, rightB);
}

// static int cnt = 0;
// static int preVal = 0;
// 遙控程式切換(現在為強制絕對角度操控)
void Chassis::opControl(){
  // int val = Controller1.ButtonB.pressing();
  // if(val == 1 && preVal == 0){
  //   cnt++;
  // }
  // preVal = val;
  if(/*cnt % 2 == 1*/true){
    fieldCentric();
    // Controller1.Screen.setCursor(2, 1);
    // Controller1.Screen.clearLine(2);
    // Controller1.Screen.print("Field Centric");
  } else {
    robotCentric();
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.clearLine(2);
    Controller1.Screen.print("Robot Centric");
  }
}

//RESET
//encoder 初始化
void Chassis::resetEnc(){
  encoderLeft.resetRotation(); encoderRight.resetRotation(); encoderBack.resetRotation();
}