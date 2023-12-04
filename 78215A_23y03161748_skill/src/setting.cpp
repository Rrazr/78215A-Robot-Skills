#include "vex.h"

using namespace vex;


double Setting::InerTial(){//Inertial degree 範圍控制 +180 ~ -180
  double d=imu.heading(degrees);
  if(d>180){
    d=d-360;
  }
  return d;
}

void Setting::setting(double deg){
  // take.setStopping(hold);//收集馬達設定速度
  // take.setPosition(0,degrees);//收集馬達初始度數
  IntakeRight.setLightPower(0, percent);
  IntakeLeft.setLightPower(0, percent);
  L_shoot.setStopping(coast);//射擊馬達設定剎車
  L_shoot.setPosition(0,degrees);
  R_shoot.setStopping(coast);
  R_shoot.setPosition(0,degrees);
  roller.setStopping(brake);
  roller.setPosition(0,degrees);
  Controller1.Screen.clearScreen();
  shoot_air.set(false);
  Color.setLightPower(50,percent);
  
  // 重置 encoder 角度數值
  encoderBack.resetRotation();
  encoderLeft.resetRotation();
  encoderRight.resetRotation();

  // Inertial 感測器重置
  imu.calibrate();
  imuFort.calibrate();
  fort.setPosition(0, degrees);
  wait(2000, msec); // 因為重置需要啟動時間
  imu.setHeading(deg,degrees);
  imuFort.setHeading(deg,degrees);
  Chassis::setStop(hold);  // 呼叫 class 裡面的函數
}

void Setting::motor_temperature(void){//溫度計，顯示在主機螢幕上

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("leftFront  = %f",leftFront.temperature(percent));
  Brain.Screen.setCursor(2,1);
  Brain.Screen.print("leftBack   = %f",leftBack.temperature(percent));
  Brain.Screen.setCursor(3,1);
  Brain.Screen.print("rightFront = %f",rightFront.temperature(percent));
  Brain.Screen.setCursor(4,1);
  Brain.Screen.print("rightBack  = %f",rightBack.temperature(percent));
  // Brain.Screen.setCursor(5,1);
  // Brain.Screen.print("take       = %f",take.temperature(percent));
  Brain.Screen.setCursor(6,1);
  Brain.Screen.print("L_shoot    = %f",L_shoot.temperature(percent));
  Brain.Screen.setCursor(7,1);
  Brain.Screen.print("R_shoot    = %f",R_shoot.temperature(percent));
  Brain.Screen.setCursor(8,1);
  Brain.Screen.print("roller     = %f",roller.temperature(percent));
}