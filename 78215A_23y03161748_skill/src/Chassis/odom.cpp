#include "vex.h"

using namespace vex;

//GLOBAL COORDINATES
Point Odom::globalPoint = {0, 0, 0};
Point Odom::prevGlobalPoint = {0, 0, 0};
Point Odom::globalDeltaPoint = {0, 0, 0};

//LOCAL COORDINATES
Point Odom::localDeltaPoint = {0, 0};

//SENSOR VALUES
//encoder
encoderType Odom::encoderVal = {0, 0, 0}; //leftEnc, rightEnc, backEnc
encoderType Odom::prevEncoderVal = {0, 0, 0};
encoderType Odom::deltaEncoderVal = {0, 0, 0};

//angle
double Odom::currentAngle = 0.0;
double Odom::prevAngle = 0.0;
double Odom::deltaAngle = 0.0;

//ODOMETRY FUNCTIONS
void Odom::updateSensors(){
  encoderVal.left = Math::degToCm(encoderLeft.rotation(deg)); //leftE
  encoderVal.right = Math::degToCm(encoderRight.rotation(deg)); //rightE
  encoderVal.back = Math::degToCm(encoderBack.rotation(deg)); //backE
  
  deltaEncoderVal.left = encoderVal.left - prevEncoderVal.left; //leftE
  deltaEncoderVal.right = encoderVal.right - prevEncoderVal.right; //rightE
  deltaEncoderVal.back = encoderVal.back - prevEncoderVal.back; //backE

  prevEncoderVal.left = encoderVal.left; //leftE
  prevEncoderVal.right = encoderVal.right; //rightE
  prevEncoderVal.back = encoderVal.back; //backe

  //deltaAngle = -(deltaEncoderVal.right - deltaEncoderVal.left) / track;
  //currentAngle = deltaAngle + prevAngle;
  //prevAngle = currentAngle;

  currentAngle = Math::getRadians(imu.rotation()); // 讀取 Inertial Rotation 數值
  deltaAngle = currentAngle - prevAngle; // 當前角度與前一個角度的誤差值
  prevAngle = currentAngle; //把這一刻的數值儲存到前一刻數值變數中
}


void Odom::updatePosition(){
  //Polar coordinates //
  localDeltaPoint.x = (deltaAngle + (deltaEncoderVal.back / backEncOffset)) * backEncOffset;
  localDeltaPoint.y = (deltaEncoderVal.left + deltaEncoderVal.right) / 2;//機器人座標

  //Cartesian coordinates | 注意 sin cos 內的數值須留意
  globalDeltaPoint.x = (localDeltaPoint.y * sin(prevAngle + deltaAngle/2)) + (localDeltaPoint.x * cos(prevAngle + deltaAngle/2)); 
  globalDeltaPoint.y = (localDeltaPoint.y * cos(prevAngle + deltaAngle/2)) - (localDeltaPoint.x * sin(prevAngle + deltaAngle/2));//場地座標
  globalDeltaPoint.angle = deltaAngle;

  //X & y Position
  globalPoint.x = globalDeltaPoint.x + prevGlobalPoint.x; // 最新的 X
  globalPoint.y = globalDeltaPoint.y + prevGlobalPoint.y; // 最新的 Y
  globalPoint.angle = currentAngle; // 最新的 Angle 直接吃 Inertial 數值

  prevGlobalPoint.x = globalPoint.x; // 將當前的數值，儲存到過去數值
  prevGlobalPoint.y = globalPoint.y; // 將當前的數值，儲存到過去數值
}

void Odom::reset(){
  encoderLeft.resetRotation(); encoderRight.resetRotation(); encoderBack.resetRotation(); // 重置encoder數值
  prevEncoderVal.left = 0.0; prevEncoderVal.right = 0.0; prevEncoderVal.back = 0.0; // 設置 變數 encoder 數值為 0
  prevAngle = 0.0; // 設置 前一刻 角度變數 數值為 0
  prevGlobalPoint.x = 0.0; prevGlobalPoint.y = 0.0; // 設置 前一刻 位置變數 數值為 0
}

void Odom::setPosition(double newX, double newY, double newAngle){
  reset();
  prevAngle = newAngle;//設定角度
  prevGlobalPoint.x = newX;//設定x
  prevGlobalPoint.y = newY;//設定y
}

//ODOMETRY THREAD
int Odom::Odometry(){
  while(true) {
    Odom::updateSensors();
    Odom::updatePosition();
    
    Brain.Screen.printAt(1, 20, "X: %f", Odom::globalPoint.x);
    Brain.Screen.printAt(1, 40, "Y: %f", Odom::globalPoint.y);
    Brain.Screen.printAt(1, 60, "Heading: %f", Odom::globalPoint.angle*(180/M_PI));
    
    Brain.Screen.printAt(1, 120, "L: %f", Odom::encoderVal.left);
    Brain.Screen.printAt(1, 140, "R: %f", Odom::encoderVal.right);
    Brain.Screen.printAt(1, 160, "B: %f", Odom::encoderVal.back);

    this_thread::sleep_for(10);//wait
  }
  return 0;
}