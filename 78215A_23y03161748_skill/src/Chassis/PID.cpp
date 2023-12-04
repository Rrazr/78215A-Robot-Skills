#include "vex.h"

using namespace vex;


double PID::kP = 0, PID::kD = 0, PID::derivative_x = 0, PID::derivative_y = 0, PID::prevError_x = 0, PID::prevError_y = 0, PID::kF = 0;
double PID::turn_kP = 0, PID::turn_kD = 0, PID::turn_derivative = 0, PID::turn_prevError = 0, PID::turn_kF = 0;
double PID::dist = 0, PID::turn_e = 0;

double PID::startAngle = 0;
double PID::distanceToTarget = 0;
double PID::angleToTarget = 0;
double PID::relativeAngleToTarget = 0;
double PID::relativeXToTarget = 0;
double PID::relativeYToTarget = 0;
double PID::movementXPower = 0;
double PID::movementYPower = 0;
double PID::heading2 = 0;
double PID::turnError = 0;
double PID::turnPower = 0;

// 設定 PID 係數
void PID::setPID(double _kP, double _kD, double _turn_kP, double _turn_kD){
  //Movement PID constants
  kP = _kP; kD = _kD; // >> 塞進 全域變數讓其他地方使用
  //Movement PID
  derivative_x = 0.0; derivative_y = 0.0; prevError_x = 0.0; prevError_y = 0.0;
  //Turn PID constants
  turn_kP = _turn_kP; turn_kD = _turn_kD; 
  //Turn PID
  turn_derivative = 0.0; turn_prevError = 0.0;
  //getError function - for debugging
  dist = 0.0; turn_e = 0.0;
}

// 使用 PID 控制 輸入參數 座標 x y angle
void PID::PIDmoveTo(double x, double y, double finalAngle){
  finalAngle = finalAngle*(M_PI/180.0); // 輸入是角度，需要轉換成徑度
  startAngle = Odom::globalPoint.angle; // 從 Odom .cpp 中傳過來的數值 Inertial 的 rotation數值（徑度）
  while(true){
    // Calculates distance and angle of the current point to the target point 計算當前點到目標點的距離和角度
    distanceToTarget = hypot(x - Odom::globalPoint.x, y - Odom::globalPoint.y); // hypot :參數平方和的平方根 斜邊，斜邊是直角三角形的最長邊
    PID::dist = distanceToTarget; //for multitasking
    // printf("dist =%f |",dist);
    angleToTarget = atan2(y - Odom::globalPoint.y, x - Odom::globalPoint.x); // 求得角度 兩點之間 斜邊 與 X軸長度 的角度（徑度） 
    relativeAngleToTarget = Math::angleWrap(angleToTarget + Odom::globalPoint.angle); // 求的目標角度與當前角度差值
    // printf("angError = %f\n",angleToTarget);

    //Calculates the x and y components of the vector
    relativeXToTarget = (cos(relativeAngleToTarget) * distanceToTarget); // 標角度與當前角度差值 分成 x 分量｜乘上 與目標距離之差
    relativeYToTarget = (sin(relativeAngleToTarget) * distanceToTarget); // 標角度與當前角度差值 分成 y 分量｜乘上 與目標距離之差

    //Movement PID
    PID::derivative_x = relativeAngleToTarget - PID::prevError_x; // Kd 需要用到
    PID::prevError_x = relativeAngleToTarget;
    PID::derivative_y = relativeAngleToTarget - PID::prevError_y;
    PID::prevError_y = relativeAngleToTarget;

    movementXPower = relativeXToTarget * PID::kP + PID::derivative_x * PID::kD;
    movementYPower = relativeYToTarget * PID::kP + PID::derivative_x * PID::kD;

    //Calculates turn angle
    heading2 = finalAngle < 0 ? finalAngle + M_PI*2 : finalAngle - M_PI*2; //(徑度如果小於零 )
    finalAngle = (fabs(Odom::globalPoint.angle) - finalAngle < 
                  fabs(Odom::globalPoint.angle) - heading2) ? finalAngle : heading2;
    Brain.Screen.printAt(1, 220, "turn amount: %f", heading2);

    //Turn PID
    turnError = -(finalAngle - Math::compressAngle(startAngle, Odom::globalPoint.angle)); PID::turn_e = turnError;
    PID::turn_derivative = turnError - PID::turn_prevError;
    PID::turn_prevError = turnError;
    turnPower = turnError * PID::turn_kP + PID::turn_derivative * PID::turn_kD;

    Brain.Screen.printAt(1, 200, "Turn Error: %f", turnError);

    if(fabs(turnError) < Math::getRadians(2) && fabs(relativeXToTarget) < 0.5 && fabs(relativeYToTarget) < 0.5) break;

    Chassis::setVel(movementYPower + movementXPower - turnPower, movementYPower - movementXPower - turnPower,
                    movementYPower - movementXPower + turnPower, movementYPower + movementXPower + turnPower);
    
    wait(10, msec);
  }
  Chassis::setVel(0, 0, 0, 0);
}


void PID::PIDreset(){
  //Movement PID
  PID::derivative_x = 0.0; PID::derivative_y = 0.0; PID::prevError_x = 0.0; PID::prevError_y = 0.0;
  //Turn PID
  PID::turn_derivative = 0.0; PID::turn_prevError = 0.0;
  //Reset function - for debugging
  PID::dist = 0.0; PID::turn_e = 0.0;
}

double  PID::PIDgetError(PIDtype type){
  if(type == PIDtype::movement){
    return dist;
  } else {
    return turn_e;
  }
}
