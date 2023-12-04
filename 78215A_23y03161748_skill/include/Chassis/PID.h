#include <vector>

using namespace vex;

class PID{

  public :
    enum PIDtype{turnAng,movement}; // 參考 enum 列舉 https://www.796t.com/content/1550367759.html

    // PID Declare Variable 先宣告變數
    static double kP, kD, derivative_x, derivative_y, prevError_x, prevError_y, kF; // derivative 導數 prev 前
    static double turn_kP, turn_kD, turn_derivative, turn_prevError, turn_kF; // kF？
    static double dist, turn_e; // dist 距離 | turn_e 轉誤差

    // PID Declare Function Constructs PID constants 先宣告function
    static void setPID(double _kP, double _kD, double _turn_kP, double _turn_kD); 
    static void PIDmoveTo(double x, double y, double finalAngle);
    static void PIDreset();
    static double PIDgetError(PIDtype type);

    static double startAngle;
    static double distanceToTarget;
    static double angleToTarget;
    static double relativeAngleToTarget;
    static double relativeXToTarget;
    static double relativeYToTarget;
    static double movementXPower;
    static double movementYPower;
    static double heading2;
    static double turnError;
    static double turnPower;




};

