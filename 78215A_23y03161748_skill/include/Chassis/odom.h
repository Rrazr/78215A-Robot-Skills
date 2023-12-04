using namespace vex;

struct Point {
  double x, y, angle; //設定該點座標及角度
};

struct encoderType{
  double left, right, back; //將Encoder 綁在一起使用
};

class Odom {
  public: 
    //CONSTANTS
    static constexpr double track = 8.8; //cm  左右Encoder的間距
    static constexpr double backEncOffset = 13.6; //cm 後方Encoder與左右Encoder的間距
    static constexpr double wheelCircumference = 2.75 * 2.54; //cm  Encoder輪子直徑

    //GLOBAL COORDINATES
    static Point globalPoint; //x, y, angle
    static Point prevGlobalPoint; //前一刻 x, y, angle
    static Point globalDeltaPoint; //change in x, change in y, change in angle

    //LOCAL COORDINATES
    static Point localDeltaPoint; //change in x, change in y

    //SENSOR VALUES
    //encoder
    static encoderType encoderVal; //leftEnc, rightEnc, backEnc
    static encoderType prevEncoderVal; //prev leftEnc, rightEnc, backEnc
    static encoderType deltaEncoderVal; //change in leftEnc, rightEnc, backEnc
    //angle
    static double currentAngle;//現在角度
    static double prevAngle;//前一刻角度
    static double deltaAngle;//角度變化

    //ODOMETRY FUNCTIONS
    static void updateSensors();//Encoder , imu 數據處理及更新
    static void updatePosition();//x, y, angle更新
    static void reset();
    static void setPosition(double newX, double newY, double newAngle);//外部設定現在機器人x, y, angle

    //ODOMETRY THREAD
    static int Odometry();//主程式，放同步執行
};