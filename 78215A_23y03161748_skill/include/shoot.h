using namespace vex;

class Shoot {

  public :
    static int shoot_air_on();

    static bool shoot_OnOff;//發射馬達狀態
    static int shoot_velocity;//發射馬達速度
    static int shooting();

    static void auto_shoot_air(void);
    static void auto_shoot_motor_on();
    static void auto_shoot_motor_off();
    static void auto_shoot_motor_set(double shoot_motor_velocity);
      
    static constexpr double pid_turn_Kp = 2.2/*2.2*/, pid_turn_Ki = 0 , pid_turn_Kd = 0;
    static constexpr double  pid_turn_I_limit = 10, pid_turn_totalError_limit = 30;
    static double pid_turn_P, pid_turn_I ,pid_turn_D ,pid_turn_power,pid_turn_totalError ;
    static double pid_turn(double turnerror, double last_turnerror);
    static void fort_pid_spin(double preferredAngle);
    static int fort_no_pid_spin_task();
    static double fortVelocity ,  fortAngle ;
    static void fort_no_pid_spin(double Angle , double fortV , bool wait);
};