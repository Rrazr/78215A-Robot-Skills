using namespace vex;

class noOdom {
  public :
    static double no_odometry_move_linear_speed, no_odometry_move_linear_Direction, no_odometry_move_linear_second;
    static int  no_odometry_move_linear_second_task();
    static void no_odometry_move_linear_time(double speed,double Direction,double second,bool Wait);

    static double find_min_angle (double firstAngle, double secondAngle);
    static double preferredAngle;
    static double pid_turn_Kp, pid_turn_Ki , pid_turn_Kd;
    static constexpr double  pid_turn_I_limit = 10, pid_turn_totalError_limit = 30;
    static double pid_turn_P, pid_turn_I ,pid_turn_D ,pid_turn_power,pid_turn_totalError ;
    static double pid_turn(double turnerror, double last_turnerror);
    static int odometry_move_pid_spin_task();
    static void odometry_move_pid_spin(double degree, double kp , double ki, double kd, bool wait);
    static void No_odometry_move_linear(int speed,double Direction,double degree);
    static void imu_No_odometry_move_linear(int speed,double Direction,double degree);
    static void No_imu_rotation_speed(int speed,double preferredAngle);
};

