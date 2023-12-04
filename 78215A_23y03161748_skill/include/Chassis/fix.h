using namespace vex;

class fix {
  public :
    static constexpr double fix_kp = 0.6,limit_kp = 0.9 ;
    static double LF_fix_V , LB_fix_V ,  RF_fix_V , RB_fix_V;
    static void fix_motor(double LF_V_want, double LB_V_want, double RF_V_want, double RB_V_want);


};

