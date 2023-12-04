using namespace vex;

class highgoal {
  public:

    static double AutomoveAtan(double deltax ,double deltay);
    static double facing_highgoal_degree ;
    static constexpr double facing_highgoal_targetx = 47.4 ,facing_highgoal_target_y = 360 - 47.4;

    static void facing_highgoal(void);
    static int facing_highgoal_driving();
};