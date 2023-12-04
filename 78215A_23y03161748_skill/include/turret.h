// using namespace vex;
extern bool autoAim_midline_io ;
extern bool move_OK;
extern bool firstRotation;
extern bool secondRotation;
class Turret {
  public:
    static int spin_turret();
    static int auto_aim_turret();
    static int turretspin();
    static int recenter();
    static void turret_on(void);
    static int autoAim_midline_task();
    static int autoAim_midline_task_2();
    static double autoAim_midline_cm;
    static double autoAim_midline_cm_2;
    static void autoAim_midline(double cm,bool LR,bool Wait);
    static double autoAim_midline_Dis;
    static double autoAim_midline_Dis_2;
    static bool LR_io;
    static bool LR_io_2;
    static void autoAim_midline_2(double cm ,bool LR,bool Wait);
    static double LF_rotation;
    static double target;
    static double tmpy_highgoal;
};
