using namespace vex;

class Roller {
  
  public:
    static int roller_spin_blue_task();
    static void roller_spin_blue(bool Wait);
    static int roller_spin_red_task();
    static void roller_spin_red(bool Wait);
    static void roller_spin(void);
    static void roller_load_on(void);
    static void roller_load_off(void);
    static void roller_load_out();
};