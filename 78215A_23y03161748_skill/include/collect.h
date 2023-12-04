using namespace vex;

class Collect{

  public :
    static int taking_task();

    static void auto_taking_in(void);
    static void auto_taking_out(void);
    static void auto_taking_off(void);

    static double auto_taking_in_second;
    static int auto_taking_in_time_task();
    static void auto_taking_in_time(double second,bool Wait);

    static double auto_taking_out_second;
    static int auto_taking_out_time_task();
    static void auto_taking_out_time(double second,bool Wait);
};