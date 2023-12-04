#include "vex.h"

using namespace vex;


double  fix::LF_fix_V = 0;
double  fix::LB_fix_V = 0;
double  fix::RF_fix_V = 0;
double  fix::RB_fix_V = 0;

void fix::fix_motor(double LF_V_want, double LB_V_want, double RF_V_want, double RB_V_want){

        LF_V_want *= fix::limit_kp;
        LB_V_want *= fix::limit_kp;
        RF_V_want *= fix::limit_kp;
        RB_V_want *= fix::limit_kp;

        // printf("want  LF = %f , LB = %f , RF = %f ,RB = %f\n",LF_V_want,LB_V_want,RF_V_want,RB_V_want);
        
        double LF_current_V = leftFront.velocity(percent);
        double LB_current_V = leftBack.velocity(percent);
        double RF_current_V = rightFront.velocity(percent);
        double RB_current_V = rightBack.velocity(percent);

        // printf("current  LF = %f , LB = %f , RF = %f ,RB = %f\n",LF_current_V,LB_current_V,RF_current_V,RB_current_V);

        double LF_delta_V = LF_V_want - LF_current_V;
        double LB_delta_V = LB_V_want - LB_current_V;
        double RF_delta_V = RF_V_want - RF_current_V;
        double RB_delta_V = RB_V_want - RB_current_V;

        // printf("delta  LF = %f , LB = %f , RF = %f ,RB = %f\n",LF_delta_V,LB_delta_V,RF_delta_V,RB_delta_V);

        fix::LF_fix_V = LF_V_want + fix::fix_kp * LF_delta_V;
        fix::LB_fix_V = LB_V_want + fix::fix_kp * LB_delta_V;
        fix::RF_fix_V = RF_V_want + fix::fix_kp * RF_delta_V;
        fix::RB_fix_V = RB_V_want + fix::fix_kp * RB_delta_V;

        // printf("fix  LF = %f , LB = %f , RF = %f ,RB = %f\n",fix::LF_fix_V,fix::LB_fix_V,fix::RF_fix_V,fix::RB_fix_V);
        wait(30,msec);

}

