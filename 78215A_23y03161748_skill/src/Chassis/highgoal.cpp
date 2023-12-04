// #include "vex.h"

// using namespace vex;

// double highgoal::AutomoveAtan(double deltax ,double deltay){
//   double p3=deltay;
//   double p4=deltax; 
//   double phi=atan(p4/p3)*180/M_PI; 
//   phi=(p3==0&&p4==0)?0:phi;
  
//   if(p3<0){
//     if(phi>0){
//       phi=phi-180; 
//     }else{
//       phi=phi+180;
//     }
//   }
//   //printf("%lf\n",phi);
//   //wait(10,msec);
//   return phi;
  
// }

// double highgoal::facing_highgoal_degree=0.0;

// void highgoal::facing_highgoal(){

 
//       highgoal::facing_highgoal_degree=AutomoveAtan(highgoal::facing_highgoal_targetx - Odom::globalPoint.x , highgoal::facing_highgoal_target_y - Odom::globalPoint.y);
//       // printf("deltax= %f,deltay= %f,deltad=%f",x1final - facing_highgoal_targetx, y1final - facing_highgoal_target_y,facing_highgoal_degree);
//       noOdom::odometry_move_pid_spin(highgoal::facing_highgoal_degree);
//       // noOdom::odometry_move_pid_spin(20.0);

// }


// int highgoal::facing_highgoal_driving(){
    
//   while(true){
//     if(Controller1.ButtonRight.pressing()){
      
//       wait(20,msec);
//       waitUntil(!Controller1.ButtonRight.pressing());
//       highgoal::facing_highgoal();

//     }
//   } 
//   return 0;
// }