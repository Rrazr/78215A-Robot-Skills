#include "vex.h"

using namespace vex;

double noOdom::no_odometry_move_linear_second = 0;
double noOdom::no_odometry_move_linear_speed = 0;
double noOdom::no_odometry_move_linear_Direction = 0;

int  noOdom::no_odometry_move_linear_second_task(){
  
  no_odometry_move_linear_second = no_odometry_move_linear_second + Brain.Timer.value();
  while(no_odometry_move_linear_second > Brain.Timer.value()){

    leftFront.spin(forward,no_odometry_move_linear_speed*sin((no_odometry_move_linear_Direction + 45 - Setting::InerTial())*M_PI/180.0),percent);
    rightFront.spin(forward,no_odometry_move_linear_speed*cos((no_odometry_move_linear_Direction + 45 - Setting::InerTial())*M_PI/180.0),percent);
    leftBack.spin(forward,no_odometry_move_linear_speed*cos((no_odometry_move_linear_Direction + 45 - Setting::InerTial())*M_PI/180.0),percent);
    rightBack.spin(forward,no_odometry_move_linear_speed*sin((no_odometry_move_linear_Direction + 45 - Setting::InerTial())*M_PI/180.0),percent);
  }
  Chassis::setStop(hold);
  Chassis::groupStop();
  return 0;
}


void noOdom::no_odometry_move_linear_time(double speed,double Direction,double second,bool Wait)
{
 
  no_odometry_move_linear_speed = speed;
  no_odometry_move_linear_Direction = Direction;
  no_odometry_move_linear_second = second;
  if (Wait){
  task no_odometry_move_linear_task_event = task(no_odometry_move_linear_second_task);
  }else{
    no_odometry_move_linear_second_task();
  }   
}


double noOdom::find_min_angle (double firstAngle, double secondAngle){
  double minimunAngle = firstAngle - secondAngle;
  if(minimunAngle > 180){
    minimunAngle = firstAngle -360 - secondAngle;
  }

  if(minimunAngle < -180){
    minimunAngle = firstAngle +(360 - secondAngle);
  }

  return minimunAngle;
}


//轉向的PID控制
double noOdom::pid_turn_P =0.0;
double noOdom::pid_turn_I =0.0;
double noOdom::pid_turn_D =0.0;
double noOdom::pid_turn_power =0.0;
double noOdom::pid_turn_totalError =0.0;

double noOdom::pid_turn(double turnerror, double last_turnerror){
 
  pid_turn_totalError = 0; 
  double derivative = turnerror - last_turnerror; 
  if(fabs(turnerror) < 30  && turnerror !=0){
    noOdom::pid_turn_totalError += turnerror;
  }
  else{
    noOdom::pid_turn_totalError = 0;
  }

  noOdom::pid_turn_P = noOdom::pid_turn_Kp * turnerror;
  noOdom::pid_turn_I = noOdom::pid_turn_totalError * noOdom::pid_turn_Ki;
  noOdom::pid_turn_D = noOdom::pid_turn_Kd * derivative;

  if(noOdom::pid_turn_I > noOdom::pid_turn_I_limit){
    noOdom::pid_turn_I = noOdom::pid_turn_I_limit;
  }
  
  noOdom::pid_turn_power = noOdom::pid_turn_P + noOdom::pid_turn_I + noOdom::pid_turn_D;
  return noOdom::pid_turn_power;

}


double noOdom::pid_turn_Kp = 2.2,  noOdom::pid_turn_Ki = 0, noOdom::pid_turn_Kd = 0;
double noOdom::preferredAngle = 0;
int noOdom::odometry_move_pid_spin_task(){

    double currentHeading =imu.heading(degrees);
    
    noOdom::pid_turn_totalError = 0;

    double turnAngle = 0;
    turnAngle = noOdom::find_min_angle(preferredAngle, currentHeading);
    double last_turnAngle = turnAngle;

    while((fabs(turnAngle) > 0.7)){

      currentHeading = imu.heading(degrees);

      turnAngle = noOdom::find_min_angle(preferredAngle, currentHeading);
      double Turnpower = noOdom::pid_turn(turnAngle, last_turnAngle);
      // if(Turnpower<20.0)Turnpower=30.0*Turnpower/fabs(Turnpower);
      Chassis::setVel(Turnpower, Turnpower,  - Turnpower, - Turnpower);
      last_turnAngle = turnAngle;
      //printf("f\n", Turnpower);
      // printf("Inertial= %f\n",Inertial.heading(degrees));

    }

    Chassis::setStop(hold);
    Chassis::groupStop();
    return 0;
}

void noOdom::odometry_move_pid_spin(double degree, double kp , double ki, double kd, bool wait){

  noOdom::preferredAngle = degree;
  noOdom::pid_turn_Kp = kp;
  noOdom::pid_turn_Ki = ki;
  noOdom::pid_turn_Kd = kd; 
  if (wait){
    task odometry_move_pid_spin_Task = task(noOdom::odometry_move_pid_spin_task);
  }else{
    noOdom::odometry_move_pid_spin_task();
  }
  



}
// void noOdom::No_imu_rotation_speed(int speed,double preferredAngle){

//     double currentHeading =imu.heading(degrees);
//     double settingAngle =preferredAngle;
//     preferredAngle+=currentHeading;
//     if(preferredAngle<360)preferredAngle+=360;
//     // printf("%f\n",preferredAngle);
//     noOdom::pid_turn_totalError = 0;

//     double turnAngle = 0;
//     turnAngle = noOdom::find_min_angle(preferredAngle, currentHeading);
//     // printf("%f\n",turnAngle);
//     double last_turnAngle = turnAngle;

//     while((fabs(turnAngle) > 0.5)){

//       currentHeading = imu.heading(degrees);

//       turnAngle = noOdom::find_min_angle(preferredAngle, currentHeading);
//       double Turnpower = speed*turnAngle/fabs(settingAngle);
//       if(Turnpower <35.0)Turnpower =35.0*turnAngle/fabs(turnAngle);
//       // printf("%f\n",Turnpower);
//       // printf("%f\n",turnAngle);
//       Chassis::setVel(Turnpower, Turnpower,  - Turnpower, - Turnpower);
//       last_turnAngle = turnAngle;
//       // printf("Inertial= %f\n",Inertial.heading(degrees));

//     }

//     Chassis::setStop(hold);
//     Chassis::groupStop();
// }

// void noOdom::No_odometry_move_linear(int speed,double Direction,double degree){
//   leftFront.setPosition(0.0,degrees);
//   rightFront.setPosition(0.0,degrees);
//   leftBack.setPosition(0.0,degrees);
//   rightBack.setPosition(0.0,degrees);
  
//   // double FL_deg = cos((Direction-45-Inertial.heading(degrees))*M_PI/180.0);
//   // double BL_deg = 0.0-sin((Direction-45-Inertial.heading(degrees))*M_PI/180.0);
//   // double FR_deg = sin((Direction-45-Inertial.heading(degrees))*M_PI/180.0);
//   // double BR_deg = (0.0-cos((Direction-45-Inertial.heading(degrees))*M_PI/180.0));

//   while(
//   degree-fabs(leftFront.position(degrees))<=2&&
//   degree-fabs(rightFront.position(degrees))<=2&&
//   degree-fabs(leftBack.position(degrees))<=2&&
//   degree-fabs(rightBack.position(degrees))<=2){
//   // frontleft.setVelocity(speed*(cos((Direction-45-Inertial.heading(degrees))*M_PI/180.0)),percent);
//   // backleft.setVelocity(speed*(0.0-sin((Direction-45-Inertial.heading(degrees))*M_PI/180.0)),percent);
//   // frontright.setVelocity(-(speed*(sin((Direction-45-Inertial.heading(degrees))*M_PI/180.0))),percent);
//   // backright.setVelocity(-speed*(((0.0-cos((Direction-45-Inertial.heading(degrees))*M_PI/180.0)))),percent);
//   double Speed;
//   if( leftFront.position(degrees) < 200.0 ){
//     Speed = (speed-30.0) * leftFront.position(degrees) / 200.0 +30.0; 
//   }
//   else if( (degree - leftFront.position(degrees)) < 200.0 ){
//     Speed = speed * (degree - leftFront.position(degrees)) / 200.0 ; 
//     if(fabs(Speed)<5){
//       if(Speed<0){
//         Speed = -5;
//       }else{
//         Speed = 5;
//       }
//     }
//   } 
//   else {
//     Speed = speed;
//   }


//   double No_odometry_move_linear_FL_V = Speed*(cos((Direction-45-imu.heading(degrees))*M_PI/180.0));
//   double No_odometry_move_linear_BL_V = Speed*(0.0-sin((Direction-45-imu.heading(degrees))*M_PI/180.0));
//   double No_odometry_move_linear_FR_V = -1*(Speed*(sin((Direction-45-imu.heading(degrees))*M_PI/180.0)));
//   double No_odometry_move_linear_BR_V = -1*Speed*(((0.0-cos((Direction-45-imu.heading(degrees))*M_PI/180.0))));

//   // double No_odometry_move_linear_FL_V = speed*(FL_deg);
//   // double No_odometry_move_linear_BL_V = speed*(BL_deg);
//   // double No_odometry_move_linear_FR_V = -(speed*(FR_deg));
//   // double No_odometry_move_linear_BR_V = -speed*(BR_deg);

//   fix::fix_motor(No_odometry_move_linear_FL_V, No_odometry_move_linear_BL_V , No_odometry_move_linear_FR_V , No_odometry_move_linear_BR_V);
  
//   /*if (fix::LF_fix_V <= 0 && fix::LF_fix_V < 5){
//     fix::LF_fix_V *= 1.5;
//   }else if (fix::LF_fix_V >= 0 && fix::LF_fix_V > -5){
//     fix::LF_fix_V *= 1.5;
//   }
//   if (fix::LB_fix_V <= 0 && fix::LB_fix_V < 5){
//     fix::LB_fix_V *= 1.5;
//   }else if (fix::LB_fix_V >= 0 && fix::LB_fix_V > -5){
//     fix::LB_fix_V *= 1.5;
//   }
//   if (fix::RF_fix_V <= 0 && fix::RF_fix_V < 5){
//     fix::RF_fix_V *= 1.5;
//   }else if (fix::RF_fix_V >= 0 && fix::RF_fix_V > -5){
//     fix::RF_fix_V *= 1.5;
//   }
//   if (fix::RB_fix_V <= 0 && fix::RB_fix_V < 5){
//     fix::RB_fix_V *= 1.5;
//   }else if (fix::RF_fix_V >= 0 && fix::RF_fix_V > -5){
//     fix::RB_fix_V *= 1.5;
//   }*/
//   leftFront.setVelocity(fix::LF_fix_V,percent);
//   leftBack.setVelocity(fix::LB_fix_V,percent);
//   rightFront.setVelocity(fix::RF_fix_V,percent);
//   rightBack.setVelocity(fix::RB_fix_V,percent);

//   // printf("%f  %f  %f  %f\n",No_odometry_move_linear_FL_V,No_odometry_move_linear_BL_V,No_odometry_move_linear_FR_V,No_odometry_move_linear_BR_V);
//   // leftFront.setVelocity(No_odometry_move_linear_FL_V,percent);
//   // leftBack.setVelocity(No_odometry_move_linear_BL_V,percent);
//   // rightFront.setVelocity(No_odometry_move_linear_FR_V,percent);
//   // rightBack.setVelocity(No_odometry_move_linear_BR_V,percent);

//   leftFront.spin(forward);
//   leftBack.spin(forward);
//   rightFront.spin(forward);
//   rightBack.spin(forward);
//   // printf("%f,%f,%f,%f\n\n\n",speed*(sin(Direction+45-InerTial())*M_PI/180.0),speed*(cos(Direction+45-InerTial())*M_PI/180.0),speed*(cos(Direction+45-InerTial())*M_PI/180.0),speed*((sin(Direction+45-InerTial()))*M_PI/180.0));
//   // waitUntil(fabs(frontleft.position(degrees))>=degree||fabs(frontright.position(degrees))>=degree||fabs(backright.position(degrees))>=degree||fabs(backleft.position(degrees))>=degree); 
  
//   printf("Stage 0\n");
//   wait(30,msec);
//   }


//   leftFront.stop();
//   rightFront.stop();
//   leftBack.stop();
//   rightBack.stop();

// }

// 沒有 odometry 的移動 中場線 輸入 (底盤速度 方位 角度）
void noOdom::imu_No_odometry_move_linear(int speed,double Direction,double degree){
  leftFront.setPosition(0.0,degrees);
  rightFront.setPosition(0.0,degrees);
  leftBack.setPosition(0.0,degrees);
  rightBack.setPosition(0.0,degrees);

  double stableHeading = imu.heading(degrees);
  double adjustForwardSpeed =0.8;
  // double FL_deg = cos((Direction-45-Inertial.heading(degrees))*M_PI/180.0);
  // double BL_deg = 0.0-sin((Direction-45-Inertial.heading(degrees))*M_PI/180.0);
  // double FR_deg = sin((Direction-45-Inertial.heading(degrees))*M_PI/180.0);
  // double BR_deg = (0.0-cos((Direction-45-Inertial.heading(degrees))*M_PI/180.0));

  while(   fabs(leftFront.position(degrees)) <=degree 
        && fabs(rightFront.position(degrees))<=degree
        && fabs(leftBack.position(degrees))  <=degree
        && fabs(rightBack.position(degrees)) <=degree){
  // frontleft.setVelocity(speed*(cos((Direction-45-Inertial.heading(degrees))*M_PI/180.0)),percent);
  // backleft.setVelocity(speed*(0.0-sin((Direction-45-Inertial.heading(degrees))*M_PI/180.0)),percent);
  // frontright.setVelocity(-(speed*(sin((Direction-45-Inertial.heading(degrees))*M_PI/180.0))),percent);
  // backright.setVelocity(-speed*(((0.0-cos((Direction-45-Inertial.heading(degrees))*M_PI/180.0)))),percent);

  // printf("moving...");
  double Speed;
  if(degree <= 400.0){
      if(fabs(leftFront.position(degrees)) < degree/2.0 || fabs(leftBack.position(degrees)) < degree/2.0){
        Speed =  std::max((speed-5) * fabs(leftFront.position(degrees)) / (degree/2.0) +5, (speed-5) * fabs(leftBack.position(degrees)) / (degree/2.0) +5); 
      }else {
         Speed = std::max( speed * (degree - fabs(leftFront.position(degrees))) / 200.0 , speed * (degree - fabs(leftBack.position(degrees))) / 200.0); 
      }

  } else if( fabs(leftFront.position(degrees)) < 200.0 || fabs(leftBack.position(degrees)) < 200.0){
    Speed = std::max((speed) * fabs(leftFront.position(degrees)) -5/ 600.0 +5 ,(speed-5) * fabs(leftBack.position(degrees)) / 600.0 +5); 
  } else if( (degree - fabs(leftFront.position(degrees))) < 200.0 ){
    Speed = std::max(speed * fabs((degree - leftFront.position(degrees))) / 200.0 , speed * fabs((degree - leftBack.position(degrees))) / 200.0 ); 
  } else {
    Speed = speed;
  }

  if(fabs(Speed)<14.0)Speed = 14.0*Speed/fabs(Speed);

  Speed=speed;
  // Direction = 0.0;

  double adjustDegree =5.0;
  double No_odometry_move_linear_FL_V = Speed*adjustForwardSpeed* 1.3 * cos((Direction - 45-imu.heading(degrees))*M_PI/180.0);
  double No_odometry_move_linear_BL_V = Speed*adjustForwardSpeed* 1.3 * cos((Direction + 45-imu.heading(degrees))*M_PI/180.0);
  double No_odometry_move_linear_FR_V = Speed*adjustForwardSpeed* 1.3 * cos((Direction + 45-imu.heading(degrees))*M_PI/180.0);
  double No_odometry_move_linear_BR_V = Speed*adjustForwardSpeed* 1.3 * cos((Direction - 45-imu.heading(degrees))*M_PI/180.0);
  // printf("FL_V = %f | ",No_odometry_move_linear_FL_V);
  // printf("BL_V = %f | ",No_odometry_move_linear_BL_V);
  // printf("FR_V = %f | ",No_odometry_move_linear_FR_V);
  // printf("BR_V = %f\n",No_odometry_move_linear_BR_V);
  printf("heading %f\n",imu.heading());
  // L_F.setVelocity(((velocity_percent * 1.3) * cos(((dir - 45.0) - Inertial12.heading(degrees)) * M_PI / 180) + Target_Error), percent);
    // L_B.setVelocity(((velocity_percent * 1.3) * (0.0 - sin(((dir - 45.0) - Inertial12.heading(degrees)) * M_PI / 180)) + Target_Error), percent);
    // R_F.setVelocity(-(((velocity_percent * 1.3) * sin(((dir - 45.0) - Inertial12.heading(degrees)) * M_PI / 180) + Target_Error)), percent);
    // R_B.setVelocity(-(((velocity_percent * 1.3) * (0.0 - cos(((dir - 45.0) - Inertial12.heading(degrees)) * M_PI / 180)) + Target_Error)), percent);

  double No_odometry_move_rotation_FL_V =     Speed*(1-adjustForwardSpeed)*(noOdom::find_min_angle (stableHeading,imu.heading(degrees)))/adjustDegree;
  double No_odometry_move_rotation_BL_V =     Speed*(1-adjustForwardSpeed)*(noOdom::find_min_angle (stableHeading,imu.heading(degrees)))/adjustDegree;
  double No_odometry_move_rotation_FR_V = 0 - Speed*(1-adjustForwardSpeed)*(noOdom::find_min_angle (stableHeading,imu.heading(degrees)))/adjustDegree;
  double No_odometry_move_rotation_BR_V = 0 - Speed*(1-adjustForwardSpeed)*(noOdom::find_min_angle (stableHeading,imu.heading(degrees)))/adjustDegree;
  // double No_odometry_move_linear_FL_V = speed*(FL_deg);
  // double No_odometry_move_linear_BL_V = speed*(BL_deg);
  // double No_odometry_move_linear_FR_V = -(speed*(FR_deg));
  // double No_odometry_move_linear_BR_V = -speed*(BR_deg);

 

  leftFront.setVelocity (No_odometry_move_linear_FL_V + No_odometry_move_rotation_FL_V ,percent);
  leftBack.setVelocity  (No_odometry_move_linear_BL_V + No_odometry_move_rotation_BL_V ,percent);
  rightFront.setVelocity(No_odometry_move_linear_FR_V + No_odometry_move_rotation_FR_V ,percent);
  rightBack.setVelocity (No_odometry_move_linear_BR_V + No_odometry_move_rotation_BR_V ,percent);

  // leftFront.setVelocity(No_odometry_move_linear_FL_V,percent);
  // leftBack.setVelocity(No_odometry_move_linear_BL_V,percent);
  // rightFront.setVelocity(No_odometry_move_linear_FR_V,percent);
  // rightBack.setVelocity(No_odometry_move_linear_BR_V,percent);


  // printf("%f  %f  %f  %f\n",No_odometry_move_linear_FL_V,No_odometry_move_linear_BL_V,No_odometry_move_linear_FR_V,No_odometry_move_linear_BR_V);
  // printf("%f  %f  %f  %f\n",No_odometry_move_rotation_FL_V,No_odometry_move_rotation_BL_V,No_odometry_move_rotation_FR_V,No_odometry_move_rotation_BR_V);
  // leftFront.setVelocity(No_odometry_move_linear_FL_V,percent);
  // leftBack.setVelocity(No_odometry_move_linear_BL_V,percent);
  // rightFront.setVelocity(No_odometry_move_linear_FR_V,percent);
  // rightBack.setVelocity(No_odometry_move_linear_BR_V,percent);

  leftFront.spin(forward);
  leftBack.spin(forward);
  rightFront.spin(forward);
  rightBack.spin(forward);
  // printf("%f,%f,%f,%f\n\n\n",speed*(sin(Direction+45-InerTial())*M_PI/180.0),speed*(cos(Direction+45-InerTial())*M_PI/180.0),speed*(cos(Direction+45-InerTial())*M_PI/180.0),speed*((sin(Direction+45-InerTial()))*M_PI/180.0));
  // waitUntil(fabs(frontleft.position(degrees))>=degree||fabs(frontright.position(degrees))>=degree||fabs(backright.position(degrees))>=degree||fabs(backleft.position(degrees))>=degree); 
  
  wait(30,msec);
  }
  leftFront.stop();
  rightFront.stop();
  leftBack.stop();
  rightBack.stop();

}
