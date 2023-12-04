#include "vex.h"

using namespace vex;

int t = -1;
int speed = 0;
double Turret::autoAim_midline_Dis =0.0;

int Turret::spin_turret(){
  while (true){
    if (Controller1.ButtonLeft.pressing()){
      fort.spin(reverse);
      waitUntil(!Controller1.ButtonLeft.pressing());
      fort.stop();
    }else if (Controller1.ButtonRight.pressing()){
      fort.spin(forward);
      waitUntil(!Controller1.ButtonRight.pressing());
      fort.stop();
    }
  }
  return 0;
}

void Turret::turret_on(void){
  //printf("1\n");
  t = 0 - t;
}

int Turret::auto_aim_turret(){
  Vision.takeSnapshot(Vision__BLUEB); // identify if the object is blue high goal
  //int t = Brain.Timer.value() + 4; // a variable to record the time 4 seconds from now (i.e. Timer.value() = 1; t = 5)
  while (t == 1){ // if x-coordinate of high goal is not centered (centerX < 153
    Vision.takeSnapshot(Vision__BLUEB);  
    if (Controller1.ButtonL2.pressing()){
      waitUntil(!Controller1.ButtonL2.pressing());
      Turret::turret_on();
    }                                                                  // or centerX > 159) and Timer.value() is less than target time (t)...
    if (Vision.objects[0].centerX < 133 && fort.position(degrees) > -520 && Vision.objects[0].exists){ // if high goal is to the left of robot (centerX < 153)...
      //spinLeft(20); // rotate left
      //printf("2\n");
      speed = Vision.objects[0].centerX > 123 ? 18 : 55;
      fort.spin(reverse, speed, percent);
      if (fort.position(degrees) < -520){
        fort.stop();
      }
    }else if (Vision.objects[0].centerX > 139 && fort.position(degrees) < 910 && Vision.objects[0].exists){ // if high goal is to the right of robot (centerX > 159)...
      //spinRight(20); // rotate right
      //printf("3\n");
      speed = Vision.objects[0].centerX < 149 ? 18 : 55;
      fort.spin(forward, speed, percent);
      if (fort.position(degrees) > 910){
        fort.stop();
      }
    }else if ((Vision.objects[0].centerX > 133 || Vision.objects[0].centerX < 139)){
      //printf("1\n");
      fort.stop();
    }
  }
  // if aligned to the high goal, all motors cease movement
  return 0;
}

int Turret::recenter(){
  if (fort.position(degrees) < 0){
    while (fabs(fort.position(degrees)) > 10){
      fort.spin(forward);
    }
    fort.spin(forward, 20, percent);
  }else{
    while (fabs(fort.position(degrees)) > 10){
      fort.spin(reverse);
    }
    fort.spin(reverse, 20, percent);
  }
  waitUntil(fort.position(degrees) > -10 && fort.position(degrees) < 10);
  fort.stop();
  return 0;
}

int Turret::turretspin(){
  while (true){ 
    printf("1\n");
    if (Controller1.ButtonL2.pressing()){
      waitUntil(!Controller1.ButtonL2.pressing());
      Turret::turret_on();
    }
    if (t == 1){
      printf("2\n");
      Turret::auto_aim_turret();
    }else{
      printf("3\n");
      Turret::recenter();
    }
  }
  return 0;
}

double inherit;
bool Turret::LR_io = false;
double Turret::autoAim_midline_cm = 0;
double Turret::target;
bool firstRotation = true;
double Turret::LF_rotation=0;
double new_LF_rotation=0;
double Turret::tmpy_highgoal;
int Turret::autoAim_midline_task(){
    
    double delta = 0; //((DistanceFront.objectDistance(mm) - DistanceBack.objectDistance(mm)))/20;//離中線偏差距離(cm))
    double x_highgoal = 200.5 + delta;// 187.5 距離highgoal 垂直距離//190.5
    // double y_highgoal = 251.76 - delta - DistanceLeft.objectDistance(mm)/10;//距離highgoal 水平距離
    // double ErrorDis = DistanceLeft.objectDistance(mm);
    // double ErrorDis = 60.0; //60 cm 單位
    double ErrorDis = hypot(DisC::DR / 10, DisC::DB / 10);
    //printf("x = %f , y = %f",3650.0 - current_x , current_y);
    double fix = 2;
    double y_highgoal = 252.5 - ErrorDis;//距離highgoal 水平距離//249.76    258.055 - ErrorDis + fix
    printf("y_first = %f \n", ErrorDis);
    tmpy_highgoal = y_highgoal;//暫存highgoal
    // fort.setPosition(225,degrees);
    //fort.setVelocity(10, percent);
    //imuFort.setHeading(225, degrees);
    leftFront.setPosition(0,degrees);
    target = atan2(tmpy_highgoal, x_highgoal)*180/M_PI;
    
    // printf("",autoAim_midline_Dis)
    double aimKp = 10.0;
    // if(Turret::LR_io){
    //     aimKp = -1*aimKp;
    //   }
    // if(autoAim_midline_io){

    firstRotation = true;
    double firstRotation_time = Brain.Timer.value();
    double vel;
    while(!move_OK){
      // printf("%f\n", target);
      //printf("LFM=%.1f , Aim=%.1f | tmpy_highgoal=%f | target=%f \n",fabs(leftFront.position(degrees)),fabs(Turret::autoAim_midline_cm*360.0)/(M_PI*3.25*2.54), tmpy_highgoal, target);
      //printf("%f\n", Turret::autoAim_midline_cm);
      target = atan2(tmpy_highgoal,x_highgoal)*180/M_PI;
     // printf("tmpy_highgoal = %f\n", tmpy_highgoal);
      autoAim_midline_Dis=hypot(tmpy_highgoal,x_highgoal);
      // printf("target= %.1f|",target);
      // printf("fort= %.1f\n",fort.position(degrees)/7.0);
      //Chassis::setVel(30,-30,-30,30);//底盤速度定速
      // if(fort.position(degrees) < (target+1)*8 && fort.position(degrees) > (target-1)*8 ){//砲台角度調整
      //     fort.stop();
      // }else{
        // fort.setVelocity(80*(target - fort.position(degrees))/(fabs(target - fort.position(degrees))),percent);
      // if (leftFront.position(degrees) > (Turret::autoAim_midline_cm*360.0)/(M_PI*3.25*2.54) - 200){
      //   target += 40;
      // }

      
      // fort.setVelocity((fabs(target) - fabs(fort.position(degrees)/7.0))*aimKp,percent);
      // fort.spin(forward);
      // }
      if (Brain.Timer.value() > firstRotation_time + 0.6 ){//2.0//1.2
        firstRotation = false; 
      }
      // if (imuFort.heading(degrees) <= 225 && firstRotation == false){
        // break;
      // s}
      // if (imuFort.heading(degrees) > target + 225){
      //   //printf("1\n");
        // vel = fabs(imuFort.heading(degrees) - (target + 225)) < 6 ? -6 : (target + 225) - imuFort.heading(degrees);
        vel =  (target + 225) - imuFort.heading(degrees);
        fort.spin(forward, 7*vel+20.1, percent);//19.7
        // fort.spin(forward, 7*vel-10, percent);
        /*if (imuFort.heading(degrees) - (target + 225) < 10){
          fort.spin(forward,-6, percent);
        }else{
          fort.spin(forward,-15, percent);
        }*/
      // }else{
      //   fort.spin(forward, 90, percent);
      // }leftFront.position(degrees)

      // if(Turret::LR_io){
      //   tmpy_highgoal = fabs(leftFront.position(degrees)*(M_PI*3.25*2.54)/360)*1.3514;
      // }else{
        new_LF_rotation=Turret::LF_rotation+leftFront.position(degrees);
        tmpy_highgoal = y_highgoal - (new_LF_rotation*(M_PI*3.25*2.54)/360)*1.3514;//水平距離更新
        // tmpy_highgoal = y_highgoal - (leftFront.position(degrees)*(M_PI*3.25*2.54)/360)*1.3514;
      // }
      // if(!autoAim_midline_io){
      //   break;
      // }
      //printf("AimIng...\n");
      wait(5,msec);
    }
    fort.stop(hold);
    // }
    Turret::LR_io = false;
    // autoAim_midline_io = false;
    leftFront.setPosition(0,degrees);
    inherit = tmpy_highgoal;
    Chassis::setVel(0, 0, 0, 0);
    printf("Aim Done...################################################################################\n");
    // Chassis::setStop(hold);
    // Chassis::groupStop();

    return 0;
}


bool Turret::LR_io_2 = false;
double Turret::autoAim_midline_cm_2 = 0;
bool secondRotation = true;
int Turret::autoAim_midline_task_2(){
    move_OK =false;
    double delta = 0; //((DistanceFront.objectDistance(mm) - DistanceBack.objectDistance(mm)))/20;//離中線偏差距離(cm))
    double x_highgoal = 200.5 + delta;// 187.5 距離highgoal 垂直距離//190.5
    // double y_highgoal = 251.76 - delta - DistanceLeft.objectDistance(mm)/10;//距離highgoal 水平距離
    // double ErrorDis = DistanceLeft.objectDistance(mm);
    // double ErrorDis = 60.0; //60 cm 單位
    double ErrorDis = hypot(current_x,current_y); 
    double y_highgoal = (-inherit - (((rightBack.position(degrees)+leftFront.position(degrees))/2))*(M_PI*3.25*2.54)/360)*1.414;//距離highgoal 水平距離//249.76    254.0 - delta - ErrorDis
     y_highgoal = -10;//-30
    printf("y_highgoal = %f\n",y_highgoal);
    printf("inherit = %f\n",inherit);
    double tmpy_highgoal = y_highgoal;//暫存highgoal
    // fort.setPosition(225,degrees);
    // fort.setVelocity(10, percent);
    //imuFort.setHeading(225, degrees);
    leftFront.setPosition(0,degrees);
    double target = atan( tmpy_highgoal / x_highgoal)*180/M_PI;
    
    // printf("",autoAim_midline_Dis)
    double aimKp = 10.0;
    // if(Turret::LR_io){
    //     aimKp = -1*aimKp;
    //   }
    // if(autoAim_midline_io){

    // printf("1\n");
    secondRotation = true;
    double secondRotation_time = Brain.Timer.value();
    double vel;
    while(!move_OK){
      // printf("%f\n", imuFort.heading(degrees));
      //printf("LFM=%.1f , Aim=%.1f | tmpy_highgoal=%f | target=%f \n",fabs(leftFront.position(degrees)),fabs(Turret::autoAim_midline_cm*360.0)/(M_PI*3.25*2.54), tmpy_highgoal, target);
      //printf("%f\n", Turret::autoAim_midline_cm);
      target = atan2(tmpy_highgoal,x_highgoal)*180/M_PI;
      autoAim_midline_Dis=hypot(tmpy_highgoal,x_highgoal);
      // printf("target= %.1f|",target);
      // printf("fort= %.1f\n",fort.position(degrees)/7.0);
      //Chassis::setVel(30,-30,-30,30);//底盤速度定速
      // if(fort.position(degrees) < (target+1)*8 && fort.position(degrees) > (target-1)*8 ){//砲台角度調整
      //     fort.stop();
      // }else{
        // fort.setVelocity(80*(target - fort.position(degrees))/(fabs(target - fort.position(degrees))),percent);
      // if (leftFront.position(degrees) > (Turret::autoAim_midline_cm*360.0)/(M_PI*3.25*2.54) - 200){
      //   target += 40;
      // }

      
      // fort.setVelocity((fabs(target) - fabs(fort.position(degrees)/7.0))*aimKp,percent);
      // fort.spin(forward);
      // }
      if (imuFort.heading(degrees) <=0 && secondRotation == false){
        break;
      }
      // if (imuFort.heading(degrees) > target + 45){
    
        // seßcondRotation = false;
      if (Brain.Timer.value() > secondRotation_time + 0.1){//2.0
        secondRotation = false;
      }
        
        // vel = fabs(imuFort.heading(degrees) - (target + 45)) < 6 ? 6 : (target + 45) - imuFort.heading(degrees);
        vel =  (target + 45) - imuFort.heading(degrees)  ;
        // printf("vel = %f\n", vel);
        //printf("tmpy_highgoal = %f\n",tmpy_highgoal);
        fort.spin(forward, 6*vel+10, percent);
        // fort.spin(forward, 6*vel+7, percent);
        /*if (imuFort.heading(degrees) - (target + 225) < 10){
          fort.spin(forward,-6, percent);
        }else{
          fort.spin(forward,-15, percent);
      //   }*/
      // }else{
      //   fort.spin(forward, 90, percent);
      // }

      // if(Turret::LR_io){
      //   tmpy_highgoal = fabs(leftFront.position(degrees)*(M_PI*3.25*2.54)/360)*1.3514;
      // }else{
        new_LF_rotation=Turret::LF_rotation+leftFront.position(degrees);
        tmpy_highgoal = y_highgoal - (new_LF_rotation*(M_PI*3.25*2.54)/360)*1.3514;//水平距離更新
        // tmpy_highgoal = y_highgoal - (leftFront.position(degrees)*(M_PI*3.25*2.54)/360)*1.3514;
      // }
      // if(!autoAim_midline_io){
      //   break;
      // }
      //printf("AimIng...\n");
      wait(5,msec);
    }
    fort.stop(hold);
    // }
    Turret::LR_io = false;
    // autoAim_midline_io = false;
    Chassis::setVel(0, 0, 0, 0);
    printf("Aim Done...\n");
    // Chassis::setStop(hold);
    // Chassis::groupStop();

    return 0;
}



// LR L<<false;R<<true
void Turret::autoAim_midline(double cm,bool LR,bool Wait){
  Turret::autoAim_midline_cm=cm*1.051;
  // autoAim_midline_io = true;
  Turret::LR_io=LR;
  if (Wait){
  printf("Turret\n");
  task autoAim_midline_tasks= task(Turret::autoAim_midline_task);
  }else{
    Turret::autoAim_midline_task();
  }   

}

void Turret::autoAim_midline_2(double cm ,bool LR,bool Wait){
  Turret::autoAim_midline_cm_2=cm*1.051;
  // autoAim_midline_io = true;
  Turret::LR_io_2=LR;
  if (Wait){
  printf("Turret\n");
  task autoAim_midline_tasks= task(Turret::autoAim_midline_task_2);
  }else{
     Turret::autoAim_midline_task_2();
  }   
}

