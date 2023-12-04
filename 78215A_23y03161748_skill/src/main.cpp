/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\cslee                                            */
/*    Created:      Sat Aug 27 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// encoderLeft          encoder       C, D            
// encoderRight         encoder       A, B            
// encoderBack          encoder       E, F            
// imu                  inertial      9               
// Controller1          controller                    
// leftFront            motor         11              
// leftBack             motor         2               
// rightFront           motor         12              
// rightBack            motor         4               
// roller               motor         8               
// shoot_air            digital_out   H               
// R_shoot              motor         7               
// L_shoot              motor         6               
// expand               digital_out   G               
// Vision               vision        15              
// imuFort              inertial      1               
// fort                 motor         5               
// DistanceFront        distance      19              
// DistanceBack         distance      20              
// DistanceLeft         distance      18              
// DistanceRight        distance      17              
// Intake               motor         3               
// Color                optical       13              
// discNumber           distance      10              
// IntakeRight          optical       16              
// IntakeLeft           optical       14              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;


double newInertial;
double Inertial_Error0;
bool move_OK; 
bool rotate_OK;
bool Inertial_Switch_Initial = false;

// New Inertial Value
int iHeading(){
  while(true){
    if(Inertial_Switch_Initial){
      newInertial = fmod(Inertial_Error0+imu.heading()+0.0001,360.0);
    }else{
      newInertial = fmod(imu.heading()- Inertial_Error0+0.0001,360.0) ;
    }
   
  }
  return 0;
}
//=============== 基本馬達底盤 函式 ====================
void Initialize(double deg){

  // 慣性感測器-校正開始
  imu.calibrate();
  wait(3,sec);
  imu.setHeading(0.0,degrees);
  wait(100,msec);
  printf("Inertial value = %f\n",imu.heading());
  wait(20,msec);
  if(imu.heading()>180.0){
    Inertial_Error0 = 360.0 - imu.heading();
    Inertial_Switch_Initial = true;
  }else{
    Inertial_Error0 = imu.heading();
    Inertial_Switch_Initial = false;
  }
  if(Inertial_Switch_Initial){
    newInertial = fmod(Inertial_Error0+imu.heading()+0.0001,360.0);
  }else{
    newInertial = fmod(imu.heading()- Inertial_Error0+0.0001,360.0) ;
  }
  imu.setHeading(deg,degrees);
  task iHeadingtask = task(iHeading);
  //("Inertial value = %f |",imu.heading());
  //("newInertial value = %f\n",newInertial);
  // 慣性感測器-校正結束
  printf("Calibrate Done !\n");
  wait(20,msec);
}

double startAngle ;
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Initialize(startAngle);
  printf("Initialize_OK\n");

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void expansion(){
  if (Brain.Timer.value() >= 95){
    expand.set(true);
    wait(300, msec);
    expand.set(false);
  }
}

void shoot_Twice(){
  /*leftFront.setVelocity(50,percent);
  leftBack.setVelocity(50,percent);
  rightFront.setVelocity(50,percent);
  rightBack.setVelocity(50,percent);*/
  Shoot::auto_shoot_air();

  leftFront.spin(reverse,50,percent);
  leftBack.spin(reverse,50,percent);
  rightFront.spin(reverse,50,percent);
  rightBack.spin(reverse,50,percent);
  wait(1.5,sec);
  /*leftFront.spinFor(reverse,400,degrees);
  leftBack.spinFor(reverse,400,degrees);
  rightFront.spinFor(reverse,400,degrees);
  rightBack.spinFor(reverse,400,degrees);*/

  leftFront.stop();
  leftBack.stop();
  rightFront.stop();
  rightBack.stop();
}

void spinLeft(int speed){
  leftFront.spin(reverse, speed, percent);
  rightFront.spin(forward, speed, percent);
  leftBack.spin(reverse, speed, percent);
  rightBack.spin(forward, speed, percent);
}

void spinRight(int speed){
  leftFront.spin(forward, speed, percent);
  rightFront.spin(reverse, speed, percent);
  leftBack.spin(forward, speed, percent);
  rightBack.spin(reverse, speed, percent);
}

void autoAim(){
  Vision.takeSnapshot(Vision__BLUEB); // identify if the object is blue high goal
  int t = Brain.Timer.value() + 4; // a variable to record the time 4 seconds from now (i.e. Timer.value() = 1; t = 5)
  while ((Vision.objects[0].centerX < 153 || Vision.objects[0].centerX > 159) && Brain.Timer.value() <= t){ // if x-coordinate of high goal is not centered (centerX < 153
    Vision.takeSnapshot(Vision__BLUEB);                                                                     // or centerX > 159) and Timer.value() is less than target time (t)...
    if (Vision.objects[0].centerX < 153){ // if high goal is to the left of robot (centerX < 153)...
      spinLeft(20); // rotate left
    }else if (Vision.objects[0].centerX > 159){ // if high goal is to the right of robot (centerX > 159)...
      spinRight(20); // rotate right
    }
  }
  // if aligned to the high goal, all motors cease movement
  leftFront.stop(brake);
  rightFront.stop(brake);
  leftBack.stop(brake);
  rightBack.stop(brake);
}

void driving(void){
  Brain.resetTimer();
  Controller1.Screen.clearScreen();
  fort.setStopping(hold);
  fort.stop();
  task collect_taking(Collect::taking_task);
  task shooting_task(Shoot::shooting); // flywheel activation (at the start of match)
  task shoot_air_on_task(Shoot::shoot_air_on); // if ButtonR1 pressed, launch disk
  //task facing_highgoal_task(highgoal::facing_highgoal_driving);// automatically face highgoal (currently not used)
  task spin_roller(Roller::roller_spin_blue_task); // automatically spin roller to blue
  Controller1.ButtonY.pressed(shoot_Twice); // fire two disks simultaneously (currently not used)
  Controller1.ButtonLeft.pressed(expansion); // activate horizontal expansion (used in endgame)
  while(true){
    // Vision.takeSnapshot(Vision__BLUEB);
    //("%d\n", Vision.objects[0].centerX);
    // //("%f\n", Color.hue());
    // if (Controller1.ButtonRight.pressing()){ // if ButtonRight is pressed...
    //   waitUntil(!Controller1.ButtonRight.pressing());
    //   autoAim(); // automatically align to blue high goal
    // }else{ // if ButtonRight is not pressed...
    //   Chassis::opControl(); // resume driver control
    // }
    //("%f\n", discnumber.hue());
    Chassis::opControl();
    //spinB();// activate roller motor (at the start of match)
    // Collect::taking();// if ButtonX pressed, activate intake mechanism
    wait(50,msec);
  }
}




// FUNCTION SS



double leftFront_Error,leftBack_Error,rightFront_Error,rightBack_Error;
double leftFront_ErrorLast,leftBack_ErrorLast,rightFront_ErrorLast,rightBack_ErrorLast;
double straight_Kp = 0.03,straight_Kd =0.1;
double leftFront_V,leftBack_V,rightFront_V,rightBack_V;
double LRDegError;
double LDV,RDV,DVKp=0.001;
double VoutleftFront,VoutleftBack,VoutrightFront,VoutrightBack;

int StraightPID(){
  while(true){

   
    LDV=fabs(leftBack.position(degrees))+fabs(rightFront.position(degrees))/2.0;
    RDV=fabs(leftFront.position(degrees))+fabs(rightBack.position(degrees))/2.0;
    LRDegError = LDV-RDV;
    //("Diff=%.2f | ",LRDegError);

    leftFront_V=leftFront_V+(LRDegError*DVKp);
    leftBack_V=leftBack_V-(LRDegError*DVKp);
    rightFront_V=rightFront_V-(LRDegError*DVKp);
    rightBack_V=rightBack_V+(LRDegError*DVKp); 
    
    //("PID\n");
    leftFront_Error = (180.0 *  leftFront_V) - (leftFront.voltage(vex::voltageUnits::mV));
    leftBack_Error = (180.0 *  leftBack_V) - (leftBack.voltage(vex::voltageUnits::mV));
    rightFront_Error = (180.0 *  rightFront_V) - (rightFront.voltage(vex::voltageUnits::mV));
    rightBack_Error = (180.0 *  rightBack_V) - (rightBack.voltage(vex::voltageUnits::mV));

    VoutleftFront = (180.0 * leftFront_V) + ( leftFront_Error * straight_Kp )  +  ((leftFront_Error - leftFront_ErrorLast) * straight_Kd ) ;
    VoutleftBack = (180.0 * leftBack_V) + ( leftBack_Error * straight_Kp )  +  ((leftBack_Error - leftBack_ErrorLast) * straight_Kd ) ;
    VoutrightFront = (180.0 * rightFront_V) + ( rightFront_Error * straight_Kp )  +  ((rightFront_Error - rightFront_ErrorLast) * straight_Kd ) ;
    VoutrightBack = (180.0 * rightBack_V) + ( rightBack_Error * straight_Kp )  +  ((rightBack_Error - rightBack_ErrorLast) * straight_Kd ) ;

    //("%.2f |",VoutleftFront);
    //("%.2f |",VoutleftBack);
    //("%.2f |",VoutrightFront);
    //("%.2f |\n",VoutrightBack);

    leftFront.spin(fwd,VoutleftFront,voltageUnits::mV);
    leftBack.spin(fwd,VoutleftBack,voltageUnits::mV);
    rightFront.spin(fwd,VoutrightFront,voltageUnits::mV);
    rightBack.spin(fwd,VoutrightBack,voltageUnits::mV);


    leftFront_ErrorLast = leftFront_Error;
    leftBack_ErrorLast = leftBack_Error;
    rightFront_ErrorLast = rightFront_Error;
    rightBack_ErrorLast = rightBack_Error;

    wait(20,msec);
  }
  return 0;
}



void set_Drivetrain_V_(double VleftFront,double VleftBack
                      ,double VrightFront,double VrightBack){
  // leftFront.voltage(vex::voltageUnits::mV);
  leftFront.spin(forward);
  leftBack.spin(forward);
  rightFront.spin(forward);
  rightBack.spin(forward);

  leftFront_V = VleftFront ;
  leftBack_V = VleftBack ;
  rightFront_V = VrightFront ;
  rightBack_V = VrightBack ;

  task StraightPIDtask=task(StraightPID);

  // leftFront.spin(fwd,VoutleftFront,voltageUnits::mV);
  // leftBack.spin(fwd,VoutleftBack,voltageUnits::mV);
  // rightFront.spin(fwd,VoutrightFront,voltageUnits::mV);
  // rightBack.spin(fwd,VoutrightBack,voltageUnits::mV);

}

void set_Drivetrain_V_org(double VleftFront,double VleftBack,double VrightFront,double VrightBack){
  // leftFront.voltage(vex::voltageUnits::mV);
  leftFront.spin(fwd,180.0*VleftFront,voltageUnits::mV);
  leftBack.spin(fwd,180.0*VleftBack,voltageUnits::mV);
  rightFront.spin(fwd,180.0*VrightFront,voltageUnits::mV);
  rightBack.spin(fwd,180.0*VrightBack,voltageUnits::mV);

}

double Vst;
double Delay = 20.0; //msec
void set_Drivetrain_V(double VleftFront,double VleftBack,double VrightFront,double VrightBack){
  Vst = 0.0;
  double time0 = Brain.Timer.value();
  double AccTime = 1.0 ;  //sec

  if (VleftFront == VrightFront && fabs(VleftFront)>30.0 ){
    if(fabs(VleftFront)>0){
      while(Brain.Timer.value()< time0+AccTime && fabs(VleftFront)>=Vst){
        if(VleftFront>0){
          Vst +=8.0;
        }else{
          Vst -=8.0;
        }
        leftFront.spin(fwd,180.0*(Vst+0.395),voltageUnits::mV);
        leftBack.spin(fwd,180.0*Vst,voltageUnits::mV);
        rightFront.spin(fwd,180.0*Vst,voltageUnits::mV);
        rightBack.spin(fwd,180.0*(Vst+0.395),voltageUnits::mV);
        
        //("Acc..\n");
        wait(Delay,msec);
        
      } 
      VleftFront =(Vst+0.39);
      VleftBack =Vst;
      VrightFront =Vst;
      VrightBack =(Vst+0.39); 

    }
  } 


  leftFront.spin(fwd,180*VleftFront,voltageUnits::mV);
  leftBack.spin(fwd,180*VleftBack,voltageUnits::mV);
  rightFront.spin(fwd,180*VrightFront,voltageUnits::mV);
  rightBack.spin(fwd,180*VrightBack,voltageUnits::mV);

}

void DrivetrainStop_mode(int mode){ // mode = 1,coast mode = 2,hold
  if(mode == 1){
    leftFront.setStopping(coast);
    leftBack.setStopping(coast);
    rightFront.setStopping(coast);
    rightBack.setStopping(coast); 
  }
  if(mode == 2){
    leftFront.setStopping(hold);
    leftBack.setStopping(hold);
    rightFront.setStopping(hold);
    rightBack.setStopping(hold);    
  }
}


void DrivetrainSpin(){
  printf("DrivetrainSpin...\n");
  leftFront.spin(forward);
  leftBack.spin(forward);
  rightFront.spin(forward);
  rightBack.spin(forward);
}

void DrivetrainStop(){
  leftFront.stop();
  leftBack.stop();
  rightFront.stop();
  rightBack.stop();
}


// 旋轉至目標角度(輸入 目標角度)
double Heading_Turn_abs_degree;
double kp = 2.1;

double  Target_Error;
double Reach_Target_Error;

int Heading_Turn_abs_() {
  // 目標角度誤差 資訊設定 （degree目標角度-（當前角度與前一刻角度誤差））
  Target_Error = Heading_Turn_abs_degree - imu.rotation(degrees);
  // 與目標之誤差設定為0
  Reach_Target_Error = 0.0;
  while (true) {
    set_Drivetrain_V((kp * Target_Error),(kp * Target_Error),-(kp * Target_Error),-(kp * Target_Error));
    Target_Error = Heading_Turn_abs_degree - imu.rotation(degrees);
    if (fabs(static_cast<float>(Target_Error)) < 1.0) {
      break;
    }  
  }
  // 底盤停止
  DrivetrainStop();
  wait(0.1, seconds);
  return 0;
}

void Heading_Turn_abs(double degree,bool wait){
  Heading_Turn_abs_degree = degree;
  if(wait){
    task Heading_Turn_abs_task = task(Heading_Turn_abs_);
  }else{
    Heading_Turn_abs_();
  }
}


//=====Andy==== Initialization Inertial =======
// 旋轉至目標角度(輸入 目標角度)

double Heading_Turn_abs_Degree;
double Target_Error_D,Target_Error_D_last;
double Kp_Turn = 2.2, Kd_Turn = 1.0;
double VLTurn,VRTurn;
double minV = 5.0;
double Timeout_turn = 4.0;


int Heading_Turn_abs_D_task() {
  double iAcc = 0.0 ,jAcc = 0.0;
  bool iAcc_io = false;
  // 目標角度誤差 資訊設定 （degree目標角度-（當前角度與前一刻角度誤差））
  Target_Error_D = Heading_Turn_abs_Degree - newInertial;
  if(fabs(Target_Error_D)>180.0){
    if(Target_Error_D < 0.0){
      Target_Error_D = Target_Error_D + 360.0;
    }else{
      Target_Error_D = Target_Error_D - 360.0;
    }
  }
  // 與目標之誤差設定為0
  Reach_Target_Error = 0.0;
  double time0 = Brain.Timer.value();
  
  // while (fabs(Target_Error_D) > 0.1 && Brain.Timer.value()<(time0+Timeout_turn)) {
  while ( Brain.Timer.value()<(time0+Timeout_turn)) {
    //("newInertial = %.2f |\n",newInertial );
    Target_Error_D = Heading_Turn_abs_Degree - newInertial;

  
    if(fabs(Target_Error_D)>=180.0){
      if(Target_Error_D < 0.0){
        Target_Error_D = Target_Error_D + 360.0 ;
      }else{
        Target_Error_D = Target_Error_D - 360.0 ;
      }
    }
   
    VLTurn =    Kp_Turn * Target_Error_D + (Target_Error_D - Target_Error_D_last) * Kd_Turn ;
    VRTurn = 0-(Kp_Turn * Target_Error_D + (Target_Error_D - Target_Error_D_last) * Kd_Turn);

    if(fabs(Target_Error_D)<1.0){
      if(VLTurn>0){
        VLTurn = minV;
        VRTurn = 0.0-minV;
      }else{
        VLTurn = 0.0-minV;
        VRTurn = minV;
      }
    }
    set_Drivetrain_V( VLTurn, VLTurn, VRTurn, VRTurn);
    //("Error =%f |",Target_Error_D);
    wait(30,msec);
   if(fabs(Target_Error_D) < 0.7){
     iAcc_io = true;
     iAcc += 1.0;
     if( iAcc/jAcc > 0.5 && jAcc >3.0){
      //  printf("ok|");
       wait(10,msec);
       break;
     }
   }
   if( iAcc_io){
    //  printf("st |");
     jAcc += 1.0;
   }
   if(jAcc>100){
      iAcc = 0.0 ;
      jAcc = 0.0;
      iAcc_io = false;
   }

   Target_Error_D_last = Target_Error_D;
  //  printf("%f %% \n",iAcc/jAcc);
  }
  // 底盤停止
  DrivetrainStop();
  printf("Turn Done!\n");
  wait(0.1, seconds);
  return 0;
}

void Heading_Turn_abs_D(double degree,bool wait){
  if(degree<0.0){
    degree = degree + 360.0;
  }
  Heading_Turn_abs_Degree = degree;
  if(wait){
    task Heading_Turn_abs_task = task(Heading_Turn_abs_D_task);
  }else{
    Heading_Turn_abs_D_task();
  }
}

double temp_phi;
double velocity_percent;
double temp_time;

// ======== 機器人移動 邊走邊轉 最終面向角度（速度,移動秒數,移動方向角度,最終面向方位角度） ==========
void move_rotate_v_sec_direction_ht_degree(double v, double secs, double dir, double ht_degree) {
  // move_OK = false;
  double Target_Error_D = 0.0;
  temp_phi = ht_degree;
  velocity_percent = v;
  
  bool turn_io = false;
  DrivetrainSpin();
  temp_time = Brain.Timer.time(sec);
  while (!((Brain.Timer.time(sec) - temp_time > secs) && turn_io )){
    //("turning...\n");
    Target_Error_D = temp_phi - newInertial;
    if(fabs(Target_Error_D)>180.0){
      if(Target_Error_D < 0.0){
        Target_Error_D = Target_Error_D + 360.0;
      }else{
        Target_Error_D = Target_Error_D - 360.0;
      }
    }
    if(fabs(Target_Error_D)>60.0){
      if(Target_Error_D >0.0){
        Target_Error_D = 60.0;
      }else{
        Target_Error_D = 0.0-60.0;
      }
    }
    //("TarError=%f |",Target_Error_D);
    leftFront.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D * 1.5), percent);
    leftBack.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D * 1.5), percent);
    rightFront.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D * 1.5), percent);
    rightBack.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D * 1.5), percent);
    
    //("V=%.2f\n",(velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D));
    
    if (Brain.Timer.time(sec) - temp_time > secs) {
      velocity_percent = 0.0;
    }  
    if(Target_Error_D<1.5){
      turn_io = true;
    }
  }
  // move_OK = true;
  printf("Turn Done!\n");
  DrivetrainStop();
}

void move_rotate_v_sec_direction_ht_degree_load(double v, double secs, double dir, double ht_degree) {
  // move_OK = false;
  double Target_Error_D = 0.0;
  temp_phi = ht_degree;
  velocity_percent = v;
  
  bool turn_io = false;
  DrivetrainSpin();
  temp_time = Brain.Timer.time(sec);
  while (!((Brain.Timer.time(sec) - temp_time > secs))){
    //("turning...\n");
    Target_Error_D = temp_phi - newInertial;
    if(fabs(Target_Error_D)>180.0){
      if(Target_Error_D < 0.0){
        Target_Error_D = Target_Error_D + 360.0;
      }else{
        Target_Error_D = Target_Error_D - 360.0;
      }
    }
    if(fabs(Target_Error_D)>50.0){
      if(Target_Error_D >0.0){
        Target_Error_D = 50.0;
      }else{
        Target_Error_D = 0.0-50.0;
      }
    }
    //("TarError=%f |",Target_Error_D);
    leftFront.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D), percent);
    leftBack.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D), percent);
    rightFront.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D), percent);
    rightBack.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D), percent);
    
    //("V=%.2f\n",(velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D));
    
    if (Brain.Timer.time(sec) - temp_time > secs) {
      velocity_percent = 0.0;
    }  
    if(Target_Error_D<1.5){
      turn_io = true;
    }
  }
  // move_OK = true;
  printf("Turn Done!\n");
  DrivetrainStop();
}

bool less_cm=false;
// ======= 機器人移動距離面向角度 (速度,移動距離,機器人移動方向,最終面向方位角度)=========
void move_v_cm_direction_ht_degrees(double v, double _cm, double dir, double ht_degree) {
  
  if(_cm<20.0){
    less_cm = true;
  }else{
    less_cm = false;
  }



  if(fabs(dir)==45||fabs(dir)==135||fabs(dir)==225||fabs(dir)==315){
    _cm=_cm*1.0;
  }

  if(fabs(dir)==0||fabs(dir)==90||fabs(dir)==180||fabs(dir)==270){
    _cm=_cm*0.877;
  }

  if(fabs(dir)==315 && fabs(ht_degree)==225){
    _cm=_cm*1.0537;
  }

  if(fabs(dir)==315 && fabs(ht_degree)==45){
    _cm=_cm*1.0537;
  }

  
  // 起步距離15cm + 煞車距離45cm = 60cm
  leftFront.setRotation(0.0, degrees);
  rightFront.setRotation(0.0, degrees);
  leftBack.setRotation(0.0, degrees);
  rightBack.setRotation(0.0, degrees);
  velocity_percent = 0.0;
  // move_OK = false;
  Target_Error = 0.0;
  temp_phi = ht_degree;
  // direction = phi
  leftFront.setStopping(hold);
  rightFront.setStopping(hold);
  leftBack.setStopping(hold);
  rightBack.setStopping(hold);
  double Movetime0 = Brain.Timer.value();


  if(less_cm){
    //("lesscm10\n");
      while (( (fabs(static_cast<float>(leftFront.rotation(degrees))) + 
                 fabs(static_cast<float>(leftBack.rotation(degrees))) + 
                 fabs(static_cast<float>(rightFront.rotation(degrees))) + 
                 fabs(static_cast<float>(rightBack.rotation(degrees))) ) / 4.0)
                < fabs(360.0 * _cm)/(M_PI*3.25*2.54) && Movetime0+1.5 > Brain.Timer.value()) {
      // 煞車距離44cm
      velocity_percent  = 12.0;
      leftFront.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D), percent);
      leftBack.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D), percent);
      rightFront.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D), percent);
      rightBack.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D), percent);

      // leftFront.setVelocity(((velocity_percent * 1.3) * cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error), percent);
      // leftBack.setVelocity(((velocity_percent * 1.3) * (0.0 - sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error), percent);
      // rightFront.setVelocity(-(((velocity_percent * 1.3) * sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error)), percent);
      // rightBack.setVelocity(-(((velocity_percent * 1.3) * (0.0 - cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error)), percent);
      leftFront.spin(forward);
      leftBack.spin(forward);
      rightFront.spin(forward);
      rightBack.spin(forward);

      //("cm...\n");
    //wait(5, msec);
    }


  }else{
    while (!((fabs(static_cast<float>(leftFront.rotation(degrees))) + (fabs(static_cast<float>(leftBack.rotation(degrees))) + (fabs(static_cast<float>(rightFront.rotation(degrees))) + fabs(static_cast<float>(rightBack.rotation(degrees)))))) / 4.0 > fabs(static_cast<float>((510.0 / 50.0) * 30.0)))) {
      // 經過計算，前15公分可以加速到100%
        //("turning...\n");
    Target_Error_D = temp_phi - newInertial;
    if(fabs(Target_Error_D)>180.0){
      if(Target_Error_D < 0.0){
        Target_Error_D = Target_Error_D + 360.0;
      }else{
        Target_Error_D = Target_Error_D - 360.0;
      }
    }
    if(fabs(Target_Error_D)>60.0){ //50
      if(Target_Error_D >0.0){
        Target_Error_D = 60.0;
      }else{
        Target_Error_D = 0.0-60.0;
      }
    }
      leftFront.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D), percent);
      leftBack.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D), percent);
      rightFront.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D), percent);
      rightBack.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D), percent);
      leftFront.spin(forward);
      leftBack.spin(forward);
      rightFront.spin(forward);
      rightBack.spin(forward);
      if (velocity_percent < v) {
        velocity_percent = velocity_percent + 2.5 * (Brain.Timer.time(sec)-Movetime0);
      }
    //wait(5, msec);
    }
    Movetime0 = Brain.Timer.value();
    while (!((fabs(static_cast<float>(leftFront.rotation(degrees))) + (fabs(static_cast<float>(leftBack.rotation(degrees))) + (fabs(static_cast<float>(rightFront.rotation(degrees))) + fabs(static_cast<float>(rightBack.rotation(degrees)))))) / 4.0 > fabs(static_cast<float>((510.0 / 50.0) * (_cm - 45.0))))) {
        //("turning...\n");
    Target_Error_D = temp_phi - newInertial;
    if(fabs(Target_Error_D)>180.0){
      if(Target_Error_D < 0.0){
        Target_Error_D = Target_Error_D + 360.0;
      }else{
        Target_Error_D = Target_Error_D - 360.0;
      }
    }
    if(fabs(Target_Error_D)>60.0){
      if(Target_Error_D >0.0){
        Target_Error_D = 60.0;
      }else{
        Target_Error_D = 0.0-60.0;
      }
    }
      leftFront.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D), percent);
      leftBack.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D), percent);
      rightFront.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D), percent);
      rightBack.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D), percent);
      leftFront.spin(forward);
      leftBack.spin(forward);
      rightFront.spin(forward);
      rightBack.spin(forward);
      if (velocity_percent < v) {
        velocity_percent = velocity_percent + 3.0 * (Brain.Timer.time(sec)-Movetime0);
      }
    //wait(5, msec);
    }

    Movetime0 = Brain.Timer.value();
    while (!((fabs(static_cast<float>(leftFront.rotation(degrees))) 
    + (fabs(static_cast<float>(leftBack.rotation(degrees))) 
    + (fabs(static_cast<float>(rightFront.rotation(degrees))) 
    + fabs(static_cast<float>(rightBack.rotation(degrees)))))) / 4.0 > fabs(static_cast<float>((510.0 / 50.0) * _cm + 0.0)) )) {
        //("turning...\n");
    Target_Error_D = temp_phi - newInertial;
    if(fabs(Target_Error_D)>180.0){
      if(Target_Error_D < 0.0){
        Target_Error_D = Target_Error_D + 360.0;
      }else{
        Target_Error_D = Target_Error_D - 360.0;
      }
    }
    if(fabs(Target_Error_D)>60.0){
      if(Target_Error_D >0.0){
        Target_Error_D = 60.0;
      }else{
        Target_Error_D = 0.0-60.0;
      }
    }
      // 煞車距離44cm
      leftFront.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D), percent);
      leftBack.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D), percent);
      rightFront.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D), percent);
      rightBack.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D), percent);
      leftFront.spin(forward);
      leftBack.spin(forward);
      rightFront.spin(forward);
      rightBack.spin(forward);
      if (50.0 < velocity_percent) {
        velocity_percent = velocity_percent + (Brain.Timer.time(sec)-Movetime0) * -3.5; //-3.5
      }
    //wait(5, msec);
    }
  }
  // move_OK = true;
  printf("move ok!\n");
  rightFront.stop(hold);
  rightBack.stop(hold);
  leftFront.stop(hold);
  leftBack.stop(hold);
  wait(10,msec); // 100
  rightFront.stop(coast);
  rightBack.stop(coast);
  leftFront.stop(coast);
  leftBack.stop(coast);
  wait(50,msec); // 100
  
}


// 

void move_v_cm_direction_ht_degrees_1st(double v, double _cm, double dir, double ht_degree) {
  
  if(_cm<20.0){
    less_cm = true;
  }else{
    less_cm = false;
  }


  if(fabs(dir)==45||fabs(dir)==135||fabs(dir)==225||fabs(dir)==315){
    _cm=_cm*1.0;
  }

  if(fabs(dir)==0||fabs(dir)==90||fabs(dir)==180||fabs(dir)==270){
    _cm=_cm*0.877;
  }

  if(fabs(dir)==315 && fabs(ht_degree)==225){
    printf("Aiming....\n");
    _cm=_cm*1.051;
  }

  if(fabs(dir)==315 && fabs(ht_degree)==45){
    printf("Aiming....\n");
    _cm=_cm*1.051;
  }

  
  // 起步距離15cm + 煞車距離45cm = 60cm
  leftFront. setRotation(0.0, degrees);
  rightFront.setRotation(0.0, degrees);
  leftBack.  setRotation(0.0, degrees);
  rightBack. setRotation(0.0, degrees);
  velocity_percent = 0.0;
  // move_OK = false;
  Target_Error = 0.0;
  temp_phi = ht_degree;
  // direction = phi
  leftFront.setStopping(hold);
  rightFront.setStopping(hold);
  leftBack.setStopping(hold);
  rightBack.setStopping(hold);
  double Movetime0 = Brain.Timer.value();


  if(less_cm){
    //("lesscm10\n");
      while (( (fabs(static_cast<float>(leftFront.position(degrees))) + 
                 fabs(static_cast<float>(leftBack.position(degrees))) + 
                 fabs(static_cast<float>(rightFront.position(degrees))) + 
                 fabs(static_cast<float>(rightBack.position(degrees))) ) / 4.0)
                < fabs(360.0 * _cm)/(M_PI*3.25*2.54) && Movetime0+1.5 > Brain.Timer.value()) {
      // 煞車距離44cm
      velocity_percent  = 12.0;
      leftFront.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D), percent);
      leftBack.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D) + 2, percent);
      rightFront.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D - 2), percent);
      rightBack.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D), percent);

      // leftFront.setVelocity(((velocity_percent * 1.3) * cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error), percent);
      // leftBack.setVelocity(((velocity_percent * 1.3) * (0.0 - sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error), percent);
      // rightFront.setVelocity(-(((velocity_percent * 1.3) * sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error)), percent);
      // rightBack.setVelocity(-(((velocity_percent * 1.3) * (0.0 - cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error)), percent);
      leftFront.spin(forward);
      leftBack.spin(forward);
      rightFront.spin(forward);
      rightBack.spin(forward);

      //("cm...\n");
    //wait(5, msec);
    }


  }else{
      while (!((fabs(static_cast<float>(leftFront.rotation(degrees))) + (fabs(static_cast<float>(leftBack.rotation(degrees))) + (fabs(static_cast<float>(rightFront.rotation(degrees))) + fabs(static_cast<float>(rightBack.rotation(degrees)))))) / 4.0 > fabs(static_cast<float>((510.0 / 50.0) * 30.0)))) {
      // 經過計算，前15公分可以加速到100%
      leftFront.setVelocity(((velocity_percent*1.3) * cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error), percent);
      leftBack.setVelocity(((velocity_percent*1.3) * (0.0 - sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error) - 1, percent);
      rightFront.setVelocity(-(((velocity_percent*1.3) * sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error)) - 2.3, percent);
      rightBack.setVelocity(-(((velocity_percent*1.3) * (0.0 - cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error)), percent);
      leftFront.spin(forward);
      leftBack.spin(forward);
      rightFront.spin(forward);
      rightBack.spin(forward);
      if (velocity_percent < v) {
        velocity_percent = velocity_percent + 2.5 * (Brain.Timer.time(sec)-Movetime0);
      }
    //wait(5, msec);
    }
    // Movetime0 = Brain.Timer.value();
  while (!((fabs(static_cast<float>(leftFront.rotation(degrees))) + (fabs(static_cast<float>(leftBack.rotation(degrees))) + (fabs(static_cast<float>(rightFront.rotation(degrees))) + fabs(static_cast<float>(rightBack.rotation(degrees)))))) / 4.0 > fabs(static_cast<float>((510.0 / 50.0) * (_cm - 45.0))))) {  

      leftFront.setVelocity(((velocity_percent*1.3) * cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error), percent);
      leftBack.setVelocity(((velocity_percent*1.3) * (0.0 - sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error) - 1, percent);
      rightFront.setVelocity(-(((velocity_percent*1.3) * sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error)) - 2.3, percent);
      rightBack.setVelocity(-(((velocity_percent*1.3) * (0.0 - cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error)), percent);
      leftFront.spin(forward);
      leftBack.spin(forward);
      rightFront.spin(forward);
      rightBack.spin(forward);
      if (velocity_percent < v) {
        velocity_percent = velocity_percent + 3.0 * (Brain.Timer.time(sec)-Movetime0);
      }
    //wait(5, msec);
    }

    // Movetime0 = Brain.Timer.value();
  while (!((fabs(static_cast<float>(leftFront.rotation(degrees))) + (fabs(static_cast<float>(leftBack.rotation(degrees))) + (fabs(static_cast<float>(rightFront.rotation(degrees))) + fabs(static_cast<float>(rightBack.rotation(degrees)))))) / 4.0 > fabs(static_cast<float>((510.0 / 50.0) * (_cm + 0.0))))) {  

      // 煞車距離44cm
      leftFront.setVelocity(((velocity_percent*1.3) * cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error), percent);
      leftBack.setVelocity(((velocity_percent*1.3) * (0.0 - sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error) - 1, percent); // - 1.5
      rightFront.setVelocity(-(((velocity_percent*1.3) * sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error)) - 2.3, percent); // - 1
      rightBack.setVelocity(-(((velocity_percent*1.3) * (0.0 - cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error)), percent);
      leftFront.spin(forward);
      leftBack.spin(forward);
      rightFront.spin(forward);
      rightBack.spin(forward);
      if (50.0 < velocity_percent) {
        velocity_percent = velocity_percent + (Brain.Timer.value()-Movetime0);
      }
    //wait(5, msec);
    }
  }
  // move_OK = true;
  printf("LFM=%.1f ",leftFront.position(degrees));
  printf("move ok!\n");
  rightFront.stop(hold);
  rightBack.stop(hold);
  leftFront.stop(hold);
  leftBack.stop(hold);
  wait(100,msec);
  rightFront.stop(coast);
  rightBack.stop(coast);
  leftFront.stop(coast);
  leftBack.stop(coast);
  wait(100,msec);
  
}
// ======= 機器人移動距離 (速度,移動距離,機器人移動方向,最終面向方位角度)=========

double leftFrontV;
double leftBackV;
double rightFrontV;
double rightBackV;
double Degree;
double avg_degree;
double Kp = 0.01;

int move_degree_(){
  leftFront.setPosition(0.0,degrees);
  leftBack.setPosition(0.0,degrees);
  rightFront.setPosition(0.0,degrees);
  rightBack.setPosition(0.0,degrees);
  set_Drivetrain_V(leftFrontV,leftBackV,rightFrontV,rightBackV); 
  while(!(fabs(leftFront.position(degrees))>=Degree&&fabs(leftBack.position(degrees))>=Degree&&fabs(rightFront.position(degrees))>=Degree&&fabs(rightBack.position(degrees))>=Degree)){
    // avg_degree = ((fabs(leftFront.position(degrees))+fabs(leftBack.position(degrees))+fabs(rightFront.position(degrees))+fabs(rightBack.position(degrees)))/4);
    // leftFrontV = leftFrontV-(fabs(leftFront.position(degrees))-avg_degree)*Kp;
    // leftBackV = leftBackV-(fabs(leftBack.position(degrees))-avg_degree)*Kp;
    // rightFrontV = rightFrontV-(fabs(rightFront.position(degrees))-avg_degree)*Kp;
    // rightBackV = rightBackV-(fabs(rightBack.position(degrees))-avg_degree)*Kp;
    //("avg = %f |",avg_degree);
    //("%f ",(fabs(leftFront.position(degrees))-avg_degree));
    //("%f ",(fabs(leftBack.position(degrees))-avg_degree));
    //("%f ",(fabs(rightFront.position(degrees))-avg_degree));
    //("%f \n",(fabs(rightBack.position(degrees))-avg_degree));
    // wait(30,msec);

    //("ing....\n");
    wait(30,msec);
    
  }
  //("stop\n");
  set_Drivetrain_V(0,0,0,0); 
  DrivetrainStop_mode(3);
  DrivetrainStop();
  wait(28,msec);
  DrivetrainStop_mode(1);
  DrivetrainStop();
  printf("move ok!\n");
  
  
  return 0;
}

void move_v_cm_direction_ht_degrees_2nd(double v, double _cm, double dir, double ht_degree) {
  
  if(_cm<20.0){
    less_cm = true;
  }else{
    less_cm = false;
  }



  if(fabs(dir)==45||fabs(dir)==135||fabs(dir)==225||fabs(dir)==315){
    _cm=_cm*1.0;
  }

  if(fabs(dir)==0||fabs(dir)==90||fabs(dir)==180||fabs(dir)==270){
    _cm=_cm*0.877;
  }

  if(fabs(dir)==315 && fabs(ht_degree)==225){
    _cm=_cm*1.051;
  }

  if(fabs(dir)==315 && fabs(ht_degree)==45){
    _cm=_cm*1.051;
  }

  
  // 起步距離15cm + 煞車距離45cm = 60cm
  leftFront.setRotation(0.0, degrees);
  rightFront.setRotation(0.0, degrees);
  leftBack.setRotation(0.0, degrees);
  rightBack.setRotation(0.0, degrees);
  velocity_percent = 0.0;
  // move_OK = false;
  Target_Error = 0.0;
  temp_phi = ht_degree;
  // direction = phi
  leftFront.setStopping(hold);
  rightFront.setStopping(hold);
  leftBack.setStopping(hold);
  rightBack.setStopping(hold);
  double Movetime0 = Brain.Timer.value();


  if(less_cm){
    //("lesscm10\n");
      while (( (fabs(static_cast<float>(leftFront.rotation(degrees))) + 
                 fabs(static_cast<float>(leftBack.rotation(degrees))) + 
                 fabs(static_cast<float>(rightFront.rotation(degrees))) + 
                 fabs(static_cast<float>(rightBack.rotation(degrees))) ) / 4.0)
                < fabs(360.0 * _cm)/(M_PI*3.25*2.54) && Movetime0+1.5 > Brain.Timer.value()) {
      // 煞車距離44cm
      velocity_percent  = 12.0;
      leftFront.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D), percent);
      leftBack.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) + Target_Error_D), percent);
      rightFront.setVelocity((velocity_percent * cos(((dir + 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D), percent);
      rightBack.setVelocity((velocity_percent * cos(((dir - 45.0) - newInertial) * M_PI / 180.0) - Target_Error_D), percent);

      // leftFront.setVelocity(((velocity_percent * 1.3) * cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error), percent);
      // leftBack.setVelocity(((velocity_percent * 1.3) * (0.0 - sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error), percent);
      // rightFront.setVelocity(-(((velocity_percent * 1.3) * sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error)), percent);
      // rightBack.setVelocity(-(((velocity_percent * 1.3) * (0.0 - cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error)), percent);
      leftFront.spin(forward);
      leftBack.spin(forward);
      rightFront.spin(forward);
      rightBack.spin(forward);

      //("cm...\n");
    //wait(5, msec);
    }


  }else{
    while (!((fabs(static_cast<float>(leftFront.rotation(degrees))) + (fabs(static_cast<float>(leftBack.rotation(degrees))) + (fabs(static_cast<float>(rightFront.rotation(degrees))) + fabs(static_cast<float>(rightBack.rotation(degrees)))))) / 4.0 > fabs(static_cast<float>((510.0 / 50.0) * 30.0)))) {
      // 經過計算，前15公分可以加速到100%
      leftFront.setVelocity(((velocity_percent * 1.3) * cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error), percent);
      leftBack.setVelocity(((velocity_percent * 1.3) * (0.0 - sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error) + 1, percent);
      rightFront.setVelocity(-(((velocity_percent * 1.3) * sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error)) + 2, percent);
      rightBack.setVelocity(-(((velocity_percent * 1.3) * (0.0 - cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error)), percent);
      leftFront.spin(forward);
      leftBack.spin(forward);
      rightFront.spin(forward);
      rightBack.spin(forward);
      if (velocity_percent < v) {
        velocity_percent = velocity_percent + 2.5 * (Brain.Timer.time(sec)-Movetime0);
      }
    //wait(5, msec);
    }
    Movetime0 = Brain.Timer.value();
    while (!((fabs(static_cast<float>(leftFront.rotation(degrees))) + (fabs(static_cast<float>(leftBack.rotation(degrees))) + (fabs(static_cast<float>(rightFront.rotation(degrees))) + fabs(static_cast<float>(rightBack.rotation(degrees)))))) / 4.0 > fabs(static_cast<float>((510.0 / 50.0) * (_cm - 45.0))))) {
      leftFront.setVelocity(((velocity_percent * 1.3) * cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error), percent);
      leftBack.setVelocity(((velocity_percent * 1.3) * (0.0 - sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error) + 1, percent);
      rightFront.setVelocity(-(((velocity_percent * 1.3) * sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error)) + 2, percent);
      rightBack.setVelocity(-(((velocity_percent * 1.3) * (0.0 - cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error)), percent);
      leftFront.spin(forward);
      leftBack.spin(forward);
      rightFront.spin(forward);
      rightBack.spin(forward);
      if (velocity_percent < v) {
        velocity_percent = velocity_percent + 3.0 * (Brain.Timer.time(sec)-Movetime0);
      }
    //wait(5, msec);
    }

    Movetime0 = Brain.Timer.value();
    while (!((fabs(static_cast<float>(leftFront.rotation(degrees))) + (fabs(static_cast<float>(leftBack.rotation(degrees))) + (fabs(static_cast<float>(rightFront.rotation(degrees))) + fabs(static_cast<float>(rightBack.rotation(degrees)))))) / 4.0 > fabs(static_cast<float>((510.0 / 50.0) * _cm + 0.0)))) {
      // 煞車距離44cm
      leftFront.setVelocity(((velocity_percent * 1.3) * cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error), percent);
      leftBack.setVelocity(((velocity_percent * 1.3) * (0.0 - sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error) + 1, percent);
      rightFront.setVelocity(-(((velocity_percent * 1.3) * sin(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180) + Target_Error)) + 1.3, percent);
      rightBack.setVelocity(-(((velocity_percent * 1.3) * (0.0 - cos(((dir - 45.0) - imu.heading(degrees)) * M_PI / 180)) + Target_Error)), percent);
      leftFront.spin(forward);
      leftBack.spin(forward);
      rightFront.spin(forward);
      rightBack.spin(forward);
      if (50.0 < velocity_percent) {
        velocity_percent = velocity_percent + (Brain.Timer.time(sec)-Movetime0) * -3.5;
      }
    //wait(5, msec);
    }
  }
  // move_OK = true;
  printf("move ok!\n");
  rightFront.stop(hold);
  rightBack.stop(hold);
  leftFront.stop(hold);
  leftBack.stop(hold);
  wait(100,msec);
  rightFront.stop(coast);
  rightBack.stop(coast);
  leftFront.stop(coast);
  leftBack.stop(coast);
  wait(100,msec);
  
}

void move_degree(double leftFrontv,double leftBackv,double rightFrontv,double rightBackv,double degree,bool wait){
  leftFrontV = leftFrontv;
  leftBackV = leftBackv;
  rightFrontV = rightFrontv;
  rightBackV = rightBackv;

  Degree = degree;
  if(wait){
    task move_degree_wait = task (move_degree_); 
  }
  else{
    move_degree_();
  }
}

int cnt =0 ;

double AimShootV;
double AimShootV_TH=8.0;
double correction;
double real_speed;
bool move_done = false;
bool SW = true;
int count = 1;
int loadcnt = 6;
int ShooterAim(){
  printf("ShooterAim start\n");
  while(SW){
    //printf("target: %f     imuFort.heading(): %f\n", Turret::target + 225, imuFort.heading(degrees));
    printf("Dis: %f         L_shoot: %f\n", Turret::tmpy_highgoal - 18.5, L_shoot.velocity(percent));
    if (loadcnt == 6){
      real_speed = 80.6;
    }else if (loadcnt == 5){
      real_speed = 80.2;
    }else{
      real_speed = 0.0001479 * ((Turret::tmpy_highgoal - 18.5) * (Turret::tmpy_highgoal - 18.5))
                 + 0.0635276 * (Turret::tmpy_highgoal - 18.5)
                 + 71.0797; //68.8797
    }
    /*real_speed =   0.0002 * (Turret::autoAim_midline_Dis * Turret::autoAim_midline_Dis) 
                 + 0.0843 * Turret::autoAim_midline_Dis
                 + 47.8; //47.391       last setting: 47.6
                //+55;*/
    correction = real_speed - L_shoot.velocity(percent);
    //("real_speed= %.2f | ",real_speed);
    AimShootV = real_speed;// + correction; // AimShootV = velocity of flywheel (目前還太慢)
    //("ActShootV= %.2f | ",L_shoot.velocity(percent));
    // Shoot::auto_shoot_motor_set(AimShootV); // setting the flywheel velocity
    //if(L_shoot.velocity(percent) < AimShootV - 4)Shoot::auto_shoot_motor_set(100);
    Shoot::auto_shoot_motor_set(AimShootV);
    //Shoot::auto_shoot_motor_set(real_speed-3.0); // setting the flywheel velocity

    //("hue=%.2f | ", discnumber.hue());
    //("V=%.2f | }", AimShootV);
    //("Dis=%.2f\n", Turret::autoAim_midline_Dis);
    if(discNumber.objectDistance(mm)<30 && L_shoot.velocity(percent) >= (real_speed-0.5) && L_shoot.velocity(percent) > real_speed - 0.1 
    && firstRotation == false && move_done == true  && fabs(imuFort.heading(degrees) - (Turret::target + 225)) <= 3.6){// launch if disc detected
      printf("Shoot %d               L_shoot: %f                                                          !!!\n", count, L_shoot.velocity(percent));
      //wait(50,msec);  //300
      Shoot::auto_shoot_air();
      count ++;
      waitUntil(discNumber.objectDistance(mm)>70);
      cnt --;
      loadcnt --;
    }
 
    if(cnt>1) {
          Intake.setVelocity(-100,percent);
          Intake.spin(forward);
    }
    else {
        Intake.setVelocity(100,percent);
        Intake.spin(forward);
    }
    wait(10,msec);
  }
  return 0;
}


int load_shoot(){
  while(SW){
    
    real_speed = 63; //47.391       last setting: 47.6
                //+55;
    correction = real_speed - L_shoot.velocity(percent);
    //("real_speed= %.2f | ",real_speed);
    AimShootV = real_speed + correction; // AimShootV = velocity of flywheel (目前還太慢)
    //("ActShootV= %.2f | ",L_shoot.velocity(percent));
    // Shoot::auto_shoot_motor_set(AimShootV); // setting the flywheel velocity
    //if(L_shoot.velocity(percent) < AimShootV - 4)Shoot::auto_shoot_motor_set(100);
    if (AimShootV > 100){
      Shoot::auto_shoot_motor_set(100);
    }else{
      Shoot::auto_shoot_motor_set(AimShootV);
    }
    //Shoot::auto_shoot_motor_set(real_speed-3.0); // setting the flywheel velocity

    //("hue=%.2f | ", discnumber.hue());
    //("V=%.2f | }", AimShootV);
    //("Dis=%.2f\n", Turret::autoAim_midline_Dis);
    wait(5,msec);
  }
  return 0;
}



double AimShootV_2;
double AimShootV_TH_2=8.0;
double correction_2;
double real_speed_2;
int ShooterAim_2(){
  
  while(SW){
    real_speed_2 =   0.000220766 * (Turret::autoAim_midline_Dis * Turret::autoAim_midline_Dis) 
                 + 0.084476200 * Turret::autoAim_midline_Dis
                 + 49.8844;
    correction_2 = real_speed_2 - L_shoot.velocity(percent);
    //("real_speed= %.2f | ",real_speed);
    AimShootV_2 = real_speed_2 + 2*correction_2; // AimShootV = velocity of flywheel (目前還太慢)
    //("ActShootV= %.2f | ",L_shoot.velocity(percent));
    // Shoot::auto_shoot_motor_set(AimShootV_2); // setting the flywheel velocity
    // Shoot::auto_shoot_motor_set(real_speed+5.0); // setting the flywheel velocity
    if(L_shoot.velocity(percent) +5 < real_speed)Shoot::auto_shoot_motor_set(100);
    else Shoot::auto_shoot_motor_set(real_speed );
    //("hue=%.2f | ", discnumber.hue());
    //("V=%.2f | }", AimShootV);
    //("Dis=%.2f\n", Turret::autoAim_midline_Dis);
    if( discNumber.objectDistance(mm)<30 && L_shoot.velocity(percent) > real_speed_2-5.0 && secondRotation == false){// launch if disc detected
      wait(100,msec);  //300
      Shoot::auto_shoot_air();
      waitUntil(discNumber.objectDistance(mm)>30);
      cnt --;
    }
     
    if(cnt>1) {
          Intake.setVelocity(-100,percent);
          Intake.spin(forward);
    }
    else {
        Intake.setVelocity(100,percent);
        Intake.spin(forward);
    }
    wait(30,msec);
  }
  return 0;
}
 
int cnt_task(){
  printf("in\n");
  while(true){
    if(IntakeRight.hue() < 60&&IntakeRight.isNearObject()){
      waitUntil(!(IntakeRight.hue()  <60));
      cnt++;
      printf("11111111111111111cnt = %d\n", cnt);
    }
    if(IntakeLeft.hue() > 30&&IntakeLeft.isNearObject()){
      waitUntil(!(IntakeLeft.hue() > 30));
      cnt++;
      printf("222222222222222cnt = %d\n", cnt);
    }
    printf("333333333333333333cnt = %d\n", cnt);
    // wait(100,msec);
  }

  return 0;
}

int fortSpin(){
  if (fort.position(degrees) < 0){
    fort.spin(forward, 20, percent);
    waitUntil(fort.position(degrees) >= 0);
    fort.stop(hold);
  }else{
    fort.spin(forward, -20, percent);
    waitUntil(fort.position(degrees) <= 0);
    fort.stop(hold);
  }
  return 0;
}

bool revReset = true;
// int fortSpinRev(){
//   while(revReset){
//     fort.spin(forward,7*( 175.0-imuFort.heading(degrees)), percent);

//   }
//   fort.stop(hold);
//   return 0;
// }

int ending(){
  move_rotate_v_sec_direction_ht_degree(100, 0.3, 225, 135 );
  return 0;
}

int endExpand(){
  while(true){
    expand.set(true);
    wait(300, msec); 
    expand.set(false);
    wait(100, msec); 
  }
}

int showMotorWatt(){
  while(true){
    printf("LF: %f     |     LB: %f     |     RF: %f     |     RB: %f\n", leftFront.power(), leftBack.power(), rightFront.power(), rightBack.power());
  }
}

double d;
double v;
int fort_spin_to_degree_task(){
  printf("Fort: %f         d: %f\n", fort.position(degrees), d);
  if (d > fort.position(degrees)){
    fort.spin(forward, v, percent);
    waitUntil(fort.position(degrees) >= d);
    fort.stop();
  }else{
    fort.spin(forward, -v, percent);
    waitUntil(fort.position(degrees) <= d);
    fort.stop();
  }
  printf("Spin done\n");
  return 0;
}

void fort_spin_to_degree(double deg, double speed, bool wait){
  d = deg;
  v = speed;
  if (wait){
    task turretspin = task(fort_spin_to_degree_task);
  }else{
    fort_spin_to_degree_task();
  }
}

double t_time;
int roller_done(){
  t_time = Brain.Timer.value();
  waitUntil(Color.hue() > 200 || Brain.Timer.value() > t_time + 3.5);
  Collect::auto_taking_off();
  return 0;
}

// ============== Skills 60sec =============

void skills(){
  wait(5, sec);//10
  // Turret::autoAim_midline(70.4,1);//在中線上邊向右移動邊瞄準highgoal (cm ,wait) 待測試
  // wait(100,sec);
  // Insert autonomous user code here.
  // task cntDisc = task(cnt_task);
  
  
  // Loader 1
  printf("%f\n", fort.position(degrees));
  Brain.Timer.reset();
  Color.setLightPower(10, percent);
  L_shoot.setMaxTorque(100,pct);
  Shoot::auto_shoot_motor_set(100); //75, 65, 69
  Collect::auto_taking_in();
  fort_spin_to_degree(-39, 30, 0);
  //Shoot::fort_no_pid_spin(-4.5,20,1);//對車 80 deg 發射 //-10 // 砲台轉向
  double time00 = Brain.Timer.value();
  double timeout = 11.0; //11
  int load = 9; //9
  while(Brain.Timer.value() - time00 <= timeout && load > 0){// && time00+7.0 > Brain.Timer.value()
    printf("L_shoot: %f\n", L_shoot.velocity(percent));
    if(load<=7){
      Shoot::auto_shoot_motor_set(67);
    }else{
      Shoot::auto_shoot_motor_set(70);
    }
    /*if (L_shoot.velocity(percent) < 56){
      Shoot::auto_shoot_motor_set(100);
    }else{
      Shoot::auto_shoot_motor_set(65);
    }*/
    //("%.2f\n", discnumber.hue());
    if((discNumber.objectDistance(mm)<17||discNumber.objectDistance(mm)>600) && L_shoot.velocity(percent) >= 63.1){//value? 63
      //("%f", discnumber.hue());
      printf("Shoot %d\n", 10 - load);
      Shoot::auto_shoot_air();
      //waitUntil(discNumber.objectDistance(mm)>30 || Brain.Timer.value() - time00 > timeout);
      load --;
      wait(20,msec);//300 //50
    }
    wait(20, msec);
  }
  if (discNumber.objectDistance(mm)<17){
    Shoot::auto_shoot_motor_set(82);
    fort_spin_to_degree(-10, 30, 0);
  }else{
    task fortspintask = task(fortSpin);
  }
  //printf("time=%f\n",Brain.Timer.value()-time00 );
  //SW = false;

  //wait(10000,sec);



  // Roller 1
  //SW = true;
  //Roller::roller_spin_red_task();
  move_rotate_v_sec_direction_ht_degree(95, 0.29, 270, 0) ;
  move_v_cm_direction_ht_degrees(95, 112, 180, 270) ;
  //Heading_Turn_abs_D(270.0, 0);
  task roller1 = task(roller_done);
  DisC::refreshDistance();
  while (DisC::DB - 226 > 73 || Color.hue() > 200){
    leftFront.spin(forward, -25, percent);
    rightFront.spin(forward, -25, percent);
    leftBack.spin(forward, -25, percent);
    rightBack.spin(forward, -25, percent);
    DisC::refreshDistance();
  }
  DrivetrainStop();
  waitUntil(Color.hue() > 200 || Brain.Timer.value() > t_time + 3.5);
  if (discNumber.objectDistance(mm)<17){
    Shoot::auto_shoot_air();
    task fortspintask = task(fortSpin);
  }
  Shoot::auto_shoot_motor_set(80.8);
  /*while (DisC::DB - 226 >= 70){
    printf("true\n");
    leftFront.setVelocity(30.0, percent);
    rightFront.setVelocity(30.0, percent);
    leftBack.setVelocity(30.0, percent);
    rightBack.setVelocity(30.0, percent);

    leftFront.spin(forward);
    rightFront.spin(forward);
    leftBack.spin(forward);
    rightBack.spin(forward);
    DisC::refreshCoordinate(1);
  }
  DrivetrainStop();*/
  //move_v_cm_direction_ht_degrees(60, (DisC::DB - 226) / 10 - 6.2, 90, 270) ;
  //wait(50,sec);
  //move_rotate_v_sec_direction_ht_degree(40, 0.2, 90, 270) ;
  // move_rotate_v_sec_direction_ht_degree(100, 0.05, 270, 270) ;
  //Roller::roller_load_out();
  //move_rotate_v_sec_direction_ht_degree(20, 0.3, 90, 270) ;
  // move_rotate_v_sec_direction_ht_degree(20, 1.55, 90, 270) ; //20,1.55
  // move_rotate_v_sec_direction_ht_degree(20, 0.7, 90, 270) ; 
  // wait(0.85,sec);
  //Collect::auto_taking_in();
  //wait(1000, sec);

  
  // Roller 2 and take 1 disc
  //Shoot::fort_no_pid_spin(45,30,1);
  move_rotate_v_sec_direction_ht_degree(95, 0.20, 270, 270) ;\
  Collect::auto_taking_in();
  //Roller::roller_load_on(); 
  move_rotate_v_sec_direction_ht_degree(95, 1.1, 214, 0) ; //0.8//0.7
  //Heading_Turn_abs_D(0.0, 0);
  //Roller::roller_load_out();
  task roller2 = task(roller_done);
  DisC::refreshDistance();
  while (DisC::DB - 226 > 73 || Color.hue() > 200){
    leftFront.spin(forward, -30, percent);
    rightFront.spin(forward, -30, percent);
    leftBack.spin(forward, -30, percent);
    rightBack.spin(forward, -30, percent);
    DisC::refreshDistance();
  }
  DrivetrainStop();
  //wait(50,sec);
  //move_rotate_v_sec_direction_ht_degree(40, 0.22, 180, 0) ;
  waitUntil(Color.hue() > 200 || Brain.Timer.value() > t_time + 3.5);
  // move_rotate_v_sec_direction_ht_degree(70, 0.3, 180, 0) ;
  // move_rotate_v_sec_direction_ht_degree(20, 0.15, 180, 0) ;
  //wait(0.3, sec);
  // move_rotate_v_sec_direction_ht_degree(20, 0.2, 180, 0) ; //30,1.8
  // move_rotate_v_sec_direction_ht_degree(30, 0.85, 180, 0) ; //30
  // wait(0.95,sec);
  //Collect::auto_taking_in();
  printf("%f\n", Brain.Timer.value());
  
  
  // Intake 1 disk and align
  move_rotate_v_sec_direction_ht_degree(20, 0.15, 0, 0) ;
  Collect::auto_taking_in();
  // DisC::refreshCoordinate(1);
  //  DisC::ACC(30,300,300);
  DisC::refreshDistance();
  move_v_cm_direction_ht_degrees(80, DisC::DR / 10 - 19.5, 90, 5) ;//56
  //Roller::roller_load_on();
  // Shoot::auto_shoot_motor_set(85);
  task ShooterTask = task(ShooterAim);
  //Roller::roller_load_on();
  // Heading_Turn_abs_D(20.0, 0);
  DisC::refreshDistance();
  printf("x= %f , y = %f\n",current_x , current_y);
  //wait(100, sec);
  Heading_Turn_abs_D(226.0, 0);
  //wait(100, sec);
  // fort.spin(forward, 20, percent);
  // waitUntil(imuFort.heading(degrees) >= 225);
  // fort.stop(hold);

  
  // Diagonal strafe across half field
  // task ShooterTask = task(ShooterAim); // flywheel speed during diagonal sweep
  printf("Turret inertial value: %f\n", imuFort.heading(degrees));
  Turret::autoAim_midline(206.0,0,1); // turret alignment during diagonal sweep
  // DisC::DR & DisC::DB laser distance sensor values
  move_v_cm_direction_ht_degrees_1st(46, 130.5 - ((hypot(DisC::DR, DisC::DB) * 0.1)) - 14, 315, 225); //1st phase of path (160.0)
  move_done = true;
  // Turret::LF_rotation=leftFront.position(degrees);
  Turret::LF_rotation=Turret::LF_rotation+leftFront.position(degrees);
  move_v_cm_direction_ht_degrees_1st(12, 43, 315, 225); //1st phase of path (160.0)
  waitUntil(loadcnt <= 4);
  move_done = false;
  Turret::LF_rotation=Turret::LF_rotation+leftFront.position(degrees);
  move_v_cm_direction_ht_degrees_1st(35, 53, 315, 225); //1st phase of path (160.0)
  move_done = true;
  waitUntil(loadcnt == 1);
  move_OK = true;
  // wait(0.4, sec);//0.4
  SW = false;
  fort_spin_to_degree(260, 30, 1);
  //wait(100, sec);
  //Heading_Turn_abs_D(90.0, 0);// mid-path rotation to face red high goal

  
  // Intake 3 disks and head to Loader 2
  Shoot::auto_shoot_motor_set(68); //75
  //Shoot::fort_no_pid_spin(20,20,1);
  move_v_cm_direction_ht_degrees(70, 61.5, 180, 270) ;
  Heading_Turn_abs_D(225.0, 0);
  Shoot::auto_shoot_air();
  Shoot::auto_shoot_motor_set(64.5);
  fort_spin_to_degree(150, 25, 1);
  move_v_cm_direction_ht_degrees(75, 42.5, 316, 226) ;
  Shoot::auto_shoot_air();
  Shoot::auto_shoot_motor_set(64.5);
  fort_spin_to_degree(-39, 20, 1);
  //task fortspintask2 = task(fortSpin);
  move_v_cm_direction_ht_degrees(80, 70, 321, 226) ;
  // move_rotate_v_sec_direction_ht_degree(95, 1.9, 284, 180) ;
  // move_rotate_v_sec_direction_ht_degree(30, 1, 284, 180) ;
  Heading_Turn_abs_D(180.0, 0);
  DisC::refreshDistance();
  double direc = atan2((209 - DisC::DF * 0.1) + 1, ((DisC::DR - 117) * 0.1)) * 180 / M_PI + 270;
  printf("direc: %f\n", direc);
  move_rotate_v_sec_direction_ht_degree(95, 1.9, direc, 180) ;
  move_rotate_v_sec_direction_ht_degree(50, 0.4, direc, 180) ;
  printf("1\n");
  DisC::refreshDistance();
  while (177.5 - (DisC::DB) / 10 >= 4){
    leftFront.spin(forward, 22, percent);
    rightFront.spin(forward, 15, percent);
    leftBack.spin(forward, 15, percent);
    rightBack.spin(forward, 24, percent);
    DisC::refreshDistance();
  }
  DrivetrainStop();
  //move_v_cm_direction_ht_degrees(70, DisC::DB / 10 - 177.5, 0, 180) ;
  move_v_cm_direction_ht_degrees(70, 2.9, 90, 180) ;
  //Heading_Turn_abs_D(182.0, 0);
  /*while (imuFort.heading(degrees) >= 176){
    fort.spin(forward, -70, percent);
  }
  fort.stop();*/
  // move_v_cm_direction_ht_degrees(40, DisC::DB / 10 - 179, 0, 180) ;
  // move_v_cm_direction_ht_degrees(40, 4, 90, 180) ;


  // Loader 2
  // Shoot::fort_no_pid_spin(-5,20,0);//對車 80 deg 發射 //-10 // 砲台轉向
  load = 10;
  timeout = 11;
  time00 = Brain.Timer.value();
  while(Brain.Timer.value() - time00 <= timeout && load > 0){// && time00+7.0 > Brain.Timer.value()
    printf("L_shoot: %f\n", L_shoot.velocity(percent));
    Shoot::auto_shoot_motor_set(68);
    /*if (L_shoot.velocity(percent) < 56){
      Shoot::auto_shoot_motor_set(100);
    }else{
      Shoot::auto_shoot_motor_set(65);
    }*/
    //("%.2f\n", discnumber.hue());
    if((discNumber.objectDistance(mm)<17||discNumber.objectDistance(mm)>600) && L_shoot.velocity(percent) >= 61.9){//value? 63
      //("%f", discnumber.hue());
      printf("Shoot %d\n", 11 - load);
      Shoot::auto_shoot_air();
      //waitUntil(discNumber.objectDistance(mm)>30 || Brain.Timer.value() - time00 > timeout);
      load --;
      wait(20,msec);//300
    }
    wait(20, msec);
  }
  printf("%f\n", Brain.Timer.value());
  //wait(1000, sec);
  printf("loader done\n");
  Shoot::auto_shoot_motor_set(82);
  

  // Roller 3 and take 1 disk
  move_rotate_v_sec_direction_ht_degree(95, 0.38, 90, 180) ;
  move_v_cm_direction_ht_degrees(95, 112, 0, 90) ;
  task roller3 = task(roller_done);
  DisC::refreshDistance();
  while (DisC::DB - 226 > 73 || Color.hue() > 200){
    leftFront.spin(forward, -25, percent);
    rightFront.spin(forward, -25, percent);
    leftBack.spin(forward, -25, percent);
    rightBack.spin(forward, -25, percent);
    DisC::refreshDistance();
  }
  DrivetrainStop();
  waitUntil(Color.hue() > 200 || Brain.Timer.value() > t_time + 3.5);


  // Roller 4
  fort_spin_to_degree(-11, 10, 1);
  move_rotate_v_sec_direction_ht_degree(95, 0.24, 90, 90) ;\
  Collect::auto_taking_in();
  move_rotate_v_sec_direction_ht_degree(95, 1.1, 34, 180) ; //0.8//0.7   30
  task roller4 = task(roller_done);
  DisC::refreshDistance();
  while (DisC::DB - 226 > 73 || Color.hue() > 200){
    leftFront.spin(forward, -32, percent);
    rightFront.spin(forward, -32, percent);
    leftBack.spin(forward, -32, percent);
    rightBack.spin(forward, -32, percent);
    DisC::refreshDistance();
  }
  DrivetrainStop();
  waitUntil(Color.hue() > 200 || Brain.Timer.value() > t_time + 3.5);


  // Expansion
  move_rotate_v_sec_direction_ht_degree(70, 0.7, 195, 180) ;
  Heading_Turn_abs_D(90.0, 0);
  Shoot::auto_shoot_air();
  Heading_Turn_abs_D(135.0, 0);
  task endingExpansion = task(endExpand);
  wait(100, msec);
  //waitUntil(Brain.Timer.value() >= 59.2);
  Heading_Turn_abs_D(180.0, 0);
  printf("time: %f\n", Brain.Timer.value());
  wait(100, sec);
      
      
      
      





      // fort.spin(forward, 20, percent); // turret alignment to red high goal
      // waitUntil(imuFort.heading(degrees) >= 45);
      // fort.stop(hold);
      SW = true;
      wait(0.4, sec);//0.4
      task ShooterTask2 = task(ShooterAim_2);
      
       Turret::autoAim_midline_2(240.0,1,1); 
       Turret::LF_rotation=leftFront.position(degrees);
       move_v_cm_direction_ht_degrees_2nd(60, 160, 313, 45);// accelerated movement (2nd phase) adjust 40
       Turret::LF_rotation=Turret::LF_rotation+leftFront.position(degrees);
      //Heading_Turn_abs_D(45.0, 0);
      // wait(100,sec);

      // task ShooterTask2 = task(ShooterAim_2); // flywheel speed during diagonal sweep
     // turret alignment during diagonal sweep
      move_v_cm_direction_ht_degrees_2nd(15, 30, 315, 45);// 3rd phase of path
      Turret::LF_rotation=Turret::LF_rotation+leftFront.position(degrees);
      move_v_cm_direction_ht_degrees_2nd(50, 50, 315, 45);// 3rd phase of path 40//adjust
       move_OK = true;
       wait(0.5, sec);
       SW = false;
       
      // wait(100, sec);
      
      
      // move_v_cm_direction_ht_degrees(40, 15, 0, 180); //先後退
      
      // move_rotate_v_sec_direction_ht_degree(50, 2, 300, 180); // 靠上角落


      // move_rotate_v_sec_direction_ht_degree(20, 1.5, 90, 180 ); // 使用roller側邊 校正
      // move_v_cm_direction_ht_degrees(20, 10, 180, 180); // 往前進

      // move_v_cm_direction_ht_degrees(20, 27, 90, 180);

      
      // 3rd roller
      
      Heading_Turn_abs_D(180.0, 0);
      DisC::refreshCoordinate(0);
      printf("x= %f , y = %f\n",current_x , current_y);
      //wait(100, sec);
      move_v_cm_direction_ht_degrees(90, 73 - DisC::DR * 0.1, 90, 180) ;
      //wait(100, sec);
      // DisC::move_distance(800,3000,5);
      //Heading_Turn_abs_D(180.0, 0);
      // move_rotate_v_sec_direction_ht_degree(20, 1.7, 270, 90 );
      // task fortspinRevtask = task(fortSpinRev);
      Roller::roller_load_out();
      move_rotate_v_sec_direction_ht_degree(100, 0.4, 0, 180);
      move_rotate_v_sec_direction_ht_degree(20, 0.2, 0, 180);
      // move_rotate_v_sec_direction_ht_degree(20, 1.7, 0, 180);adjust
      // move_rotate_v_sec_direction_ht_degree(30, 0.8, 0, 180);
      // wait(0.9,sec);
      //wait(100,sec);
      

      // 4th roller
      
      move_rotate_v_sec_direction_ht_degree(25, 0.8, 180, 180 ); //轉完roller往前開一點準備轉向
      //Heading_Turn_abs_D(135.0, 0);
      Roller::roller_load_on();
      move_v_cm_direction_ht_degrees(90, 49, 225, 90); // 斜走 準備到下一個roller 49
      //Heading_Turn_abs_D(90.0, 0);
      Roller::roller_load_out();
      move_rotate_v_sec_direction_ht_degree(100, 0.4, 270, 90 );
      move_rotate_v_sec_direction_ht_degree(20, 0.3, 270, 90 );
      // move_rotate_v_sec_direction_ht_degree(20, 1.7, 270, 90 );
      // move_rotate_v_sec_direction_ht_degree(40, 1.2, 270, 90 );adjust
      // wait(0.5,sec);


      // Strafe to loader
       move_rotate_v_sec_direction_ht_degree(50, 0.4, 90, 90);
      // move_rotate_v_sec_direction_ht_degree(25, 0.8, 90, 90); adjust
      //Heading_Turn_abs_D(180.0, 0);
      //task fortspinRevtask = task(fortSpinRev);
      move_v_cm_direction_ht_degrees(90, 100, 180, 180);//107
      Roller::roller_load_on();
      move_rotate_v_sec_direction_ht_degree(40, 1, 270, 180 ); // 靠牆準備發射 7個
      move_v_cm_direction_ht_degrees(20, 4.5, 90, 180);
      printf("fort =%.1f\n",fort.position(degrees));

      Shoot::auto_shoot_motor_set(72); //75
      Collect::auto_taking_in();
      // Shoot::fort_no_pid_spin(-5,20,0);//對車 80 deg 發射 //-10 // 砲台轉向
      revReset = false;
      load = 11;
      time00 = Brain.Timer.value();
      while(Brain.Timer.value() - time00 < 9.0){// && time00+7.0 > Brain.Timer.value()
        //("%.2f\n", discnumber.hue());
        if(discNumber.objectDistance(mm)<30 && L_shoot.velocity(percent) > 60.2){//value? 63
          //("%f", discnumber.hue());
          wait(30,msec);//300 //100
          printf("2\n");
          Shoot::auto_shoot_air();
          waitUntil(discNumber.objectDistance(mm)>30);
          load --;
          
        }
      }
      
      // autoAim_midline_io = false;
      /*
      if(fort.position(degrees)>0){
        fort.setVelocity(-100,percent);
      }else{
        fort.setVelocity(-100,percent);
      }
      fort.spinToPosition(0,degrees);
      // Loader
      Shoot::auto_shoot_motor_set(70);
      // Collect::auto_taking_in();
      Shoot::fort_no_pid_spin(-4,5,0);//對車 80 deg 發射 //-10 // 砲台轉向
      // time00 = Brain.Timer.value();
      load = 7;
      */
      /*while(load > 0){// && time00+7.0 > Brain.Timer.value()
        printf("%.2f\n", discnumber.hue());
        if(discnumber.isNearObject() && discnumber.hue()<80 && L_shoot.velocity(percent) > 63.0){//value?
          printf("%f", discnumber.hue());
          wait(800,msec);
          printf("2\n");
          load --;
          Shoot::auto_shoot_air();
        }
      }*/

      //printf("fort =%.1f\n",fort.position(degrees));

      
      // Strafe to expansion position
      move_rotate_v_sec_direction_ht_degree(100, 1.65, 25, 135 );
      task end = task(ending);
      expand.set(true);
      wait(300, msec); 
      expand.set(false);
      wait(100, msec); 
      expand.set(true);
      wait(300, msec); 
      expand.set(false);
      //printf("time = %f\n",Brain.Timer.value());



      // // wait(100, sec);
      // //take 3 disc
      // Shoot::fort_no_pid_spin(10,20,1);//0
    
      // Heading_Turn_abs_D(225.0,0);
      // move_v_cm_direction_ht_degrees(40, 110, 315, 225 ) ;

    

      // //shoot 3 disc
      // Shoot::auto_shoot_air(); 
      // wait(0.5,sec);
      // Shoot::auto_shoot_air(); 
      // wait(0.5,sec);      
      // Shoot::auto_shoot_air(); 
      // wait(0.2,sec);
      
      // //load 
      
      // move_rotate_v_sec_direction_ht_degree(40, 3, 280, 180 );

      // Shoot::auto_shoot_motor_set(50);
      // Shoot::fort_no_pid_spin(-10,20,1);//-10

      // //second round
      // time00 = Brain.Timer.value();
      // load = 9;
      // while(load > 0 && time00+7.0 > Brain.Timer.value()){
      //   printf("%.2f\n", discnumber.hue());
      //   if(discnumber.hue()>55 && discnumber.hue()<70 && L_shoot.velocity(percent) > 70.0){//value?
      //     wait(800,msec);
      //     printf("2\n");
      //     load --;
      //     Shoot::auto_shoot_air();
      //   }
      // }

      // //roller 3rd
      // move_v_cm_direction_ht_degrees(40, 10, 90, 180 ) ;
      // move_v_cm_direction_ht_degrees(40, 105, 0, 180 ) ;
      // Heading_Turn_abs_D(90,0);
      // move_v_cm_direction_ht_degrees(40, 5, 270, 90 ) ;
      // Roller::roller_spin_red(0);
      // move_v_cm_direction_ht_degrees(40, 10, 90, 90 ) ;
      // //take 2 disc
      // Collect::auto_taking_in();

      // Heading_Turn_abs_D(135,0);
      
      // move_v_cm_direction_ht_degrees(40, 50, 45, 135 ) ;

      // //roller 4th
      // Heading_Turn_abs_D(180,0);
      // Shoot::auto_shoot_motor_set(60.5);
      // move_v_cm_direction_ht_degrees(40, 5,0 , 180 ) ;
      // Roller::roller_spin_red(0);
      // move_v_cm_direction_ht_degrees(40, 5,180 , 180 ) ;

      // Heading_Turn_abs_D(90,0);
      // // shoot 2 disc

      // Shoot::fort_no_pid_spin(0,20,0);//-10
      // Shoot::auto_shoot_air(); 
      // wait(0.5,sec);
      // Shoot::auto_shoot_air(); 
      // wait(0.2,sec);


      // //expansion
      // move_v_cm_direction_ht_degrees(40, 5,225 , 135 ) ;

      // expand.set(true);
      // wait(300, msec); 
      // expand.set(false);

}

void LeftRed15sec(){

      imu.setHeading(0,degrees);
      // Shoot::auto_shoot_motor_set(40);
      // wait(0.2,sec);
      // Shoot::auto_shoot_motor_set(70);
      // wait(0.2,sec);
      Shoot::auto_shoot_motor_set(100);
      set_Drivetrain_V_org(-30,-30,-30,-30);
      wait(400,msec);
      set_Drivetrain_V_org(-5,-5,-5,-5);
      
        // >>> 機器人移動距離面向角度 (速度,移動距離,機器人移動方向,最終面向方位角度)
      // Roller::roller_spin_red(0);
      Collect::auto_taking_in();
      move_rotate_v_sec_direction_ht_degree(20, 0.5, 90, 270) ;
      move_v_cm_direction_ht_degrees(65, 5, 0, 0) ;

      //砲塔轉向(角度,速度,布林值)
      Shoot::fort_no_pid_spin(-20,40,0);//-10

      move_v_cm_direction_ht_degrees(60, 15, 90, 0) ;
      
      //射擊
      Shoot::auto_shoot_air(); 
      Shoot::auto_shoot_motor_set(100);
      wait(1.2,sec);
      Shoot::auto_shoot_air(); 
      
      Heading_Turn_abs_D(315,0);
      Collect::auto_taking_in();
      //move_v_cm_direction_ht_degrees(30, 5, 0, 315) ;
      

      Shoot::fort_no_pid_spin(67,20,1);//-10
      //砲塔轉向(角度,速度,布林值)

      //Shoot::auto_shoot_air(); 
      move_v_cm_direction_ht_degrees(70, 80, 45, 315) ;

      Shoot::auto_shoot_motor_set(75);

      move_v_cm_direction_ht_degrees(15, 35, 45, 315) ;
      //Shoot::auto_shoot_motor_set(93);
      
      

      

      wait(1,sec);
      Shoot::auto_shoot_air(); 
      wait(1.4,sec);
      Shoot::auto_shoot_air(); 
      wait(1.4,sec);
      Shoot::auto_shoot_air();
     

      // move_v_cm_direction_ht_degrees(40, 102, 180, 0) ;

}
void RightRed15sec(){

    // imu.setHeading(270,degrees);
    move_v_cm_direction_ht_degrees(40, 70, 0, 270) ;
    Shoot::auto_shoot_motor_set(100);
    set_Drivetrain_V_org(-30,-30,-30,-30);
    wait(500,msec);
    set_Drivetrain_V_org(-5,-5,-5,-5);
    
      // >>> 機器人移動距離面向角度 (速度,移動距離,機器人移動方向,最終面向方位角度)
      
    // Roller::roller_spin_red(0);
    Collect::auto_taking_in();
    move_rotate_v_sec_direction_ht_degree(20, 0.5, 90, 270) ;

    move_v_cm_direction_ht_degrees(30,5, 270, 270) ;

    Shoot::auto_shoot_motor_set(100);
      //砲塔轉向(角度,速度,布林值)
    Shoot::fort_no_pid_spin(12,20,0);//-10

    
    Shoot::auto_shoot_air(); 
    Shoot::auto_shoot_motor_set(100);
    wait(2.0,sec);
    Shoot::auto_shoot_air(); 
    wait(2.0,sec);
    Shoot::auto_shoot_air();

    move_v_cm_direction_ht_degrees(30,10, 180, 270) ;
    Heading_Turn_abs_D(315,0);

    Collect::auto_taking_in();
    move_v_cm_direction_ht_degrees(20, 280, 225, 315) ;
    // move_v_cm_direction_ht_degrees(20, 40, 225, 315) ;
    // move_v_cm_direction_ht_degrees(50, 30, 225, 315) ;
    // move_v_cm_direction_ht_degrees(20, 40, 225, 315) ;
    // move_v_cm_direction_ht_degrees(50, 30, 225, 315) ; 
    // move_v_cm_direction_ht_degrees(20, 40, 225, 315) ;

    Shoot::auto_shoot_motor_set(90);
      //砲塔轉向(角度,速度,布林值)
    Shoot::fort_no_pid_spin(2,20,0);//-10

    
    Shoot::auto_shoot_air(); 
    Shoot::auto_shoot_motor_set(90);
    wait(2.0,sec);
    Shoot::auto_shoot_air(); 
    wait(2.0,sec);
    Shoot::auto_shoot_air();


}



void autonomous(void) {
  //  LeftRed15sec();
  //  RightRed15sec();
  skills();
  // ..........................................................................
  // ============== Skills 60sec =============

// 可以使用function

// >>>>>>> 前進後退 >>>>>>>>
// move_degree(-30,-30,-30,-30,720,0);

// >>>>>>> 邊移動便轉向 >>>>>>>>>
// move_rotate_v_sec_direction_ht_degree(40, 3, 90.0, 270);  // okokokokokokokokokokokok

// >>>>>>> 自轉 >>>>>>>>>
// Heading_Turn_abs_D(90.0,0);
//

  // ========== 函式說明區 =========
  // >>> 轉 Roller 目前是固定秒數了>> 應該使用optical
  // 

  // >>> 發射幾個 Disc
  // 

  // >>> 吸取Disc ??
  // 

  // >>> 機器人移動距離面向角度 (速度,移動距離,機器人移動方向,最終面向方位角度)
  // move_v_cm_direction_ht_degrees(double v, double _cm, double dir, double ht_degree) 

  // >>> 機器人移動 邊走邊轉 最終面向角度（速度,移動秒數,移動方向角度,最終面向方位角度）
  // move_rotate_v_sec_direction_ht_degree(double v, double secs, double dir, double ht_degree);

  // >>> 陀螺儀旋轉角度 (error 最為速度)
  // Heading_Turn_abs(double degree,bool wait)

  // >>> 基本底盤控制
  // DrivetrainSpin()
  // DrivetrainStop()

  // >>> 設定馬達停止模式
  // DrivetrainStop_mode(int mode){ // bracke mode = 1,coast  mode = 2,hold  mode = 3

  // >>> 設定底盤左右速度
  // set_Drivetrain_V(double VL,double VR)

  // Heading_Turn_abs_D(90.0,0);
  // wait(500,msec);


  // move_v_cm_direction_ht_degrees(40, 100, 315, 0) ;
  // wait(500,msec);

  // // move_v_cm_direction_ht_degrees(40, 120, 0, 0) ;
  // // wait(500,msec);

// Heading_Turn_abs_D(270.0,0);
//       move_v_cm_direction_ht_degrees(10, 5, 90, 270) ; 

// // move_v_cm_direction_ht_degrees(40, 2, 270, 0) ;

//   wait(2000,sec);



  

  
}

void usercontrol(void) {
  // User control code here, inside the loop
  // Chassis::setStop(brake);
  // driving();
  skills();
}

DisC DisC;
int main() {
  pre_auton();
  // Initialize(0);
  Setting::setting(0);
  // wait(3, sec);
  // skills();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
    
}