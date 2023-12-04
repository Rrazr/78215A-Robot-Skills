#include "vex.h"

using namespace vex;


//遙控的氣動程式
int Shoot::shoot_air_on(){
  //Controller1.Screen.clearScreen();
  while(true){

    if(Controller1.ButtonR1.pressing()){
      // wait(20,msec);
      shoot_air.set(true);//氣動推出
      waitUntil(!Controller1.ButtonR1.pressing());//等待R1按下及放開
      // shoot_air.set(true);//氣動推出
      // wait(200,msec);//使力傳至盤子
      shoot_air.set(false);//氣動縮回

    }
    wait(50,msec);//迴圈緩衝
  }
  
  return 0;
}

//發射馬達初始化
bool Shoot::shoot_OnOff = true;//初始狀態
int  Shoot::shoot_velocity = 65;//初速
double correction_Driving = 0;
double real_speed_Driving = 0;

void launch(){
  Shoot::shoot_OnOff=!Shoot::shoot_OnOff;
}

int Shoot::shooting(){
  //Controller1.Screen.clearScreen();
  Controller1.ButtonA.pressed(launch);
  while(true){
    correction_Driving = Shoot::shoot_velocity - L_shoot.velocity(percent);
    real_speed_Driving = Shoot::shoot_velocity + correction_Driving;
    if(Controller1.ButtonUp.pressing()){
        Shoot::shoot_velocity++;//long range shooting
        // Shoot::shoot_velocity = 60;
    }
    if(Controller1.ButtonDown.pressing()){
        Shoot::shoot_velocity--;//close range shooting
        // Shoot::shoot_velocity = 65;
    }  
    /*if(Controller1.ButtonA.pressing()){
      Shoot::shoot_OnOff=!Shoot::shoot_OnOff;//馬達開關狀態切換
      waitUntil(!Controller1.ButtonA.pressing());
    }*/
    if(Shoot::shoot_OnOff){
      L_shoot.spin(forward, real_speed_Driving*120, voltageUnits::mV);
      R_shoot.spin(forward, real_speed_Driving*120, voltageUnits::mV);
    }else{
      L_shoot.stop();
      R_shoot.stop();
    }
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("-> = %.01f",L_shoot.velocity(percent));
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print("%f", L_shoot.velocity(percent));
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print("%d", Shoot::shoot_velocity);
    // Controller1.Screen.setCursor(2,1);
    // Controller1.Screen.print("input = %.01d",shoot_velocity);
    wait(50,msec);
  }
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
// 自動氣動推出即縮回
void Shoot::auto_shoot_air(void){
  //Controller1.Screen.clearScreen();
      shoot_air.set(true);
      wait(200,msec);
      shoot_air.set(false);
      wait(200,msec);
}

//////////////////////////////////////////////////////////////  
// 自動發射馬達開啟
void Shoot::auto_shoot_motor_on(){
    L_shoot.spin(forward);
    R_shoot.spin(forward);
}

//////////////////////////////////////////////////////////////////
// 自動發射馬達關閉
void Shoot::auto_shoot_motor_off(){
      L_shoot.stop();
      R_shoot.stop();
}

//////////////////////////////////////////////////////////////////////
// 自動發射馬達速度設定
void Shoot::auto_shoot_motor_set(double shoot_motor_velocity){
  L_shoot.spin(forward, shoot_motor_velocity*120, voltageUnits::mV);
  R_shoot.spin(forward, shoot_motor_velocity*120, voltageUnits::mV);
}

double Shoot::pid_turn_P =0.0;
double Shoot::pid_turn_I =0.0;
double Shoot::pid_turn_D =0.0;
double Shoot::pid_turn_power =0.0;
double Shoot::pid_turn_totalError =0.0;

double Shoot::pid_turn(double turnerror, double last_turnerror){
 
  pid_turn_totalError = 0; 
  double derivative = turnerror - last_turnerror; 
  if(fabs(turnerror) < 30  && turnerror !=0){
    Shoot::pid_turn_totalError += turnerror;
  }
  else{
    Shoot::pid_turn_totalError = 0;
  }

  Shoot::pid_turn_P = Shoot::pid_turn_Kp * turnerror;
  Shoot::pid_turn_I = Shoot::pid_turn_totalError * Shoot::pid_turn_Ki;
  Shoot::pid_turn_D = Shoot::pid_turn_Kd * derivative;

  if(Shoot::pid_turn_I > Shoot::pid_turn_I_limit){
    Shoot::pid_turn_I = Shoot::pid_turn_I_limit;
  }
  
  Shoot::pid_turn_power = Shoot::pid_turn_P + Shoot::pid_turn_I + Shoot::pid_turn_D;
  return Shoot::pid_turn_power;

}

// ===== 砲台調整 =====

void Shoot::fort_pid_spin(double preferredAngle){

    double currentHeading =imuFort.heading(degrees);
    
    Shoot::pid_turn_totalError = 0;

    double turnAngle = 0;
    turnAngle = noOdom::find_min_angle(preferredAngle, currentHeading);
    double last_turnAngle = turnAngle;

    while((fabs(turnAngle) > 0.7)){

      currentHeading = imuFort.heading(degrees);

      turnAngle = noOdom::find_min_angle(preferredAngle, currentHeading);
      double Turnpower = Shoot::pid_turn(turnAngle, last_turnAngle);
      // if(Turnpower<20.0)Turnpower=30.0*Turnpower/fabs(Turnpower);
      fort.spin(forward,Turnpower,percent);
      last_turnAngle = turnAngle;
      // printf("Inertial= %f\n",Inertial.heading(degrees));

    }

    fort.setStopping(hold);
    fort.stop();
}


double Shoot::fortVelocity = 0 ,  Shoot::fortAngle = 0;

int Shoot::fort_no_pid_spin_task(){

    fort.setStopping(hold);
    //fort.setPosition(0,degrees);
    double targetDeg = Shoot::fortAngle >= 0 ? Shoot::fortAngle*84/12 + fort.position(degrees) : Shoot::fortAngle*84/12 - fort.position(degrees);
    if (fabs(Shoot::fortAngle) < 90.0){
        while(fabs (fort.position(degrees)) < fabs (targetDeg)){
          fort.spin(forward , Shoot::fortVelocity * fabs(Shoot::fortAngle) / Shoot::fortAngle, percent);
        }
    }
    
    fort.stop();
    return 0;
}

void Shoot::fort_no_pid_spin(double Angle , double fortV , bool wait){

  Shoot::fortVelocity = fortV ;
  Shoot::fortAngle = Angle;
  if (wait){
    task fort_no_pid_spin_event = task( Shoot::fort_no_pid_spin_task );
  }else{
    Shoot::fort_no_pid_spin_task();
  }   
}