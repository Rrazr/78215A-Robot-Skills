#include "vex.h"

using namespace vex;


// 自動轉藍roller
int Roller::roller_spin_blue_task(){

  double waitTime = Brain.Timer.value();
  while(waitTime + 1 > Brain.Timer.value()){
    if (Color.hue() > 5 && Color.hue() < 90 && Color.isNearObject()){ // if both the roller and "red" color are detected...
      roller.spin(reverse, 20, percent);
      waitUntil(Color.hue() > 205 && Color.hue() < 230 || !Color.isNearObject()); // spin the roller until "blue" color is detected or if the roller is no longer visible
      roller.stop(); // stop roller
      wait(1, sec);
    }else if (Color.hue() > 150 && Color.hue() <= 360 && Color.isNearObject()){ // if both the roller and "blue" color is detected...
      roller.spin(forward, 20, percent);
      waitUntil(Color.hue() > 7 && Color.hue() < 30 || !Color.isNearObject()); // spin the roller until "red" color is detected or if the roller is no longer visible
      roller.stop(); // stop roller
      wait(1, sec);
    }
  }

  //double waittime = Brain.Timer.value()+1.2;
  // Chassis::setVel(-80,-80,-80,-80);
  // roller.setVelocity(33,percent);
  // roller.spin(forward);
  
  // wait(0.5,sec);
  //waitUntil(Color.hue() > 30||Brain.Timer.value()>waittime);
  //waitUntil(Color.hue() < 30||Brain.Timer.value()>waittime);
  

  // if(Color.hue() > 190 && Color.hue() < 280){

  //     roller.setVelocity(30,percent);
  //     roller.spin(forward);
  //     // printf("00\n");
  //     waitUntil(Color.hue() < 30||Brain.Timer.value()>waittime);
  //     roller.stop();
  // }
  // else {

      

  //       roller.setVelocity(-50,percent);
  //       roller.spin(forward);
  //       waitUntil(Color.hue() > 30||Brain.Timer.value()>waittime);
  //       roller.setVelocity(30,percent);
  //       roller.spin(forward);
  //       // printf("00\n");
  //       waitUntil(Color.hue() < 30||Brain.Timer.value()>waittime);
  //       roller.stop();
      
      

  // }
  // Chassis::groupStop();
  // roller.stop();
  return 0;
}
// 同步與否
void Roller::roller_spin_blue(bool Wait){
  if (Wait){
  task roller_spin_blue_task_event = task( Roller::roller_spin_blue_task );
  }else{
    Roller::roller_spin_blue_task();
  }   
}

// 自動轉紅roller
int Roller::roller_spin_red_task(){
   
  //  double waittime = Brain.Timer.value()+1.2;
  // Chassis::setVel(-80,-80,-80,-80);
  roller.setVelocity(100,percent);
  roller.spin(forward);
  wait(1,sec);
  //  waitUntil(Color.hue() < 30||Brain.Timer.value()>waittime);
  //  waitUntil(Color.hue() > 30||Brain.Timer.value()>waittime);
  // if(Color.hue() < 30){
  //     roller.setVelocity(15,percent);
  //     roller.spin(forward);
  //     // printf("00\n");
  //     waitUntil(Color.hue() > 30||Brain.Timer.value()>waittime);
  //     roller.stop();
  // }
  // else {
  //     roller.setVelocity(-50,percent);
  //     roller.spin(forward);
  //     // printf("00\n");
  //     waitUntil(Color.hue() < 30||Brain.Timer.value()>waittime);
  //     roller.stop();
  //     roller.setVelocity(15,percent);
  //     roller.spin(forward);
  //     // printf("00\n");
  //     waitUntil(Color.hue() > 30||Brain.Timer.value()>waittime);
  //     roller.stop();
  // }
  // Chassis::groupStop();
   roller.stop();
  return 0;
}

// 同步與否
void Roller::roller_spin_red(bool Wait)
{
  if (Wait){
  task roller_spin_red_task_event = task( Roller::roller_spin_red_task );
  }else{
    Roller::roller_spin_red_task();
  }   
}


//遙控轉動roller
void Roller::roller_spin(void){
  //roller.spin(forward, 50, percent);
  if(Controller1.ButtonL1.pressing()){
    roller.setVelocity(50,percent);//roller轉向朝外
    roller.spin(forward);
  }else if(Controller1.ButtonL2.pressing()){
    roller.setVelocity(-50,percent);//roller轉向朝內
    roller.spin(forward);
  }else{
    roller.stop();
  }
}

void Roller::roller_load_on(){
  printf("1\n");
  roller.setVelocity(100,percent);
  roller.spin(forward);
  Intake.setVelocity(70,percent);//55
  Intake.spin(forward);
}

void Roller::roller_load_off(){
  roller.setVelocity(0,percent);
  roller.spin(forward);
  Intake.setVelocity(0,percent);
  Intake.spin(forward);
}


void Roller::roller_load_out(){
  roller.setVelocity(-100,percent);
  roller.spin(forward);
  Intake.setVelocity(80,percent);
  Intake.spin(forward);
}
