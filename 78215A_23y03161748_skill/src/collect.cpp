#include "vex.h"

using namespace vex;


// 遙控吸入吐出馬達
int Collect::taking_task(){
  while (true){
    // if(Controller1.ButtonL1.pressing()){
    //   // waitUntil(!Controller1.ButtonL1.pressing());
    //   // while(true){
    //     if (Color.hue() > 5 && Color.hue() < 90 && Color.isNearObject()){ // if both the roller and "red" color are detected...
    //       roller.spin(reverse, 60, percent);
    //       // waitUntil(Color.hue() > 205 && Color.hue() < 230 || !Color.isNearObject()); // spin the roller until "blue" color is detected or if the roller is no longer visible
    //       // roller.stop(); // stop roller
    //       // wait(1, sec);
    //     }else if (Color.hue() > 150 && Color.hue() <= 360 && Color.isNearObject()){ // if both the roller and "blue" color is detected...
    //       roller.spin(forward, 60, percent);
    //       // waitUntil(Color.hue() > 7 && Color.hue() < 30 || !Color.isNearObject()); // spin the roller until "red" color is detected or if the roller is no longer visible
    //       // roller.stop(); // stop roller
    //       // wait(1, sec);
    //     }
    //     printf("roller  %.2f\n",Color.hue());
    //     if(Controller1.ButtonL1.pressing()){
    //       if(!Controller1.ButtonL1.pressing()){
    //         break;
    //       }
    //     }
    //   // }
    //   roller.stop();
    // }
    if(Controller1.ButtonX.pressing()){
      roller.setVelocity(100,percent);//向內吸入
      roller.spin(forward);
    }
    else if(Controller1.ButtonB.pressing()){
      roller.setVelocity(-100,percent);//向外吐出
      roller.spin(forward);
    }
    else if(Controller1.ButtonL1.pressing()){
      Intake.setVelocity(100,percent);
      Intake.spin(forward);
    }
    else if(Controller1.ButtonL2.pressing()){
      Intake.setVelocity(-100,percent);
      Intake.spin(forward);      
    }
    else if(Controller1.ButtonRight.pressing()){
      Intake.setVelocity(0,percent);
      Intake.spin(forward);      
    }
    else if(Controller1.ButtonR2.pressing()){
      roller.stop();//停止
    }
  wait(30,msec);
  }
  return 0;
}  

//自動吸入
void Collect::auto_taking_in(void){
    roller.setVelocity(100,percent);
    roller.spin(forward);
    Intake.spin(forward, 100, percent);
  
} 


//自動吐出
void Collect::auto_taking_out(void){
    roller.setVelocity(-100,percent);
    roller.spin(forward);
    Intake.spin(forward, 100, percent);
  
} 

//自動停止
void Collect::auto_taking_off(void){
    roller.stop();
    Intake.stop();
} 


double Collect::auto_taking_in_second = 0;//初始化

// 在指定時間內自動吸入
int Collect::auto_taking_in_time_task(){

    Collect::auto_taking_in_second = auto_taking_in_second + Brain.Timer.value();//計時器初始化
    roller.setVelocity(100,percent);
    roller.spin(forward);
    waitUntil(auto_taking_in_second < Brain.Timer.value());// 計時器判斷
    roller.stop();
  
  return 0;
} 

//同步與否
void Collect::auto_taking_in_time(double second,bool Wait)
{
  Collect::auto_taking_in_second=second;
  if (Wait){
  task auto_taking_in_time_task_event = task( Collect::auto_taking_in_time_task );
  }else{
    Collect::auto_taking_in_time_task();
  }   
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

double Collect::auto_taking_out_second=0;//初始化

// 在指定時間內自動吐出
int Collect::auto_taking_out_time_task(){

  auto_taking_out_second = auto_taking_out_second + Brain.Timer.value();// 計時器初始化
    roller.setVelocity(100,percent);
    roller.spin(forward);
    waitUntil(auto_taking_out_second < Brain.Timer.value());//計時器判斷
    roller.stop();
  
  return 0;
} 

// 同步與否
void Collect::auto_taking_out_time(double second,bool Wait)
{
  auto_taking_out_second=second;
  if (Wait){
  task auto_taking_out_time_task_event = task( Collect::auto_taking_out_time_task );
  }else{
    Collect::auto_taking_out_time_task();
  }   
}