#include "vex.h"
using namespace vex;





double current_x = 0;
double current_y = 0;
double DisC::d = 0;
double DisC::DR = 0;
double DisC::DF = 0;
double DisC::DL = 0;
double DisC::DB = 0;
double DisC::PLF = 0,DisC::PLB = 0,DisC::PRF = 0,DisC::PRB  = 0;

//加減速[量值,單顆]

double DisC::ACC(double max_speed,double Distance,double input_distance){
  input_distance=fabs(input_distance);
  //max_speed[理想最高速度]
  double m=0.1;//斜率[(最佳最高速度-0.0[初始速度])/移動距離)]
  double high_speed=0.0+((Distance/2.0)*m);//high_speed[實際最高速度]
  double output_speed=0;
  if (high_speed>max_speed){
    high_speed=max_speed;
  }

  if (input_distance<((high_speed-0.0)/m)){//加速
    output_speed=0.0+(input_distance*m);
  }
  else if(input_distance>(Distance-(high_speed/m))){//減速
    output_speed=fabs(high_speed-((input_distance-(Distance-(high_speed/m)))*m));
  }
  else{//等速
    output_speed=high_speed;
  }
  
  return output_speed;
}

//四顆馬達的角度

void DisC::refreshPosition(){
  PLF=fabs(leftFront.position(degrees));
  PLB=fabs(leftBack.position(degrees));
  PRF=fabs(rightFront.position(degrees));
  PRB=fabs(rightBack.position(degrees));
}

//四顆馬達移動 未完成

void DisC::moveACC(double SLF,double SLB,double SRF,double SRB,double Distance){
  DisC::refreshPosition();
  while (PLF<Distance&&PLB<Distance&&PRF<Distance&&PRB<Distance){
    refreshPosition();
    leftFront.spin(forward,ACC(SLF,Distance,PLF),percent);
    leftBack.spin(forward,ACC(SLB,Distance,PLB),percent);
    rightFront.spin(forward,ACC(SRF,Distance,PRF),percent);
    rightBack.spin(forward,ACC(SRB,Distance,PRB),percent);
  }
}

//更新現在牆壁to中心點的值

void DisC::refreshDistance(){
  DR=DistanceRight.objectDistance(mm)+DR_to_center;
  DB=DistanceBack.objectDistance(mm)+DB_to_center;
  DL=DistanceLeft.objectDistance(mm)+DL_to_center;
  DF=DistanceFront.objectDistance(mm)+DF_to_center;
  //printf("DR=%f , DB=%f , DF = %f\n", DR, DB, DF);
}

//方向 -179~180

double DisC::Heading(){

  return imu.heading()-((imu.heading()>180)*360.0);

}

//自轉(方向)

void DisC::InertialSpin(double Direction){

  double error=0.0;
  while(fabs(Direction-Heading())>0.5){
    double SpinV=(Direction-Heading())/fabs(Direction-Heading())*(30+fabs(Direction-Heading())*0.9+(fabs(Direction-Heading())-error)*53.0);
    leftFront.spin(forward,SpinV,percent);
    leftBack.spin(forward,SpinV,percent);
    rightFront.spin(forward,-SpinV,percent);
    rightBack.spin(forward,-SpinV,percent);
    error=fabs(Direction-Heading());
    }
  leftFront.stop(hold);
  leftBack.stop(hold);
  rightFront.stop(hold);
  rightBack.stop(hold);
  wait(100,msec);

}

//更新座標位置(是否要轉正)

void DisC::refreshCoordinate(int Calibrate){
  if (Calibrate==1){
  if (Heading()>0){
    if(Heading()>45){
      if(Heading()>135){
        InertialSpin(180);
        d=2;
      }
      else{
        InertialSpin(90);
        d=1;
      }
    }
    else{
      InertialSpin(0);
      d=0;
    }
  }
  else {
    if (Heading()<-45){
      if(Heading()<-135){
        InertialSpin(-180);
        d=2;
      }
      else{
        InertialSpin(-90);
        d=3;
      }
    }
    else {
      InertialSpin(0);
      d=0;
    }
  }
  }
    refreshDistance();//丟入座標值
    if(d==0){
      current_x=DL*(DL<5000)+(3560.0-DR)*(DL>5000);
      current_y=DB*(DB<5000)+(3560.0-DF)*(DB>5000);
    }
    if(d==1){
      current_x=DB*(DB<5000)+(3560.0-DF)*(DB>5000);
      current_y=DR*(DR<5000)+(3560.0-DL)*(DR>5000);
    }
    if(d==2){
      current_x=DR*(DR<5000)+(3560.0-DL)*(DR>5000);
      current_y=DF*(DF<5000)+(3560.0-DB)*(DF>5000);
    }
    if(d==-1){
      current_x=DF*(DF<5000)+(3560.0-DB)*(DF>5000);
      current_y=DL*(DL<5000)+(3560.0-DR)*(DL>5000);
    }
  
  printf("%f\n", d);
}

//用distance_sensor走到目標位置(目標座標x[mm],目標座標y[mm],速度)

void DisC::move_distance(double target_x,double target_y,int speed){ 
  float kp = 0.05;
  float kpr = 0.05;
  refreshCoordinate(1);
    while (fabs(current_x-target_x)>2||fabs(current_y-target_y)>2){
      refreshCoordinate(0);
      double error_x=target_x-current_x,error_y=target_y-current_y;
      leftFront.spin(forward,(((-1+(2*(d==0||d==1))))*error_x*kp)+(((-1+(2*(d==0||d==-1))))*error_y*kp)+(((d*90)-Heading())*kpr),percent);
      leftBack.spin(forward,(((-1+(2*(d==1||d==2))))*error_x*kp)+(((-1+(2*(d==0||d==1))))*error_y*kp)+(((d*90)-Heading())*kpr),percent);
      rightFront.spin(forward,(((-1+(2*(d==1||d==2))))*error_x*kp)+(((-1+(2*(d==0||d==1))))*error_y*kp)-(((d*90)-Heading())*kpr),percent);
      rightBack.spin(forward,(((-1+(2*(d==0||d==1))))*error_x*kp)+(((-1+(2*(d==0||d==-1))))*error_y*kp)-(((d*90)-Heading())*kpr),percent);
      wait(10,msec);
    }
  leftFront.stop(hold);
  leftBack.stop(hold);
  rightFront.stop(hold);
  rightBack.stop(hold);
  wait(100,msec);  
}

//用distance_sensor走到目標位置(目標座標x[mm],目標座標y[mm],速度)[加減速] 

void DisC::move_distance_ACC(double target_x,double target_y,int max_speed){ 
  refreshCoordinate(1);//校正方向 算出現在座標值
  double error_x=target_x-current_x,error_y=target_y-current_y;//算出起點到終點總長
  double kp = 0.3;//車向偏移kp
  double original_x=current_x,original_y=current_y;//起點座標值
  double refresh_error_x=target_x-current_x,refresh_error_y=target_y-current_y;//現在值到終點總長
    while (fabs(refresh_error_x)>2||fabs(refresh_error_y)>2){

      refreshCoordinate(0);//算出現在座標值
      /////////////加減速(最高速度)(起點到終點總長換算成馬達角度[x座標]])///(起點到現在座標值的長度換算成馬達角度[x座標])/////////////////////加減速(最高速度)(起點到終點總長換算成馬達角度[y座標])///(起點到現在座標值的長度換算成馬達角度[y座標])///////
      double speed_x=ACC(max_speed,Math::CmToDeg(error_x*sqrt(2.0)/10.0),Math::CmToDeg((current_x-original_x)*sqrt(2.0)/10.0)),speed_y=ACC(max_speed,Math::CmToDeg(error_y*sqrt(2.0)/10.0),Math::CmToDeg((current_y-original_y)*sqrt(2.0)/10.0));
      refresh_error_x=target_x-current_x,refresh_error_y=target_y-current_y;//現在值到終點總長
      //////////////////////( 馬達轉向[現在值到終點總長 == 車的朝向{d*90}]  )/加減速////(馬達轉向[現在值到終點總長==車的朝向{d*90}])//////加減速////(       車向校正       )/////////
      leftFront.spin(forward,(-1+(((refresh_error_x>=0)==(d==0||d==1))*2))*speed_x+((-1+(((refresh_error_y>=0)==(d==0||d==-1))*2))*speed_y)+(((d*90)-Heading())*kp),percent);
      leftBack.spin(forward,(-1+(((refresh_error_x>=0)==(d==1||d==2))*2))*speed_x+((-1+(((refresh_error_y>=0)==(d==0||d==1))*2))*speed_y)+(((d*90)-Heading())*kp),percent);
      rightFront.spin(forward,(-1+(((refresh_error_x>=0)==(d==1||d==2))*2))*speed_x+((-1+(((refresh_error_y>=0)==(d==0||d==1))*2))*speed_y)-(((d*90)-Heading())*kp),percent);
      rightBack.spin(forward,(-1+(((refresh_error_x>=0)==(d==0||d==1))*2))*speed_x+((-1+(((refresh_error_y>=0)==(d==0||d==-1))*2))*speed_y)-(((d*90)-Heading())*kp),percent);
      wait(5,msec);
    }
  leftFront.stop(hold);
  leftBack.stop(hold);
  rightFront.stop(hold);
  rightBack.stop(hold);
  wait(100,msec);  
}

//移動

/*void move(){
  while 
  leftFront.spin(forward,,percent);
  leftBack.spin(forward,,percent);
  rightFront.spin(forward,,percent);
  rightBack.spin(forward,,percent);
}
+*/

//朝向highGoal

void DisC::FaceHighGoal(){

    refreshCoordinate(1);
    InertialSpin(atan((current_x-457.0)/(current_y-3123.0))*180.0/M_PI);

  }

/*
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  imu.calibrate();
  wait(2,sec);
  imu.setHeading(0,degrees);
  
  //double time0=Brain.Timer.value();
  //InertialSpin(90);
  move_distance(400.0,400.0,30);
  wait(1,sec);
  //move_distance(1800.0,1200.0,30);
  wait(1,msec);
  move_distance(1200.0,600.0,30);
  FaceHighGoal();
  //FaceHighGoal();
  while(1){
  printf("x=%lf;y=%lf,dis=%lf,d=%lf\n",current_x,current_y,Heading(),atan((current_x-460.0)/(current_y-3123.0))*180.0/M_PI);
  wait(10,msec);
  }
  
}
*/