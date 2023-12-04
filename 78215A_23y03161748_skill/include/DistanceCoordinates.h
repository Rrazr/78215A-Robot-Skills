#ifndef _DISTANCECOORDINATE_H_
#define _DISTANCECOORDINATE_H_

extern double current_x,current_y;//現在座標值(x,y)
class DisC{
  public:
    static constexpr double DR_to_center=117.0;//右
    static constexpr double DB_to_center=226.0;//後
    static constexpr double DL_to_center=130.0;//左
    static constexpr double DF_to_center=206.0;//前
    static double DR,DB,DL,DF,d;//牆的距離
    static double PLF,PLB,PRF,PRB;//角度
    // static double current_x,current_y;//現在座標值(x,y)
     
    static void refreshPosition();//四顆馬達的角度
    static void refreshDistance();//更新現在牆壁to中心點的值
    static double Heading();//方向 -179~180
    static void InertialSpin(double Direction);//自轉(方向)
    static void refreshCoordinate(int Calibrate);//更新座標位置(是否要轉正)
    static void move_distance(double target_x,double target_y,int speed);//用distance_sensor走到目標位置(目標座標x,目標座標y,速度)
    static void move_distance_ACC(double target_x,double target_y,int speed);//用distance_sensor走到目標位置(目標座標x,目標座標y,速度)[加減速] 未測試
    static void FaceHighGoal();//朝向highGoal
    static double ACC(double max_speed,double Distance,double input);//加減速
    static void moveACC(double LF,double LB,double RF,double RB,double Distance);//四顆馬達移動 未完成
};

#endif