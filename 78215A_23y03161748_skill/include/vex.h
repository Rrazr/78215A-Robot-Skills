/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"
//Chassis
#include "Chassis/odom.h"
#include "Chassis/PID.h"
#include "Chassis/chassis.h"
#include "Chassis/fix.h"
#include "Chassis/noOdom.h"
#include "Chassis/highgoal.h"
//Math-Functions.h
#include "Math-Functions.h"
//Roller
#include "roller.h"
//collect 
#include "collect.h"
//shoot 
#include "shoot.h"
//setting
#include "setting.h"
//turret
#include "turret.h"

//DisCoordinat
#include "DistanceCoordinates.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)