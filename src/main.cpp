/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Clawbot Competition Template                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Left_Motor1          motor         1               
// Left_Motor2          motor         2               
// Left_Motor3          motor         3               
// Right_Motor1         motor         10              
// Right_Motor2         motor         9               
// Right_Motor3         motor         8               
// SHAI                 motor         7               
// DINGBU               motor         11              
// Inertial             inertial      16              
// A                    digital_out   A               
// B                    digital_out   B               
// Rotation19           rotation      19              
// Rotation20           rotation      20              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "PID.h"
#include "Move_Motor.h"
#include "Auxiliary.h"
#include "Auto.h"
#include "map.h"
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) 
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  EncoderReset();
  EncoderInit();
  Inertial.calibrate();
  wait(2000, timeUnits::msec);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


  


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) 
{ 





  //initLocalization();这里





  
  RunAuto();
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void thread_Move()
{
  int Axis_1,Axis_2,Axis_3,Axis_4;
  bool Button_R1,Button_R2;
  bool Button_L1,Button_L2;
  while (1) 
  {
    Axis_1=Controller1.Axis1.value();
    Axis_2=Controller1.Axis2.value();
    Axis_3=Controller1.Axis3.value();
    Axis_4=Controller1.Axis4.value();

    Button_R1=Controller1.ButtonR1.pressing();
    Button_R2=Controller1.ButtonR2.pressing();
    Button_L1=Controller1.ButtonL1.pressing();
    Button_L2=Controller1.ButtonL2.pressing();
    
    //====================================================手柄定义
    //updatePosition(); // 即使在手动控制时也更新位置

    if (abs(Axis_3)<5 && abs(Axis_1)>5)
    {
      Move(0.7 * Axis_1,- 0.7 * Axis_1);
    }
    else if (abs(Axis_3)>5)
    {
      Move(Axis_3 + 0.7 * Axis_1,Axis_3 - 0.7 * Axis_1);
    }
    else
    {
      Move_Stop_brake();
    }//底盘设置

    if (Button_R1)
    {
     SHAIControl(-100);
    }
    else if (Button_R2) 
    {
      SHAIControl(100);
    }
    else if(Button_L1==0)
    {
      SHAIControl(0);
    }//吸吐球


if (Button_L1)
    {
     DINGBUControl(-100);
     SHAIControl(-100);
    }
    else if (Button_L2) 
    {
      DINGBUControl(100);
    }
    else 
    {
      DINGBUControl(0);
    }
  }
}//底盘控制

void drivercontrol(void) 
{
  int f = 0;
  int g = 0;
  thread task1,task2;
  bool Button_A,Button_B,Button_Y,Button_X;
  bool Button_L1,Button_L2,Button_R1,Button_R2;
  bool Button_Up,Button_Down,Button_Left,Button_Right;
    EncoderReset();
  //====================================================手动参数定义
  // User control code here, inside the loop
  while (1) 
  {
    Button_A=Controller1.ButtonA.pressing();
    Button_Y=Controller1.ButtonY.pressing();
    
    Button_L1=Controller1.ButtonL1.pressing();    
    Button_L2=Controller1.ButtonL2.pressing();
    Button_R1=Controller1.ButtonR1.pressing();    
    Button_R2=Controller1.ButtonR2.pressing();

    Button_Up=Controller1.ButtonUp.pressing();
    Button_Down=Controller1.ButtonDown.pressing();
    Button_Left=Controller1.ButtonLeft.pressing();
    Button_Right=Controller1.ButtonRight.pressing();

    Button_X=Controller1.ButtonX.pressing();
    Button_A=Controller1.ButtonA.pressing();
      Button_B=Controller1.ButtonB.pressing();
    //====================================================手柄定义
     task1 = thread(thread_Move);
        
     if(Button_A and f == 0){
       A.set(1);
       waitUntil(!Controller1.ButtonA.pressing());
       f = 1;
     }
     else if(Button_A and f == 1){
       A.set(0);
       waitUntil(!Controller1.ButtonA.pressing());
       f = 0;
     }
     else if(f == 0){
     A.set(0);
     }//导入
     
     if(Button_Y and g == 0){
       B.set(1);
       waitUntil(!Controller1.ButtonY.pressing());
       g = 1;
     }
     else if(Button_Y and g == 1){
       B.set(0);
       waitUntil(!Controller1.ButtonY.pressing());
       g = 0;
     }
     else if(g == 0){
     B.set(0);
     }//arm

     

    //余下部分逻辑在这之下续写
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    }
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(drivercontrol);

  // 初始化定位系统和卡尔曼滤波
  initLocalization();
  
  init();
  printf("build time： %s\n\n",__TIME__);
  Competition.autonomous(autonomous);
  
  // Run the pre-autonomous function.
  while(1) {
    wait(100,msec);
    if(!Inertial.installed()) {
      printf("no Inertial!\n");
    }
  }
}
