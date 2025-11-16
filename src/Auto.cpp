#include "Move_Motor.h"
#include "PID.h"
#include "vex.h"
#include "map.h"

void RunAuto() {

Inertial.setRotation(0,deg);
A.set(0);
SHAI.spin(fwd,-100,pct);
wait(100,msec);
PID_Turn(23);
Inertial.setRotation(0,deg);
SHAI.spin(fwd,0,pct);
PID_Move(30,30,450,1000);
PID_Move(10,10,200,1000);
SHAI.spin(fwd,-100,pct);
PID_Turn(-45);
PID_Move(50,50,325,800);
SHAI.spin(fwd,100,pct);
wait(1000,msec);
PID_Turn(135);
PID_Move(50,50,900,1000);

  /*// 示例路径点
  double waypoints[][3] = {
    {500, 300, 45},    // X, Y, 朝向
    {1000, 600, 90},
    {800, 1200, 180},
    {300, 800, 270}
  };
  
  int numWaypoints = sizeof(waypoints) / sizeof(waypoints[0]);
  
  for (int i = 0; i < numWaypoints; i++) {
    // 设置目标点
    setTarget(waypoints[i][0], waypoints[i][1], waypoints[i][2]);
    
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Moving to waypoint %d: X:%.0f Y:%.0f", 
                       i+1, waypoints[i][0], waypoints[i][1]);
    
    // 移动到目标点
    int timeout = 0;
    int maxTimeout = 8000; // 8秒超时
    
    while (!moveToTarget() && timeout < maxTimeout) {
      updatePosition(); // 持续更新位置估计
      wait(20, msec);
      timeout += 20;
    }
    
    if (timeout >= maxTimeout) {
      Brain.Screen.setCursor(6, 1);
      Brain.Screen.print("Waypoint %d timeout", i+1);
    } else {
      Brain.Screen.setCursor(6, 1);
      Brain.Screen.print("Waypoint %d reached", i+1);
    }
    
    // 在路径点停顿
    stopChassis();
    wait(500, msec);
  }
  
  Brain.Screen.setCursor(7, 1);
  Brain.Screen.print("Autonomous routine completed");

//移动到单一目标点
setTarget(10000,0 , 0); 
  while (!moveToTarget()) {
    updatePosition();
    vex::wait(20, vex::msec);
  }

ArmControl(-100);
task::sleep(850);
ArmControl(0);//抬臂

PID_Move(60,60, 200,1000);//直行

ArmControl(100);
task::sleep(600);
ArmControl(0);//降臂

//PID_Turn(-10);//左转

PID_Move(30,50, -950,1200);//后退

A.set(1);//夹子

ArmControl(100);
task::sleep(300);
ArmControl(0);//降臂

PID_Turn(200);
task::sleep(1200);
Move_Stop_brake();

Roller_Motor.spin(fwd,100,pct);
 Roller_Motor2.spin(fwd,100,pct);

PID_Move(50,50, 500,1200);*/

 






// Reverse_Degree(40, 40, -450);
// A.set(1);
// wait(500, timeUnits::msec);

// Roller_Motor.spin(fwd,100,pct);
// Roller_Motor2.spin(fwd,100,pct);
// Roller_Motor3.spin(fwd,100,pct);

// PID_Turn(70);
// Forward_Degree(50, 50,400);

// PID_Turn(150);

// Forward_Degree(50, 50,250);

// Reverse_Degree(50, 50, -100);

// PID_Turn(120);

// Forward_Degree(50, 50,120);
// wait(500, timeUnits::msec);
// Reverse_Degree(50, 50, -100);

// PID_Turn(45);

// wait(1500, timeUnits::msec);



// Reverse_Degree(100, 100, -300);
// Forward_Degree(100, 100, 200);

// Reverse_Degree(50, 50, -350);

// Roller_Motor.stop();
// Roller_Motor2.stop();
// Roller_Motor3.stop();
// A.set(0);
// Roller_Motor.spin(fwd,100,pct); 
// Forward_Degree(80, 80, 150);
// wait(500, timeUnits::msec);

// Reverse_Degree(80, 80, -730);
// PID_Turn(-40);



}