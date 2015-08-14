#include <windows.h>
#include <rtapi.h>
#include <wchar.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>
#include <conio.h>
#include <vector>
//#include <ctype.h>
#include <math.h>
//#include <errno.h>
//#include <Eigen/Dense>
#include "ControlLita.h"
#include "Gripper.h"

#include "finger.h"

//define function in main

//LuoLitaArm Global Constant---
//define-----------------------
#define kb_ESC 27
#define kb_Space 32


#define SAMPLING_TIME_DISPLAY 50

//const------------------------
const float RVmax = 100;//(degree/sec)
const float RAmax = 100;//
const float LVmax = 200;//(mm/sec)
const float LAmax = 200;
//-----------------------------

//LuoLitaArm Global Variable---
//-----------------------------
extern char kbCmd;
extern int mode_cmd;
extern int mode_display;

extern float q_trgt[ROBOT_DOF];
extern Eigen::Matrix4f T_trgt;
extern Eigen::Matrix4f T_ready3;
extern Vector7f q_ready3;

extern Gripper LitaHand;

//-----------------------------

//LuoLitaArmFunc Declaration
void init_LuoLita_1();
void init_LuoLita_2();

void Holding();
void Move_J_Abs( float (&qd)[ROBOT_DOF] );
void Move_J_Rel( float (&delta_qd)[ROBOT_DOF] );
void Move_J_Abs_sameTime( float (&qd)[ROBOT_DOF], float time );
void Move_J_Rel_sameTime( float (&delta_qd)[ROBOT_DOF], float time );
void Move_L_Abs( const Eigen::Matrix4f& Td , float Psi_d);
void Move_Contour();// const Eigen::Matrix4f& Td , float Psi_d);
void Move_L_Rel( const Vector6f& deltaTd ,float delta_Psi_d);

void GoToReadyPose1();
void GoToReadyPose2();
void BackToReadyPose1();
void BackToReadyPose2();
void BackToDHhome();
void ByeBye();

void MainLoop_keyboard();
void MyLoop_keyboard();

void DisplayLoop(float);

void OutputData();

void setDefaultArmSpeed(float percentage = 1.0f);
void readDrawPoints();
string int2str(int);
vector<string> split(string, char);


