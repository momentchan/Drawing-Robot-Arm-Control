#include "LuoLitaArm.h"

using namespace std;
using namespace Eigen;

//LuoLitaArm Global Variable---
//-----------------------------
char kbCmd;
char kbCmd_1;
char kbCmd_2;

int kbDelay = 500;
int kbSpeed = 40;

//int mode_cmd = -1;// 0:go to ready pose 1:Joint 2:Cartesian
//int mode_cmd_1 = -1;//-1:need to record last target command

int mode_cmd = 0;
int mode_cmd_1 = 0;

int sub_mode_cmd = 0;
int sub_mode_cmd_1 = 0;

bool mode_free = false;
bool mode_imped = false;

bool movetoReady = false;

bool CmdType = true;//  true:Abs false:Rel

int ParameterIndex = 0;

float q_trgt[ROBOT_DOF];
Matrix4f T_trgt;
Matrix4f T_trgt_backup;

Matrix4f Td_start_record;
Matrix4f Td_end_record;
float Psid_start_record;
float Psid_end_record;
int TrgtRecNum = 8;
Matrix4f Td_record[8];
float Psid_record[8];

float Psi_trgt = 0.0f;

Vector6f delta_T_trgt;

Vector7f q_ready1;
Vector7f q_ready2;
Vector7f q_ready3;
Matrix4f T_ready3;
Vector7f q_byebye;

float RV_percentage = 10.0f;
float RA_percentage =  5.0f;
float LV_percentage = 10.0f;
float LA_percentage =  5.0f;

float Vel_Limit_ENC_default[ROBOT_DOF];
float Acc_Limit_ENC_default[ROBOT_DOF];
float Dec_Limit_ENC_default[ROBOT_DOF];

float Lin_Vel_limit_default;
float Lin_Acc_limit_default;
float Lin_Dec_limit_default;

float Ang_Vel_limit_default;
float Ang_Acc_limit_default;
float Ang_Dec_limit_default;

float Jn_Vel_limit_default;
float Jn_Acc_limit_default;
float Jn_Dec_limit_default;

float qSpeed;

float LinSpeed;
float AngSpeed;

int mode_display = 0;

int DisplayIndex = 0;

int TestNumber = 1;
int TestStep = 0;

Gripper LitaHand;
	

//-----------------------------

//function in main
void init_LuoLita_1()
{
	cout << endl << " please enlarge the console window (press Space to continue)" << endl;
	
	while(1)
	{
		if( _kbhit() )
			 kbCmd = _getch();
		if( kbCmd == kb_Space)
		{
			break;
		}
	}
	kbCmd ='~';

	cout << endl << " initialize..." << endl;

	Init_RobotLita();
	Init_ControlLita();
	
	//qSpeed = 0.01f*RV_percentage * deg2rad*RVmax * nkbSpeed /1000;
	qSpeed = 0.01f*0.5f*RV_percentage * deg2rad*RVmax/10;

	for( int i = 0; i < ROBOT_DOF; i++ )
	{
		Vel_Limit_ENC_default[i] = 10.0f*( 0.01f*0.5f*RV_percentage * deg2ENC[i]*RVmax / 1000.0f ) *SAMPLING_TIME;
		Acc_Limit_ENC_default[i] = ((0.01f*0.5f*RV_percentage * deg2ENC[i]*RAmax / 1000.0f ) *SAMPLING_TIME) /1000 *SAMPLING_TIME;
		Dec_Limit_ENC_default[i] = 0.5f*Acc_Limit_ENC_default[i];

		Vel_Limit_ENC[i] = Vel_Limit_ENC_default[i];
		Acc_Limit_ENC[i] = Acc_Limit_ENC_default[i];
		Dec_Limit_ENC[i] = Dec_Limit_ENC_default[i];
	}	
	
	//LinSpeed = 0.01f*LV_percentage * 0.001f*LVmax * nkbSpeed /1000;;
	//AngSpeed = 0.01f*RV_percentage * deg2rad*RVmax * nkbSpeed /1000;;
	LinSpeed = 0.01f*LV_percentage * 0.001f*LVmax/10.0f;
	AngSpeed = 0.01f*RV_percentage * deg2rad*RVmax/10.0f;

	Ang_Vel_limit_default = ( 0.01f*RV_percentage * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C;
	Ang_Acc_limit_default = ((0.01f*RA_percentage * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
	Ang_Dec_limit_default = Ang_Acc_limit_default;
	Lin_Vel_limit_default = ( 0.01f*LV_percentage * 0.001f*LVmax / 1000.0f ) *SAMPLING_TIME_C;
	Lin_Acc_limit_default = ((0.01f*LA_percentage * 0.001f*LAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
	Lin_Dec_limit_default = Lin_Acc_limit_default;
	Jn_Vel_limit_default = Ang_Vel_limit_default;
	Jn_Acc_limit_default = Ang_Acc_limit_default;
	Jn_Dec_limit_default = Jn_Acc_limit_default;

	Ang_Vel_limit = Ang_Vel_limit_default;
	Ang_Acc_limit = Ang_Acc_limit_default;
	Ang_Dec_limit = Ang_Dec_limit_default;
	Lin_Vel_limit = Lin_Vel_limit_default;
	Lin_Acc_limit = Lin_Acc_limit_default;
	Lin_Dec_limit = Lin_Dec_limit_default;
	Jn_Vel_limit = Jn_Vel_limit_default;
	Jn_Acc_limit = Jn_Acc_limit_default;
	Jn_Dec_limit = Jn_Dec_limit_default;

	TOL_R = Ang_Dec_limit / 2.0f;
	if( TOL_R > 0.00005f)
		TOL_R = 0.00005f;
	
	TOL_L = Lin_Dec_limit / 2.0f;
	if( TOL_L > 0.00005f)
		TOL_L = 0.00005f;

	q_ready1 << 0.0f, -pi/4.0f,  0.0f,  5.0f*pi/6.0f,  0.0f, -pi/12.0f,  0.0f;
	q_ready2 << 0.0f, pi/12.0f,  0.0f,  2.0f*pi/3.0f,  0.0f,   pi/4.0f,  0.0f;

	T_ready3 << -1.0f,  0.0f,  0.0f,  0.56f,//0.56
				 0.0f,  1.0f,  0.0f,  0.0f,
				 0.0f,  0.0f, -1.0f,  0.004f,//-0.050
				 0.0f,  0.0f,  0.0f,  1.0f;

	for(int i = 0; i<TrgtRecNum; i++)
	{
		Td_record[i] = T_ready3;
		Psid_record[i] = 0.0f;
	}

	
	q_byebye << 0.0f, -pi/2.0f + deg2rad*0.5f, 0.0f, pi - deg2rad*0.12f, 0.0f, 0.0f, 0.0f;

	cout << endl << " arm parameter: "<< endl << endl;
	
	cout << " qSpeed: " << qSpeed << endl;
	cout << " Vel_Limit_ENC[0,1]: " << Vel_Limit_ENC_default[0] << endl;
	cout << " Vel_Limit_ENC[2~6]: " << Vel_Limit_ENC_default[2] << endl;
	cout << " Acc_Limit_ENC[0,1]: " << Acc_Limit_ENC_default[0] << endl;
	cout << " Acc_Limit_ENC[2~6]: " << Acc_Limit_ENC_default[2] << endl;
	
	cout << " LinSpeed: " << LinSpeed << endl;
	cout << " AngSpeed: " << AngSpeed << endl;
	cout << " Lin_Vel_limit: " << Lin_Vel_limit << endl;
	cout << " Lin_Acc_limit: " << Lin_Acc_limit << endl;
	cout << " Ang_Vel_limit: " << Ang_Vel_limit << endl;
	cout << " Ang_Acc_limit: " << Ang_Acc_limit << endl;

	cout << " TOL_L: " << TOL_L << endl;
	cout << " TOL_R: " << TOL_R << endl;

	cout << MOVJ <<endl << " move to ready pose (press Space to continue)" << endl;

	for( int i = 0; i < USAGE_CHANNELS; i++)
		cout << summ_error_qm_limit_ENC[i] << endl;
	//cout.precision(4);
	//cout.fixed;

	//mode_display = 1;

	//cout << MOVJ <<endl << " move to ready pose (press Space to continue)" << endl;
	//while(1)
	//{
	//	if( _kbhit() )
	//		kbCmd = _getche();

	//	if( kbCmd == kb_Space)
	//	{
	//		MOVJ == false;
	//		break;
	//	}
	//	
	//	Sleep(30);
	//	/*Sleep(29);
	//	system("cls");
	//	DisplayLoop(0);*/
	//}
}

void init_LuoLita_2()
{

	ByeBye();
	
	while(1)
	{
		if( _kbhit() )
			kbCmd = _getche();

		if( kbCmd == kb_Space)
		{
			
			
			break;
		}
		Sleep(30);
	}
	Sleep(1000);

	cout.precision(4);
	cout.fixed;
	mode_display = 1;

	int wait_time = 0;
	while(1)
	{
		Sleep(100);
		wait_time++;
		if( MOVJ == false && wait_time >= 10)
			break;
		if ( wait_time >= 20)
		{
			MOVJ = false; 
			break;
		}
		system("cls");
		DisplayLoop(0);
	}

	movetoReady = true;
	GoToReadyPose1();
	wait_time = 0;
	while(1)
	{
		Sleep(100);
		wait_time++;
		if( MOVJ == false && wait_time >= 50)
			break;
		if ( wait_time >= 100)
		{
			MOVJ = false; 
			break;
		}
		system("cls");
		DisplayLoop(0);
	}

	GoToReadyPose2();
	wait_time = 0;
	while(1)
	{
		Sleep(100);
		wait_time++;
		if( MOVJ == false && wait_time >= 50)
			break;
		if ( wait_time >= 100)
		{
			MOVJ = false; 
			break;
		}
		system("cls");
		DisplayLoop(0);
	}

	mode_cmd = 2;
	ModeArm = 2;
	T_trgt = T_ready3;
	Psi_trgt = 0.0f;
	Move_L_Abs(T_trgt, 0.0f);
	wait_time = 0;
	while(1)
	{
		Sleep(100);
		wait_time++;
		if( MOVJ == false && MOVL == false && wait_time >= 20)
			break;
		if ( wait_time >= 40)
		{ 
			MOVJ = false;
			MOVL = false;
			break;
		}
		system("cls");
		DisplayLoop(0);
	}

	mode_cmd = 1;
	for( int i = 0; i < ROBOT_DOF; i++)
		q_trgt[i] = qlCmd_rad(i,0);	
	
	for( int i = 0; i < ROBOT_DOF; i++)
		q_ready3(i,0) = q_trgt[i];
	
	ModeArm = 1;
	movetoReady = false;
}

void Holding()
{
	for (int i = 0; i < ROBOT_DOF; i++)
	{
		qmTarget_ENC[i] = qmCmd_ENC[i];
		//MOV[i] = false;
	}
	ModeArm = 1;
}

void Move_J_Abs( float (&qd)[ROBOT_DOF] )
{
	/*for (int i = 0; i < ROBOT_DOF; i++)
	{
		Vel_Limit_ENC[i] = Vel_Limit_ENC_default[i];
		Acc_Limit_ENC[i] = Acc_Limit_ENC_default[i];
		Dec_Limit_ENC[i] = Acc_Limit_ENC_default[i];
	}*/
	for (int i = 0; i < ROBOT_DOF; i++)
	{
		qmTarget_ENC[i] = roundf( rad2ENC[i]*qd[i] ) - q_home_ENC[i];
		noMOV_count[i] = 0;
		//MOV[i] = true;
	}
	MOVJ_flag = true;
	MOVJ = true;

	ModeArm = 1;
}

void Move_J_Rel( float (&delta_qd)[ROBOT_DOF] )
{
	/*for (int i = 0; i < ROBOT_DOF; i++)
	{
		Vel_Limit_ENC[i] = Vel_Limit_ENC_default[i];
		Acc_Limit_ENC[i] = Acc_Limit_ENC_default[i];
		Dec_Limit_ENC[i] = Acc_Limit_ENC_default[i];
	}*/
	for (int i = 0; i < ROBOT_DOF; i++)
	{
		//qmTarget_ENC[i] = qmTarget_ENC[i] + roundf( rad2ENC[i] * delta_qd[i] );
		qmTarget_ENC[i] = qmCmd_ENC[i] + roundf( rad2ENC[i] * delta_qd[i] );
		noMOV_count[i] = 0;
		//MOV[i] = true;
	}
	MOVJ_flag = true;
	MOVJ = true;

	ModeArm = 1;
}

void Move_J_Abs_sameTime( float (&qd)[ROBOT_DOF], float time )
{
	long target[ROBOT_DOF];
	long delta[ROBOT_DOF];

	float TimeAcc[ROBOT_DOF];
	float TimeAccMax;
	float TimeLinDec[ROBOT_DOF];
	float TimeLinDecMax;
	float TimeDec;
	float MovingTime;
	float tmp;
	float tmax = 0.0f;
	
	MovingTime = time*1000/SAMPLING_TIME;

	for (int i = 0; i < ROBOT_DOF; i++)
	{
		Vel_Limit_ENC[i] = Vel_Limit_ENC_default[i];
		Acc_Limit_ENC[i] = Acc_Limit_ENC_default[i];
		Dec_Limit_ENC[i] = Acc_Limit_ENC_default[i];

		q_trgt[i] = qd[i];

		target[i] = roundf( rad2ENC[i]*qd[i] ) - q_home_ENC[i];
		
		delta[i] = target[i] - qmCmd_ENC[i];

		if( (delta[i] > 0 && dqmCmd_ENC[i] > 0) || (delta[i] < 0 && dqmCmd_ENC[i] < 0) )
		{
			TimeAcc[i] = (float)( Vel_Limit_ENC[i] - abs(dqmCmd_ENC[i]) ) /(float)Acc_Limit_ENC[i];
			TimeLinDec[i] = (float)abs(delta[i]) / (float)Vel_Limit_ENC[i]
						  + (float)( ( Vel_Limit_ENC[i] - abs(dqmCmd_ENC[i]) ) /(float)Vel_Limit_ENC[i] ) *(float)( Vel_Limit_ENC[i] - abs(dqmCmd_ENC[i]) ) /2.0f /(float)Acc_Limit_ENC[i]
						  + (float)Vel_Limit_ENC[i] /2.0f /(float)Dec_Limit_ENC[i]
						  - TimeAcc[i];
		}
		else if( (delta[i] >= 0 && dqmCmd_ENC[i] < 0) || (delta[i] <= 0 && dqmCmd_ENC[i] > 0) )
		{
			TimeAcc[i] = (float)( Vel_Limit_ENC[i] + abs(dqmCmd_ENC[i]) ) /(float)Acc_Limit_ENC[i];
			TimeLinDec[i] = (float)abs(delta[i]) /(float)Vel_Limit_ENC[i]
						  + (float)( ( Vel_Limit_ENC[i] + abs(dqmCmd_ENC[i]) ) /(float)Vel_Limit_ENC[i] ) *(float)( Vel_Limit_ENC[i] + abs(dqmCmd_ENC[i]) ) /2.0f /(float)Acc_Limit_ENC[i]
						  + (float)Vel_Limit_ENC[i] /2.0f /(float)Dec_Limit_ENC[i]
						  - TimeAcc[i];
		}
		else if( delta[i] = 0 && dqmCmd_ENC[i] < 0 )
		{
			TimeAcc[i] = (float)( Vel_Limit_ENC[i] - dqmCmd_ENC[i] ) /(float)Acc_Limit_ENC[i];
			TimeLinDec[i] =
						  + (float)( ( Vel_Limit_ENC[i] - dqmCmd_ENC[i] ) /(float)Vel_Limit_ENC[i] ) *(float)(Vel_Limit_ENC[i] - dqmCmd_ENC[i]) /2.0f /(float)Acc_Limit_ENC[i]
						  + Vel_Limit_ENC[i] /2 /Dec_Limit_ENC[i]
						  - TimeAcc[i];
		}
		else
		{
			TimeAcc[i] = (float)Vel_Limit_ENC[i] /(float)Acc_Limit_ENC[i];
			TimeLinDec[i] = (float)delta[i] /(float)Vel_Limit_ENC[i] + (float)Vel_Limit_ENC[i] /2.0f /(float)Dec_Limit_ENC[i] - (float)Vel_Limit_ENC[i] /2.0f /(float)Acc_Limit_ENC[i];
		}

	}
	
	TimeAccMax = 0.0f;
	for (int i = 0; i < ROBOT_DOF; i++)
	{
		tmp = TimeAcc[i];
		
		if( tmp >= TimeAccMax)
			TimeAccMax = tmp;
	}
	
	TimeLinDecMax = 0.0f;
	for (int i = 0; i < ROBOT_DOF; i++)
	{
		tmp = TimeLinDec[i];
		
		if( tmp >= TimeLinDecMax)
			TimeLinDecMax = tmp;
	}
	
	tmp = TimeAccMax + TimeLinDecMax;
	if( MovingTime < tmp )
		MovingTime = tmp;
	 
	for (int i = 0; i < ROBOT_DOF; i++)
	{		TimeDec = roundf( (float)Vel_Limit_ENC[i] /(float)Dec_Limit_ENC[i] );
		Vel_Limit_ENC[i] = roundf( ( delta[i] - dqmCmd_ENC[i]*TimeAccMax /2.0f ) /(MovingTime - TimeAccMax/2 - TimeDec/2.0f) );
		Acc_Limit_ENC[i] = roundf( (float)( Vel_Limit_ENC[i] - dqmCmd_ENC[i] )/ TimeAccMax );
		Dec_Limit_ENC[i] = roundf( (float)Vel_Limit_ENC[i] / TimeAcc[i] );	
	}
	for (int i = 0; i < ROBOT_DOF; i++)
	{
		qmTarget_ENC[i] = target[i];
		MOV[i] = true;
	}
	MOVJ = true;

	ModeArm = 1;

	/*long tc = 0;
	while(1)
	{
		if ( _kbhit() )
			 kbCmd = _getch();
		if( kbCmd == 'z')
		{
			mode_MC = 0;
			break;
		
		}
		else if( kbCmd == 'x')
		{
			MOVJ = false;
			break;
		}
		Sleep(1);
		tc++;
		if(tc > MovingTime)
		{
			MOVJ = false;
			break;
		}
	}*/
}

void Move_J_Rel_sameTime( float (&delta_qd)[ROBOT_DOF], float time )
{
	float target[ROBOT_DOF];
	for (int i = 0; i < ROBOT_DOF; i++)
	{
		//target[i] = ENC2rad[i]*qmTarget_ENC[i] + delta_qd[i] + q_home(i,0);
		target[i] = ENC2rad[i]*qmCmd_ENC[i] + delta_qd[i] + q_home(i,0);
	}
	Move_J_Abs_sameTime( target, time);
}
extern vector<vector<float>> sketch_pos_y;
extern vector<vector<float>> sketch_pos_x;
extern int sketchIndex;
extern float z_axis;
extern float speed;

void Move_Contour(){//const Matrix4f& Td , float Psi_d){
	//T07Target = Td;
	//PsiTarget = Psi_d;

	MOVL_newflag=true;
	MOVL_flag = true;
	MOVL = true;
	if( mode_imped == true)
		ModeArm = 3;
	else
		ModeArm = 2;
}
void Move_L_Abs( const Matrix4f& Td , float Psi_d)
{
	// Td : target position
	T07Target = Td;
	//qmTarget_ENC[2] = roundf( rad2ENC[2]*q3d ) - q_home_ENC[2];
	PsiTarget = Psi_d;

	/*MOVJ3  = true;
	MOVL_L = true;
	MOVL_R = true;*/
	MOVL_newflag=false;
	MOVL_flag = true;
	MOVL = true;
	
	if( mode_imped == true)
		ModeArm = 3;
	else
		ModeArm = 2;
}

void Move_L_Rel( const Vector6f& delta_Td ,float delta_Psi_d)
{
	/*Vel_Limit_ENC[2] = Vel_Limit_ENC_rec[2];
	Acc_Limit_ENC[2] = Acc_Limit_ENC_rec[2];
	Dec_Limit_ENC[2] = Acc_Limit_ENC_rec[2];*/

	Matrix4f Tt;
	//Tt = T07Target;
	Tt = T07Cmd;
	T07Target = delta2tr(delta_Td ,Tt);
	//qmTarget_ENC[2] = qmTarget_ENC[2] + rad2ENC[2]*delta_q3d;
	//qmTarget_ENC[2] = qmCmd_ENC[2] + rad2ENC[2]*delta_q3d;
	PsiTarget = PsiCmd + delta_Psi_d;

	/*MOVJ3  = true;
	MOVL_L = true;
	MOVL_R = true;*/
	MOVL_flag = true;
	MOVL = true;

	ModeArm = 2;
}

void GoToReadyPose1()
{
	mode_cmd = 1;
	for (int i = 0; i < ROBOT_DOF; i++)
		q_trgt[i] = q_ready1(i,0);
	
	//Move_J_Abs_sameTime( qr1 ,5);
	Move_J_Abs( q_trgt );
	//Sleep(10000);
}

void GoToReadyPose2()
{
	mode_cmd = 1;
	for (int i = 0; i < ROBOT_DOF; i++)
		q_trgt[i] = q_ready2(i,0);
	
	//Move_J_Abs_sameTime( qr2 ,5);
	Move_J_Abs( q_trgt );
	//Sleep(10000);
}

void GoToReadyPose3()
{
	mode_cmd = 1;
	for (int i = 0; i < ROBOT_DOF; i++)
		q_trgt[i] = q_ready3(i,0);
	
	//Move_J_Abs_sameTime( qr2 ,5);
	Move_J_Abs( q_trgt );
	//Sleep(10000);

	T_trgt = T_ready3;
}

void GoToZeroPose()
{
	mode_cmd = 1;
	for (int i = 0; i < ROBOT_DOF; i++)
		q_trgt[i] = 0.0f;
	Move_J_Abs( q_trgt );
	//Sleep(5000);
}

void GoToInitPose()
{
	GoToReadyPose2();
	Sleep(5000);
	GoToReadyPose1();
	Sleep(5000);
	float qz[ROBOT_DOF];
	for (int i = 0; i < ROBOT_DOF; i++)
		qz[i] = q_home(i,0);
	qz[1] = qz[1] + 0.05f;
	qz[3] = qz[3] - 0.05f;
	Move_J_Abs( qz );
	Sleep(5000);
}

void BackToReadyPose1()
{
	float qr1[ROBOT_DOF];
	for (int i = 0; i < ROBOT_DOF; i++)
	{
		qr1[i] = ENC2rad[i]*qmCmd_ENC[i] + q_home(i,0);
	}
	for (int j = 0; j < ROBOT_DOF; j++)
	{
		for (int i = 0; i < j; i++)
			qr1[i] = q_ready1(i,0);
		//Move_J_Abs_sameTime( qr1 ,5);
		Move_J_Abs( qr1 );
		Sleep(5000);
	}
}

void BackToReadyPose2()
{
	float qr2[ROBOT_DOF];
	for (int i = 0; i < ROBOT_DOF; i++)
	{
		qr2[i] = ENC2rad[i]*qmCmd_ENC[i] + q_home(i,0);
	}
	for (int j = 0; j < ROBOT_DOF; j++)
	{
		for (int i = 0; i < j; i++)
			qr2[i] = q_ready1(i,0);
		//Move_J_Abs_sameTime( qr1 ,5);
		Move_J_Abs( qr2 );
		Sleep(5000);
	}
}

void BackToInitPose()
{
	BackToReadyPose2();
	BackToReadyPose1();
	float qz[ROBOT_DOF];
}

void ByeBye()
{
	mode_cmd = 1;
	for (int i = 0; i < ROBOT_DOF; i++)
		q_trgt[i] = q_byebye(i,0);
	
	//Move_J_Abs_sameTime( qr1 ,5);
	Move_J_Abs( q_trgt );
	//Sleep(10000);
}

void motion_kbc_J( int i, bool dir)
{
	if( CmdType == true)
	{
		if(dir == 0)
			q_trgt[i] = q_trgt[i] + qSpeed;
		else
			q_trgt[i] = q_trgt[i] - qSpeed;
		Move_J_Abs(q_trgt);
	}
	else
	{
		float delta_q_trgt[ROBOT_DOF];
		for (int j = 0; j < ROBOT_DOF; j++)
			delta_q_trgt[j] = 0;
		if(dir == 0)
			delta_q_trgt[i] = qSpeed;
		else
			delta_q_trgt[i] = -qSpeed;
		Move_J_Rel(delta_q_trgt);
		q_trgt[i] = ENC2rad[i]*qmTarget_ENC[i] + q_home(i,0);
	}
}

void motion_kbc_C(int u, bool dir)
{
	Vector6f delta_Tt;
	delta_Tt << 0,0,0,0,0,0;

	if( u < 6 )
	{
		if( u < 3 )
		{
			if( dir == 0)
				delta_Tt(u,0) = LinSpeed;
			else
				delta_Tt(u,0) = -LinSpeed;
		}
		else
		{
			if( dir == 0)
				delta_Tt(u,0) = AngSpeed;
			else
				delta_Tt(u,0) = -AngSpeed;
		}
		if( CmdType == true)
		{
			Matrix4f Tt;
			Tt = T_trgt;
			T_trgt = delta2tr( delta_Tt, Tt);
			Move_L_Abs( T_trgt, Psi_trgt );
		}
		else
		{
			Move_L_Rel( delta_Tt, 0 );
			T_trgt = T07Target;
		}
	}
	else
	{
		if( CmdType == true)
		{
			if( dir == 0)
				Psi_trgt = Psi_trgt + qSpeed;
				//q_trgt[Jn_index] = q_trgt[Jn_index] + qSpeed;
			else
				Psi_trgt = Psi_trgt - qSpeed;
				//q_trgt[Jn_index] = q_trgt[Jn_index] - qSpeed;
			Move_L_Abs( T_trgt, Psi_trgt );
		}
		else
		{
			if( dir == 0)
				Move_L_Rel( delta_Tt, qSpeed );
			else
				Move_L_Rel( delta_Tt, -qSpeed );
			//q_trgt[Jn_index] = ENC2rad[2]*qmTarget_ENC[2] + q_home(2,0);
			Psi_trgt = PsiTarget;
		}
	}
}


void MainLoop_keyboard()
{
		switch( kbCmd )
		{
		case 'z':
			//Sleep(500);
			break;

		case 'x':
			if( ModeArm != 0)
				STOP = true;
			
			break;

		case 'c':

			MOVJ = false;
			MOVL = false;
			
			break;

		case 'v':
			if( TestStep == 0)
				TestNumber++;
			if( TestNumber > 7)
				TestNumber = 0;
			break;

		case 'b':
			switch( TestNumber)
			{
			case 1: // trun round by VOTG
				TestStep++;
				if( TestStep > 3)
				{
					TestStep = 0;
				}
				if( TestStep == 0)
				{
					Lin_Vel_limit = Lin_Vel_limit_default;
					Lin_Acc_limit = Lin_Acc_limit_default;
					Lin_Dec_limit = Lin_Dec_limit_default;
				}
				else if( TestStep == 1)
				{
					mode_cmd = 2;
					Lin_Vel_limit = ( 0.01f*80 * 0.001f*LVmax / 1000.0f ) *SAMPLING_TIME_C;
					Lin_Acc_limit = ((0.01f*120 * 0.001f*LAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Lin_Dec_limit = Lin_Acc_limit;
					
					Matrix4f Td;
					Td << -1.0f,  0.0f,  0.0f,  0.40f, //
						   0.0f,  1.0f,  0.0f,  0.24f, //
						   0.0f,  0.0f, -1.0f,  0.0f, //
						   0.0f,  0.0f,  0.0f,  1.0f;

					Move_L_Abs( Td ,0.0f);
				}
				else if( TestStep == 2)
				{
					data_count = 0;
					q_DATA_RECORD = true;
					Matrix4f Td;
					Td << -1.0f,  0.0f,  0.0f,  0.40f, //
						   0.0f,  1.0f,  0.0f, -0.24f, //
						   0.0f,  0.0f, -1.0f,  0.0f, //
						   0.0f,  0.0f,  0.0f,  1.0f;
					Move_L_Abs( Td ,0.0f);
					Sleep(100);
					while(1)
					{
						Sleep(10);
						if( MOVLDEC_L == true)
						{
							break;
						}
					}
					Td << -1.0f,  0.0f,  0.0f,  0.70f, //
						   0.0f,  1.0f,  0.0f, -0.24f, //
						   0.0f,  0.0f, -1.0f,  0.0f, //
						   0.0f,  0.0f,  0.0f,  1.0f;
					Move_L_Abs( Td ,0.0f);
					Sleep(100);
					while(1)
					{
						Sleep(10);
						if( MOVLDEC_L == true)
						{
							break;
						}
					}
					Td << -1.0f,  0.0f,  0.0f,  0.70f, //
						   0.0f,  1.0f,  0.0f,  0.24f, //
						   0.0f,  0.0f, -1.0f,  0.0f, //
						   0.0f,  0.0f,  0.0f,  1.0f;
					Move_L_Abs( Td ,0.0f);
					Sleep(100);
					while(1)
					{
						Sleep(10);
						if( MOVLDEC_L == true)
						{
							break;
						}
					}
					Td << -1.0f,  0.0f,  0.0f,  0.40f, //
						   0.0f,  1.0f,  0.0f,  0.24f, //
						   0.0f,  0.0f, -1.0f,  0.0f, //
						   0.0f,  0.0f,  0.0f,  1.0f;
					Move_L_Abs( Td ,0.0f);
				}
				else if( TestStep == 3)
				{
					q_DATA_RECORD = false;
					Matrix4f Td;
					Td << -1.0f,  0.0f,  0.0f,  0.56f, //
						   0.0f,  1.0f,  0.0f,  0.0f, //
						   0.0f,  0.0f, -1.0f,  0.004f, //
						   0.0f,  0.0f,  0.0f,  1.0f;
					Move_L_Abs( Td ,0.0f);
				}
				break;   

			case 2: // fast line movement for position control evaluation
				TestStep++;
				if( TestStep > 4)
				{
					TestStep = 0;
				}
				if( TestStep == 1)
				{
					mode_cmd = 1;
					GoToReadyPose3();
				}
				else if( TestStep == 2)
				{
					mode_cmd = 2;
					Lin_Vel_limit = ( 0.01f*50 * 0.001f*LVmax / 1000.0f ) *SAMPLING_TIME_C;
					Lin_Acc_limit = ((0.01f*50 * 0.001f*LAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Lin_Dec_limit = Lin_Acc_limit;

					Matrix4f Td;
					Td << -1.0f,  0.0f,  0.0f,  0.25f, //0.18
						   0.0f,  1.0f,  0.0f,  0.32f, //0.4
						   0.0f,  0.0f, -1.0f, -0.2f, //-0.25
						   0.0f,  0.0f,  0.0f,  1.0f;

					Move_L_Abs( Td ,0.8727);//0.8727,1.1345
				}
				else if( TestStep == 3)
				{
					Ang_Vel_limit = ( 0.01f*100 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C;
					Ang_Acc_limit = ((0.01f*100 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Ang_Dec_limit = Ang_Acc_limit;
					Lin_Vel_limit = ( 0.01f*160 * 0.001f*LVmax / 1000.0f ) *SAMPLING_TIME_C;
					Lin_Acc_limit = ((0.01f*320 * 0.001f*LAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Lin_Dec_limit = Lin_Acc_limit;
					Jn_Vel_limit = ( 0.01f*25* deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C;
					Jn_Acc_limit = ((0.01f*25* deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Jn_Dec_limit = Jn_Acc_limit;

					Matrix4f Td;
					Td << -1.0f,  0.0f,  0.0f,  0.634f, //0.7
						   0.0f,  1.0f,  0.0f, -0.32f,  //-0.4
						   0.0f,  0.0f, -1.0f,  0.088f, //0.05
						   0.0f,  0.0f,  0.0f,  1.0f;

					mode_cmd = 2;
					data_count = 0;
					q_DATA_RECORD = true;
					Tq_DATA_RECORD = true;
					
					Move_L_Abs( Td , 0.08727f);//0.08727
				}
				else if( TestStep == 4)
				{
					Ang_Vel_limit = Ang_Vel_limit_default;
					Ang_Acc_limit = Ang_Acc_limit_default;
					Ang_Dec_limit = Ang_Dec_limit_default;
					Lin_Vel_limit = Lin_Vel_limit_default;
					Lin_Acc_limit = Lin_Acc_limit_default;
					Lin_Dec_limit = Lin_Dec_limit_default;
					Jn_Vel_limit = Jn_Vel_limit_default;
					Jn_Acc_limit = Jn_Acc_limit_default;
					Jn_Dec_limit = Jn_Dec_limit_default;
					
					q_DATA_RECORD = false;
					Tq_DATA_RECORD = false;
		
					GoToReadyPose3();
				}
				break;

			case 3: // auto self motion joint 6 limit avoidance
				TestStep++;
				if( TestStep > 5)
				{
					TestStep = 0;
				}
				if( TestStep == 0)
				{
					Ang_Vel_limit = Ang_Vel_limit_default;
					Ang_Acc_limit = Ang_Acc_limit_default;
					Ang_Dec_limit = Ang_Dec_limit_default;
					Lin_Vel_limit = Lin_Vel_limit_default;
					Lin_Acc_limit = Lin_Acc_limit_default;
					Lin_Dec_limit = Lin_Dec_limit_default;
					Jn_Vel_limit = Jn_Vel_limit_default;
					Jn_Acc_limit = Jn_Acc_limit_default;
					Jn_Dec_limit = Jn_Dec_limit_default;
					mode_cmd = 1;
					GoToReadyPose3();
				}
				else if( TestStep == 1)
				{
					Ang_Vel_limit = ( 0.01f*25 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C;
					Ang_Acc_limit = ((0.01f*25 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Ang_Dec_limit = Ang_Acc_limit;
					Lin_Vel_limit = ( 0.01f*25 * 0.001f*LVmax / 1000.0f ) *SAMPLING_TIME_C;
					Lin_Acc_limit = ((0.01f*25 * 0.001f*LAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Lin_Dec_limit = Lin_Acc_limit;
					Jn_Vel_limit = ( 0.01f*25 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C;
					Jn_Acc_limit = ((0.01f*25 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Jn_Dec_limit = Jn_Acc_limit;

					Matrix4f Td;
					Td << -1.0f,  0.0f,  0.0f,  0.550f, //0.7
						   0.0f,  1.0f,  0.0f,  0.100f,  //-0.4
						   0.0f,  0.0f, -1.0f, -0.150f, //0.05
						   0.0f,  0.0f,  0.0f,  1.0f;
					mode_cmd = 2;
					Move_L_Abs( Td , -1.0472f);
				}
				else if( TestStep == 2)
				{
					Matrix4f Td;
					//Td << -0.9659f,  0.0f, -0.2588f,  0.550f, //0.7
					//	      0.0f,  1.0f,     0.0f, -0.100f,  //-0.4
					//	   0.2588f,  0.0f, -0.9659f,  0.020f, //0.05
					//	      0.0f,  0.0f,  0.0f,  1.0f;
					Td << -1.0f,  0.0f,  0.0f,  0.550f, //0.7
						   0.0f,  1.0f,  0.0f,  0.100f,  //-0.4
						   0.0f,  0.0f, -1.0f,  0.150f, //0.05
						   0.0f,  0.0f,  0.0f,  1.0f;
					Move_L_Abs( Td , -1.0472f);
				}
				else if( TestStep == 3)
				{
					Matrix4f Td;
					Td << -1.0f,  0.0f,  0.0f,  0.550f, //0.7
						   0.0f,  1.0f,  0.0f,  0.100f,  //-0.4
						   0.0f,  0.0f, -1.0f, -0.150f, //0.05
						   0.0f,  0.0f,  0.0f,  1.0f;
					Move_L_Abs( Td , -1.0472f);
				}
				else if( TestStep == 4)
				{
					Jn_Vel_limit = ( 0.01f*50 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C;
					Jn_Acc_limit = ((0.01f*50 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Jn_Dec_limit = Jn_Acc_limit;
					AutoSelfMotion_Enable = true;
					Matrix4f Td;
					//Td << -0.9659f,  0.0f, -0.2588f,  0.550f, //0.7
					//	      0.0f,  1.0f,     0.0f, -0.100f,  //-0.4
					//	   0.2588f,  0.0f, -0.9659f,  0.020f, //0.05
					//	      0.0f,  0.0f,  0.0f,  1.0f;
					Td << -1.0f,  0.0f,  0.0f,  0.550f, //0.7
						   0.0f,  1.0f,  0.0f,  0.100f,  //-0.4
						   0.0f,  0.0f, -1.0f,  0.150f, //0.05
						   0.0f,  0.0f,  0.0f,  1.0f;
					Move_L_Abs( Td , -1.0472f);

				}
				else if( TestStep == 5)
				{
					PsiTarget = PsiCmd;
					Psi_trgt = PsiTarget;
					AutoSelfMotion_Enable = false;
				}
				break;

			case 4: // auto self motion singular point (q6==0) avoidance
				TestStep++;
				if( TestStep > 3)
				{
					TestStep = 0;
				}
				if( TestStep == 0)
				{
					Ang_Vel_limit = Ang_Vel_limit_default;
					Ang_Acc_limit = Ang_Acc_limit_default;
					Ang_Dec_limit = Ang_Dec_limit_default;
					Lin_Vel_limit = Lin_Vel_limit_default;
					Lin_Acc_limit = Lin_Acc_limit_default;
					Lin_Dec_limit = Lin_Dec_limit_default;
					Jn_Vel_limit = Jn_Vel_limit_default;
					Jn_Acc_limit = Jn_Acc_limit_default;
					Jn_Dec_limit = Jn_Dec_limit_default;
					mode_cmd = 1;
					GoToReadyPose3();
				}
				else if( TestStep == 1)
				{
					/*Ang_Vel_limit = ( 0.01f*80 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C;
					Ang_Acc_limit = ((0.01f*80 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Ang_Dec_limit = Ang_Acc_limit;
					Lin_Vel_limit = ( 0.01f*50 * 0.001f*LVmax / 1000.0f ) *SAMPLING_TIME_C;
					Lin_Acc_limit = ((0.01f*50 * 0.001f*LAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Lin_Dec_limit = Lin_Acc_limit;*/
					Jn_Vel_limit = ( 0.01f*25 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C;
					Jn_Acc_limit = ((0.01f*25 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Jn_Dec_limit = Jn_Acc_limit;
				}
				else if( TestStep == 2)
				{
					/*Jn_Vel_limit = ( 0.01f*40 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C;
					Jn_Acc_limit = ((0.01f*40 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Jn_Dec_limit = Jn_Acc_limit;*/
					AutoSelfMotion_Enable = true;
				}
				else if( TestStep == 3)
				{
					PsiTarget = PsiCmd;
					Psi_trgt = PsiTarget;
					AutoSelfMotion_Enable = false;
				}
				break;

			case 5: // teach two point and replay P2P with safty stop enable
				TestStep++;
				if( TestStep > 7)
				{
					TestStep = 0;
				}
				if( TestStep == 0)
				{
					Ang_Vel_limit = Ang_Vel_limit_default;
					Ang_Acc_limit = Ang_Acc_limit_default;
					Ang_Dec_limit = Ang_Dec_limit_default;
					Lin_Vel_limit = Lin_Vel_limit_default;
					Lin_Acc_limit = Lin_Acc_limit_default;
					Lin_Dec_limit = Lin_Dec_limit_default;
					Jn_Vel_limit = Jn_Vel_limit_default;
					Jn_Acc_limit = Jn_Acc_limit_default;
					Jn_Dec_limit = Jn_Dec_limit_default;

					q_DATA_RECORD = false;
					Tq_DATA_RECORD = false;

					Integrator_Enable = true;
					GoToReadyPose3();
				}
				else if( TestStep == 1)
				{
					mode_cmd = 2;           
					T_trgt.block(0,0,3,3) = R07Cmd;
					T_trgt.block(0,3,3,1) = P07Cmd;
					Psi_trgt = PsiCmd;

					ModeArm = 2;

					 Integrator_Enable = false;
					//then teach
				}
				else if( TestStep == 2)
				{
					Td_start_record.block(0,0,3,3) =  R07Cmd;
					Td_start_record.block(0,3,3,1) =  P07Cmd;
					Psid_start_record = PsiCmd;
				}
				else if( TestStep == 3)
				{
					Td_end_record.block(0,0,3,3) =  R07Cmd;
					Td_end_record.block(0,3,3,1) =  P07Cmd;
					Psid_end_record = PsiCmd;
				}
				else if( TestStep == 4)
				{
					Ang_Vel_limit = ( 0.01f*40 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C;
					Ang_Acc_limit = ((0.01f*40 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Ang_Dec_limit = Ang_Acc_limit;
					Lin_Vel_limit = ( 0.01f*80 * 0.001f*LVmax / 1000.0f ) *SAMPLING_TIME_C;
					Lin_Acc_limit = ((0.01f*120 * 0.001f*LAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Lin_Dec_limit = Lin_Acc_limit;
					Jn_Vel_limit = ( 0.01f*10 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C;
					Jn_Acc_limit = ((0.01f*10 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Jn_Dec_limit = Jn_Acc_limit;

					mode_cmd = 1;
					for( int i = 0; i < ROBOT_DOF; i++)
					q_trgt[i] = qlCmd_rad(i,0);
					ModeArm  = 1;
					Sleep(100);

					mode_cmd = 2;
					T_trgt.block(0,0,3,3) = R07Cmd;
					T_trgt.block(0,3,3,1) = P07Cmd;
					Psi_trgt = PsiCmd;
					ModeArm = 2;
					Sleep(100);

					Move_L_Abs( Td_start_record , Psid_start_record);
				}
				else if( TestStep == 5)
				{
					data_count = 0;
					q_DATA_RECORD = true;
					Tq_DATA_RECORD = true;

					Move_L_Abs( Td_end_record , Psid_end_record);
					Sleep(100);
					int wait_count = 0;
					while(1)
					{
						wait_count++;
						Sleep(10);
						if( MOVL == false || wait_count > 1000)
							break;
					}
					Sleep(500);
					Move_L_Abs( Td_start_record , Psid_start_record);
				}
				else if( TestStep == 6) //replay
				{
					SaftySTOP_Enable = true;
					Move_L_Abs( Td_end_record , Psid_end_record);
					Sleep(100);
					/*int wait_count = 0;
					while(1)
					{
						wait_count++;
						Sleep(10);
						if( MOVL == false || wait_count > 1000)
							break;
					}
					Sleep(500);
					Move_L_Abs( Td_start_record , Psid_start_record);*/
				}
				else if( TestStep == 7) //replay
				{
					SaftySTOP_Enable = false;
					Move_L_Abs( Td_end_record , Psid_end_record);
					Sleep(100);
					int wait_count = 0;
					while(1)
					{
						wait_count++;
						Sleep(10);
						if( MOVL == false || wait_count > 1000)
							break;
					}
					Sleep(500);
					Move_L_Abs( Td_start_record , Psid_start_record);
				}
				break;

			case 6:
				TestStep++;
				if( TestStep > TrgtRecNum+2)
				{
					TestStep = 0;
				}
				if( TestStep == 0)
				{
					Ang_Vel_limit = Ang_Vel_limit_default;
					Ang_Acc_limit = Ang_Acc_limit_default;
					Ang_Dec_limit = Ang_Dec_limit_default;
					Lin_Vel_limit = Lin_Vel_limit_default;
					Lin_Acc_limit = Lin_Acc_limit_default;
					Lin_Dec_limit = Lin_Dec_limit_default;
					Jn_Vel_limit = Jn_Vel_limit_default;
					Jn_Acc_limit = Jn_Acc_limit_default;
					Jn_Dec_limit = Jn_Dec_limit_default;

					q_DATA_RECORD = false;
					Tq_DATA_RECORD = false;

					Integrator_Enable = true;
					GoToReadyPose3();
					LitaHand.GripperMove_Abs_To( 55.0f, 200);
				}
				else if( TestStep == 1)
				{
					mode_cmd = 2;           
					T_trgt.block(0,0,3,3) = R07Cmd;
					T_trgt.block(0,3,3,1) = P07Cmd;
					Psi_trgt = PsiCmd;

					ModeArm = 2;

					Integrator_Enable = false;
					//then teach
				}
				else if( TestStep >= 2 && TestStep <= 2+TrgtRecNum-1)
				{
					Td_record[TestStep-2].block(0,0,3,3) =  R07Cmd;
					Td_record[TestStep-2].block(0,3,3,1) =  P07Cmd;
					Psid_record[TestStep-2] = PsiCmd;
					T_trgt.block(0,0,3,3) = Td_record[TestStep-2].block(0,0,3,3);
					T_trgt.block(0,3,3,1) = Td_record[TestStep-2].block(0,3,3,1);
					Psi_trgt = Psid_record[TestStep-2];
					// Grasp
					if( TestStep == 4)
					{
						LitaHand.GripperMove_Abs_To( 48.0f, 200);
					}
					else if( TestStep == 8)
					{
						LitaHand.GripperMove_Abs_To( 60.0f, 200);
					}
				}
				else if( TestStep == TrgtRecNum+2)
				{
					if( ModeArm == 1)
					{
						 SubMode = 0;
					}
					else if( ModeArm == 3 )
					{
						SubMode = 0;
					}
					else if( ModeArm == 2 )
					{
						if( SubMode ==2)
						{
							SubMode = 1;
							Sleep(25);
						}
						SubMode = 0;
					}
					Ang_Vel_limit = ( 0.01f*40 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C;
					Ang_Acc_limit = ((0.01f*40 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Ang_Dec_limit = Ang_Acc_limit;
					Lin_Vel_limit = ( 0.01f*80 * 0.001f*LVmax / 1000.0f ) *SAMPLING_TIME_C;
					Lin_Acc_limit = ((0.01f*120 * 0.001f*LAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Lin_Dec_limit = Lin_Acc_limit;
					Jn_Vel_limit = ( 0.01f*10 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C;
					Jn_Acc_limit = ((0.01f*10 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C;
					Jn_Dec_limit = Jn_Acc_limit;

					Integrator_Enable = true;

					mode_cmd = 1;
					for( int i = 0; i < ROBOT_DOF; i++)
					q_trgt[i] = qlCmd_rad(i,0);
					ModeArm  = 1;
					Sleep(100);

					mode_cmd = 2;
					T_trgt.block(0,0,3,3) = R07Cmd;
					T_trgt.block(0,3,3,1) = P07Cmd;
					Psi_trgt = PsiCmd;
					ModeArm = 2;
					Sleep(100);

					int TrgtIndex = 0;
					while(1)
					{
						Move_L_Abs( Td_record[TrgtIndex] , Psid_record[TrgtIndex]);
						Sleep(100);
						int wait_count = 0;
						while(1)
						{
							wait_count++;
							Sleep(10);
							if( MOVL == false || wait_count > 1000)
								break;
						}
						//grasp
						if( TrgtIndex == 0)
						{
							LitaHand.GripperMove_Abs_To( 55.0f, 200);
						}
						else if( TrgtIndex == 2)
						{
							LitaHand.GripperMove_Abs_To( 48.0f, 200);
						}
						else if( TrgtIndex == 6)
						{
							LitaHand.GripperMove_Abs_To( 55.0f, 200);
						}

						TrgtIndex++;
						if( TrgtIndex >= TrgtRecNum )
							break;
					}
					Move_L_Abs( Td_record[0] , Psid_record[0]);
				}
				break;
			//case 6: // record q data
			//	TestStep++;
			//	if( TestStep > 1)
			//	{
			//		TestStep = 0;
			//	}
			//	if( TestStep == 0)
			//	{
			//		q_DATA_RECORD = false;
			//	}
			//	else if( TestStep == 1)
			//	{
			//		data_count = 0;
			//		q_DATA_RECORD = true;
			//	}
			//	
			//	break;

			case 7: // z direction F/P hybrid control exp
				TestStep++;
				if( TestStep > 3)
				{
					TestStep = 0;
				}
				if( TestStep == 0)
				{
					FORCE_CONTROL = false;
					//FORCE_LIMIT = false;
					Integrator_Enable = true;
					Fz_DATA_RECORD = false;
					q_DATA_RECORD = false;

					mode_cmd = 1;
					GoToReadyPose3();
				}
				else if( TestStep == 1)
				{
					Matrix3f Rd;
					Rd <<  0.0f,  1.0f,  0.0f,
						   1.0f,  0.0f,  0.0f,
						   0.0f,  0.0f, -1.0f;
					T_trgt.block(0,0,3,3) = Rd;
					Move_L_Abs(T_trgt,Psi_trgt);
						   
				}
				else if( TestStep == 2)
				{
					T_trgt(2,3) = T_trgt(2,3) - 0.04f;
					
					/*Fz_th = -20.0f;
					Fz_task = -100.0f;*/
					Fz_th = -120.0f;
					Fz_task = -60.0f;

					FORCE_CONTROL = true;
					//FORCE_LIMIT = true;
					Integrator_Enable = false;
					
					data_count = 0;
					q_DATA_RECORD = true;
					Fz_DATA_RECORD = true;
					//m
					Move_L_Abs(T_trgt,Psi_trgt);
	
					int wait_count = 0;
					while(1)
					{
						wait_count++;
						Sleep(10);
						if(FORCE_TRACKING == true || wait_count > 1000)
						//if( wait_count >= 400)
						{
							Sleep(4000);
							T_trgt(1,3) = T_trgt(1,3) - 0.36f;
							Move_L_Abs(T_trgt,Psi_trgt);
							break;
						}
					}
				}
				else if(TestStep == 3)
				{
					//m
					T_trgt(2,3) = T_trgt(2,3) + 0.06f;
					Move_L_Abs(T_trgt,Psi_trgt);
					
					/*FORCE_CONTROL = false;
					d3_d = d3_0;
					Sleep(10);
					d3_d = d3_0;
					Fz_DATA_RECORD = false;
					q_DATA_RECORD = false;*/
				}
				break;

			//case 8: // auto detect end-effector pay load
			//	break;	
			}
			break;

		case 'a':
			if( ModeArm == 3 || ModeArm == 2)
			{
				SubMode = 1;
			}
			break;
		case 's':
			if( ModeArm == 3 )
			{
				SubMode = 2;
			}
			else if ( ModeArm == 2)
			{
				SubMode = 1;
				Sleep(25);
				SubMode = 2;
			}
			break;
		case 'd':
			if( ModeArm == 3 || ModeArm == 2)
			{
				SubMode = 3;
			}
			break;
		case 'f':
			if( ModeArm == 3 )
			{
				SubMode = 4;
			}
			else if( ModeArm == 2 )
			{
				if( SubMode ==2)
				{
					SubMode = 1;
					Sleep(25);
					SubMode = 0;
					Sleep(25);
				}
				SubMode = 4;
			}
			break;
		case 'g':
			if( ModeArm == 3 )
			{
				SubMode = 5;
			}
			break;
		case 'h':
			if( ModeArm == 3 || ModeArm == 2)
			{
				SubMode = 6;
			}
			break;
		case 'j':
			if( ModeArm == 3 )
			{
				SubMode = 0;
			}
			else if( ModeArm == 2 )
			{
				if( SubMode ==2)
				{
					SubMode = 1;
					Sleep(25);
				}
				SubMode = 0;
			}
			break;
		
		case 'n':

			if( ModeArm == 1)
			{
				mode_cmd = 0;
			    ModeArm = 0;
			}
			else if( ModeArm == 0)
			{
				mode_cmd = 1;
				
				for( int i = 0; i < ROBOT_DOF; i++)
					q_trgt[i] = qlCmd_rad(i,0);

				ModeArm  = 1;
			}

			if( ModeArm == 3)
			{
				//nullspace aux
				if( nullAuxTq == false)
					nullAuxTq = true;
				else
					nullAuxTq = false;
			}
			break;

		case 'm':
			if( MOVJ == false && MOVL == false)
			{
				if( mode_cmd == 2 || mode_cmd == 0)
				{
					mode_cmd = 1;
					for( int i = 0; i < ROBOT_DOF; i++)
						q_trgt[i] = qlCmd_rad(i,0);
					
					SubMode = 0;
					ModeArm = 1;
				}
				else if(mode_cmd == 1)
				{
					mode_cmd = 2;
					T_trgt.block(0,0,3,3) = R07Cmd;
					T_trgt.block(0,3,3,1) = P07Cmd;
					Psi_trgt = PsiCmd;

					ModeArm = 2;
				}
				//Sleep(500);
			}
			break;

		case 'k':
			if( ModeArm ==2 && MOVL == false )
			{
				SubMode = 0;
				mode_imped = true;
				ModeArm = 3;
			}
			else if( ModeArm == 3 && MOVL == false)
			{
				SubMode = 0;
				mode_imped = false;
				ModeArm = 2;
			}
			break;

		case 'l':
			//dir aux
			break;

		case '>':
			
			break;
		case '<':

			break;
		case '1':
			if( mode_cmd == 1)
			{
				motion_kbc_J(0,0);
				//Sleep(500);
			}
			break;
		case 'q':
			if( mode_cmd == 1)
			{
				motion_kbc_J(0,1);
				//Sleep(500);
			}
			break;
		case '2':
			if( mode_cmd == 1)
			{
				motion_kbc_J(1,0);
				//Sleep(500);
			}
			break;
		case 'w':
			if( mode_cmd == 1)
			{
				motion_kbc_J(1,1);
				//Sleep(500);
			}
			break;
		case '3':
			if( mode_cmd == 1)
			{
				motion_kbc_J(2,0);
				//Sleep(500);
			}
			if( mode_cmd == 2)
			{
				motion_kbc_C(6,0);
				//Sleep(500);
			}
			break;
		case 'e':
			if( mode_cmd == 1)
			{
				motion_kbc_J(2,1);
				//Sleep(500);
			}
			if( mode_cmd == 2)
			{
				motion_kbc_C(6,1);
				//Sleep(500);
			}
			break;
		case '4':
			if( mode_cmd == 1)
			{
				motion_kbc_J(3,0);
				//Sleep(500);
			}
			break;
		case 'r':
			if( mode_cmd == 1)
			{
				motion_kbc_J(3,1);
				//Sleep(500);
			}
			break;
		case '5':
			if( mode_cmd == 1)
			{
				motion_kbc_J(4,0);
				//Sleep(500);
			}
			if( mode_cmd == 2)
			{
				motion_kbc_C(3,0);
				//Sleep(500);
			}
			break;
		case 't':
			if( mode_cmd == 1)
			{
				motion_kbc_J(4,1);
				//Sleep(500);
			}
			if( mode_cmd == 2)
			{
				motion_kbc_C(3,1);
				//Sleep(500);
			}
			break;
		case '6':
			if( mode_cmd == 1)
			{
				motion_kbc_J(5,0);
				//Sleep(500);
			}
			if( mode_cmd == 2)
			{
				motion_kbc_C(4,0);
				//Sleep(500);
			}
			break;
		case 'y':
			if( mode_cmd == 1)
			{
				motion_kbc_J(5,1);
				//Sleep(500);
			}
			if( mode_cmd == 2)
			{
				motion_kbc_C(4,1);
				//Sleep(500);
			}
			break;
		case '7':
			if( mode_cmd == 1)
			{
				motion_kbc_J(6,0);
				//Sleep(500);
			}
			if( mode_cmd == 2)
			{
				motion_kbc_C(5,0);
				//Sleep(500);
			}
			break;
		case 'u':
			if( mode_cmd == 1)
			{
				motion_kbc_J(6,1);
				//Sleep(500);
			}
			if( mode_cmd == 2)
			{
				motion_kbc_C(5,1);
				//Sleep(500);
			}
			break;
		case '8':
			if( mode_cmd == 2)
			{
				motion_kbc_C(0,0);
				//Sleep(500);
			}
			break;
		case 'i':
			if( mode_cmd == 2)
			{
				motion_kbc_C(0,1);
				//Sleep(500);
			}
			break;
		case '9':
			if( mode_cmd == 2)
			{
				motion_kbc_C(1,0);
				//Sleep(500);
			}
			break;
		case 'o':
			if( mode_cmd == 2)
			{
				motion_kbc_C(1,1);
				//Sleep(500);
			}
			break;
		case '0':
			if( mode_cmd == 2)
			{
				motion_kbc_C(2,0);
				//Sleep(500);
			}
			break;
		case 'p':
			if( mode_cmd == 2)
			{
				motion_kbc_C(2,1);
				//Sleep(500);
			}
			break;
		}

		kbCmd_1 = kbCmd;
		kbCmd = ' ';
	//}
}

/*void MyLoop_keyboard(){
	Vector6f t;
	Matrix4f T;
	int loop = 10;
	float PI = 3.1415926f;
	float R = 0.3f;
	
	switch( kbCmd ){
		case 'w':
			t << 0,0.01f,0,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 's':
			t << 0,-0.01f,0,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 'a':
			t << -0.01f,0,0,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 'd':
			t << 0.01f,0,0,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 'r':
			t << 0,0,0.01f,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 'f':
			t << 0,0,-0.01f,0,0,0;
			Move_L_Rel(t,0);
			break;
/*
		case 'k':
			t << 0,-0.01f,0,0,0,0;
			for(int i=0;i<loop;i++){
				Move_L_Rel(t,0);
				while(MOVL){}
			}
			break;
		case 'l':
			T <<  -1.0f,  0.0f,  0.0f,  0.4f, //
				   0.0f,  1.0f,  0.0f,  0.0f, //
				   0.0f,  0.0f, -1.0f,  0.0f, //
				   0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			Sleep(100);
			for(float i=2;i<=90;i+=2){
				T(0,3) = 0.4f + R * sin(i*PI/180);
				T(1,3) = R - R * cos(i*PI/180);
				Move_L_Abs(T,0);
				while(MOVL){}
			}
			break;
		case 'm':
			T <<  -1.0f,  0.0f,  0.0f,  0.4f, //
				   0.0f,  1.0f,  0.0f,  0.3f, //
				   0.0f,  0.0f, -1.0f,  0.0f, //
				   0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			T <<  -1.0f,  0.0f,  0.0f,  0.7f, //
				   0.0f,  1.0f,  0.0f,  0.3f, //
				   0.0f,  0.0f, -1.0f,  0.0f, //
				   0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			break;
		case 'n':
			t << 0,0.3f,0,0,0,0;
			Move_L_Rel(t,0);
			t << 0.3f,0,0,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 'y':
			T <<  -1.0f,  0.0f,  0.0f,  0.5f, //
				   0.0f,  1.0f,  0.0f,  0.0f, //
				   0.0f,  0.0f, -1.0f,  0.0f, //
				   0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			break;
		case 'h':
			T <<  -1.0f,  0.0f,  0.0f,  0.5f, //
				   0.0f,  1.0f,  0.0f,  0.0f, //
				   0.0f,  0.0f, -1.0f,  -0.26f, //
				   0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			break;
		case 'o':
			for(int i=0;i<pos_x.size();i++){
			Matrix4f T;
			T <<  -1.0f,  0.0f,  0.0f,  pos_x[i], //
				   0.0f,  1.0f,  0.0f,  pos_y[i], //
				   0.0f,  0.0f, -1.0f,  -0.26f, //
				   0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			while(MOVL){}
			}
			break;
	}

	kbCmd_1 = kbCmd;
	kbCmd = ' ';
}*/

void DisplayLoop(float speed)
{

	if( mode_display == 1)
	{
			//1.
			cout << "  Luo Lita Arm" << endl;
			//2.
			cout << "       qTarget      qCmd      q(rad)   error_ENC   TqComp.(mNm)  SetValue(V)" << endl;
			//3.
			cout << endl;

			if(mode_cmd == 1 ||mode_cmd == 0)
			{
			//4.q1
			cout << "  1 " << setw(10) << rad2deg*q_trgt[0] <<  setw(10) << qCmd_deg[0] << setw(10) << q_deg[0] << setw(10) << dqmTrgt_ENC[0] << setw(6) << error_qm_ENC[0] << setw(12) <<						 '0' << setw(14) << SetValue[0] << endl;
			//5.q2
			cout << "  2 " << setw(10) << rad2deg*q_trgt[1] <<  setw(10) << qCmd_deg[1] << setw(10) << q_deg[1] << setw(10) << dqmTrgt_ENC[1] << setw(6) << error_qm_ENC[1] << setw(12) <<torque2setvalue[1]*Tq_g[1] << setw(14) << SetValue[1] << endl;
			//6.q3
			cout << "  3 " << setw(10) << rad2deg*q_trgt[2] <<  setw(10) << qCmd_deg[2] << setw(10) << q_deg[2] << setw(10) << dqmTrgt_ENC[2] << setw(6) << error_qm_ENC[2] << setw(12) <<torque2setvalue[2]*Tq_g[2] << setw(14) << SetValue[2] << endl;
			//7.
			cout << "  4 " << setw(10) << rad2deg*q_trgt[3] <<  setw(10) << qCmd_deg[3] << setw(10) << q_deg[3] << setw(10) << dqmTrgt_ENC[3] << setw(6) << error_qm_ENC[3] << setw(12) <<torque2setvalue[3]*Tq_g[3] << setw(14) << SetValue[3] << endl;
			//8.
			cout << endl;
			//9.q5
			cout << "  5 " << setw(10) << rad2deg*q_trgt[4] <<  setw(10) << qCmd_deg[4] << setw(10) << q_deg[4] << setw(10) << dqmTrgt_ENC[4] << setw(6) << error_qm_ENC[4] << setw(12) <<torque2setvalue[4]*Tq_g[4] << setw(14) << SetValue[4] << endl;
			//10.q6
			cout << "  6 " << setw(10) << rad2deg*q_trgt[5] <<  setw(10) << qCmd_deg[5] << setw(10) << q_deg[5] << setw(10) << dqmTrgt_ENC[5] << setw(6) << error_qm_ENC[5] << setw(12) <<torque2setvalue[5]*Tq_g[5] << setw(14) << SetValue[5] << endl;
			//11.q7
			cout << "  7 " << setw(10) << rad2deg*q_trgt[6] <<  setw(10) << qCmd_deg[6] << setw(10) << q_deg[6] << setw(10) << dqmTrgt_ENC[6] << setw(6) << error_qm_ENC[6] << setw(12) <<						 '0' << setw(14) << SetValue[6] << endl;
				//12.
			cout << endl;
			//13.
			cout << "  Arm Angle: Psi Target:" << setw(10) <<  Psi_trgt << "-" << setw(10) << PsiCmd << "  Psi:" << setw(10) << Psi << endl;
			}
			else
			{
			//4.q1
			cout << "  1 " << setw(10) <<				"-" <<  setw(10) << qCmd_deg[0] << setw(10) << q_deg[0] << setw(12) << error_qm_ENC[0] << setw(16) <<						   '0' << setw(14) << -SetValue[0] << endl;
			//5.q2
			cout << "  2 " << setw(10) <<				"-" <<  setw(10) << qCmd_deg[1] << setw(10) << q_deg[1] << setw(12) << error_qm_ENC[1] << setw(16) <<  torque2setvalue[1]*Tq_g[1] << setw(14) <<  SetValue[1] << endl;
			//6.q3
			cout << "  3 " << setw(10) <<				"-" <<  setw(10) << qCmd_deg[2] << setw(10) << q_deg[2] << setw(12) << error_qm_ENC[2] << setw(16) << -torque2setvalue[2]*Tq_g[2] << setw(14) << -SetValue[2] << endl;
			//7.
			cout << "  4 " << setw(10) <<				"-" <<  setw(10) << qCmd_deg[3] << setw(10) << q_deg[3] << setw(12) << error_qm_ENC[3] << setw(16) <<  torque2setvalue[3]*Tq_g[3] << setw(14) <<  SetValue[3] << endl;
			//8.
			cout << endl;
			//9.q5
			cout << "  5 " << setw(10) <<				"-" <<  setw(10) << qCmd_deg[4] << setw(10) << q_deg[4] << setw(12) << error_qm_ENC[4] << setw(16) << -torque2setvalue[4]*Tq_g[4] << setw(14) << -SetValue[4] << endl;
			//10.q6
			cout << "  6 " << setw(10) <<				"-" <<  setw(10) << qCmd_deg[5] << setw(10) << q_deg[5] << setw(12) << error_qm_ENC[5] << setw(16) <<  torque2setvalue[5]*Tq_g[5] << setw(14) <<  SetValue[5] << endl;
			//11.q7
			cout << "  7 " << setw(10) <<				"-" <<  setw(10) << qCmd_deg[6] << setw(10) << q_deg[6] << setw(12) << error_qm_ENC[6] << setw(16) <<						   '0' << setw(14) << -SetValue[6] << endl;
				//12.
			cout << endl;
			//13.
			cout << "  Arm Angle: Psi Target:" << setw(8) <<  Psi_trgt << "  Psi_Cmd:" << setw(8) << PsiCmd << "  Psi:" << setw(8) << Psi << " dPsi:" << setw(8) << dPsi*100.0f << endl;
			}
			
			//14.
			cout << "  Ttarget:" << endl;
			if(mode_cmd == 2)
			{
			//15.x
			cout << setw(10) << T_trgt(0,0) << setw(8) << T_trgt(0,1) << setw(8) << T_trgt(0,2) << setw(10) << 1000.0f*T_trgt(0,3) << endl;
			//16.y
			cout << setw(10) << T_trgt(1,0) << setw(8) << T_trgt(1,1) << setw(8) << T_trgt(1,2) << setw(10) << 1000.0f*T_trgt(1,3) << endl;
			//17.z
			cout << setw(10) << T_trgt(2,0) << setw(8) << T_trgt(2,1) << setw(8) << T_trgt(2,2) << setw(10) << 1000.0f*T_trgt(2,3) << endl;
			//18.
			cout << setw(10) << "0" << setw(8) << "0" << setw(8) << "0" << setw(10) << "1" << endl;
			}
			else
			{
			//15.x
			cout << "  -" << endl;
			//16.y
			cout << "  -" << endl;
			//17.z
			cout << "  -" << endl;
			//18.
			cout << "  -" << endl;
			}
			//19.
			cout << setw(48) << "F_imped:" << setw(10) << "Tq_imped:" << setw(10) << "Tq_null:" << endl;
			//20.
			cout << "  Tcmd: " << endl;
			//21.
			cout << setw(10) << T07Cmd(0,0) << setw(8) << T07Cmd(0,1) << setw(8) << T07Cmd(0,2) << setw(10) << 1000.0f*T07Cmd(0,3) << "; Fx:" << setw(6) << F0wCmd(0,0) << ";Tq_1:"<< setw(6) << -torque2setvalue[0]*Tq_imped(0,0)/MT0_GEAR_RATIO << ";" << setw(6) << -torque2setvalue[0]*Tq_null[0] << endl;
			//22.
			cout << setw(10) << T07Cmd(1,0) << setw(8) << T07Cmd(1,1) << setw(8) << T07Cmd(1,2) << setw(10) << 1000.0f*T07Cmd(1,3) << "; Fy:" << setw(6) << F0wCmd(1,0) << ";Tq_2:"<< setw(6) <<  torque2setvalue[1]*Tq_imped(1,0)/MT1_GEAR_RATIO << ";" << setw(6) <<  torque2setvalue[1]*Tq_null[1] << endl;
			//23.
			cout << setw(10) << T07Cmd(2,0) << setw(8) << T07Cmd(2,1) << setw(8) << T07Cmd(2,2) << setw(10) << 1000.0f*T07Cmd(2,3) << "; Fz:" << setw(6) << F0wCmd(2,0) << ";Tq_3:"<< setw(6) << -torque2setvalue[2]*Tq_imped(2,0)/MT2_GEAR_RATIO << ";" << setw(6) << -torque2setvalue[2]*Tq_null[2] << endl;
			//24.
			cout << setw(10) << "0" << setw(8) << "0" << setw(8) << "0" << setw(10) << "1" << setw(17) << ";Tq_4:"<< setw(6) << torque2setvalue[3]*Tq_imped(3,0)/MT3_GEAR_RATIO << ";" << setw(6) << torque2setvalue[3]*Tq_null[3] << endl;
			//25.
			cout << endl;
			//26.
			//cout << "  T:                        dT:    (mm/s)  (rad/s)" << endl;
			//26.
			cout << "     T: " << endl;
			//27.
			cout << setw(10) << T07(0,0) << setw(8) << T07(0,1) << setw(8) << T07(0,2) << setw(10) << 1000.0f*T07(0,3) << "; Mx:" << setw(6) << M0wCmd(0,0) << ";Tq_5:"<< setw(6) << -torque2setvalue[4]*Tq_imped(4,0)/MT4_GEAR_RATIO << ";" << setw(6) << -torque2setvalue[4]*Tq_null[4] << endl;
			//28.
			cout << setw(10) << T07(1,0) << setw(8) << T07(1,1) << setw(8) << T07(1,2) << setw(10) << 1000.0f*T07(1,3) << "; My:" << setw(6) << M0wCmd(1,0) << ";Tq_6:"<< setw(6) <<  torque2setvalue[5]*Tq_imped(5,0)/MT5_GEAR_RATIO << ";" << setw(6) <<  torque2setvalue[5]*Tq_null[5] << endl;
			//29.
			cout << setw(10) << T07(2,0) << setw(8) << T07(2,1) << setw(8) << T07(2,2) << setw(10) << 1000.0f*T07(2,3) << "; Mz:" << setw(6) << M0wCmd(2,0) << ";Tq_7:"<< setw(6) << -torque2setvalue[6]*Tq_imped(6,0)/MT6_GEAR_RATIO << ";" << setw(6) << -torque2setvalue[6]*Tq_null[6] << endl;
			//30.
			cout << setw(10) << "0" << setw(8) << "0" << setw(8) << "0" << setw(10) << "1" << endl;
			//31.
			cout << endl;
			//32.
			cout << "  parameter: ";
			cout << endl;
	        /*switch(ParameterIndex)
			{
			case 0:
				cout << endl;
				break;
			case 1:
				cout << endl;
				break;
			case 2:
				cout << endl;
				break;
			case 3:
				cout << endl;
				break;
			case 4:
				cout << endl;
				break;
			case 5:
				cout << endl;
				break;
			}*/
			//33.
			cout << endl;
			//34.
			if( mode_cmd == 1)
			{
				if( movetoReady == true)
					cout << "  moving to ready pose..." << endl;
				else
					cout << "  Joint Mode" << endl;
			}
			else if( mode_cmd == 2)
			{
				if( mode_imped == true)
					cout << "  Impedence Mode" << endl;
				else
					cout << "  Cartesian Mode" << endl;
			}
			else
				cout << "  mode: " << mode_cmd << endl;
			//35.
			if( mode_cmd == 1)
				cout << "  ModeArm: " << ModeArm << endl;
			else
				cout << "  ModeArm: " << ModeArm << "  SubMode: " << SubMode << "  nullAuxTq: " << nullAuxTq << endl;
			//36.
			cout << "  modeMotion_J: " << modeMotion_J << "  modeMotion_C: " << modeMotion_C << endl;
			//37.
			cout << "  MOVJ: " << MOVJ << "  MOV: " << setw(6) << MOV[0] << setw(6) << MOV[1] << setw(6) << MOV[2] << setw(6) << MOV[3] << setw(6) << MOV[4] << setw(6) << MOV[5] << setw(6) << MOV[6] << endl;
			//38.
			cout << setw(16) <<"noMOV_count:" <<setw(6)<<noMOV_count[0] <<setw(6)<<noMOV_count[1] <<setw(6)<<noMOV_count[2] <<setw(6)<<noMOV_count[3] <<setw(6)<<noMOV_count[4] <<setw(6)<<noMOV_count[5] <<setw(6)<<noMOV_count[6] << endl;
			//39
			cout << "  MOVL: " << MOVL << "  MOVL_L: " << MOVL_L << "  MOVL_R: " << MOVL_R << "  MOVJN: " << MOVJN << "  DEC_L: " << MOVLDEC_L << "  TestComputeLose: " << TestComputeLose << " TestNumber: " << TestNumber << endl;
			//40.			
			cout << FORCE_TRACKING << "  d3: " << d3 << "  ENC7: " << setw(4) << ENC7 << setw(6) << ENC7_BL << setw(8) << ENC7_e << "  Fz: " << Fz << " TestStep: " << TestStep;
			cout << endl <<"  Moving speed: "<< speed;
	}

}

void OutputData()
{
	ofstream ofile;
	if ( TestNumber == 1)
	{
		ofile.open("qENC_data.txt");
		for( int i = 0; i < DataLength; i++)
		{
			ofile << qENC_data[i][0];
			for( int j = 1; j < USAGE_CHANNELS; j++)
			{
				ofile << " " << qENC_data[i][j];
			}
			ofile << "\n";
		}
		ofile.close();

		ofile.open("qCmd_data.txt");
		for( int i = 0; i < DataLength; i++)
		{
			ofile << qCmd_data[i][0];
			for( int j = 1; j < USAGE_CHANNELS; j++)
			{
				ofile << " " << qCmd_data[i][j];
			}
			ofile << "\n";
		}
		ofile.close();
	}
	else if ( TestNumber == 2 || TestNumber == 5)
	{
		ofile.open("qENC_data.txt");
		for( int i = 0; i < DataLength; i++)
		{
			ofile << qENC_data[i][0];
			for( int j = 1; j < USAGE_CHANNELS; j++)
			{
				ofile << " " << qENC_data[i][j];
			}
			ofile << "\n";
		}
		ofile.close();

		ofile.open("qCmd_data.txt");
		for( int i = 0; i < DataLength; i++)
		{
			ofile << qCmd_data[i][0];
			for( int j = 1; j < USAGE_CHANNELS; j++)
			{
				ofile << " " << qCmd_data[i][j];
			}
			ofile << "\n";
		}
		ofile.close();

		ofile.open("TqCmd_data.txt");
		for( int i = 0; i < DataLength; i++)
		{
			ofile << TqCmd_data[i][0];
			for( int j = 1; j < USAGE_CHANNELS; j++)
			{
				ofile << " " << TqCmd_data[i][j];
			}
			ofile << "\n";
		}
		ofile.close();
	}
	else if( TestNumber==6)
	{
		ofile.open("qENC_data.txt");
		for( int i = 0; i < DataLength; i++)
		{
			ofile << qENC_data[i][0];
			for( int j = 1; j < USAGE_CHANNELS; j++)
			{
				ofile << " " << qENC_data[i][j];
			}
			ofile << "\n";
		}
		ofile.close();

		ofile.open("qCmd_data.txt");
		for( int i = 0; i < DataLength; i++)
		{
			ofile << qCmd_data[i][0];
			for( int j = 1; j < USAGE_CHANNELS; j++)
			{
				ofile << " " << qCmd_data[i][j];
			}
			ofile << "\n";
		}
		ofile.close();
		
		ofile.open("Fz_data.txt");
		for( int i = 0; i < DataLength; i++)
		{
			ofile << Fz_data[i] << "\n";
		}
		ofile.close();
	}
}
