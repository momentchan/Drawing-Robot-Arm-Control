#include "Gripper.h"

using namespace std;

Gripper::Gripper()
{
	mm2enc = 7075.15f;
	initpos = 85.6f; // mm
	Init_Gripper();
}

void Gripper::Init_Gripper()
{
	openPort( string("COM2"), 9600 );
	
	writePort(string("0ANSW0\n"));
	writePort(string("0HO\n"));
	writePort(string("0SP30000\n"));
	writePort(string("0AC30000\n"));
	writePort(string("0DEC30000\n"));
}

void Gripper::SetGripperMotorCurrentLimit(int I_limit_mA, int sleepTime_ms)
{
	char MotorCmdBuffer[16];
	sprintf_s(MotorCmdBuffer, "0LCC%d\n", I_limit_mA );
	writePort( string(MotorCmdBuffer));
	Sleep( sleepTime_ms);
}

void Gripper::SetGripperMotorSpeed(int motor_rpm, int sleepTime_ms)
{
	char MotorCmdBuffer[16];
	sprintf_s(MotorCmdBuffer, "0sp%d\n", motor_rpm );
	writePort( string(MotorCmdBuffer));
	Sleep( sleepTime_ms);
}

void Gripper::SetGripperMotorAcc(int motor_acc, int sleepTime_ms )
{
	char MotorCmdBuffer[16];
	sprintf_s(MotorCmdBuffer, "0ac%d\n", motor_acc );
	writePort( string(MotorCmdBuffer));
	Sleep( sleepTime_ms);
}

void Gripper::SetGripperMotorDec(int motor_dec, int sleepTime_ms )
{
	char MotorCmdBuffer[16];
	sprintf_s(MotorCmdBuffer, "0dec%d\n", motor_dec);
	writePort( string(MotorCmdBuffer));
	Sleep( sleepTime_ms);
}

int Gripper::GetGripperMotorCurrent_mA( int waitTime_ms)
{
	int current;
	writePort( string("0GRC\n"));
	Sleep( waitTime_ms);
	current = atoi(readPort().c_str());
	return current;
}

int Gripper::GetGripperPosition_enc( int waitTime_ms)
{
	int enc;
	writePort( string("0pos\n"));
	Sleep( waitTime_ms);
	enc = atoi(readPort().c_str());
	return enc;
}

float Gripper::GetGripperPosition_mm( int waitTime_ms)
{
	int enc;
	enc = GetGripperPosition_enc( waitTime_ms );
	return initpos - (int)(enc/mm2enc);
}

void Gripper::GripperMove_Abs_To(float mm, int sleepTime_ms)
{
	char MotorCmdBuffer[16];

	if( mm > initpos )
		mm = initpos;
	int dd = (int)(mm2enc*(initpos - mm));

	sprintf_s(MotorCmdBuffer, "0LA%d\n", dd);
	writePort( string(MotorCmdBuffer));
	writePort( string("0m\n"));
	
	if(sleepTime_ms>=10)
		Sleep( sleepTime_ms);
}

void Gripper::GripperMove_Rel_To(float dmm, int sleepTime_ms)
{
	char MotorCmdBuffer[16];

	int ddd = -((int)mm2enc*dmm);

	sprintf_s(MotorCmdBuffer, "0LR%d\n", ddd);
	writePort( string(MotorCmdBuffer));
	writePort( string("0m\n"));
	
	if(sleepTime_ms>=10)
		Sleep( sleepTime_ms);
}

void Gripper::GripperStop(int sleepTime_ms)
{
	writePort( string("0v0"));
	Sleep( sleepTime_ms);
}

void Gripper::GripperGrasp(int motor_rpm, int sleepTime_ms)
{
	char MotorCmdBuffer[16];
	sprintf_s(MotorCmdBuffer, "0v%d\n", motor_rpm);
	writePort( string(MotorCmdBuffer));
	Sleep( sleepTime_ms);
}

void Gripper::GripperLoose(int motor_rpm, int sleepTime_ms)
{
	char MotorCmdBuffer[16];
	sprintf_s(MotorCmdBuffer, "0v%d\n", -motor_rpm);
	writePort( string(MotorCmdBuffer));
	Sleep( sleepTime_ms);
}

void Gripper::GripperGoHome()
{
	//SetGripperMotorSpeed(1000);
	//GripperMove_Abs_To( 0, 20);
	writePort( string("0LA0\n"));
	Sleep(10);
	writePort( string("0m\n"));
}

void Gripper::GripperByeBye()
{
	GripperGoHome();
	int enc;
	while(1)
	{
		enc = GetGripperPosition_enc(20);

		if( abs(enc) < 3 )
			break;
	}
	writePort( string("DI\n"));
	Sleep(10);
	//DI
}