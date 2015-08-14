#include "SerialPortCom.hpp"

//const float mm2enc = 7075.15f//21973.65f;
	
class Gripper: public SerialPortCom
{
public:

	Gripper();
	
	void Init_Gripper();

	void SetGripperMotorCurrentLimit(int I_limit_mA, int sleepTime_ms = 10);

	void SetGripperMotorSpeed(int motor_rpm, int sleepTime_ms = 10);

	void SetGripperMotorAcc(int motor_acc, int sleepTime_ms = 10);

	void SetGripperMotorDec(int motor_dec, int sleepTime_ms = 10);


	int GetGripperMotorCurrent_mA(int waitTime_ms = 20);

	int GetGripperPosition_enc(int waitTime_ms = 20);
	
	float GetGripperPosition_mm(int waitTime_ms = 20);

	void GripperMove_Abs_To(float mm, int sleepTime_ms = 10);

	void GripperMove_Rel_To(float dmm, int sleepTime_ms = 10);

	void GripperStop(int sleepTime_ms = 10);

	void GripperGrasp(int motor_rpm, int sleepTime_ms = 10);

	void GripperLoose(int motor_rpm, int sleepTime_ms = 10);

	void GripperGoHome();

	void GripperByeBye();

private:
	float mm2enc;// = 7075.15f;
	float initpos;// = 85.6f; // mm
};


