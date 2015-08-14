#include "ControlLita.h"
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
using namespace std;
using namespace Eigen;

extern ofstream outputFile;
extern bool write;
extern vector<Eigen::Vector3f> trace;
extern vector<vector<Eigen::Vector2f>> sketchPositionXY;
extern float z_axis_black;
extern int sketchIndex;
int traceIndex=0;

//function in RTtimer

//initial variable
long qENC_data[DataLength][USAGE_CHANNELS];
long qCmd_data[DataLength][USAGE_CHANNELS];
float TqCmd_data[DataLength][USAGE_CHANNELS];
float Fz_data[DataLength]= {0};

bool q_DATA_RECORD = false;
bool Tq_DATA_RECORD = false;
bool Fz_DATA_RECORD = false;
long data_count = 0;

long t_count = 0;

long t_count_B = 0;
long t_count_C = 0;
long t_count_D = 0;

float ENC2rad[USAGE_CHANNELS];
float rad2ENC[USAGE_CHANNELS];
float ENC2deg[USAGE_CHANNELS];
float deg2ENC[USAGE_CHANNELS];
float torque2setvalue[USAGE_CHANNELS];
float setvalue2torque[USAGE_CHANNELS];

float K_vel_P[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float K_vel_p2I[USAGE_CHANNELS]= {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

float K_pos_P[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float K_pos_p2I[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float K_pos_p2D[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

float K_posvel_P[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float K_posvel_p2I[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float K_posvel_p2D[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

float K_pos[USAGE_CHANNELS] = {0.00020f, 0.00020f, 0.00010f, 0.00020f, 0.00012f, 0.00030f, 0.000005f};
float PID_pos[3] = {1.0f, 0.0f, 0.0f};//{P,I,D}
float K_vel[USAGE_CHANNELS] = {0};
float PID_vel[3] = {1.0f, 0.0f, 0.0f};//{P,I,D}

float TorqueFriction[USAGE_CHANNELS];

float K_damper[USAGE_CHANNELS] = {0.010f, 0.010f, 0.003f, 0.005f, 0.005f, 0.005f, 0.0001f};

float K_gravity[USAGE_CHANNELS];

long q_home_ENC[USAGE_CHANNELS];

long q_pLimit_ENC[USAGE_CHANNELS];
long q_nLimit_ENC[USAGE_CHANNELS];

bool highsetvalue = false;
long TestComputeLose = 0;

int ModeArm;
int ModeArm_1;
int SubMode;

bool ModeArm_Z = false;
bool ModeArm_G = false;
bool ModeArm_J = false;
bool ModeArm_C = false;
bool ModeArm_K = false;
bool ModeArm_Z_1 = false;
bool ModeArm_G_1 = false;
bool ModeArm_J_1 = false;
bool ModeArm_C_1 = false;
bool ModeArm_K_1 = false;

bool Free_All = false;
bool Free_P_psi = false;
bool Free_P = false;
bool Free_O = false;
bool Free_psi = false;
bool Free_All_1 = false;
bool Free_P_psi_1 = false;
bool Free_P_1 = false;
bool Free_O_1 = false;
bool Free_psi_1 = false;

bool ModeArm_C_nullTq = false;
bool ModeArm_C_nullTq_1 = false;

bool nullAuxTq = false;
bool dirAuxF = false;

bool modeControl_G = false;
bool modeControl_P = false;
bool modeControl_K = false;
bool modeControl_G_1 = false;
bool modeControl_P_1 = false;
bool modeControl_K_1 = false;

bool modeMotion_J = false;
bool modeMotion_C = false;
bool modeMotion_J_1 = false;
bool modeMotion_C_1 = false;

bool STOP = false;

bool SaftySTOP_Enable = false;

bool Integrator_Enable = false;

bool MOVJ = false;
bool MOVJ_flag = false;

bool MOV[USAGE_CHANNELS] = {false,false,false,false,false,false,false};

long qm_ENC_1[USAGE_CHANNELS] = {0,0,0,0,0,0,0};
long qm_ENC_2[USAGE_CHANNELS] = {0,0,0,0,0,0,0};
long qm_ENC_3[USAGE_CHANNELS] = {0,0,0,0,0,0,0};

float dqm_ENC[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float dqm_ENC_1[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

float ddqm_ENC[USAGE_CHANNELS]  = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

long qmCmd_ENC[USAGE_CHANNELS] = {0,0,0,0,0,0,0};
long qmCmd_ENC_1[USAGE_CHANNELS] = {0,0,0,0,0,0,0};
long qmCmd_ENC_2[USAGE_CHANNELS] = {0,0,0,0,0,0,0};

float dqmCmd_ENC[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float dqmCmd_ENC_1[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float dqmCmdCmd_ENC[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float ddqmCmd_ENC[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

long  qmTrgt_ENC[USAGE_CHANNELS] = {0,0,0,0,0,0,0};
float dqmTrgt_ENC[USAGE_CHANNELS] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

long  qmTarget_ENC[USAGE_CHANNELS] = {0,0,0,0,0,0,0};

long  error_qm_ENC[USAGE_CHANNELS] = {0};		long  error_qm_ENC_1[USAGE_CHANNELS] = {0};
float error_dqm_ENC[USAGE_CHANNELS] = {0};		float error_dqm_ENC_1[USAGE_CHANNELS] = {0};
long  diff_error_qm_ENC[USAGE_CHANNELS] = {0};  long  diff_error_qm_ENC_1[USAGE_CHANNELS] = {0};
float diff_error_dqm_ENC[USAGE_CHANNELS] = {0}; float diff_error_dqm_ENC_1[USAGE_CHANNELS] = {0};
long  summ_error_qm_ENC[USAGE_CHANNELS] = {0};  long  summ_error_qm_ENC_1[USAGE_CHANNELS] = {0};
float summ_error_dqm_ENC[USAGE_CHANNELS] = {0}; float summ_error_dqm_ENC_1[USAGE_CHANNELS] = {0};

long summ_error_qm_limit_ENC[USAGE_CHANNELS];
float summ_error_dqm_limit_ENC[USAGE_CHANNELS];

int noMOV_count[USAGE_CHANNELS] = {0};

float Vel_Limit_ENC[USAGE_CHANNELS]; //500unit: count/ms
float Acc_Limit_ENC[USAGE_CHANNELS]; //5unit: count/(ms)^2
float Dec_Limit_ENC[USAGE_CHANNELS];
float qVelMax;


float Kp_imped_F[3];
float Kv_imped_F[3];
float Kp_imped_M[3];
float Kv_imped_M[3];
float K_aux_F[3];
float Kp_imped_dq_null[USAGE_CHANNELS];
float K_aux_dq_null[USAGE_CHANNELS];

bool MOVL = false;
bool MOVL_1 = false;
bool MOVL_flag = false;
bool MOVL_newflag=false;
bool MOVL_L = false;
bool MOVL_R = false;
bool MOVJN = false;

bool MOVLDEC_L = false;

Vector7f ql_rad;
Vector7f ql_rad_1;
Vector7f dql_rad;

Vector7f qlCmd_rad;
Vector7f qlCmd_rad_1;
Vector7f qlCmd_next_rad;
Vector7f dqlCmd_rad;

Vector7f qlTrgt_rad;

float cosq[ROBOT_DOF];
float sinq[ROBOT_DOF];
float cosqCmd[ROBOT_DOF];
float sinqCmd[ROBOT_DOF];

float c1 = 1.0f; float c2 =-1.0f; float c3 = 1.0f; float c4 = 0.0f;
float c5 = 1.0f; float c6 = 1.0f; float c7 = 1.0f;
float s1 = 0.0f; float s2 = 0.0f; float s3 = 0.0f; float s4 =-1.0f;
float s5 = 0.0f; float s6 = 0.0f; float s7 = 0.0f;

float Psi = 0.0f;
float Psi_1 = 0.0f;
float dPsi = 0.0f;
float dPsi_1 = 0.0f;

float Psi_psi0 = 0.0f;
float Psi_psi1 = 0.0f;

Vector7f ql_psi0;
Vector7f ql_psi1;

Vector7f delta_ql_null_rad;
Vector7f delta_ql_null_rad_1;

Vector7f dql_null_rad;
Vector7f dql_null_rad_1;

Matrix3f Su;
Matrix3f As;
Matrix3f Bs;
Matrix3f Cs;
Matrix3f Aw;
Matrix3f Bw;
Matrix3f Cw;

Matrix3f R03_o_psi0;
Matrix3f R03_psi1;
Matrix3f R34_psi0;
Matrix3f R47_psi1;
Matrix3f R07_psi0;

float q1o_psi0;
float q2o_psi0;

float q1o = 0.0f;
float q1o_1 = 0.0f;
float q2o;

Matrix3f R03_o;
Matrix3f R_psi;

Matrix3f R03;
Matrix3f R34;
Matrix3f R47;
Matrix3f R04;
Matrix3f R07;
Matrix3f R07_1;

Vector3f u0w;

Vector3f P0w;
Vector3f P0w_1;
Vector3f P07;
Vector3f P07_1;
float P0w_norm;

Vector3f V0w;
Vector3f V07;
Vector3f W07;
float V07_norm;
float W07_norm;


float PsiCmd = 0.0f;
float PsiCmd_1 = 0.0f;
float dPsiCmd = 0.0f;
float dPsiCmd_1 = 0.0f;

float PsiCmd_temp;

float q1oCmd= 0.0f;
float q1oCmd_1 = 0.0f;
float q2oCmd;

Matrix3f R03_oCmd;

Matrix3f R03Cmd;
Matrix3f R34Cmd;
Matrix3f R04Cmd;
Matrix3f R47Cmd;
Matrix3f R07Cmd;
Matrix3f R07Cmd_1;

Vector3f u0wCmd;

Vector3f P0pCmd;	// keke
Vector3f P0eCmd;	// keke
Vector3f P0aCmd;	// keke
Vector3f P0wCmd;
Vector3f P0wCmd_1;
Vector3f P07Cmd;
Vector3f P07Cmd_1;
float P0wCmd_norm;

Vector3f V0wCmd;
Vector3f V07Cmd;
Vector3f W07Cmd;
Vector3f V0wCmd_1;
Vector3f V07Cmd_1;
Vector3f W07Cmd_1;
float V07Cmd_norm;
float W07Cmd_norm;
float V07Cmd_norm_1;
float W07Cmd_norm_1;

float PsiTrgt = 0.0f;
float dPsiTrgt = 0.0f;

float PsiTarget = 0.0f;

Vector3f P07Trgt;
Matrix3f R07Trgt;
Vector3f dP07Trgt;
Vector3f dR07Trgt;
Vector3f dP07Trgt_Normalized;
Vector3f dR07Trgt_Normalized;
Vector3f V07Trgt;
Vector3f W07Trgt;
Vector3f V07Trgt_Normalized;
Vector3f W07Trgt_Normalized;
Vector3f dV07Trgt;
Vector3f dW07Trgt;
float dP07Trgt_norm;
float dR07Trgt_norm;
float V07Trgt_norm;
float W07Trgt_norm;
float dV07Trgt_norm;
float dW07Trgt_norm;

Matrix4f T07Trgt;
Vector6f dT07Trgt;

Matrix4f T07Target;

float Lin_Vel_limit;
float Lin_Acc_limit;
float Lin_Dec_limit;

float Ang_Vel_limit;
float Ang_Acc_limit;
float Ang_Dec_limit;

float Jn_Vel_limit;
float Jn_Acc_limit;
float Jn_Dec_limit;

float LinearSpeedMax;
float AngularSpeedMax;
float JnSpeedMax;

float TOL_L;
float TOL_R;

Matrix43f JV0wt;
Matrix3f JW4wt;
Matrix67f J07;
Matrix67f J07Cmd;

Vector3f delta_P0w;
Vector3f delta_R07;
Vector3f F0wCmd;
Vector3f M0wCmd;

Vector7f Tq_imped;

float TqmTrgt[USAGE_CHANNELS] = {0.0f};

float TqmCmd[USAGE_CHANNELS] = {0.0f};

float Tq_fric[USAGE_CHANNELS] = {0.0f};

float Tq_g[ROBOT_DOF] = {0.0f};

float Tq_null[ROBOT_DOF] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};


bool AutoSelfMotion = false;
bool AutoSelfMotion_Enable = false;
Matrix3f JJT_v;
Matrix3f JJT_w;
float Vv_sngl = 1.0f;
float Vw_sngl = 1.0f;
float V_sngl = 1.0f;
float V_sngl_1 = 1.0f;
float V_sngl_next = 1.0f;
float dV_sngl = 0.0f;
float H_jlim6 = 1.0f;
float H_jlim6_1 = 1.0f;
float H_jlim6_next_p = 1.0f;
float H_jlim6_next_n = 1.0f;
float dH_jlim6 = 0.0f;
float Psi_jlim6_next_p = 0.0f;
float Psi_jlim6_next_n = 0.0f;

bool FORCE_CONTROL = false;
bool FORCE_LIMIT = false;
//bool FORCE_CONTROL_1 = false;
bool FORCE_TRACKING = false;
bool FC_flag = false;
//bool FT_flag = false;
bool ENC7_BL_flag = false;
int dir_ENC7_BL = 0;
long ENC7_BL = 0;
long ENC7_BL_1 = 0;
long ENC7_1;
long ENC7_2;
float z_ENC7_BL = 0.0f;
float ENC7_e = 0.0f;
float ENC7_e_1 = 0.0f;
float ENC7_e_2 = 0.0f;
float deltaZ_FS = 0.0f;
float z07Cmd_task;
float z0wCmd_temp;
float z07Cmd_temp;
float z07Cmd_temp_1;
float z_init;
float Fz_init;
float Fz_th = -100.0f;
float Fz = 0.0f;
float Fz_1 = 0.0f;
float Fz_task;
float error_Fz;
float error_Fz_1;
float summ_error_Fz;
//float diff_error_Fz;


float q_deg[ROBOT_DOF];
float qCmd_deg[ROBOT_DOF];
Matrix4f T07;
Matrix4f T07Cmd;
//

// keke
float qr1_temp;

void Init_ControlLita()
{
	for( int i = 0; i < DataLength; i++)
	{
		for( int j = 0; j < USAGE_CHANNELS; j++)
		{
			qENC_data[i][j] = 0;
			qCmd_data[i][j] = 0;
			TqCmd_data[i][j] = 0.0f;
			
		}
	}

	//factor
	rad2ENC[0] = MT0_GEAR_RATIO * MT0_ENCODER_4X / rev2rad;
	rad2ENC[1] = MT1_GEAR_RATIO * MT1_ENCODER_4X / rev2rad;
	rad2ENC[2] = MT2_GEAR_RATIO * MT2_ENCODER_4X / rev2rad;
	rad2ENC[3] = MT3_GEAR_RATIO * MT3_ENCODER_4X / rev2rad;
    rad2ENC[4] = MT4_GEAR_RATIO * MT4_ENCODER_4X / rev2rad;
	rad2ENC[5] = MT5_GEAR_RATIO * MT5_ENCODER_4X / rev2rad;
	rad2ENC[6] = MT6_GEAR_RATIO * MT6_ENCODER_4X / rev2rad;
	
	ENC2rad[0] = rev2rad / MT0_GEAR_RATIO / MT0_ENCODER_4X; // M => J
	ENC2rad[1] = rev2rad / MT1_GEAR_RATIO / MT1_ENCODER_4X;
	ENC2rad[2] = rev2rad / MT2_GEAR_RATIO / MT2_ENCODER_4X;
	ENC2rad[3] = rev2rad / MT3_GEAR_RATIO / MT3_ENCODER_4X;
	ENC2rad[4] = rev2rad / MT4_GEAR_RATIO / MT4_ENCODER_4X;
	ENC2rad[5] = rev2rad / MT5_GEAR_RATIO / MT5_ENCODER_4X;
	ENC2rad[6] = rev2rad / MT6_GEAR_RATIO / MT6_ENCODER_4X;
	
	deg2ENC[0] = MT0_GEAR_RATIO * MT0_ENCODER_4X / 360.0f;
	deg2ENC[1] = MT1_GEAR_RATIO * MT1_ENCODER_4X / 360.0f;
	deg2ENC[2] = MT2_GEAR_RATIO * MT2_ENCODER_4X / 360.0f;
	deg2ENC[3] = MT3_GEAR_RATIO * MT3_ENCODER_4X / 360.0f;
	deg2ENC[4] = MT4_GEAR_RATIO * MT4_ENCODER_4X / 360.0f;
	deg2ENC[5] = MT5_GEAR_RATIO * MT5_ENCODER_4X / 360.0f;
	deg2ENC[6] = MT6_GEAR_RATIO * MT6_ENCODER_4X / 360.0f;

	ENC2deg[0] = 360.0f / MT0_GEAR_RATIO / MT0_ENCODER_4X;
	ENC2deg[1] = 360.0f / MT1_GEAR_RATIO / MT1_ENCODER_4X;
	ENC2deg[2] = 360.0f / MT2_GEAR_RATIO / MT2_ENCODER_4X;
	ENC2deg[3] = 360.0f / MT3_GEAR_RATIO / MT3_ENCODER_4X;
	ENC2deg[4] = 360.0f / MT4_GEAR_RATIO / MT4_ENCODER_4X;
	ENC2deg[5] = 360.0f / MT5_GEAR_RATIO / MT5_ENCODER_4X;
	ENC2deg[6] = 360.0f / MT6_GEAR_RATIO / MT6_ENCODER_4X;

	torque2setvalue[0] = MT0_DAC_POLAR*DAC_MAX_VOLTAGE / MT0_MAX_CURRENT / MT0_TORQUE_CONSTANT;
	torque2setvalue[1] = MT1_DAC_POLAR*DAC_MAX_VOLTAGE / MT1_MAX_CURRENT / MT1_TORQUE_CONSTANT;
	torque2setvalue[2] = MT2_DAC_POLAR*DAC_MAX_VOLTAGE / MT2_MAX_CURRENT/ MT2_TORQUE_CONSTANT;
	torque2setvalue[3] = MT3_DAC_POLAR*DAC_MAX_VOLTAGE / MT3_MAX_CURRENT / MT3_TORQUE_CONSTANT;
	torque2setvalue[4] = MT4_DAC_POLAR*DAC_MAX_VOLTAGE / MT4_MAX_CURRENT / MT4_TORQUE_CONSTANT;
	torque2setvalue[5] = MT5_DAC_POLAR*DAC_MAX_VOLTAGE / MT5_MAX_CURRENT / MT5_TORQUE_CONSTANT;
	torque2setvalue[6] = MT6_DAC_POLAR*DAC_MAX_VOLTAGE / MT6_MAX_CURRENT / MT6_TORQUE_CONSTANT;

	setvalue2torque[0] = MT0_DAC_POLAR * MT0_MAX_CURRENT * MT0_TORQUE_CONSTANT / DAC_MAX_VOLTAGE;
	setvalue2torque[1] = MT1_DAC_POLAR * MT1_MAX_CURRENT * MT1_TORQUE_CONSTANT / DAC_MAX_VOLTAGE;
	setvalue2torque[2] = MT2_DAC_POLAR * MT2_MAX_CURRENT * MT2_TORQUE_CONSTANT / DAC_MAX_VOLTAGE;
	setvalue2torque[3] = MT3_DAC_POLAR * MT3_MAX_CURRENT * MT3_TORQUE_CONSTANT / DAC_MAX_VOLTAGE;
	setvalue2torque[4] = MT4_DAC_POLAR * MT4_MAX_CURRENT * MT4_TORQUE_CONSTANT / DAC_MAX_VOLTAGE;
	setvalue2torque[5] = MT5_DAC_POLAR * MT5_MAX_CURRENT * MT5_TORQUE_CONSTANT / DAC_MAX_VOLTAGE;
	setvalue2torque[6] = MT6_DAC_POLAR * MT6_MAX_CURRENT * MT6_TORQUE_CONSTANT / DAC_MAX_VOLTAGE;


	for( int i = 0; i < USAGE_CHANNELS; i++)
	{
		q_home_ENC[i] = roundf( rad2ENC[i]*q_home(i,0) );
		q_pLimit_ENC[i] = roundf( deg2ENC[i]*( q_p_limit_deg(i,0) - q_home_deg(i,0) ) );
		q_nLimit_ENC[i] = roundf( deg2ENC[i]*( q_n_limit_deg(i,0) - q_home_deg(i,0) ) );
	}

	//PID

	K_vel_P[0] = 0.012f;//0.010 0.020f;
	K_vel_P[1] = 0.012f;//0.024f;
	K_vel_P[2] = 0.008f;//0.010f;
	K_vel_P[3] = 0.010f;//0.016f;
	K_vel_P[4] = 0.004f;//0.008f;
	K_vel_P[5] = 0.005f;//0.010f;
	K_vel_P[6] = 0.002f;//0.004f;

	K_vel_p2I[0] = 0.0060f;//0.010f;
	K_vel_p2I[1] = 0.0080f;//0.010f;
	K_vel_p2I[2] = 0.0030f;//0.005f;
	K_vel_p2I[3] = 0.0050f;//0.005f;
	K_vel_p2I[4] = 0.0008f;//0.0008f;
	K_vel_p2I[5] = 0.0010f;//0.0008f;
	K_vel_p2I[6] = 0.0008f;//0.0008f;

	K_pos_P[0] = 0.00020f;
	K_pos_P[1] = 0.00025f;
	K_pos_P[2] = 0.00015f;
	K_pos_P[3] = 0.00020f;
	K_pos_P[4] = 0.00015f;
	K_pos_P[5] = 0.00030f;
	K_pos_P[6] = 0.00004f;

	K_pos_p2I[0] = 0.0f;//0.004f;//0.080f;
	K_pos_p2I[1] = 0.0f;//0.004f;//0.080f;
	K_pos_p2I[2] = 0.0f;//0.003f;//0.060f;
	K_pos_p2I[3] = 0.0f;//0.003f;//0.060f;
	K_pos_p2I[4] = 0.0f;//0.002f;//0.040f;
	K_pos_p2I[5] = 0.0f;//0.002f;//0.040f;
	K_pos_p2I[6] = 0.0f;//0.001f;//0.030f;
	
	K_pos_p2D[0] = 0.0f;//0.100f;
	K_pos_p2D[1] = 0.0f;//0.100f;
	K_pos_p2D[2] = 0.0f;//0.100f;
	K_pos_p2D[3] = 0.0f;//0.100f;
	K_pos_p2D[4] = 0.0f;//0.100f;
	K_pos_p2D[5] = 0.0f;//0.100f;
	K_pos_p2D[6] = 0.0f;//0.100f;

	K_posvel_P[0] = 0.06f;//0.010f;
	K_posvel_P[1] = 0.06f;//0.012f;
	K_posvel_P[2] = 0.04f;//0.008f;
	K_posvel_P[3] = 0.05f;//0.010f;
	K_posvel_P[4] = 0.03f;//0.008f;
	K_posvel_P[5] = 0.04f;//0.016f;
	K_posvel_P[6] = 0.02f;//0.002f;

	K_posvel_p2I[0] = 0.0050f;//0.004 0.080f;
	K_posvel_p2I[1] = 0.0060f;//0.006 0.080f;
	K_posvel_p2I[2] = 0.0030f;//0.003 0.060f;
	K_posvel_p2I[3] = 0.0060f;//0.006 0.060f;
	K_posvel_p2I[4] = 0.0020f;//0.002 0.040f;
	K_posvel_p2I[5] = 0.0030f;//0.003 0.040f;
	K_posvel_p2I[6] = 0.0010f;//0.001 0.030f;

	K_posvel_p2D[0] = 0.100f;//0.100f;
	K_posvel_p2D[1] = 0.100f;//0.100f;
	K_posvel_p2D[2] = 0.100f;//0.100f;
	K_posvel_p2D[3] = 0.100f;//0.100f;
	K_posvel_p2D[4] = 0.100f;//0.100f;
	K_posvel_p2D[5] = 0.100f;//0.100f;
	K_posvel_p2D[6] = 0.100f;//0.100f;
	
	//Anti-intergrator
	for( int i = 0; i < USAGE_CHANNELS; i++)
	{
		summ_error_qm_limit_ENC[i] = roundf( abs(1.0f*10.0f*setvalue2torque[i]/K_vel_P[i]/K_posvel_P[i]/K_posvel_p2I[i]));
		summ_error_dqm_limit_ENC[i] = abs(1.0f*10.0f*setvalue2torque[i]/K_vel_P[i]/K_vel_p2I[i]);

		/*summ_error_qm_limit_ENC[i] = roundf( abs(0.1f*10.0f*setvalue2torque[i]/K_vel_P[i]/K_posvel_P[i]/K_posvel_p2I[i]));
		summ_error_dqm_limit_ENC[i] = abs(0.1f*10.0f*setvalue2torque[i]/K_vel_P[i]/K_vel_p2I[i]);*/
	}

	//Dead Zone
	for( int i = 0; i < USAGE_CHANNELS; i++)
	{
		TorqueFriction[i] = abs(setvalue2torque[i]*V_th[i]);
	}

	//damper
	K_damper[0] = 0.0080f;//0.012f;
	K_damper[1] = 0.0060f;//0.010f;
	K_damper[2] = 0.0040f;//0.005f;
	K_damper[3] = 0.0030f;//0.005f;
	K_damper[4] = 0.0040f;//0.005f;
	K_damper[5] = 0.0030f;//0.005f;
	K_damper[6] = 0.0002f;//0.0002f;
	
	// gravity
	K_gravity[0] = gain_GC_grav[0] / MT0_GEAR_RATIO;
	K_gravity[1] = gain_GC_grav[1] / MT1_GEAR_RATIO;
	K_gravity[2] = gain_GC_grav[2] / MT2_GEAR_RATIO;
	K_gravity[3] = gain_GC_grav[3] / MT3_GEAR_RATIO;
	K_gravity[4] = gain_GC_grav[4] / MT4_GEAR_RATIO;
	K_gravity[5] = gain_GC_grav[5] / MT5_GEAR_RATIO;
	K_gravity[6] = gain_GC_grav[6] / MT6_GEAR_RATIO;

	//Impedence
	Kp_imped_F[0] = 100000.0f; // 100000
	Kp_imped_F[1] = 100000.0f; // 100000
	Kp_imped_F[2] = 0.0f; // 100000
	
	Kv_imped_F[0] =  8000.0f; //800
	Kv_imped_F[1] =  8000.0f;
	Kv_imped_F[2] =  8000.0f;

	Kp_imped_M[0] =  3000.0f; //1500
	Kp_imped_M[1] =  3000.0f;
	Kp_imped_M[2] =  3000.0f;
	
	Kv_imped_M[0] =  1000.0f; //12
	Kv_imped_M[1] =  1000.0f;
	Kv_imped_M[2] =  1000.0f;
	
	K_aux_F[0] = 10.0f;
	K_aux_F[1] = 10.0f;
	K_aux_F[2] = 10.0f;

	Kp_imped_dq_null[0] = 360.0f / MT0_GEAR_RATIO;
	Kp_imped_dq_null[1] = 360.0f / MT1_GEAR_RATIO;
	Kp_imped_dq_null[2] = 300.0f / MT2_GEAR_RATIO;
	Kp_imped_dq_null[3] =   1.0f / MT3_GEAR_RATIO;
	Kp_imped_dq_null[4] = 80.0f / MT4_GEAR_RATIO;
	Kp_imped_dq_null[5] = 80.0f / MT5_GEAR_RATIO;
	Kp_imped_dq_null[6] = 80.0f / MT6_GEAR_RATIO;
	
	K_aux_dq_null[0] = 20.0f / MT0_GEAR_RATIO;
	K_aux_dq_null[1] = 20.0f / MT1_GEAR_RATIO;
	K_aux_dq_null[2] = 16.0f / MT2_GEAR_RATIO;
	K_aux_dq_null[3] =  1.0f / MT3_GEAR_RATIO;
	K_aux_dq_null[4] = 10.0f / MT4_GEAR_RATIO;
	K_aux_dq_null[5] = 10.0f / MT5_GEAR_RATIO;
	K_aux_dq_null[6] = 10.0f / MT6_GEAR_RATIO;

	Integrator_Enable = true;

	ModeArm = -1;
	SubMode = 0;
}

void Reset_ControlLita()
{
}

inline long roundf( float x )
{
	if( x >= 0.0f )
		return (long)(x + 0.5f);
	else 
		return (long)(x - 0.5f);
}

inline void MotorPosPIDVelPIControl( int i )
{
	// PositionMode ( P-loop => ( V-loop => (driver) ) )

	//error
	error_qm_ENC[i] = qmCmd_ENC[i] - qm_ENC[i]; // "qmCmd_ENC_1" ?

	//erroe difference	
	diff_error_qm_ENC[i] = error_qm_ENC[i] - error_qm_ENC_1[i];

	//error accumulate
	if( Integrator_Enable == true)
	{
		if(abs(error_qm_ENC[i]) < 2 )
			summ_error_qm_ENC[i] = summ_error_qm_ENC_1[i];
		else
			summ_error_qm_ENC[i] = (error_qm_ENC[i] + error_qm_ENC_1[i])/2 + summ_error_qm_ENC_1[i];
	
		if( summ_error_qm_ENC[i] > summ_error_qm_limit_ENC[i])
			summ_error_qm_ENC[i] = summ_error_qm_limit_ENC[i];
		else if( summ_error_qm_ENC[i] < -summ_error_qm_limit_ENC[i])
			summ_error_qm_ENC[i] = -summ_error_qm_limit_ENC[i];

	//( P-loop => (V-loop) )
		dqmCmdCmd_ENC[i] = K_posvel_P[i]*error_qm_ENC[i] + K_posvel_p2I[i]*K_posvel_P[i]*summ_error_qm_ENC[i] + K_posvel_p2D[i]*K_posvel_P[i]*diff_error_qm_ENC[i];
	}
	else
	{
		dqmCmdCmd_ENC[i] = K_posvel_P[i]*error_qm_ENC[i] + K_posvel_p2D[i]*K_posvel_P[i]*diff_error_qm_ENC[i];
	}

	//error
	//error_dqm_ENC[i] = dqmCmdCmd_ENC[i] - dqm_ENC[i];
	//error_dqm_ENC[i] = dqmCmdCmd_ENC[i] + dqmCmd_ENC[i] - dqm_ENC[i]; // "dqmCmd_ENC_1"
	error_dqm_ENC[i] = 1.0f*(dqmCmdCmd_ENC[i] + dqmCmd_ENC[i]) - dqm_ENC[i];
	
	//erroe difference
	diff_error_dqm_ENC[i] = error_dqm_ENC[i] - error_dqm_ENC_1[i];
	
	//error accumulate
	/*if( Integrator_Enable == true)
	{
		if(abs(error_qm_ENC[i]) < 5 )
			summ_error_dqm_ENC[i] = summ_error_dqm_ENC_1[i];
		else
			summ_error_dqm_ENC[i] = 0.5f*(error_dqm_ENC[i] + error_dqm_ENC_1[i]) + summ_error_dqm_ENC_1[i];
	
		if( summ_error_dqm_ENC[i] > summ_error_dqm_limit_ENC[i])
			summ_error_dqm_ENC[i] = summ_error_dqm_limit_ENC[i];
		else if( summ_error_dqm_ENC[i] < -summ_error_dqm_limit_ENC[i])
			summ_error_dqm_ENC[i] = -summ_error_dqm_limit_ENC[i];
	
	//PI motor Vel controller ( V-loop => (driver) )
		TqmCmd[i] = K_vel_P[i]*error_dqm_ENC[i] ;//- K_damper[i]*(qm_ENC[i] - qm_ENC_1[i]);//damper
	}
	else
	{
	//PI motor Vel controller ( V-loop => (driver) )
		TqmCmd[i] = K_vel_P[i]*error_dqm_ENC[i] ;//- K_damper[i]*(qm_ENC[i] - qm_ENC_1[i]);//damper
	}*/
	TqmCmd[i] = K_vel_P[i]*error_dqm_ENC[i];

	//record last data (error)
			error_qm_ENC_1[i]=error_qm_ENC[i];			  error_dqm_ENC_1[i]=error_dqm_ENC[i];
	diff_error_qm_ENC_1[i]=diff_error_qm_ENC[i]; diff_error_dqm_ENC_1[i]=diff_error_dqm_ENC[i];
	summ_error_qm_ENC_1[i]=summ_error_qm_ENC[i]; //summ_error_dqm_ENC_1[i]=summ_error_dqm_ENC[i];

	//if( CompstGravity == true )
		//TqmCmd[i] = TqmCmd[i] + Tq_g[i]; //Torque_compansate
	
	if( SaftySTOP_Enable == true) // error_th need to depend on velocity command
	{ 
		if( abs(error_qm_ENC[0]) > 300 || abs(error_qm_ENC[1]) > 300 || abs(error_qm_ENC[2]) > 300 || abs(error_qm_ENC[3]) > 300)
			STOP = true;
	}
}

inline void CartesianControl()
{
	if( Free_P == false && Free_psi == false && Free_O == true)
	{
		FrictionComp();
		for( int i = 0; i < 4; i++)
			MotorPosPIDVelPIControl( i );
		for( int i = 4; i < USAGE_CHANNELS; i++)
			TqmCmd[i] = Tq_g[i] + Tq_fric[i];
	}
	else if( Free_P == true && Free_O == false)
	{
		FrictionComp();
		if( Free_psi == true)
		{
			for( int i = 0; i < 4; i++)
				TqmCmd[i] = Tq_g[i] + Tq_fric[i];
		}
		else
		{
			for( int i = 0; i < 4; i++)
				TqmCmd[i] = Tq_g[i] + Tq_fric[i] + Tq_null[i];
		}
		for( int i = 4; i < USAGE_CHANNELS; i++)
			MotorPosPIDVelPIControl( i );
	}
	else if ( Free_P == true && Free_O == true && Free_psi == true)
	{
		FrictionComp();

		for( int i = 0; i < USAGE_CHANNELS; i++)
			TqmCmd[i] = Tq_g[i] + Tq_fric[i];
	}
	else
	{
		for( int i = 0; i < USAGE_CHANNELS; i++)
			MotorPosPIDVelPIControl( i );
	}

	if( ql_rad(3,0) < q4_U_snglr) 
		TqmCmd[3] = TqmCmd[3] + 20.0f*(q4_U_snglr - ql_rad(3,0)); //
	else if( ql_rad(3,0) > q4_L_snglr)
		TqmCmd[3] = TqmCmd[3] + 20.0f*(q4_L_snglr - ql_rad(3,0)); //

	if( ql_rad(5,0) > q_p_limit(5,0)) 
		TqmCmd[5] = TqmCmd[5] + 40.0f*(q_p_limit(5,0) - ql_rad(5,0)); //
	else if( ql_rad(5,0) < q_n_limit(5,0))
		TqmCmd[5] = TqmCmd[5] + 40.0f*(q_n_limit(5,0) - ql_rad(5,0)); //
}

inline void ImpedenceTorque()
{
	FrictionComp();

	if( Free_P == false && Free_O == false)
	{
		delta_P0w = P0wCmd - P0w;

		delta_R07 = rr2delta(R07Cmd,R07);

		for( int i = 0; i < 3; i++)
			F0wCmd(i,0) = Kp_imped_F[i]*delta_P0w(i,0) + Kv_imped_F[i]*( V0wCmd(i,0) - V0w(i,0));
		
		for( int i = 0; i < 3; i++)
			M0wCmd(i,0) = Kp_imped_M[i]*delta_R07(i,0) + Kv_imped_M[i]*( W07Cmd(i,0) - W07(i,0));

		// z dir force control by force limit****************************************************//
		if ( FORCE_LIMIT == true)
		{
			if( F0wCmd(2,0) <= 1.5f*Fz_task) // adjust gain
			{
				FORCE_TRACKING = true;
				F0wCmd(2,0) = 1.4f*Fz_task;
			}
			else
				FORCE_TRACKING = false;
		}
		//Endof: z dir force control by force limit**********************************************//

		Tq_imped.block(0,0,4,1) = JV0wt*F0wCmd;
		Tq_imped.block(4,0,3,1) = JW4wt*R04.transpose()*M0wCmd;

		if( Free_psi == true)
		{
			TqmCmd[0] = Tq_imped(0,0)/MT0_GEAR_RATIO + Tq_null[0] + Tq_g[0] + 0.60f * Tq_fric[0]; //
			TqmCmd[1] = Tq_imped(1,0)/MT1_GEAR_RATIO + Tq_null[1] + Tq_g[1] + 0.60f * Tq_fric[1]; //
			TqmCmd[2] = Tq_imped(2,0)/MT2_GEAR_RATIO + Tq_null[2] + Tq_g[2] + 0.48f * Tq_fric[2]; //
			TqmCmd[3] = Tq_imped(3,0)/MT3_GEAR_RATIO + Tq_null[3] + Tq_g[3] + 0.48f * Tq_fric[3]; //
			
			TqmCmd[4] = Tq_imped(4,0)/MT4_GEAR_RATIO + Tq_null[4] + Tq_g[4] + 0.12f * Tq_fric[4]; //
			TqmCmd[5] = Tq_imped(5,0)/MT5_GEAR_RATIO + Tq_null[5] + Tq_g[5] + 0.12f * Tq_fric[5]; //
			TqmCmd[6] = Tq_imped(6,0)/MT6_GEAR_RATIO + Tq_null[6] + Tq_g[6] + 0.12f * Tq_fric[6]; //
		}
		else
		{
			TqmCmd[0] = Tq_imped(0,0)/MT0_GEAR_RATIO + Tq_null[0] + Tq_g[0] + 0.8f * Tq_fric[0]; //0.1
			TqmCmd[1] = Tq_imped(1,0)/MT1_GEAR_RATIO + Tq_null[1] + Tq_g[1] + 0.8f * Tq_fric[1]; //0.1
			TqmCmd[2] = Tq_imped(2,0)/MT2_GEAR_RATIO + Tq_null[2] + Tq_g[2] + 0.6f * Tq_fric[2]; //0.05
			TqmCmd[3] = Tq_imped(3,0)/MT3_GEAR_RATIO + Tq_null[3] + Tq_g[3] + 0.6f * Tq_fric[3]; //0.05
			
			TqmCmd[4] = Tq_imped(4,0)/MT4_GEAR_RATIO + Tq_null[4] + Tq_g[4] + 0.0f * Tq_fric[4]; //0.05
			TqmCmd[5] = Tq_imped(5,0)/MT5_GEAR_RATIO + Tq_null[5] + Tq_g[5] + 0.0f * Tq_fric[5]; //0.05
			TqmCmd[6] = Tq_imped(6,0)/MT6_GEAR_RATIO + Tq_null[6] + Tq_g[6] + 0.0f * Tq_fric[6]; //0.05
		}

	}
	else if( Free_P == false && Free_O == true)
	{
		//P0wCmd = P07Cmd - R07*L7wt;
		//delta_P0w = P0wCmd - P0w;
		delta_P0w = P07Cmd - R07*L7wt- P0w;

		for( int i = 0; i < 3; i++)
			F0wCmd(i,0) = Kp_imped_F[i]*delta_P0w(i,0) + Kv_imped_F[i]*( V0wCmd(i,0) - V0w(i,0));

		Tq_imped.block(0,0,4,1) = JV0wt*F0wCmd;
		Tq_imped(4,0) = 0.0f;
		Tq_imped(5,0) = 0.0f;
		Tq_imped(6,0) = 0.0f;

		TqmCmd[0] = Tq_imped(0,0)/MT0_GEAR_RATIO + Tq_null[0] + Tq_g[0] + 0.10f * Tq_fric[0]; //
		TqmCmd[1] = Tq_imped(1,0)/MT1_GEAR_RATIO + Tq_null[1] + Tq_g[1] + 0.10f * Tq_fric[1]; //
		TqmCmd[2] = Tq_imped(2,0)/MT2_GEAR_RATIO + Tq_null[2] + Tq_g[2] + 0.05f * Tq_fric[2]; //
		TqmCmd[3] = Tq_imped(3,0)/MT3_GEAR_RATIO + Tq_null[3] + Tq_g[3] + 0.05f * Tq_fric[3]; //
		
		for( int i = 4; i < USAGE_CHANNELS; i++)
			TqmCmd[i] = Tq_null[i] + Tq_g[i] + 0.90f * Tq_fric[i]; //

	}
	else if( Free_P == true && Free_O == false)
	{
		delta_R07 = rr2delta(R07Cmd,R07);

		for( int i = 0; i < 3; i++)
			M0wCmd(i,0) = Kp_imped_M[i]*delta_R07(i,0) + Kv_imped_M[i]*( W07Cmd(i,0) - W07(i,0));

		if( dirAuxF == true)
		{
			Tq_imped(0,0) = 0.0f;
			Tq_imped(1,0) = 0.0f;
			Tq_imped(2,0) = 0.0f;
			Tq_imped(3,0) = 0.0f;
		}
		else
		{
			Tq_imped(0,0) = 0.0f;
			Tq_imped(1,0) = 0.0f;
			Tq_imped(2,0) = 0.0f;
			Tq_imped(3,0) = 0.0f;
		}
		Tq_imped.block(4,0,3,1) = JW4wt*R04.transpose()*M0wCmd;

		for( int i = 0; i < 4; i++)
			TqmCmd[i] = Tq_null[i] + Tq_g[i] + 1.00f * Tq_fric[i]; //
		
		TqmCmd[4] = Tq_imped(4,0)/MT4_GEAR_RATIO + Tq_null[4] + Tq_g[4] + 0.05f * Tq_fric[4]; //
		TqmCmd[5] = Tq_imped(5,0)/MT5_GEAR_RATIO + Tq_null[5] + Tq_g[5] + 0.05f * Tq_fric[5]; //
		TqmCmd[6] = Tq_imped(6,0)/MT6_GEAR_RATIO + Tq_null[6] + Tq_g[6] + 0.05f * Tq_fric[6]; //

	}
	else
	{
		Tq_imped << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

		for( int i = 0; i < USAGE_CHANNELS; i++)
			TqmCmd[i] =  Tq_g[i] + Tq_fric[i];
	}

	//if( abs(ql_rad(1,0)) < 0.05f  ) //
	//{
	//	TqmCmd[0] = 0.0f;
	//	TqmCmd[2] = 0.0f;
	//}
	//if( abs(ql_rad(5,0)) < 0.05f  ) //
	//{
	//	TqmCmd[4] = 0.0f;
	//	TqmCmd[6] = 0.0f;
	//}
	if( ql_rad(3,0) < q4_U_snglr) 
		TqmCmd[3] = TqmCmd[3] + 20.0f*(q4_U_snglr - ql_rad(3,0)); //
	else if( ql_rad(3,0) > q4_L_snglr)
		TqmCmd[3] = TqmCmd[3] + 20.0f*(q4_L_snglr - ql_rad(3,0)); //

	if( ql_rad(5,0) > q_p_limit(5,0)) 
		TqmCmd[5] = TqmCmd[5] + 40.0f*(q_p_limit(5,0) - ql_rad(5,0)); //
	else if( ql_rad(5,0) < q_n_limit(5,0))
		TqmCmd[5] = TqmCmd[5] + 40.0f*(q_n_limit(5,0) - ql_rad(5,0)); //
}

inline void TqCmd2SetValue( int i )
{
	SetValue[i] = torque2setvalue[i]*TqmCmd[i];
	
	// Saturator
	if( SetValue[i] >= DAC_MAX_VOLTAGE)
		SetValue[i] = DAC_MAX_VOLTAGE;
	else if( SetValue[i] <= -DAC_MAX_VOLTAGE)
		SetValue[i] = -DAC_MAX_VOLTAGE;
	
}

inline void MotorPosRampGenerator( int i )
{
	//dqmTrgt_ENC[i] = (float)(qmTrgt_ENC[i] - qmCmd_ENC_1[i]);

	if( MOV[i] == true )
	{
		if( qmTrgt_ENC[i] > q_pLimit_ENC[i])
			qmTrgt_ENC[i] = q_pLimit_ENC[i];
		else if( qmTrgt_ENC[i] < q_nLimit_ENC[i])
			qmTrgt_ENC[i] = q_nLimit_ENC[i];

		dqmTrgt_ENC[i] = (float)(qmTrgt_ENC[i] - qmCmd_ENC[i]);

		if( ( abs(dqmTrgt_ENC[i]) < 2.5f || abs(dqmTrgt_ENC[i]) < 0.5f*Dec_Limit_ENC[i] ) && abs(dqmCmd_ENC[i]) < 0.5f*Dec_Limit_ENC[i] )
		{
			dqmTrgt_ENC[i] = 0.0f;
			qmCmd_ENC[i] = qmTrgt_ENC[i];
			dqmCmd_ENC[i] = 0.0f;
			MOV[i] = false;
		}
		else
		{
			qVelMax = sqrt( 2.0f* abs( dqmTrgt_ENC[i] )* Dec_Limit_ENC[i] );

			if( qVelMax < Vel_Limit_ENC[i])
			{
				if( dqmTrgt_ENC[i] > qVelMax)
					dqmTrgt_ENC[i] = qVelMax;
				else if( dqmTrgt_ENC[i] < -qVelMax)
					dqmTrgt_ENC[i] = -qVelMax;

				if( dqmTrgt_ENC[i] - dqmCmd_ENC_1[i] > Dec_Limit_ENC[i])
					dqmCmd_ENC[i] = dqmCmd_ENC_1[i] + Dec_Limit_ENC[i];
				else if( dqmTrgt_ENC[i] - dqmCmd_ENC_1[i] < -Dec_Limit_ENC[i])
					dqmCmd_ENC[i] = dqmCmd_ENC_1[i] - Dec_Limit_ENC[i];
				else
					dqmCmd_ENC[i] = dqmTrgt_ENC[i];
			}
			else
			{
				if( dqmTrgt_ENC[i] > Vel_Limit_ENC[i])
					dqmTrgt_ENC[i] = Vel_Limit_ENC[i];
				else if( dqmTrgt_ENC[i] < -Vel_Limit_ENC[i])
					dqmTrgt_ENC[i] = -Vel_Limit_ENC[i];

				if( dqmTrgt_ENC[i] - dqmCmd_ENC_1[i] > Acc_Limit_ENC[i])
					dqmCmd_ENC[i] = dqmCmd_ENC_1[i] + Acc_Limit_ENC[i];
				else if( dqmTrgt_ENC[i] - dqmCmd_ENC_1[i] < -Acc_Limit_ENC[i])
					dqmCmd_ENC[i] = dqmCmd_ENC_1[i] - Acc_Limit_ENC[i];
				else
					dqmCmd_ENC[i] = dqmTrgt_ENC[i];
			}
			ddqmCmd_ENC[i] = dqmCmd_ENC[i] - dqmCmd_ENC_1[i];

			qmCmd_ENC[i] = qmCmd_ENC_1[i] + roundf( dqmCmd_ENC[i] + 0.5f*ddqmCmd_ENC[i] );
		}

		// test
		if( qmCmd_ENC[i] == qmCmd_ENC_1[i])
			noMOV_count[i] = noMOV_count[i] + 1;

		if( noMOV_count[i] > 5000 )
		{
			dqmTrgt_ENC[i] = 0.0f;
			dqmCmd_ENC[i] = 0.0f;
			noMOV_count[i] = 0;
			MOV[i] = false;
		}
		//
	}
}

//inline void LinVelGenerator()
//{
//
//}

inline void PositionRampGenerator()
{
	if(!MOVL_newflag){
		if( MOVL_L == true)
		{
			//P07Trgt = T07Trgt.block(0,3,3,1);
			dP07Trgt = P07Trgt - P07Cmd;
			dP07Trgt_norm = dP07Trgt.norm();
			//V07Cmd_norm = V07Cmd.norm();

			if( dP07Trgt_norm <= TOL_L)
			{
				if( V07Cmd_norm <= TOL_L)
					dP07Trgt_Normalized << 0.0f, 0.0f, 0.0f;
				else
					dP07Trgt_Normalized = V07Cmd / V07Cmd_norm;
			}
			else
			{
				dP07Trgt_Normalized = dP07Trgt / dP07Trgt_norm;
			}

			if( dP07Trgt_norm < 0.5f*Lin_Acc_limit && V07Cmd_norm < 0.5f*Lin_Acc_limit)
			{
				// go to final point
				MOVLDEC_L = false;
				MOVL_L = false;
				P07Cmd = P07Trgt;
				V07Cmd << 0.0f, 0.0f, 0.0f;
				V07Cmd_norm = 0.0f;
			}
			else
			{
				LinearSpeedMax = sqrt( 2.0f* dP07Trgt_norm * Lin_Dec_limit );

				if( LinearSpeedMax < Lin_Vel_limit)
				{
					MOVLDEC_L = true;

					if( dP07Trgt_norm > LinearSpeedMax)
						dP07Trgt_norm = LinearSpeedMax;

					if( dP07Trgt_norm - V07Cmd_norm > Lin_Dec_limit)
						V07Trgt_norm = V07Cmd_norm + Lin_Dec_limit;
					else if( dP07Trgt_norm - V07Cmd_norm < -Lin_Dec_limit)
						V07Trgt_norm = V07Cmd_norm - Lin_Dec_limit;
					else
						V07Trgt_norm = dP07Trgt_norm;
				}
				else
				{
					if( dP07Trgt_norm < 0.64*V07Cmd_norm*V07Cmd_norm/Lin_Acc_limit)
					//if( dP07Trgt_norm < Lin_Vel_limit*Lin_Vel_limit/Lin_Acc_limit )
						MOVLDEC_L = true;
					else
						MOVLDEC_L = false;

					if( dP07Trgt_norm > Lin_Vel_limit)
						dP07Trgt_norm = Lin_Vel_limit;

					if( dP07Trgt_norm - V07Cmd_norm > Lin_Acc_limit)
						V07Trgt_norm = V07Cmd_norm + Lin_Acc_limit;
					else if( dP07Trgt_norm - V07Cmd_norm < -Lin_Acc_limit)
						V07Trgt_norm = V07Cmd_norm - Lin_Acc_limit;
					else
						V07Trgt_norm = dP07Trgt_norm;
				}

				V07Trgt = V07Trgt_norm*dP07Trgt_Normalized;
				dV07Trgt = V07Trgt - V07Cmd;
				dV07Trgt_norm = dV07Trgt.norm();

				if( dV07Trgt_norm <= TOL_L)
				{
					V07Cmd = V07Cmd_1;
					V07Cmd_norm = V07Cmd.norm();
				}
				else if( dV07Trgt_norm > Lin_Acc_limit)
				{
					V07Cmd = V07Cmd_1 + Lin_Acc_limit*dV07Trgt/dV07Trgt_norm;
					V07Cmd_norm = V07Cmd.norm();
					if( V07Cmd_norm > V07Trgt_norm)
					{
						V07Cmd = V07Trgt_norm*V07Cmd/V07Cmd_norm;
						V07Cmd_norm = V07Trgt_norm;
					}
				}
				else
				{
					V07Cmd = V07Trgt;
					V07Cmd_norm = V07Cmd.norm();
				}
			}
			
			P07Cmd = P07Cmd + 1.5f*V07Cmd - 0.5f* V07Cmd_1;
			if(write)
				outputFile<<P07Cmd(0)<<" "<<P07Cmd(1)<<" "<<P07Cmd(2)<<endl;
		}
	}else{
		if(traceIndex!=sketchPositionXY[sketchIndex].size()-1){
			Vector3f point;
			point(0) = sketchPositionXY[sketchIndex][traceIndex](0);
			point(1) = sketchPositionXY[sketchIndex][traceIndex](1);
			point(2) = z_axis_black;
			P07Cmd = point;
			cout<<P07Cmd(0)<<" "<<P07Cmd(1)<<" "<<P07Cmd(2)<<endl;
			traceIndex++;
		}else{
			MOVLDEC_L = false;
			MOVL_L = false;
		}

		/*if(abs(1000.0f*(T07Cmd(0,3)-T07(0,3)))<1.0&&abs(1000.0f*(T07Cmd(1,3)-T07(1,3)))<1.0){
			MOVLDEC_L = false;
			MOVL_L = false;
		}
		P07Cmd = P07Trgt;*/
	}

}

inline void OrientationRampGenerator()
{
	if( MOVL_R == true)
	{
		//R07Trgt = T07Trgt.block(0,0,3,3);
		dR07Trgt = rr2delta( R07Trgt,R07Cmd );
		dR07Trgt_norm = dR07Trgt.norm();
		//W07Cmd_norm = W07Cmd.norm();

		if( dR07Trgt_norm <= TOL_R)
		{
			if( W07Cmd_norm <= TOL_R)
				dR07Trgt_Normalized << 0.0f, 0.0f, 0.0f;
			else
				dR07Trgt_Normalized = W07Cmd / W07Cmd_norm;
		}
		else
		{
			dR07Trgt_Normalized = dR07Trgt / dR07Trgt_norm;
		}

		if( dR07Trgt_norm < 0.5f*Ang_Acc_limit && W07Cmd_norm < 0.5f*Ang_Acc_limit)
		{
			MOVL_R = false;
			R07Cmd = R07Trgt;
			W07Cmd << 0.0f, 0.0f, 0.0f;
			W07Cmd_norm = 0.0f;
		}
		else
		{
			AngularSpeedMax = sqrt( 2.0f* dR07Trgt_norm * Ang_Dec_limit );

			if( AngularSpeedMax < Ang_Vel_limit)
			{
				if( dR07Trgt_norm > AngularSpeedMax)
					dR07Trgt_norm = AngularSpeedMax;

				if( dR07Trgt_norm - W07Cmd_norm > Ang_Dec_limit)
					W07Trgt_norm = W07Cmd_norm + Ang_Dec_limit;
				else if( dR07Trgt_norm - W07Cmd_norm < -Ang_Dec_limit)
					W07Trgt_norm = W07Cmd_norm - Ang_Dec_limit;
				else
					W07Trgt_norm = dR07Trgt_norm;
			}
			else
			{
				if( dR07Trgt_norm > Ang_Vel_limit)
					dR07Trgt_norm = Ang_Vel_limit;

				if( dR07Trgt_norm - W07Cmd_norm > Ang_Acc_limit)
					W07Trgt_norm = W07Cmd_norm + Ang_Acc_limit;
				else if( dR07Trgt_norm - W07Cmd_norm < -Ang_Acc_limit)
					W07Trgt_norm = W07Cmd_norm - Ang_Acc_limit;
				else
					W07Trgt_norm = dR07Trgt_norm;
			}

			W07Trgt = W07Trgt_norm*dR07Trgt_Normalized;
			dW07Trgt = W07Trgt - W07Cmd;
			dW07Trgt_norm = dW07Trgt.norm();

			if( dW07Trgt_norm <= TOL_L)
			{
				W07Cmd = W07Cmd_1;
				W07Cmd_norm = W07Cmd.norm();
			}
			else if( dW07Trgt_norm > Ang_Acc_limit)
			{
				W07Cmd = W07Cmd_1 + Ang_Acc_limit*dW07Trgt/dW07Trgt_norm;
				W07Cmd_norm = W07Cmd.norm();
				if( W07Cmd_norm > W07Trgt_norm)
				{
					W07Cmd = W07Trgt_norm*W07Cmd/W07Cmd_norm;
					W07Cmd_norm = W07Trgt_norm;
				}
			}
			else
			{
				W07Cmd = W07Trgt;
				W07Cmd_norm = W07Cmd.norm();
			}
		}
		R07Cmd = delta2rr( 1.5f*W07Cmd-0.5f*W07Cmd_1, R07Cmd_1);
	}
}

inline void ArmAngleRampGenerator()
{
	if( MOVJN == true )
	{
		if(PsiTrgt > Psi_p_limit)
			PsiTrgt = Psi_p_limit;
		else if( PsiTrgt < Psi_n_limit)
			PsiTrgt = Psi_n_limit;

		dPsiTrgt = PsiTrgt - PsiCmd;
		
		if( abs(dPsiTrgt) < 0.5f*Jn_Dec_limit && abs(dPsiCmd) < 0.5f*Jn_Dec_limit )
		{
			PsiCmd = PsiTrgt;
			dPsiCmd = 0.0f;
			MOVJN = false;
		}
		else
		{
			JnSpeedMax = sqrt( 2.0f* abs( dPsiTrgt )* Jn_Dec_limit );

			if( JnSpeedMax < Jn_Vel_limit)
			{
				if( dPsiTrgt > JnSpeedMax)
					dPsiTrgt = JnSpeedMax;
				else if( dPsiTrgt < -JnSpeedMax)
					dPsiTrgt = -JnSpeedMax;

				if( dPsiTrgt - dPsiCmd > Jn_Dec_limit)
					dPsiCmd = dPsiCmd + Jn_Dec_limit;
				else if( dPsiTrgt - dPsiCmd < -Jn_Dec_limit)
					dPsiCmd = dPsiCmd - Jn_Dec_limit;
				else
					dPsiCmd = dPsiTrgt;
			}
			else
			{
				if( dPsiTrgt > Jn_Vel_limit)
					dPsiTrgt = Jn_Vel_limit;
				else if( dPsiTrgt < -Jn_Vel_limit)
					dPsiTrgt = -Jn_Vel_limit;

				if( dPsiTrgt - dPsiCmd > Jn_Acc_limit)
					dPsiCmd = dPsiCmd + Jn_Acc_limit;
				else if( dPsiTrgt - dPsiCmd < -Jn_Acc_limit)
					dPsiCmd = dPsiCmd - Jn_Acc_limit;
				else
					dPsiCmd = dPsiTrgt;
			}
			PsiCmd = PsiCmd_1 + 1.5f*dPsiCmd - 0.5f*dPsiCmd_1;
		}
	}//End if(MOVJN == true)
}

inline void CosSinUpdate( bool isCMD)
{
	if( isCMD == true )
	{
		c1 = cosqCmd[0]; c2 = cosqCmd[1]; c3 = cosqCmd[2]; c4 = cosqCmd[3]; c5 = cosqCmd[4]; c6 = cosqCmd[5]; c7 = cosqCmd[6];
		s1 = sinqCmd[0]; s2 = sinqCmd[1]; s3 = sinqCmd[2]; s4 = sinqCmd[3]; s5 = sinqCmd[4]; s6 = sinqCmd[5]; s7 = sinqCmd[6];
	}
	else
	{
		c1 = cosq[0]; c2 = cosq[1]; c3 = cosq[2]; c4 = cosq[3]; c5 = cosq[4]; c6 = cosq[5]; c7 = cosq[6];
		s1 = sinq[0]; s2 = sinq[1]; s3 = sinq[2]; s4 = sinq[3]; s5 = sinq[4]; s6 = sinq[5]; s7 = sinq[6];
	}
}

inline void FwdKineUpdate_1( bool isCMD)
{
	if( isCMD == true)
	{
		R03Cmd(0,0) = c1*c2*c3 - s1*s3; R03Cmd(0,1) = -c1*s2; R03Cmd(0,2) = -c3*s1 - c1*c2*s3;
		R03Cmd(1,0) = c1*s3 + c2*c3*s1; R03Cmd(1,1) = -s1*s2; R03Cmd(1,2) =  c1*c3 - c2*s1*s3;
		R03Cmd(2,0) =           -c3*s2; R03Cmd(2,1) =    -c2; R03Cmd(2,2) =             s2*s3;

		R34Cmd(0,0) =   c4; R34Cmd(0,1) = 0.0f; R34Cmd(0,2) =   s4;
		R34Cmd(1,0) =   s4; R34Cmd(1,1) = 0.0f; R34Cmd(1,2) =  -c4;
		R34Cmd(2,0) = 0.0f; R34Cmd(2,1) = 1.0f; R34Cmd(2,2) = 0.0f;

		R47Cmd(0,0) = c5*c6*c7 - s5*s7; R47Cmd(0,1) = -c7*s5 - c5*c6*s7; R47Cmd(0,2) = c5*s6;
		R47Cmd(1,0) = c5*s7 + c6*c7*s5; R47Cmd(1,1) =  c5*c7 - c6*s5*s7; R47Cmd(1,2) = s5*s6;
		R47Cmd(2,0) =           -c7*s6; R47Cmd(2,1) =             s6*s7; R47Cmd(2,2) =    c6;

		R04Cmd = R03Cmd*R34Cmd;
		
		P0wCmd = R03Cmd*L3se + R04Cmd*L4ew;

		R07Cmd = R04Cmd*R47Cmd;

		P07Cmd = P0wCmd + R07Cmd*L7wt;
	}
	else
	{
		//FK
		R03(0,0) = c1*c2*c3 - s1*s3; R03(0,1) = -c1*s2; R03(0,2) = -c3*s1 - c1*c2*s3;
		R03(1,0) = c1*s3 + c2*c3*s1; R03(1,1) = -s1*s2; R03(1,2) =  c1*c3 - c2*s1*s3;
		R03(2,0) =           -c3*s2; R03(2,1) =    -c2; R03(2,2) =             s2*s3;

		R34(0,0) =   c4; R34(0,1) = 0.0f; R34(0,2) =   s4;
		R34(1,0) =   s4; R34(1,1) = 0.0f; R34(1,2) =  -c4;
		R34(2,0) = 0.0f; R34(2,1) = 1.0f; R34(2,2) = 0.0f;

		R47(0,0) = c5*c6*c7 - s5*s7; R47(0,1) = -c7*s5 - c5*c6*s7; R47(0,2) = c5*s6;
		R47(1,0) = c5*s7 + c6*c7*s5; R47(1,1) =  c5*c7 - c6*s5*s7; R47(1,2) = s5*s6;
		R47(2,0) =           -c7*s6; R47(2,1) =             s6*s7; R47(2,2) =    c6;

		/*
		R03 = [
			cos(q1)*cos(q2)*cos(q3) - sin(q1)*sin(q3), -cos(q1)*sin(q2), - cos(q3)*sin(q1) - cos(q1)*cos(q2)*sin(q3);
			cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1), -sin(q1)*sin(q2),   cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3);
									 -cos(q3)*sin(q2),         -cos(q2),                             sin(q2)*sin(q3)
				];
		R34 = [ cos(q4), 0,  sin(q4);
				sin(q4), 0, -cos(q4);
					  0, 1,        0];
		R47 = [
			cos(q5)*cos(q6)*cos(q7) - sin(q5)*sin(q7), - cos(q7)*sin(q5) - cos(q5)*cos(q6)*sin(q7), cos(q5)*sin(q6);
			cos(q5)*sin(q7) + cos(q6)*cos(q7)*sin(q5),   cos(q5)*cos(q7) - cos(q6)*sin(q5)*sin(q7), sin(q5)*sin(q6);
									- cos(q7)*sin(q6),                             sin(q6)*sin(q7),         cos(q6)
				]; 
		*/
		R04 = R03*R34;
		
		P0w = R03*L3se + R04*L4ew;

		R07 = R04*R47;

		P07 = P0w + R07*L7wt;
	}
}

inline void FwdKineUpdate_2( bool isCMD)
{
	float x_w, y_w, z_w, r_2, r_w, d_2, cq1o, sq1o ,cq2o ,sq2o, cpsi, spsi;

	if( isCMD == true)
	{
		x_w = P0wCmd(0,0);
		y_w = P0wCmd(1,0);
		z_w = P0wCmd(2,0);
		r_2 = x_w*x_w + y_w*y_w;
		r_w = sqrt(r_2);
		d_2 = r_2 + z_w*z_w;
		P0wCmd_norm = sqrt(d_2);

		if( r_w < 0.000001f)
			q1oCmd = q1oCmd_1;
		else
			q1oCmd = atan2f( y_w, x_w);

		q2oCmd = B1 - acosf( (d1a2+d_2-d2a2)/(2.0f*d1a*P0wCmd_norm) ) - atan2f(z_w,r_w);

		cq1o = cosf(q1oCmd); sq1o = sinf(q1oCmd);
		cq2o = cosf(q2oCmd); sq2o = sinf(q2oCmd);

		R03_oCmd(0,0) = cq1o*cq2o; R03_oCmd(0,1) = -cq1o*sq2o; R03_oCmd(0,2) = -sq1o;
		R03_oCmd(1,0) = cq2o*sq1o; R03_oCmd(1,1) = -sq1o*sq2o; R03_oCmd(1,2) =  cq1o;
        R03_oCmd(2,0) =     -sq2o; R03_oCmd(2,1) =      -cq2o; R03_oCmd(2,2) =  0.0f;

		u0wCmd = P0wCmd/P0wCmd_norm;

		R_psi = R03Cmd*R03_oCmd.transpose();
			
		cpsi = (R_psi.trace() - 1.0f) / 2.0f;
		
		if( abs(u0wCmd(0,0)) > abs(u0wCmd(1,0)) && abs(u0wCmd(0,0)) > abs(u0wCmd(2,0)) )
			spsi = (R_psi(2,1) - R_psi(1,2)) / u0wCmd(0,0) / 2.0f; 
		else if( abs(u0wCmd(1,0)) > abs(u0wCmd(2,0)) && abs(u0wCmd(1,0)) > abs(u0wCmd(0,0)) )
			spsi = (R_psi(0,2) - R_psi(2,0)) / u0wCmd(1,0) / 2.0f;
		else if( abs(u0wCmd(2,0)) > abs(u0wCmd(0,0)) && abs(u0wCmd(2,0)) > abs(u0wCmd(1,0)) )
			spsi = (R_psi(1,0) - R_psi(0,1)) / u0wCmd(2,0) / 2.0f; 
		else
			spsi = (R_psi(2,1) - R_psi(1,2)) / u0wCmd(0,0) / 2.0f; 

		PsiCmd = atan2f( spsi, cpsi);
	}
	else
	{
		x_w = P0w(0,0);
		y_w = P0w(1,0);
		z_w = P0w(2,0);
		r_2 = x_w*x_w + y_w*y_w;
		r_w = sqrt(r_2);
		d_2 = r_2 + z_w*z_w;
		P0w_norm = sqrt(d_2);

		if( r_w < 0.000001f)
			q1o = q1o_1;
		else
			q1o = atan2f( y_w, x_w);

		q2o = B1 - acosf( (d1a2+d_2-d2a2)/(2.0f*d1a*P0w_norm) ) - atan2f(z_w,r_w);

		cq1o = cosf(q1o); sq1o = sinf(q1o);
		cq2o = cosf(q2o); sq2o = sinf(q2o);

		R03_o(0,0) = cq1o*cq2o; R03_o(0,1) = -cq1o*sq2o; R03_o(0,2) = -sq1o;
		R03_o(1,0) = cq2o*sq1o; R03_o(1,1) = -sq1o*sq2o; R03_o(1,2) =  cq1o;
        R03_o(2,0) =     -sq2o; R03_o(2,1) =      -cq2o; R03_o(2,2) =  0.0f;

		u0w = P0w/P0w_norm;

		R_psi = R03*R03_o.transpose();

		cpsi = (R_psi.trace() - 1.0f) / 2.0f;

		if( abs(u0w(0,0)) > abs(u0w(1,0)) && abs(u0w(0,0)) > abs(u0w(2,0)) )
			spsi = (R_psi(2,1) - R_psi(1,2)) / u0w(0,0) / 2.0f; 
		else if( abs(u0w(1,0)) > abs(u0w(2,0)) && abs(u0w(1,0)) > abs(u0w(0,0)) )
			spsi = (R_psi(0,2) - R_psi(2,0)) / u0w(1,0) / 2.0f;
		else if( abs(u0w(2,0)) > abs(u0w(0,0)) && abs(u0w(2,0)) > abs(u0w(1,0)) )
			spsi = (R_psi(1,0) - R_psi(0,1)) / u0w(2,0) / 2.0f; 
		else
			spsi = (R_psi(2,1) - R_psi(1,2)) / u0w(0,0) / 2.0f; 

		Psi = atan2f( spsi, cpsi);
	}
}

inline void InvKineCompute_1()
{
	float x_w, y_w, z_w, r_2, r_w, d_2, cq1o, sq1o ,cq2o ,sq2o, cq4, sq4;

	x_w = P0wCmd(0,0);
	y_w = P0wCmd(1,0);
	z_w = P0wCmd(2,0);
	r_2 = x_w*x_w + y_w*y_w;
	r_w = sqrt(r_2);
	d_2 = r_2 + z_w*z_w;
	P0wCmd_norm = sqrt(d_2);
	
	qlCmd_rad(3,0) = B3 - acosf((d1a2 + d2a2- d_2)/(2.0f*d1a*d2a));

	cq4 = cosf(qlCmd_rad(3,0));
	sq4 = sinf(qlCmd_rad(3,0));

	R34Cmd(0,0) =  cq4; R34Cmd(0,1) = 0.0f; R34Cmd(0,2) =  sq4;
	R34Cmd(1,0) =  sq4; R34Cmd(1,1) = 0.0f; R34Cmd(1,2) = -cq4;
	R34Cmd(2,0) = 0.0f; R34Cmd(2,1) = 1.0f; R34Cmd(2,2) = 0.0f;
	
	if( r_w < 0.000001f)
		q1oCmd = q1oCmd_1;
	else
		q1oCmd = atan2f( y_w, x_w);
	q2oCmd = B1 - acosf( (d1a2+d_2-d2a2)/(2.0f*d1a*P0wCmd_norm) ) - atan2f(z_w,r_w);

	cq1o = cosf(q1oCmd); sq1o = sinf(q1oCmd);
	cq2o = cosf(q2oCmd); sq2o = sinf(q2oCmd);

	R03_oCmd(0,0) = cq1o*cq2o; R03_oCmd(0,1) = -cq1o*sq2o; R03_oCmd(0,2) = -sq1o;
	R03_oCmd(1,0) = cq2o*sq1o; R03_oCmd(1,1) = -sq1o*sq2o; R03_oCmd(1,2) =  cq1o;
    R03_oCmd(2,0) =     -sq2o; R03_oCmd(2,1) =      -cq2o; R03_oCmd(2,2) =  0.0f;

	u0wCmd = P0wCmd/P0wCmd_norm;

	Su = skew( u0wCmd );

	As = Su*R03_oCmd;
	Bs = -Su*As;
	Cs = u0wCmd*u0wCmd.transpose()*R03_oCmd;

}

inline void InvKineCompute_2( bool isCMD)
{
	float sp, cp, rs_r, rc_r, rs_p, rc_p, rs_y, rc_y;//, q_temp;
	float qr1, qp1, qy1, qr2, qp2, qy2;
	
	if( abs(PsiCmd) < 0.000001f ) //
	{
		R03Cmd = R03_oCmd;
		qlCmd_rad(0,0) = q1oCmd;
		qlCmd_rad(1,0) = q2oCmd;
		qlCmd_rad(2,0) = 0.0f;
	}
	else
	{
		sp =sinf(PsiCmd);
		cp =cosf(PsiCmd);

		R03Cmd = sp*As + cp*Bs + Cs;

		rs_r = -R03Cmd(1,1);
		rc_r = -R03Cmd(0,1);
		rc_p = -R03Cmd(2,1);
		rs_y =  R03Cmd(2,2);
		rc_y = -R03Cmd(2,0);

		qr1 = atan2f(rs_r, rc_r);
		qp1 = acosf(rc_p);
		qy1 = atan2f(rs_y, rc_y);
		
		//qr2 = atan2f(-rs_r, -rc_r);
		//qp2 = -qp1;
		//qy2 = atan2f(-rs_y, -rc_y);
		
		qlCmd_rad(0,0) = qr1;
		qlCmd_rad(1,0) = qp1;
		qlCmd_rad(2,0) = qy1;

		/*if( abs(qr1-qlCmd_rad_1(0,0)) + abs(qy1-qlCmd_rad_1(2,0)) < abs(qr2-qlCmd_rad_1(0,0)) + abs(qy2-qlCmd_rad_1(2,0)) )
		{
			qlCmd_rad(0,0) = qr1;
			qlCmd_rad(1,0) = qp1;
			qlCmd_rad(2,0) = qy1;
		}
		else if( abs(qr1-qlCmd_rad_1(0,0)) + abs(qy1-qlCmd_rad_1(2,0)) > abs(qr2-qlCmd_rad_1(0,0)) + abs(qy2-qlCmd_rad_1(2,0)) )
		{
			qlCmd_rad(0,0) = qr2;
			qlCmd_rad(1,0) = qp2;
			qlCmd_rad(2,0) = qy2;
		}
		else
		{
			if( abs(qp1 - qlCmd_rad_1(1,0)) <= abs(qp2 - qlCmd_rad_1(1,0)) )
			{
				qlCmd_rad(0,0) = qr1;
				qlCmd_rad(1,0) = qp1;
				qlCmd_rad(2,0) = qy1;
			}
			else
			{
				qlCmd_rad(0,0) = qr2;
				qlCmd_rad(1,0) = qp2;
				qlCmd_rad(2,0) = qy2;
			}
		}*/

		//if( abs(1.0f - abs(rc_p)) < 0.000001f ) //
		//{
		//	qlCmd_rad(0,0) = qlCmd_rad_1(0,0);
		//	qlCmd_rad(1,0) = 0.0f;
		//	qlCmd_rad(2,0) = qlCmd_rad_1(2,0);
		//}
		//else
		//{
		//	q_temp = atan2f( rs_r, rc_r);
		//	if( q_temp - qlCmd_rad_1(0,0) > 3.14f )
		//		qlCmd_rad(0,0) = q_temp - pi;
		//	else if( q_temp - qlCmd_rad_1(0,0) < -3.14f )
		//		qlCmd_rad(0,0) = q_temp + pi;
		//	else
		//		qlCmd_rad(0,0) = q_temp;

		//	q_temp = atan2f( rs_y, rc_y);
		//	if( q_temp - qlCmd_rad_1(2,0) > 3.14f )
		//		qlCmd_rad(2,0) = q_temp - pi;
		//	else if( q_temp - qlCmd_rad_1(2,0) < -3.14f )
		//		qlCmd_rad(2,0) = q_temp + pi;
		//	else
		//		qlCmd_rad(2,0) = q_temp;

		//	if( abs(rc_r) > abs(rs_r) )
		//	{
		//		rs_p = 	rc_r / cosf(qlCmd_rad(0,0)); 
		//	}
		//	else
		//	{
		//		rs_p = 	rs_r / sinf(qlCmd_rad(0,0)); 
		//	}
		//
		//	qlCmd_rad(1,0) = atan2( rs_p, rc_p);
		//}
	}

	/*qlCmd_rad(0,0) = atan2f((-As(1,1)*sp-Bs(1,1)*cp-Cs(1,1)),(-As(0,1)*sp-Bs(0,1)*cp-Cs(0,1)));
	qlCmd_rad(1,0) = acosf(-As(2,1)*sp-Bs(2,1)*cp-Cs(2,1));
	qlCmd_rad(2,0) = atan2f(( As(2,2)*sp+Bs(2,2)*cp+Cs(2,2)),(-As(2,0)*sp-Bs(2,0)*cp-Cs(2,0)));*/
	
	/*Aw = R34Cmd.transpose()*As.transpose()*R07Cmd;
	Bw = R34Cmd.transpose()*Bs.transpose()*R07Cmd;
	Cw = R34Cmd.transpose()*Cs.transpose()*R07Cmd;
	
	R47Cmd = sp*Aw + cp*Bw + Cw;*/
	if( isCMD == true)
	{
		R47Cmd = R34Cmd.transpose()*R03Cmd.transpose()*R07Cmd;
		P0eCmd = R03Cmd*L3se;  	            // position of elbow 
		P0pCmd = P0eCmd / 2;  	            // position of posterior limb (bwtween elbow and shoulder) while Ps = [0;0;0]
		P0aCmd = (P0eCmd + P0wCmd)/2;  	            // position of anterior limb (bwtween elbow and wrist)
	}
	else
	{
		R47Cmd = R34.transpose()*R03.transpose()*R07Cmd;
	}

	rs_r =  R47Cmd(2,1);
	rc_r = -R47Cmd(2,0);
	rc_p =  R47Cmd(2,2);
	rs_y =  R47Cmd(1,2);
	rc_y =  R47Cmd(0,2);

	qr1 = atan2f(rs_r, rc_r);
	qp1 = acosf(rc_p);
	qy1 = atan2f(rs_y, rc_y);
		
	/*qr2 = atan2f(-rs_r, -rc_r);
	qp2 = -qp1;
	qy2 = atan2f(-rs_y, -rc_y);*/

// special filter for 7th dof
	/*if (qr1 - qr1_temp > 2*pi-0.0001f ){
		qr1 = qr1 - 2*pi;
	}
	else if(qr1 - qr1_temp < -2*pi+0.0001f){
		qr1 = qr1 + 2*pi;
	}*/
// end of special filter
	qlCmd_rad(6,0) = qr1;
	qlCmd_rad(5,0) = qp1;
	qlCmd_rad(4,0) = qy1;

	qr1_temp = qr1;
	/*if( qr1<0.0f )
	{
		if( abs( qr1 + rev2rad - qlCmd_rad_1(6,0) ) < abs( qr1 - qlCmd_rad_1(6,0) ) )
		{
			qlCmd_rad(6,0) = qr1 + rev2rad;
		}
		else
		{
			qlCmd_rad(6,0) = qr1;
		}
	}
	else if( qr1>0.0f )
	{
		if( abs( qr1 - rev2rad - qlCmd_rad_1(6,0) ) < abs( qr1 - qlCmd_rad_1(6,0) ) )
		{
			qlCmd_rad(6,0) = qr1 - rev2rad;
		}
		else
		{
			qlCmd_rad(6,0) = qr1;
		}
	}*/

	//if( abs(qlCmd_rad(6,0) - qlCmd_rad_1(6,0)) > pi )
	if( abs(qlCmd_rad(6,0) - qlCmd_rad_1(6,0)) > 2.0f )
	{
		if( qlCmd_rad(6,0) < 0 )
		{
			qlCmd_rad(6,0) = qlCmd_rad(6,0) + rev2rad;
		}
		else if( qlCmd_rad(6,0) > 0 )
		{
			qlCmd_rad(6,0) = qlCmd_rad(6,0) - rev2rad;
		}
	}


	/*if( abs(qr1-qlCmd_rad_1(6,0)) + abs(qy1-qlCmd_rad_1(4,0)) < abs(qr2-qlCmd_rad_1(6,0)) + abs(qy2-qlCmd_rad_1(4,0)) )
	{
		qlCmd_rad(6,0) = qr1;
		qlCmd_rad(5,0) = qp1;
		qlCmd_rad(4,0) = qy1;
	}
	else if( abs(qr1-qlCmd_rad_1(6,0)) + abs(qy1-qlCmd_rad_1(4,0)) > abs(qr2-qlCmd_rad_1(6,0)) + abs(qy2-qlCmd_rad_1(4,0)) )
	{
		qlCmd_rad(6,0) = qr2;
		qlCmd_rad(5,0) = qp2;
		qlCmd_rad(4,0) = qy2;
	}
	else
	{
		if( abs(qp1 - qlCmd_rad_1(5,0)) <= abs(qp2 - qlCmd_rad_1(5,0)) )
		{
			qlCmd_rad(6,0) = qr1;
			qlCmd_rad(5,0) = qp1;
			qlCmd_rad(4,0) = qy1;
		}
		else
		{
			qlCmd_rad(6,0) = qr2;
			qlCmd_rad(5,0) = qp2;
			qlCmd_rad(4,0) = qy2;
		}
	}*/

	/*if( abs(1.0f - abs(rc_p)) < 0.000001f)
	{
		qlCmd_rad(4,0) = qlCmd_rad_1(4,0);
		qlCmd_rad(5,0) = 0.0f;
		qlCmd_rad(6,0) = qlCmd_rad_1(6,0);
	}
	else
	{
		q_temp = atan2f( rs_r, rc_r);
		if( q_temp - qlCmd_rad_1(6,0) > 3.14f )
			qlCmd_rad(6,0) = q_temp - pi;
		else if( q_temp - qlCmd_rad_1(6,0) < -3.14f )
			qlCmd_rad(6,0) = q_temp + pi;
		else
			qlCmd_rad(6,0) = q_temp;

		q_temp = atan2f( rs_y, rc_y);
		if( q_temp - qlCmd_rad_1(4,0) > 3.14f )
			qlCmd_rad(4,0) = q_temp - pi;
		else if( q_temp - qlCmd_rad_1(4,0) < -3.14f )
			qlCmd_rad(4,0) = q_temp + pi;
		else
			qlCmd_rad(4,0) = q_temp;
		
		if( abs(rc_r) > abs(rs_r) )
		{
			rs_p = 	rc_r / cosf(qlCmd_rad(6,0)); 
		}
		else
		{
			rs_p = 	rs_r / sinf(qlCmd_rad(6,0)); 
		}
		
		qlCmd_rad(5,0) = atan2( rs_p, rc_p);
	}*/

	/*qlCmd_rad(4,0) = atan2f((Aw(1,2)*sp+Bw(1,2)*cp+Cw(1,2)),( Aw(0,2)*sp+Bw(0,2)*cp+Cw(0,2)));
	qlCmd_rad(5,0) = acosf(Aw(2,2)*sp-Bw(2,2)*cp-Cw(2,2));
	qlCmd_rad(6,0) = atan2f((Aw(2,1)*sp+Bw(2,1)*cp+Cw(2,1)),(-Aw(2,0)*sp-Bw(2,0)*cp-Cw(2,0)));*/

}

inline void NullSpaceKineCompute_1( bool isCMD)
{
	if( isCMD == true)
	{
		Psi_psi0 = PsiCmd;

		ql_psi0 = qlCmd_rad;

		ql_psi1(3,0) = ql_psi0(3,0);

		q1o_psi0 = q1oCmd;
		q2o_psi0 = q2oCmd;

		R03_o_psi0 = R03_oCmd;

		R34_psi0 = R34Cmd;
		R07_psi0 = R07Cmd;

		Su = skew(u0wCmd);
	}
	else
	{
		Psi_psi0 = Psi;

		ql_psi0 = ql_rad;

		ql_psi1(3,0) = ql_psi0(3,0);

		q1o_psi0 = q1o;
		q2o_psi0 = q2o;

		R03_o_psi0 = R03_o;

		R34_psi0 = R34;
		R07_psi0 = R07;

		Su = skew(u0w);
	}
}

inline void NullSpaceKineCompute_2( bool isCMD)
{
	float sp1, cp1, rs_r, rc_r, rs_p, rc_p, rs_y, rc_y, q_temp;

	if ( abs(Psi_psi1) < 0.000001f) //
	{
		R03_psi1 = R03_o_psi0;
		
		ql_psi1(0,0) = q1o_psi0;
		ql_psi1(1,0) = q2o_psi0;
		ql_psi1(2,0) = 0.0f;
	}
	else
	{
		As = Su*R03_o_psi0;
		Bs = -Su*As;
		if( isCMD == true)
			Cs = u0wCmd*u0wCmd.transpose()*R03_o_psi0;
		else
			Cs = u0w*u0w.transpose()*R03_o_psi0;

		sp1 =sinf(Psi_psi1);
		cp1 =cosf(Psi_psi1);

		R03_psi1 = sp1*As + cp1*Bs + Cs;

		rs_r = -R03_psi1(1,1);
		rc_r = -R03_psi1(0,1);
		rc_p = -R03_psi1(2,1);
		rs_y =  R03_psi1(2,2);
		rc_y = -R03_psi1(2,0);

		if( abs(1.0f - abs(rc_p)) < 0.000001f) //
		{
			ql_psi1(0,0) = ql_psi0(0,0);
			ql_psi1(1,0) = 0.0f;
			ql_psi1(2,0) = ql_psi0(2,0);
		}
		else
		{
			q_temp = atan2f( rs_r, rc_r);
			if( q_temp - ql_psi0(0,0) > 3.14f )
				ql_psi1(0,0) = q_temp - pi;
			else if( q_temp - ql_psi0(0,0) < -3.14f )
				ql_psi1(0,0) = q_temp + pi;
			else
				ql_psi1(0,0) = q_temp;

			q_temp = atan2f( rs_y, rc_y);
			if( q_temp - ql_psi0(2,0) > 3.14f )
				ql_psi1(2,0) = q_temp - pi;
			else if( q_temp - ql_psi0(2,0) < -3.14f )
				ql_psi1(2,0) = q_temp + pi;
			else
				ql_psi1(2,0) = q_temp;

			if( abs(rc_r) > abs(rs_r) )
			{
				rs_p = 	rc_r / cosf(ql_psi1(0,0)); 
			}
			else
			{
				rs_p = 	rs_r / sinf(ql_psi1(0,0)); 
			}
		
			ql_psi1(1,0) = atan2( rs_p, rc_p);
		}
	}

	/*ql_psi1(0,0) = atan2f((-As(1,1)*sp1-Bs(1,1)*cp1-Cs(1,1)),(-As(0,1)*sp1-Bs(0,1)*cp1-Cs(0,1)));
	ql_psi1(1,0) = acosf(-As(2,1)*sp1-Bs(2,1)*cp1-Cs(2,1));
	ql_psi1(2,0) = atan2f(( As(2,2)*sp1+Bs(2,2)*cp1+Cs(2,2)),(-As(2,0)*sp1-Bs(2,0)*cp1-Cs(2,0)));*/
	
	/*Aw = R34.transpose()*As.transpose()*R07;
	Bw = R34.transpose()*Bs.transpose()*R07;
	Cw = R34.transpose()*Cs.transpose()*R07;*/

	R47_psi1 = R34_psi0.transpose()*R03_psi1.transpose()*R07_psi0;

	rs_r =  R47_psi1(2,1);
	rc_r = -R47_psi1(2,0);
	rc_p =  R47_psi1(2,2);
	rs_y =  R47_psi1(1,2);
	rc_y =  R47_psi1(0,2);

	if( abs(1.0f - abs(rc_p)) < 0.000001f ) //
	{
		ql_psi1(4,0) = ql_psi0(4,0);
		ql_psi1(5,0) = 0.0f;
		ql_psi1(6,0) = ql_psi0(6,0);
	}
	else
	{
		q_temp = atan2f( rs_r, rc_r);
		if( q_temp - ql_psi0(6,0) > 3.14f )
			ql_psi1(6,0) = q_temp - pi;
		else if( q_temp - ql_psi0(6,0) < -3.14f )
			ql_psi1(6,0) = q_temp + pi;
		else
			ql_psi1(6,0) = q_temp;

		q_temp = atan2f( rs_y, rc_y);
			if( q_temp - ql_psi0(4,0) > 3.14f )
				ql_psi1(4,0) = q_temp - pi;
			else if( q_temp - ql_psi0(4,0) < -3.14f )
				ql_psi1(4,0) = q_temp + pi;
			else
				ql_psi1(4,0) = q_temp;

		if( abs(rc_r) > abs(rs_r) )
		{
			rs_p = 	rc_r / cosf(ql_psi1(6,0)); 
		}
		else
		{
			rs_p = 	rs_r / sinf(ql_psi1(6,0)); 
		}
		
		ql_psi1(5,0) = atan2( rs_p, rc_p);
	}

	/*ql_psi1(4,0) = atan2f((Aw(1,2)*sp1+Bw(1,2)*cp1+Cw(1,2)),( Aw(0,2)*sp1+Bw(0,2)*cp1+Cw(0,2)));
	ql_psi1(5,0) = acosf(Aw(2,2)*sp1-Bw(2,2)*cp1-Cw(2,2));
	ql_psi1(6,0) = atan2f((Aw(2,1)*sp1+Bw(2,1)*cp1+Cw(2,1)),(-Aw(2,0)*sp1-Bw(2,0)*cp1-Cw(2,0)));*/

	delta_ql_null_rad = ql_psi1 - ql_psi0;
}

inline void JacobUpdate()
{
	bool b_tmp = false;
	float s4_tmp, c4_tmp;
	
	if( ql_rad(3,0) < q4_U_snglr)
	{
		s4_tmp = s4;
		s4 = sinf(q4_U_snglr);
		c4_tmp = c4;
		c4 = cosf(q4_U_snglr);
		b_tmp = true;
	}
	else if( ql_rad(3,0) > q4_L_snglr)
	{
		s4_tmp = s4;
		s4 = sinf(q4_L_snglr);
		c4_tmp = c4;
		c4 = cosf(q4_L_snglr);
		b_tmp = true;
	}

	JV0wt(0,0) = a*c4*(c1*s3 + c2*c3*s1) - a*c1*s3 - d1*s1*s2 - d2*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) - a*c2*c3*s1 - a*s1*s2*s4;
	JV0wt(0,1) = d1*c1*s2 - d2*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - a*s1*s3 + a*c4*(s1*s3 - c1*c2*c3) + a*c1*c2*c3 + a*c1*s2*s4;
	JV0wt(0,2) = 0.0f;
	JV0wt(1,0) = d2*(c1*c2*c4 - c1*c3*s2*s4) + d1*c1*c2 - a*c1*c3*s2 + a*c1*c2*s4 + a*c1*c3*c4*s2;
	JV0wt(1,1) = d2*(c2*c4*s1 - c3*s1*s2*s4) + d1*c2*s1 - a*c3*s1*s2 + a*c2*s1*s4 + a*c3*c4*s1*s2;
	JV0wt(1,2) = a*c2*c3*c4 - d1*s2 - a*c2*c3 - a*s2*s4 - d2*(c4*s2 + c2*c3*s4);
	JV0wt(2,0) = a*c4*(c3*s1 + c1*c2*s3) - a*c3*s1 - d2*s4*(c3*s1 + c1*c2*s3) - a*c1*c2*s3;
	JV0wt(2,1) = d2*s4*(c1*c3 - c2*s1*s3) + a*c1*c3 - a*c4*(c1*c3 - c2*s1*s3) - a*c2*s1*s3;
	JV0wt(2,2) = a*s2*s3 - a*c4*s2*s3 + d2*s2*s3*s4;
	JV0wt(3,0) = a*c1*c4*s2 - a*s4*(s1*s3 - c1*c2*c3) - d2*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4);
	JV0wt(3,1) = d2*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + a*s4*(c1*s3 + c2*c3*s1) + a*c4*s1*s2;
	JV0wt(3,2) = a*c2*c4 - d2*(c2*s4 + c3*c4*s2) - a*c3*s2*s4;
	
	if(b_tmp == true)
	{
		s4 = s4_tmp;
		c4 = c4_tmp;
	}
//[ a*cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - a*cos(q1)*sin(q3) - d1*sin(q1)*sin(q2) - d2*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) - a*cos(q2)*cos(q3)*sin(q1) - a*sin(q1)*sin(q2)*sin(q4), d2*(cos(q1)*cos(q2)*cos(q4) - cos(q1)*cos(q3)*sin(q2)*sin(q4)) + d1*cos(q1)*cos(q2) - a*cos(q1)*cos(q3)*sin(q2) + a*cos(q1)*cos(q2)*sin(q4) + a*cos(q1)*cos(q3)*cos(q4)*sin(q2), a*cos(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)) - a*cos(q3)*sin(q1) - d2*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)) - a*cos(q1)*cos(q2)*sin(q3), a*cos(q1)*cos(q4)*sin(q2) - a*sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - d2*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4))]
//[ d1*cos(q1)*sin(q2) - d2*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - a*sin(q1)*sin(q3) + a*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + a*cos(q1)*cos(q2)*cos(q3) + a*cos(q1)*sin(q2)*sin(q4), d2*(cos(q2)*cos(q4)*sin(q1) - cos(q3)*sin(q1)*sin(q2)*sin(q4)) + d1*cos(q2)*sin(q1) - a*cos(q3)*sin(q1)*sin(q2) + a*cos(q2)*sin(q1)*sin(q4) + a*cos(q3)*cos(q4)*sin(q1)*sin(q2), d2*sin(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)) + a*cos(q1)*cos(q3) - a*cos(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)) - a*cos(q2)*sin(q1)*sin(q3), d2*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + a*sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + a*cos(q4)*sin(q1)*sin(q2)]
//[                                                                                                                                                                                                                                           0,                                                 a*cos(q2)*cos(q3)*cos(q4) - d1*sin(q2) - a*cos(q2)*cos(q3) - a*sin(q2)*sin(q4) - d2*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)),                                                                                     a*sin(q2)*sin(q3) - a*cos(q4)*sin(q2)*sin(q3) + d2*sin(q2)*sin(q3)*sin(q4),                                                                         a*cos(q2)*cos(q4) - d2*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - a*cos(q3)*sin(q2)*sin(q4)]
//[ a*cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - a*cos(q1)*sin(q3) - d1*sin(q1)*sin(q2) - d2*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) - a*cos(q2)*cos(q3)*sin(q1) - a*sin(q1)*sin(q2)*sin(q4), d2*(cos(q1)*cos(q2)*cos(q4) - cos(q1)*cos(q3)*sin(q2)*sin(q4)) + d1*cos(q1)*cos(q2) - a*cos(q1)*cos(q3)*sin(q2) + a*cos(q1)*cos(q2)*sin(q4) + a*cos(q1)*cos(q3)*cos(q4)*sin(q2), a*cos(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)) - a*cos(q3)*sin(q1) - d2*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)) - a*cos(q1)*cos(q2)*sin(q3), a*cos(q1)*cos(q4)*sin(q2) - a*sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - d2*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)), 0, 0, 0]
//[ d1*cos(q1)*sin(q2) - d2*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - a*sin(q1)*sin(q3) + a*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + a*cos(q1)*cos(q2)*cos(q3) + a*cos(q1)*sin(q2)*sin(q4), d2*(cos(q2)*cos(q4)*sin(q1) - cos(q3)*sin(q1)*sin(q2)*sin(q4)) + d1*cos(q2)*sin(q1) - a*cos(q3)*sin(q1)*sin(q2) + a*cos(q2)*sin(q1)*sin(q4) + a*cos(q3)*cos(q4)*sin(q1)*sin(q2), d2*sin(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)) + a*cos(q1)*cos(q3) - a*cos(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)) - a*cos(q2)*sin(q1)*sin(q3), d2*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + a*sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + a*cos(q4)*sin(q1)*sin(q2), 0, 0, 0]
//[                                                                                                                                                                                                                                           0,                                                 a*cos(q2)*cos(q3)*cos(q4) - d1*sin(q2) - a*cos(q2)*cos(q3) - a*sin(q2)*sin(q4) - d2*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)),                                                                                     a*sin(q2)*sin(q3) - a*cos(q4)*sin(q2)*sin(q3) + d2*sin(q2)*sin(q3)*sin(q4),                                                                         a*cos(q2)*cos(q4) - d2*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - a*cos(q3)*sin(q2)*sin(q4), 0, 0, 0]
 
	JW4wt(0,0) =  0.0f;
	JW4wt(0,1) =  0.0f;
	JW4wt(0,2) =  1.0f;
	JW4wt(1,0) = -s5;
	JW4wt(1,1) =  c5;
	JW4wt(1,2) =  0.0f;
	JW4wt(2,0) =  c5*s6;
	JW4wt(2,1) =  s5*s6;
	JW4wt(2,2) =  c6;
//[ 0, -sin(q5), cos(q5)*sin(q6)]
//[ 0,  cos(q5), sin(q5)*sin(q6)]
//[ 1,        0,         cos(q6)]
}

inline void GravityCompUpdate()
{
	Tq_g[0] = 0.0f;
	Tq_g[1] = K_gravity[1] * (- M2*g0_z*(L_m2*(c4*s2 + c2*c3*s4) + s2*d1 + c2*c3*a + s2*s4*a - c2*c3*c4*a) - M1b*g0_z*(s2*d1 + c2*c3*a) - M1a*g0_z*s2*L_m1a );
	Tq_g[2] = K_gravity[2] * (M2*g0_z*(s2*s3*a + s2*s3*s4*L_m2 - c4*s2*s3*a) + M1b*g0_z*s2*s3*a );
	Tq_g[3] = K_gravity[3] * (- M2*g0_z*(L_m2*(c2*s4 + c3*c4*s2) - c2*c4*a + c3*s2*s4*a) );
	Tq_g[4] = K_gravity[4] * (M3*g0_z*s6*L_m3*(s5*(c2*s4 + c3*c4*s2) + c5*s2*s3) );
	Tq_g[5] = K_gravity[5] * (- M3*g0_z*L_m3*(c6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) + s6*(c2*c4 - c3*s2*s4)) );
	Tq_g[6] = 0.0f;
}

inline void FrictionComp()
{
	/*float V_bound[USAGE_CHANNELS] = {1.0f, 1.7f, 4.0f, 4.0f, 4.0f, 3.8f ,4.0f};
	float gain_GC[USAGE_CHANNELS] = {1.0f, 1.2f, 1.4f, 1.5f, 1.2f, 1.2f, 1.0f};
	float gain_FT_p[USAGE_CHANNELS] = {1.0f, 1.1f, 0.1f, 0.8f, 0.9f, 0.9f, 0.6f};
	float gain_FT_n[USAGE_CHANNELS] = {1.0f, 0.5f, 0.1f, 0.1f, 0.9f, 0.6f, 0.6f};
	float gain_DP[USAGE_CHANNELS] = {2.0f, 1.6f, 4.0f, 3.2f, 1.2f, 1.6f, 1.2f};*/

	for( int i = 0; i < USAGE_CHANNELS; i++)
	{
		if( dqm_ENC[i] > V_bound_fric[i] )
		{
			if( Tq_g[i] > 0 )
				Tq_fric[i] = gain_FT_n_fric[i]*TorqueFriction[i] - gain_DP_fric[i]*K_damper[i]*(qm_ENC[i] - qm_ENC_1[i]);
			else
				Tq_fric[i] = gain_FT_p_fric[i]*TorqueFriction[i] - gain_DP_fric[i]*K_damper[i]*(qm_ENC[i] - qm_ENC_1[i]);
		}
		else if( dqm_ENC[i] < -V_bound_fric[i] )
		{
			if( Tq_g[i] < 0 )
				Tq_fric[i] = - gain_FT_p_fric[i]*TorqueFriction[i] - gain_DP_fric[i]*K_damper[i]*(qm_ENC[i] - qm_ENC_1[i]);
			else
				Tq_fric[i] = - gain_FT_n_fric[i]*TorqueFriction[i] - gain_DP_fric[i]*K_damper[i]*(qm_ENC[i] - qm_ENC_1[i]);
		}
		else
		{
			Tq_fric[i] = - gain_DP_fric[i]*K_damper[i]*(qm_ENC[i] - qm_ENC_1[i]);
		}
	}
}

void ServoLoop()
{
	//sevroloop start !
	IMC_DAC_SetOutputVoltage(7, 5);

	TestComputeLose++;

	t_count_B++;
	t_count_C++;
	t_count_D++;

	// sampling
	ReadEncoder();
	//for( int i = 0; i < USAGE_CHANNELS; i++)
	//	IMC_ENC_ReadCounter(i, qm_ENC+i); //&qm_ENC[i]
	
	//read force Z
	IMC_ENC_ReadCounter( 7, &ENC7 );

	IMC_DAC_SetOutputVoltage(7, 4);

	for( int i = 0; i < USAGE_CHANNELS; i++)
	{
		dqm_ENC[i] = 1.625f*qm_ENC[i] - 2.375f*qm_ENC_1[i] + 0.001f*(875*qm_ENC_2[i] - 125*qm_ENC_3[i]);

		ddqm_ENC[i] = dqm_ENC[i] - dqm_ENC_1[i];
		
		//record last data (qm_ENC)
		qm_ENC_3[i]=qm_ENC_2[i]; qm_ENC_2[i]=qm_ENC_1[i]; qm_ENC_1[i]=qm_ENC[i];
		
		dqm_ENC_1[i] = dqm_ENC[i];
	}
	for( int i = 0; i < USAGE_CHANNELS; i++)
	{
		ql_rad(i,0) = ENC2rad[i]*qm_ENC[i] + q_home(i,0);
		dql_rad(i,0) = ENC2rad[i]*dqm_ENC[i]*SamplingTime_C;
		
		/*cosq[i] = cosf(ql_rad(i,0));
		sinq[i] = sinf(ql_rad(i,0));*/
	}

	// z dir force control **********************************************************************//
	/*deltaZ_FS = 0.019 - 0.06*sin( 0.3066763 - rev2rad*ENC7/267168 );
	d3 = d3_0 - deltaZ_FS;*/
	// z dir force control **********************************************************************//

	// Control law
	if( modeControl_G == true )
	{
		FrictionComp();
		for( int i = 0; i < USAGE_CHANNELS; i++)
			TqmCmd[i] = Tq_g[i] + Tq_fric[i];
	}
	else if( modeControl_P == true ) // PositionMode
	{
		if( ModeArm_C == true)
		{
			CartesianControl();

			// z dir force control by force limit************************************************//
			if( FORCE_LIMIT == true )
			{
				F0wCmd << 0.0f, 0.0f, 0.8f*Fz_task; // adjust gain
				Tq_imped.block(0,0,4,1) = JV0wt*F0wCmd;

				float Tq_limit[4];
				Tq_limit[0] = Tq_imped(0,0)/MT0_GEAR_RATIO + Tq_g[0];
				Tq_limit[1] = Tq_imped(1,0)/MT1_GEAR_RATIO + Tq_g[1];
				Tq_limit[2] = Tq_imped(2,0)/MT2_GEAR_RATIO + Tq_g[2];
				Tq_limit[3] = Tq_imped(3,0)/MT3_GEAR_RATIO + Tq_g[3];

				for( int i = 0; i < 4; i++)
				{
					if( Tq_limit[i] >= 0.0f)
					{
						if( TqmCmd[i] > Tq_limit[i])
							TqmCmd[i] = Tq_limit[i];
					}
					else 
					{
						if( TqmCmd[i] < Tq_limit[i])
							TqmCmd[i] = Tq_limit[i];
					}
				}
			}
			//Endof: z dir force control by force limit******************************************//
		}
		else
		{
			for( int i = 0; i < USAGE_CHANNELS; i++)
				MotorPosPIDVelPIControl( i );
		}
	}
	else if( modeControl_K  == true) // ImpedenceMode
	{
		for( int i = 0; i < ROBOT_DOF; i++)
		{
			cosq[i] = cosf(ql_rad(i,0));
			sinq[i] = sinf(ql_rad(i,0));
		}
		CosSinUpdate( 0 );
		FwdKineUpdate_1(0);
		V07 = (P07 - P07_1)*SamplingTime_C;
		V0w = (P0w - P0w_1)*SamplingTime_C;
		W07 = (rr2delta(R07,R07_1))*SamplingTime_C;

		//impedence law
		ImpedenceTorque();
		//test!
		/*for( int i = 0; i < USAGE_CHANNELS; i++)
			TqmCmd[i] =  Tq_g[i] + Tq_fric[i];*/
	}
	else // zero torque
	{
		for( int i = 0; i < USAGE_CHANNELS; i++)
			TqmCmd[i] = 0.0f;
	}

	// conpute setvalue (SetValue)
	for( int i = 0; i < USAGE_CHANNELS; i++)
		TqCmd2SetValue( i );
	
	IMC_DAC_SetOutputVoltage(7, 3);

	// DA output
	SetOutputDA();
	//for( int i = 0; i < USAGE_CHANNELS; i++)
	//	IMC_DAC_SetOutputVoltage(i, SetValue[i]);

	IMC_DAC_SetOutputVoltage(7, 2);

	// record data ******************************************************************************//
	if( q_DATA_RECORD == true)
	{
		for( int i = 0; i < USAGE_CHANNELS; i++)
		{
			qENC_data[data_count][i] = qm_ENC[i];
			qCmd_data[data_count][i] = qmCmd_ENC[i];
		}
	}
	if( Tq_DATA_RECORD == true)
	{
		for( int i = 0; i < USAGE_CHANNELS; i++)
		{
			TqCmd_data[data_count][i] = TqmCmd[i];
			//SetV_data[data_count][i] = SetValue[i];  //process of SetValue[] must before SetOutputDA()//
		}
	}
	//ENDof: record data ************************************************************************//

	// Sampling_C and Mode Change Setting *******************************************************//
	if( t_count_C == 1 )
	{
		ModeArm_G    = false; ModeArm_J    = false; ModeArm_C    = false; ModeArm_K = false;

		Free_psi     = false; Free_P       = false; Free_O       = false;

		modeControl_G= false; modeControl_P= false; modeControl_K= false;

		modeMotion_J = false; modeMotion_C = false;

		/*if( qlCmd_rad(3,0) < q4_snglr)
		{
			ModeArm = ModeArm_1;
		}*/

		if( ModeArm == 0)
		{
			ModeArm_G = true;
			modeControl_G = true;
		}
		else if( ModeArm == 1)
		{
			ModeArm_J = true;
			modeControl_P = true;
			modeMotion_J = true;
		}
		else if( ModeArm == 2)
		{
			ModeArm_C = true;
			if( ModeArm_C_1 == false)
			{
				SubMode = 0;
			}
			modeControl_P = true;
			modeMotion_C = true;
		}
		else if( ModeArm == 3 )
		{
			
			ModeArm_K = true;
			if( ModeArm_K_1 == false)
			{
				SubMode = 0;
			}
			modeControl_K = true;
			modeMotion_C = true;
		}
		else
		{
			ModeArm_Z = true;
		}

		if( SubMode == 1)
		{	
			Free_P = true; Free_psi = true;
		}
		else if( SubMode == 2)
			Free_P = true;
		else if( SubMode == 3)
			Free_psi = true;
		else if( SubMode == 4)
			Free_O = true;
		else if( SubMode == 5)
		{
			Free_O = true; Free_psi = true;
		}
		else if( SubMode == 6)
		{
			Free_psi = true; Free_P = true; Free_O = true;
		}
		else
			;
		
		if( Free_psi_1 == true)
		{
			PsiTrgt = PsiCmd;
		}
		if( Free_P_1 == true)
		{
			P07Trgt = P07Cmd;
		}
		if( Free_O_1 == true)
		{
			R07Trgt = R07Cmd;
		}

		/*if( modeControl_G == true && modeControl_G_1 == false)
		{
		}*/
		if( modeControl_P == true )
		{
			if( modeControl_P_1 == false )
			{
				for( int i = 0; i < USAGE_CHANNELS; i++)
				{
					qmCmd_ENC[i] = qm_ENC[i];
					dqmCmd_ENC[i] = dqm_ENC[i];
				
					summ_error_qm_ENC[i] = 0;
					summ_error_qm_ENC_1[i] = 0;
					summ_error_dqm_ENC[i] = 0.0f;
					summ_error_dqm_ENC_1[i] = 0.0f;
				}
				PsiCmd = Psi;
				PsiCmd_1= Psi_1;
				dPsiCmd = dPsi;
				dPsiCmd_1 = dPsi;
				P07Cmd = P07;
				P07Cmd_1 = P07_1;
				V07Cmd = V07;
				V07Cmd_1 = V07;
				R07Cmd = R07;
				R07Cmd_1 = R07_1;
				W07Cmd = V07;
				W07Cmd_1 = V07;
			}
		}
		if( modeControl_K == true )
		{
			if( modeControl_K_1 == false )
			{
				for( int i = 0; i < ROBOT_DOF; i++)
				{
					Tq_null[i] = 0.0f;
				}
				PsiCmd = Psi;
				PsiCmd_1= Psi_1;
				dPsiCmd = dPsi;
				dPsiCmd_1 = dPsi;
				P07Cmd = P07;
				P07Cmd_1 = P07_1;
				V07Cmd = V07;
				V07Cmd_1 = V07;
				R07Cmd = R07;
				R07Cmd_1 = R07_1;
				W07Cmd = V07;
				W07Cmd_1 = V07;
			}
		}

		if( modeMotion_J == true )
		{
			if( modeMotion_J_1 == false )
			{
				for( int i = 0; i < USAGE_CHANNELS; i++)
					qmTrgt_ENC[i] = qmCmd_ENC[i];
			}
		}
		if( modeMotion_C == true )
		{
			if( modeMotion_C_1 == false )
			{
				//qlCmd_next_rad = qlCmd_rad;
				PsiTrgt = PsiCmd;
				P07Trgt = P07Cmd;
				R07Trgt = R07Cmd;
				//T07Trgt = T07Cmd;
			}
		}

		if( ModeArm_C == true && Free_P == true && Free_O == false && Free_psi == false)
		{
			ModeArm_C_nullTq = true;
		}
		else
		{
			ModeArm_C_nullTq = false;
			if( ModeArm_C_nullTq_1 == true)
			{
				PsiCmd = Psi;
				PsiTrgt = PsiCmd;
				R07Cmd = R07;
				P07Cmd = P07;
				for( int i = 0; i < USAGE_CHANNELS; i++)
				{
					qmCmd_ENC[i] = qm_ENC[i];
					dqmCmd_ENC[i] = dqm_ENC[i];
				
					summ_error_qm_ENC[i] = 0;
					summ_error_qm_ENC_1[i] = 0;
					summ_error_dqm_ENC[i] = 0.0f;
					summ_error_dqm_ENC_1[i] = 0.0f;
				}
			}
		}
		ModeArm_C_nullTq_1 = ModeArm_C_nullTq;

		if( AutoSelfMotion_Enable == true)
			AutoSelfMotion = true;
		else
			AutoSelfMotion = false;
	}
	//ENDof: Sampling_C and Mode Change Setting *************************************************//

	// z dir force estimate**********************************************************************//
	if( ENC7 > ENC7_1 && ENC7_1 <= ENC7_2 )
	{
		ENC7_BL_flag = false;
		dir_ENC7_BL = 1;
	}
	else if( ENC7 < ENC7_1 && ENC7_1 >= ENC7_2 )
	{
		ENC7_BL_flag = false;
		dir_ENC7_BL = -1;
	}
	else if( ENC7_1 == ENC7_2 && ENC7 == ENC7_1)
	{
		dir_ENC7_BL = 0;
		if( ENC7_BL_flag == false)
		{
			z_ENC7_BL = P07(2,0);
			ENC7_BL_flag = true;
		}
	}
	else
	{
		//dir_ENC7_BL = 0;
		ENC7_BL_flag = false;
	}
	ENC7_2 = ENC7_1; ENC7_1 = ENC7;
	// z dir force estimate**********************************************************************//

	// Motion Generate **************************************************************************//
	if( modeMotion_J == true)
	{
		if( MOVJ == true)
		{
			if( MOVJ_flag == true)
			{
				for( int i = 0; i < USAGE_CHANNELS; i++)
				{
					qmTrgt_ENC[i]  = qmTarget_ENC[i]; 
					MOV[i] = true;
				}
				MOVJ_flag = false;
			}

			if( STOP == true )
			{
				for( int i = 0; i < USAGE_CHANNELS; i++)
				{
					qmTrgt_ENC[i] = qmCmd_ENC[i] + roundf( (float)( dqmCmd_ENC[i]*abs(dqmCmd_ENC[i]) ) /(float)(2.0f*Dec_Limit_ENC[i]) );
				}
				STOP = false;
			}

			if( MOV[0] == false && MOV[1] == false && MOV[2] == false && MOV[3] == false && MOV[4] == false && MOV[5] == false && MOV[6] == false )
				MOVJ = false;

		}
		else
		{
			for( int i = 0; i < USAGE_CHANNELS; i++)
				MOV[i] = false;
		}

		for( int i = 0; i < USAGE_CHANNELS; i++)
		{
			MotorPosRampGenerator( i );
		}

		if( t_count_C == 10)
		{
			for( int i = 0; i < USAGE_CHANNELS; i++)
				qlCmd_rad(i,0) = ENC2rad[i]*qmCmd_ENC[i] + q_home(i,0);
				
			dqlCmd_rad = qlCmd_rad - qlCmd_rad_1;
		}
	}
	else if( modeMotion_C == true)
	{
		if( t_count_C == 1)
		{
			if( MOVL == true)
			{
				if( MOVL_flag == true)
				{
					R07Trgt = T07Target.block(0,0,3,3);
					P07Trgt = T07Target.block(0,3,3,1);
					
					if( AutoSelfMotion != true )
						PsiTrgt = PsiTarget;

					MOVJN = true;
					MOVL_L = true;
					MOVL_R = true;

					MOVL_flag = false;
				}
				if( MOVJN == false && MOVL_L == false && MOVL_R == false )
					MOVL = false;

			}
			else
			{
				MOVJN = false;
				MOVL_L = false;
				MOVL_R = false;
				for( int i = 0; i < USAGE_CHANNELS; i++)
					MOV[i] = false;
			}

			if( ModeArm_C == true)
			{
				for( int i = 0; i < USAGE_CHANNELS; i++)
					dqmCmd_ENC[i] = rad2ENC[i]*dqlCmd_rad(i,0)/SamplingTime_C;
			}
		}
		else if( t_count_C == 4)
		{
			if( MOVL == true)
			{
				if(STOP == true)
				{
					PsiTrgt = PsiCmd + dPsiCmd*abs(dPsiCmd)/2.0f/Jn_Dec_limit;
						
					Vector3f bd;
						
					bd = W07Cmd*abs(W07Cmd.norm())/2.0f/Ang_Dec_limit;
					R07Trgt = delta2rr(bd,R07Cmd);
						
					bd = V07Cmd*abs(V07Cmd.norm())/2.0f/Lin_Dec_limit;
					P07Trgt = P07Cmd + bd;

					STOP = false;
				}
			}
			if( Free_psi == true)
			{
				PsiCmd = Psi;
				dPsiCmd = dPsi;

				MOVL = true;
			}
			else
			{
				if( AutoSelfMotion != true )
					ArmAngleRampGenerator();
			}
		}
		else if( t_count_C == 5)
		{	
			if( Free_O == true)
			{
				R07Cmd = R07;
				W07Cmd = rr2delta(R07Cmd,R07Cmd_1);

				MOVL = true;
			}
			else
			{
				//if( MOVL == true)
				OrientationRampGenerator();
			}

			// z dir force estimate****************************************************************//
			if( dir_ENC7_BL == 1 )
			{
				ENC7_BL = ENC7_BL + 10; //
				if(ENC7_BL > 341) //341
					ENC7_BL = 341;
			}
			else if( dir_ENC7_BL == -1 )
			{
				ENC7_BL = ENC7_BL - 10; //
				if( ENC7_BL < -341)
					ENC7_BL = -341;
			}
			if( FORCE_TRACKING == true && ENC7_BL_flag == true)
			{
				//ENC7_e = (float)(ENC7 + ENC7_BL);
				ENC7_e = (float)(ENC7 + ENC7_BL) + 0.4f*267168.0f*(0.3066763f - atan2f(0.019f - z_ENC7_BL + P07(2,0), 0.06f) );
				//ENC7_e = 1.05f*ENC7_e_1 - 0.1f*ENC7_e_1 + 0.05*ENC7_e_2;
			}
			else
			{
				ENC7_e = (float)(ENC7 + ENC7_BL);
			}
			//ENC7_e = 1.1f*(ENC7 + ENC7_BL) - 0.2f*ENC7_1 + 0.1*ENC7_2;
			ENC7_e_2 = ENC7_e_1; ENC7_e_1 = ENC7_e;
			
			deltaZ_FS = 0.019 - 0.06*tanf( 0.3066763f - rev2rad*ENC7_e/267168.0f );
			//deltaZ_FS = 0.019 - 0.06*tanf( 0.3066763f - rev2rad*((float)ENC7)/267168.0f );

			Fz = -deltaZ_FS*Kz_FS;
			Fz = 0.5f*Fz + 0.5f*Fz_1;
			Fz_1 = Fz;
			//ENDof: z dir force estimate***********************************************************//
			
			if( FORCE_CONTROL == true)
			{
				d3_d = d3_0 - 0.019 + 0.06*tanf( 0.3066763 - rev2rad*((float)ENC7)/267168.0f );
				//d3_d = d3_0 - 0.019 + 0.06*tanf( 0.3066763 - rev2rad*((float)(ENC7+ENC7_BL))/267168.0f );
				//d3_d = d3_0 - deltaZ_FS;
			}
			else if( FORCE_LIMIT == true)
			{
				d3_d = d3_0;
			}
			else
				d3_d = d3_0 - 0.019 + 0.06*tanf( 0.3066763 - rev2rad*((float)ENC7)/267168.0f );

			float err_d3 = 0.0f;
			float d3_Vel_limit = 0.001f; //
			err_d3 = d3_d - d3;
			if( abs(err_d3) < d3_Vel_limit)
				d3 = d3_d;
			else
			{
				if( err_d3 > 0.0f)
					d3 = d3 + d3_Vel_limit;
				else if( err_d3 < 0.0f)
					d3 = d3 - d3_Vel_limit;

				err_d3 = d3_d - d3;
				if( abs(err_d3) < d3_Vel_limit)
				d3 = d3_d;
			}
			L7wt(2,0) = d3;
		}
		else if( t_count_C == 6)
		{
			if( Free_P == true)
			{
				P07Cmd = P07;
				V07Cmd = P07Cmd - P07Cmd_1;
				
				MOVL = true;
			}
			else
			{
				//if( MOVL == true)
				PositionRampGenerator();
			
			}
			P0wCmd = P07Cmd - R07Cmd*L7wt;
				
			P0wCmd_norm = P0wCmd.norm();
			if( P0wCmd_norm > P0w_norm_U_limit || P0wCmd_norm < P0w_norm_L_limit)
				P0wCmd = P0wCmd_1;
			
			V0wCmd = P0wCmd - P0wCmd_1;

			// z dir force control***************************************************************//			
			if( FORCE_CONTROL == true)
			{
				z0wCmd_temp = P0wCmd(2,0);
				
				if( Fz <= Fz_th )// abs(Fz_th) >= abs(Fz_task)
				{
					FORCE_TRACKING = true;
				}

				if( FORCE_TRACKING == true)// abs(Fz_th) > abs(Fz_task)
				{
					if(FC_flag == true)
					{
						z_init = P07(2,0);
						Fz_init = Fz;
						z07Cmd_temp_1 = P07Cmd(2,0);

						FC_flag = false;
					}
					//PositionRampGenerator();

					if( P07Cmd(2,0) - z07Cmd_temp > 0.0f)  // dot(P07Cmd_task - P07Cmd_force, F_task) < 0 
						FORCE_TRACKING = false;
					 
					error_Fz = Fz_task - Fz;
					if( abs(error_Fz) <= 0.1f)
						z07Cmd_temp = z07Cmd_temp_1;
						//P07Cmd(2,0) = P07Cmd_1(2,0);
					else if( abs(error_Fz) > 0.1f && abs(error_Fz) <= 0.4f)
					{
						z07Cmd_temp = z07Cmd_temp_1 + 0.01f*( error_Fz )/Kz_FS;
						//P07Cmd(2,0) = P07Cmd_1(2,0) + 0.001f*( error_Fz )/Kz_FS;
					}
					else if( abs(error_Fz) > 0.4f && abs(error_Fz) <= 0.8f)
					{
						summ_error_Fz = error_Fz_1 + error_Fz;
						z07Cmd_temp = z07Cmd_temp_1 + ( 0.02f*error_Fz + 0.005*summ_error_Fz )/Kz_FS;
						//P07Cmd(2,0) = z07Cmd_temp_1 + ( 0.002f*error_Fz + 0.005*summ_error_Fz )/Kz_FS;
						//P07Cmd(2,0) = P07Cmd_1(2,0) + ( 0.002f*error_Fz + 0.005*summ_error_Fz )/Kz_FS;
					}
					else
					{
						summ_error_Fz = error_Fz_1 + error_Fz;
						z07Cmd_temp = z07Cmd_temp_1 + ( 0.025f*error_Fz + 0.01*summ_error_Fz )/Kz_FS;
						//P07Cmd(2,0) = P07Cmd_1(2,0) + ( 0.0025f*error_Fz + 0.01*summ_error_Fz )/Kz_FS;
						//P07Cmd(2,0) = P07Cmd_1(2,0) + 0.16f*( error_Fz )/Kz_FS;
						//P07Cmd(2,0) = P07(2,0) + 0.01f*( error_Fz )/Kz_FS;
					}
					error_Fz_1 = error_Fz;
					z07Cmd_temp_1 = z07Cmd_temp;

					P0wCmd(2,0) = z07Cmd_temp + d3;
				}
				else
				{
					FC_flag = true;
					//FT_flag = true;
					//FORCE_TRACKING = false;
				}

			}
			else
			{
				FORCE_TRACKING = false;
			}
			//ENDof: z dir force control*********************************************************//
		}

		if( ModeArm_C == true)
		{
			if( MOVL == true || MOVL_1 == true )//|| ModeArm_C_nullTq_1 == true)// && FORCE_TRACKING != true)
			{
				if( t_count_C == 9)
				{
					InvKineCompute_1();
				}
				else if( t_count_C == 10)
				{
					if( Free_P == true || FORCE_LIMIT == true)
					{
						InvKineCompute_2(0);
					}
					else
					{
						InvKineCompute_2(1);
					}
					dqlCmd_rad = qlCmd_rad - qlCmd_rad_1;

					if( FORCE_CONTROL == true && FORCE_TRACKING == true)
					{
						P0wCmd(2,0) = z0wCmd_temp;
					}
				}
			}
			if( MOVL_1 == true )// && FORCE_TRACKING != true)
			{
				for( int i = 0; i < USAGE_CHANNELS; i++)
				{
					//qmCmd_ENC[i] = roundf( rad2ENC[i] *( qlCmd_rad_1(i,0) + ( qlCmd_next_rad(i,0) - qlCmd_rad_1(i,0) )*((float)(t_count_C)) / SamplingTime_C) ) - q_home_ENC[i];
					qmCmd_ENC[i] = roundf( rad2ENC[i] *qlCmd_rad_1(i,0) + ((float)t_count_C)*dqmCmd_ENC[i] - q_home_ENC[i]); 
					for( int i = 0; i < USAGE_CHANNELS; i++)
					{
						if(qmCmd_ENC[i] != qmCmd_ENC_1[i])
							MOV[i] = true;
						else
							MOV[i] = false;
					}
				}
			}
		}
		else if( ModeArm_K == true)
		{
			for( int i = 0; i < USAGE_CHANNELS; i++)
			{
				qmCmd_ENC[i] = qm_ENC[i];
				dqmCmd_ENC[i] = dqm_ENC[i];
			}
			if( t_count_C == 10)
			{
				for( int i = 0; i < USAGE_CHANNELS; i++)
					qlCmd_rad(i,0) = ENC2rad[i]*qmCmd_ENC[i] + q_home(i,0);
				
				dqlCmd_rad = qlCmd_rad - qlCmd_rad_1;
			}
		}

	}
	else
	{
		for( int i = 0; i < USAGE_CHANNELS; i++)
		{
			qmCmd_ENC[i] = qm_ENC[i];
			dqmCmd_ENC[i] = dqm_ENC[i];
		}
		if( t_count_C == 10)
		{
			for( int i = 0; i < USAGE_CHANNELS; i++)
				qlCmd_rad(i,0) = ENC2rad[i]*qmCmd_ENC[i] + q_home(i,0);
				
			dqlCmd_rad = qlCmd_rad - qlCmd_rad_1;
		}
	}
	//ENDof: Motion Generate ********************************************************************//
	
	// record data ******************************************************************************//
	if( Fz_DATA_RECORD == true)
	{
		Fz_data[data_count] = Fz;
	}
	data_count++;
	if( data_count > DataLength)
	{
		data_count = 0;
		q_DATA_RECORD = false;
		Tq_DATA_RECORD = false;
		Fz_DATA_RECORD = false;
	}
	//ENDof: record data ************************************************************************//
	
	// Sampling_B and Null Space Torque *********************************************************//
	if( t_count_B == 1)
	{
		if( modeControl_K == true)
		{
			FwdKineUpdate_2(0);
			dPsi = (Psi - Psi_1)*SamplingTime_C/SamplingTime_B;

			NullSpaceKineCompute_1(0);
		}

		// Null space torque in ModeArm = 2 *****************************************************//
		if( ModeArm_C_nullTq == true)
		{
			for( int i = 0; i < ROBOT_DOF; i++)
			{
				cosq[i] = cosf(ql_rad(i,0));
				sinq[i] = sinf(ql_rad(i,0));
			}
			CosSinUpdate( 0 );
			FwdKineUpdate_1(0);
			V07 = (P07 - P07_1)*SamplingTime_C/SamplingTime_B;
			V0w = (P0w - P0w_1)*SamplingTime_C/SamplingTime_B;
			W07 = (rr2delta(R07,R07_1))*SamplingTime_C/SamplingTime_B;

			FwdKineUpdate_2(0);
			dPsi = (Psi - Psi_1)*SamplingTime_C/SamplingTime_B;

			NullSpaceKineCompute_1(0);
		}
		// Null space torque in ModeArm = 2 *****************************************************//

	}
	else if( t_count_B == 2)
	{
		if( modeControl_K  == true)
		{
			//Null Space impedence law
			
			if( Free_psi == true )
			{
				if( nullAuxTq == true )
				{
					float K_psi;
					K_psi = 0.5f*( 1.5f - 0.5f*cosf( 2.0f*Psi ) );
					
					if( Psi > 0.0002f) //
					{
						if ( dPsi > 0.00002f) //
							Psi_psi1 = Psi_psi0 + 0.040f * K_psi - 10.0f*dPsi;
						else if( dPsi < -0.00002f) //
							Psi_psi1 = Psi_psi0 - 0.025f * K_psi - 10.0f*dPsi;
						else
							Psi_psi1 = Psi_psi0; //
					}
					else if( Psi < -0.0002f) //
					{
						if ( dPsi < -0.00002f) //
							Psi_psi1 = Psi_psi0 - 0.040f * K_psi - 10.0f*dPsi;
						else if( dPsi > 0.00002f) //
							Psi_psi1 = Psi_psi0 + 0.025f * K_psi - 10.0f*dPsi;
						else
							Psi_psi1 = Psi_psi0; //
					}
					else
					{
						if ( dPsi > 0.00002f) //
							Psi_psi1 = Psi_psi0 + 0.01f * K_psi;
						else if( dPsi < -0.00002f) //
							Psi_psi1 = Psi_psi0 - 0.01f * K_psi;
						else
							Psi_psi1 = Psi_psi0; //
					}
					NullSpaceKineCompute_2(0);
					/*for( int i = 0; i < ROBOT_DOF; i++)
						Tq_null[i] = Kp_imped_dq_null[i]*delta_ql_null_rad(i,0);*/
						//Tq_null[i] = K_aux_dq_null[i]*delta_ql_null_rad(i,0);
					if( Free_P == false && Free_O == false )
					{
						for( int i = 0; i < 4; i++)
							Tq_null[i] = Kp_imped_dq_null[i]*delta_ql_null_rad(i,0); //
						for( int i = 4; i < ROBOT_DOF; i++)
							Tq_null[i] = 0.5f * Kp_imped_dq_null[i]*delta_ql_null_rad(i,0); //
					
					}
					else if( Free_P == false && Free_O == true)
					{
						for( int i = 0; i < 4; i++)
							Tq_null[i] = 1.0f * Kp_imped_dq_null[i]*delta_ql_null_rad(i,0); //
						for( int i = 4; i < ROBOT_DOF; i++)
							Tq_null[i] = 0.0f; //
					}
					else if( Free_P == true && Free_O == false)
					{
						for( int i = 0; i < 4; i++)
							Tq_null[i] = 0.25f * Kp_imped_dq_null[i]*delta_ql_null_rad(i,0); //
						for( int i = 4; i < ROBOT_DOF; i++)
							Tq_null[i] = 0.75f * Kp_imped_dq_null[i]*delta_ql_null_rad(i,0); //
					}
					else
					{
						for( int i = 0; i < ROBOT_DOF; i++)
							Tq_null[i] = 0.0f;
					}
				}
				else
				{
					Psi_psi1 =Psi_psi0;
					for( int i = 0; i < ROBOT_DOF; i++)
						Tq_null[i] = 0.0f;
				}
			}
			else
			{		
				Psi_psi1 = PsiCmd;
				NullSpaceKineCompute_2(0);
				/*for( int i = 0; i < ROBOT_DOF; i++)
					Tq_null[i] = Kp_imped_dq_null[i]*delta_ql_null_rad(i,0);*/
				if( Free_P == false && Free_O == false )
				{
					for( int i = 0; i < ROBOT_DOF; i++)
						Tq_null[i] = Kp_imped_dq_null[i]*delta_ql_null_rad(i,0);
				}
				else if( Free_P == false && Free_O == true)
				{
					for( int i = 0; i < 4; i++)
						Tq_null[i] = 1.0f * Kp_imped_dq_null[i]*delta_ql_null_rad(i,0); //
					for( int i = 4; i < ROBOT_DOF; i++)
						Tq_null[i] = 0.0f;
				
				}
				else if( Free_P == true && Free_O == false)
				{
					for( int i = 0; i < 4; i++)
							Tq_null[i] = 1.25f * Kp_imped_dq_null[i]*delta_ql_null_rad(i,0); //
						for( int i = 4; i < ROBOT_DOF; i++)
							Tq_null[i] = 0.50f * Kp_imped_dq_null[i]*delta_ql_null_rad(i,0); //
				}
				else
				{
					for( int i = 0; i < ROBOT_DOF; i++)
						Tq_null[i] = 0.0f;
				}
			}
			Tq_null[3] = 0.0f;
		}

		// Null space torque in ModeArm = 2 *****************************************************//
		if( ModeArm_C_nullTq == true)
		{
			//Null Space impedence law
			Psi_psi1 = PsiCmd;
			NullSpaceKineCompute_2(0);
			/*for( int i = 0; i < ROBOT_DOF; i++)
				Tq_null[i] = Kp_imped_dq_null[i]*delta_ql_null_rad(i,0);*/
			
			for( int i = 0; i < 4; i++)
				Tq_null[i] = 1.5f * Kp_imped_dq_null[i]*delta_ql_null_rad(i,0); //
			for( int i = 4; i < ROBOT_DOF; i++)
				Tq_null[i] = 0.0f; //
			
			Tq_null[3] = 0.0f;
		}
		//ENDof: Null space torque in ModeArm = 2 ***********************************************//

	}
	else if( t_count_B == 5)
	{
		//if( modeControl_K  == true || ModeArm_C_nullTq == true)
		//{
		//	Psi_1 = Psi;
		//	/*PsiCmd_1 = PsiCmd;
		//	dPsiCmd_1 = dPsiCmd;*/
		//}
	}
	//ENDof: Sampling_B and Null Space Torque ***************************************************//

	// auto self motion joint 6 limit avoidance**************************************************//
	if( modeMotion_C == true && AutoSelfMotion == true)
	{
		if( t_count_C == 3)
		{
			//if( qlCmd_rad(5,0) < q_p_limit(5,0) && qlCmd_rad(5,0) > q_n_limit(5,0) )
			if( abs(qlCmd_rad(5,0)) < q_p_limit(5,0) && abs(qlCmd_rad(5,0)) > 0 )
			{
				H_jlim6_1 = pi*pi / 16.0f / ( pi/2.0f - abs(qlCmd_rad(5,0)) ) / abs(qlCmd_rad(5,0));
				if(H_jlim6_1 > 101.0f)
					H_jlim6_1 = 101.0f;
			}
			else
			{
				H_jlim6_1 = 101.0f;
			}
			
			NullSpaceKineCompute_1(1);
		}
		else if( t_count_C == 4)
		{
			Psi_psi1 = PsiCmd + 0.0175f;
			NullSpaceKineCompute_2(1);   
			H_jlim6_next_p = pi*pi / 16.0f / ( pi/2.0f - abs(ql_psi1(5,0)) ) / abs(ql_psi1(5,0));
		}
		else if( t_count_C == 7)
		{
			Psi_psi1 = PsiCmd - 0.0175f;
			NullSpaceKineCompute_2(1);
			H_jlim6_next_n = pi*pi / 16.0f / ( pi/2.0f - abs(ql_psi1(5,0)) ) / abs(ql_psi1(5,0));
		}
		else if( t_count_C == 8)
		{
			if( H_jlim6_next_p < H_jlim6_1 && H_jlim6_next_n >= H_jlim6_1)
			{
				PsiTrgt = PsiCmd + deg2rad*0.16f*(H_jlim6_1-1.0f);
				//PsiCmd = PsiCmd_1 + deg2rad*0.16f*(H_jlim6_1-1.0f);
			}
			else if( H_jlim6_next_n < H_jlim6_1 && H_jlim6_next_p >= H_jlim6_1)
			{
				PsiTrgt = PsiCmd - deg2rad*0.1f*(H_jlim6_1-1.0f);
				//PsiCmd = PsiCmd_1 - deg2rad*0.1f*(H_jlim6_1-1.0f);
			}

			if( abs(PsiTrgt - PsiCmd) < 0.00175f)
				PsiTrgt = PsiCmd;

			MOVJN = true;
			ArmAngleRampGenerator();
			MOVL = true;
		}
	}
	//ENDof: auto self motion joint 6 limit avoidance********************************************//

	// Sampling_C and Nonlinear Computing and State Update **************************************//
	switch( t_count_C )
	{
	case 1:
		if( ModeArm_K != true && ModeArm_C_nullTq != true)
		{
			for( int i = 0; i < ROBOT_DOF; i++)
			{
				cosq[i] = cosf(ql_rad(i,0));
				sinq[i] = sinf(ql_rad(i,0));
			}
			CosSinUpdate(0);
		}
		for( int i = 0; i < USAGE_CHANNELS; i++)
		{
			cosqCmd[i] = cosf(qlCmd_rad(i,0));
			sinqCmd[i] = sinf(qlCmd_rad(i,0));
		}
		break;

	case 2:
		if( ModeArm_K != true && ModeArm_C_nullTq != true && ModeArm_C_nullTq_1 != true)
		{
			FwdKineUpdate_1( 0 );
			FwdKineUpdate_2( 0 );
		}
		break;

	case 3:
		if( ModeArm_K != true && ModeArm_C_nullTq != true && ModeArm_C_nullTq_1 != true)
		{
			dPsi = Psi-Psi_1;
			V07 = P07 - P07_1;
			W07 = rr2delta(R07,R07_1);
		}

		JacobUpdate();

		//if( ModeArm_C == true )
		//{
		//	JJT_v = JV0wt.transpose()*JV0wt;
		//	JJT_w = JW4wt.transpose()*JW4wt;
		//	Vv_sngl = sqrtf( JJT_v.determinant() );
		//	Vw_sngl = sqrtf( JJT_w.determinant() );
		//	//V_sngl = sqrtf( Vv_sngl*Vv_sngl + Vw_sngl*Vw_sngl );
		//	V_sngl = Vw_sngl;
		//}

		break;

	case 4:
		GravityCompUpdate();

		break;

	/*case 5:
		break;*/

	case 6:
		if( ModeArm_C != true && ModeArm_K != true )
		{
			CosSinUpdate( 1 );
		}
		break;

	case 7:
		if( ModeArm_C != true && ModeArm_K != true )
		{
			FwdKineUpdate_1( 1 );
			FwdKineUpdate_2( 1 );
		}
		break;

	case 8:
		if( ModeArm_C != true && ModeArm_K != true )
		{
			dPsiCmd = PsiCmd - PsiCmd_1;
			V07Cmd = P07Cmd - P07Cmd_1;
			W07Cmd = rr2delta(R07Cmd,R07Cmd_1);
		}
		if( ModeArm_K == true)
			JacobUpdate();

		//test
		/*if( ModeArm_C == true)
			JacobUpdate();*/
		//
		break;

	/*case 9:
		break;*/

	/*case 10:
		break;*/
	}
	//ENDof: Sampling_C and Nonlinear Computing and State Update ********************************//

	// record last data
	for( int i = 0; i < USAGE_CHANNELS; i++)
	{
		qmCmd_ENC_2[i] = qmCmd_ENC_1[i]; qmCmd_ENC_1[i] = qmCmd_ENC[i];
		dqmCmd_ENC_1[i] = dqmCmd_ENC[i];
	}

	if( t_count_C == 5)
	{
		if( ModeArm_K == true || ModeArm_C_nullTq == true)
		{
			Psi_1 = Psi;
			/*PsiCmd_1 = PsiCmd;
			dPsiCmd_1 = dPsiCmd;*/
		}
		
		t_count_B = 0;
	}
	else if( t_count_C == 10)
	{
		if( modeMotion_C == true)
			MOVL_1 = MOVL;

		PsiCmd_1 = PsiCmd;
		dPsiCmd_1 = dPsiCmd;

		q1oCmd_1 = q1oCmd;
		qlCmd_rad_1 = qlCmd_rad;
		P0wCmd_1 = P0wCmd;
		P07Cmd_1 = P07Cmd;
		R07Cmd_1 = R07Cmd;
		
		V07Cmd_1 = V07Cmd;
		W07Cmd_1 = W07Cmd;

		if( ModeArm_K != true && ModeArm_C_nullTq != true)
		{
			Psi_1 = Psi;
			/*PsiCmd_1 = PsiCmd;
			dPsiCmd_1 = dPsiCmd;*/
		}
		q1o_1 = q1o;
		ql_rad_1 = ql_rad;
		P0w_1 = P0w;
		P07_1 = P07;
		R07_1 = R07;

		ModeArm_1 = ModeArm;
		ModeArm_G_1 = ModeArm_G;
		ModeArm_J_1 = ModeArm_J;
		ModeArm_C_1 = ModeArm_C;
		ModeArm_K_1 = ModeArm_K;

		modeControl_G_1 = modeControl_G;
		modeControl_P_1 = modeControl_P;
		modeControl_K_1 = modeControl_K;
		modeMotion_J_1 = modeMotion_J;
		modeMotion_C_1 = modeMotion_C;

		t_count_B = 0;
		t_count_C = 0;
	}

	if( t_count_D == 5)
	{
		for( int i = 0; i < ROBOT_DOF; i++)
		{
			q_deg[i] = rad2deg*ql_rad(i,0);
			qCmd_deg[i] = ENC2deg[i]*qmCmd_ENC[i] + q_home_deg(i,0);
		}
		T07.block(0,0,3,3) = R07;
		T07.block(0,3,3,1) = P07;
		
		T07Cmd.block(0,0,3,3) = R07Cmd;
		T07Cmd.block(0,3,3,1) = P07Cmd;
	}
	else if( t_count_D == 20)
	{
		t_count_D = 0;
	}

	TestComputeLose--;

	IMC_DAC_SetOutputVoltage(7, 0);
	//sevroloop complete !
}