#ifndef _CONTROLLITA_H
#define _CONTROLLITA_H

#include "IMPCard.h"
#include "RobotLita.h"

#define SAMPLING_TIME   1   // unit: ms
#define SAMPLING_TIME_A 2
#define SAMPLING_TIME_B 5
#define SAMPLING_TIME_C 10
#define SAMPLING_TIME_D 20
#define SAMPLING_TIME_E 50

const float SamplingTime_B = 5.0f;
const float SamplingTime_C = 10.0f;

//const float GearRatio[ROBOT_DOF] = { 483.0f, 483.0f, 404.0f, 404.0f, 404.0f, 404.0f, 404.0f };

const float V_th[USAGE_CHANNELS] = {0.8f, 1.4f, 0.8f, 1.4f, 1.2f, 2.2f, 2.0f};//deadzone bondary

//const float V_bound_fric[  USAGE_CHANNELS] = {1.0f, 1.7f, 4.0f, 4.0f, 4.0f, 3.8f, 4.0f};
//const float gain_FT_p_fric[USAGE_CHANNELS] = {1.0f, 1.1f, 0.1f, 0.8f, 0.9f, 0.9f, 0.6f};
//const float gain_FT_n_fric[USAGE_CHANNELS] = {1.0f, 0.5f, 0.1f, 0.1f, 0.9f, 0.6f, 0.6f};
//const float gain_DP_fric[  USAGE_CHANNELS] = {2.0f, 1.6f, 6.0f, 4.8f, 1.2f, 2.0f, 1.2f}; 
const float V_bound_fric[  USAGE_CHANNELS] = {1.2f, 1.8f, 4.0f, 4.0f, 4.0f, 4.0f, 4.0f};
const float gain_FT_p_fric[USAGE_CHANNELS] = {0.9f, 1.2f, 0.1f, 0.8f, 0.9f, 0.7f, 0.5f};
const float gain_FT_n_fric[USAGE_CHANNELS] = {0.9f, 0.2f, 0.1f, 0.09f, 0.9f, 0.4f, 0.5f};
const float gain_DP_fric[  USAGE_CHANNELS] = {2.0f, 1.6f, 6.0f, 4.8f, 1.2f, 2.0f, 1.2f}; 
// {2.0f, 1.6f, 4.0f, 3.2f, 1.2f, 1.6f, 1.2f}

const float gain_GC_grav[USAGE_CHANNELS] = {1.0f, 1.2f, 1.4f, 1.4f, 2.2f, 2.4f, 1.0f};

//const int Jn_index = 2; // redundant axis, maybe 0~6(1~7), 0(1),2(3) better

const float Kz_FS = 12500; //N/m

const int DataLength = 50000;

//define function in RTtimer
void Init_ControlLita();
void Reset_ControlLita();

inline long roundf( float x );

inline void MotorPosPIDVelPIControl( int i );

inline void CartesianControl();

inline void ImpedenceTorque();

inline void TqCmd2SetValue( int i );

inline void MotorPosRampGenerator( int i );

//inline void LinVelGenerator();
inline void PositionRampGenerator();
inline void OrientationRampGenerator();
inline void ArmAngleRampGenerator();

inline void CosSinUpdate( bool isCMD);

inline void FwdKineUpdate_1( bool isCMD);
inline void FwdKineUpdate_2( bool isCMD);

inline void InvKineCompute_1();
inline void InvKineCompute_2( bool isCMD);

inline void NullSpaceKineCompute_1( bool isCMD);
inline void NullSpaceKineCompute_2( bool isCMD);

inline void JacobUpdate();
inline void GravityCompUpdate();
inline void FrictionComp();

// CONTROLL:
// modeContorl:
//-1: default ( ZeroTorqueMode )
// 0: ZeroTorqueMode
// 1: TorqueCompMode
// 2: VelocityMode
// 3: PositionMode
// 4: 
//
// MOTION:
// modeMotion:
//-1: default ( not generate motion, but only record motion )
// 0: JointMode
// 1: CartesianMode (Line)
// 2: 
//

void ServoLoop();
//

//DATA
extern long qENC_data[DataLength][USAGE_CHANNELS];
extern long qCmd_data[DataLength][USAGE_CHANNELS];
extern float TqCmd_data[DataLength][USAGE_CHANNELS];

extern float Fz_data[DataLength];

extern bool q_DATA_RECORD;
extern bool Tq_DATA_RECORD;
extern bool Fz_DATA_RECORD;
extern long data_count;

//
extern float ENC2rad[USAGE_CHANNELS];
extern float rad2ENC[USAGE_CHANNELS];
extern float ENC2deg[USAGE_CHANNELS];
extern float deg2ENC[USAGE_CHANNELS];
extern float torque2setvalue[USAGE_CHANNELS];

extern float K_vel_P[USAGE_CHANNELS];
extern float K_vel_p2I[USAGE_CHANNELS];

extern float K_pos_P[USAGE_CHANNELS];
extern float K_pos_p2I[USAGE_CHANNELS];
extern float K_pos_p2D[USAGE_CHANNELS];

extern float K_posvel_P[USAGE_CHANNELS];
extern float K_posvel_p2I[USAGE_CHANNELS];
extern float K_posvel_p2D[USAGE_CHANNELS];

extern float TorqueFriction[USAGE_CHANNELS];

extern float damper_gain[USAGE_CHANNELS];

extern long q_home_ENC[USAGE_CHANNELS];

extern long q_pLimit_ENC[USAGE_CHANNELS];
extern long q_nLimit_ENC[USAGE_CHANNELS];

extern bool highsetvalue;
extern long TestComputeLose;

extern int ModeArm;
extern int SubMode;

extern bool ModeArm_G;
extern bool ModeArm_J;
extern bool ModeArm_C;
extern bool ModeArm_K;

extern bool Free_All;
extern bool Free_P_psie;
extern bool Free_P;
extern bool Free_O;
extern bool Free_psi;

extern bool nullAuxTq;
extern bool dirAuxF;

extern bool modeControl_G;
extern bool modeControl_P;
extern bool modeControl_K;

extern bool modeMotion_J;
extern bool modeMotion_C;

extern bool STOP;

extern bool SaftySTOP_Enable;

extern bool Integrator_Enable;

extern bool MOVJ;
extern bool MOVJ_flag;

extern bool MOV[USAGE_CHANNELS];

extern long qm_ENC_1[USAGE_CHANNELS];

extern float dqm_ENC[USAGE_CHANNELS];			
extern float dqm_ENC_1[USAGE_CHANNELS];

extern float ddqm_ENC[USAGE_CHANNELS];

extern long qmCmd_ENC[USAGE_CHANNELS];

extern float dqmCmd_ENC[USAGE_CHANNELS];
extern float dqmCmdCmd_ENC[USAGE_CHANNELS];
extern float ddqmCmd_ENC[USAGE_CHANNELS];

extern long  qmTrgt_ENC[USAGE_CHANNELS];
extern float dqmTrgt_ENC[USAGE_CHANNELS];

extern long  qmTarget_ENC[USAGE_CHANNELS];

extern long  error_qm_ENC[USAGE_CHANNELS];		
extern float error_dqm_ENC[USAGE_CHANNELS];		
extern long  diff_error_qm_ENC[USAGE_CHANNELS]; 
extern float diff_error_dqm_ENC[USAGE_CHANNELS];
extern long  summ_error_qm_ENC[USAGE_CHANNELS]; 
extern float summ_error_dqm_ENC[USAGE_CHANNELS];

extern long summ_error_qm_limit_ENC[USAGE_CHANNELS];
extern float summ_error_dqm_limit_ENC[USAGE_CHANNELS];

extern int noMOV_count[USAGE_CHANNELS];

extern float Vel_Limit_ENC[USAGE_CHANNELS];
extern float Acc_Limit_ENC[USAGE_CHANNELS];
extern float Dec_Limit_ENC[USAGE_CHANNELS];
extern float qVelMax;


extern float Kp_imped_F[3];
extern float Kv_imped_F[3];
extern float Kp_imped_M[3];
extern float Kv_imped_M[3];
extern float K_aux_F[3];
extern float Kp_imped_dq_null[USAGE_CHANNELS];
extern float K_aux_dq_null[USAGE_CHANNELS];

extern bool MOVL;
extern bool MOVL_flag;
extern bool MOVL_newflag;
extern bool MOVL_L;
extern bool MOVL_R;
extern bool MOVJN;

extern bool MOVLDEC_L;

extern Vector7f ql_rad;
extern Vector7f dql_rad;

extern Vector7f qlCmd_rad;
extern Vector7f dqlCmd_rad;

extern Vector7f qlTrgt_rad;

extern float cosq[ROBOT_DOF];
extern float sinq[ROBOT_DOF];
extern float cosqCmd[ROBOT_DOF];
extern float sinqCmd[ROBOT_DOF];

extern float Psi;
extern float dPsi;

extern float Psi_psi0;
extern float Psi_psi1;

extern Vector7f ql_psi0;
extern Vector7f ql_psi1;

extern Vector7f delta_ql_null_rad;;

extern Vector7f dql_null_rad;

extern Eigen::Matrix3f R03_o_psi0;

extern Eigen::Matrix3f R03_o;

extern Eigen::Matrix3f R03;
extern Eigen::Matrix3f R34;
extern Eigen::Matrix3f R47;
extern Eigen::Matrix3f R04;
extern Eigen::Matrix3f R07;

extern Eigen::Vector3f u0w;

extern Eigen::Vector3f P0w;
extern Eigen::Vector3f P07;
extern float P0w_norm;

extern Eigen::Vector3f V0w;
extern Eigen::Vector3f V07;
extern Eigen::Vector3f W07;
extern float V07_norm;
extern float W07_norm;

extern Eigen::Matrix4f T07;

extern float PsiCmd;
extern float dPsiCmd;

extern Eigen::Matrix3f R03_oCmd;

extern Eigen::Matrix3f R03Cmd;
extern Eigen::Matrix3f R34Cmd;
extern Eigen::Matrix3f R04Cmd;
extern Eigen::Matrix3f R47Cmd;
extern Eigen::Matrix3f R07Cmd;

extern Eigen::Vector3f u0wCmd;

extern Eigen::Vector3f P0wCmd;
extern Eigen::Vector3f P07Cmd;
extern float P0wCmd_norm;

extern Eigen::Vector3f V0wCmd;
extern Eigen::Vector3f V07Cmd;
extern Eigen::Vector3f W07Cmd;
extern float V07Cmd_norm;
extern float W07Cmd_norm;

extern Eigen::Matrix4f T07Cmd;

extern float PsiTrgt;
extern float dPsiTrgt;

extern float PsiTarget;

extern Eigen::Vector3f P07Trgt;
extern Eigen::Matrix3f R07Trgt;
extern Eigen::Vector3f V07Trgt;
extern Eigen::Vector3f W07Trgt;
extern float V07Trgt_norm;
extern float W07Trgt_norm;

extern Eigen::Matrix4f T07Target;

extern bool MOVL;
extern bool MOVL_flag;
extern bool MOVL_newflag;
extern bool MOVL_L;
extern bool MOVL_R;
extern bool MOVJN;

extern float Lin_Vel_limit;
extern float Lin_Acc_limit;
extern float Lin_Dec_limit;

extern float Ang_Vel_limit;
extern float Ang_Acc_limit;
extern float Ang_Dec_limit;

extern float Jn_Vel_limit;
extern float Jn_Acc_limit;
extern float Jn_Dec_limit;

extern float LinearSpeedMax;
extern float AngularSpeedMax;
extern float JnSpeedMax;

extern float TOL_L;
extern float TOL_R;

extern Matrix43f JV0wt;
extern Eigen::Matrix3f JW4wt;

extern Eigen::Vector3f delta_P0w;
extern Eigen::Vector3f delta_R07;
extern Eigen::Vector3f F0wCmd;
extern Eigen::Vector3f M0wCmd;

extern Vector7f Tq_imped;

extern float TqmTrgt[USAGE_CHANNELS];

extern float TqmCmd[USAGE_CHANNELS];

extern float Tq_fric[USAGE_CHANNELS];

extern float Tq_g[ROBOT_DOF];

extern float Tq_null[ROBOT_DOF];

extern float q_deg[ROBOT_DOF];
extern float qCmd_deg[ROBOT_DOF];
extern Eigen::Matrix4f T07;
extern Eigen::Matrix4f T07Cmd;

extern bool AutoSelfMotion_Enable;
extern bool AutoSelfMotion;
extern float Vv_sngl;
extern float Vw_sngl;
extern float V_sngl;
extern float H_jlim6;

//float H_jlim6_next_p = 1.0f;
//float H_jlim6_next_n = 1.0f;
//float dH_jlim6 = 0.0f;
//float Psi_jlim6_next_p = 0.0f;
//float Psi_jlim6_next_n = 0.0f;

extern bool FORCE_CONTROL;
extern bool FORCE_LIMIT;
extern bool FORCE_TRACKING;
extern bool FC_flag;
extern int dir_ENC7_BL;
extern long ENC7_BL;
extern float ENC7_e;
extern float deltaZ_FS;
extern float z_init;
extern float Fz_init;
extern float Fz_th;
extern float Fz;
extern float Fz_task;


#endif