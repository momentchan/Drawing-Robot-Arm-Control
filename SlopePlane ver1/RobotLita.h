#ifndef _ROBOTLITA_H
#define _ROBOTLITA_H

#include <math.h>
#include <Eigen/Dense>
#include "ConstNum.h"

#define SPACE_DOF 6
#define ROBOT_DOF 7

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 7, 1> Vector7f;
typedef Eigen::Matrix<float, 9, 1> Vector9f;
typedef Eigen::Matrix<float, 3, 4> Matrix34f;
typedef Eigen::Matrix<float, 4, 3> Matrix43f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 6, 7> Matrix67f;
typedef Eigen::Matrix<float, 7, 6> Matrix76f;
typedef Eigen::Matrix<float, 7, 7> Matrix7f; 

//RobotLita Global Constant-------------
const float TOL_trim = 0.000000001f;
//arm DH parameter----------------------
 const float d1 = 0.500f;
 const float  a = 0.050f;
 const float d2 = 0.400f;
extern float d3;
extern float d3_0;
extern float d3_1;
extern float d3_d;
extern float  b;
extern float d1a;
extern float d1a2;
extern float d2a;
extern float d2a2;
extern float  B1;
extern float  B2;
extern float  B3;

const float eye3[3][3] = {{1.0f,0.0f,0.0f},{0.0f,1.0f,0.0f},{0.0f,0.0f,1.0f}};

const float g0_z = 9.81f; // m(m/s^2)
const float M1a = 4.0f; // kg
const float M1b = 2.0f; // kg
const float M2 = 5.0f; // kg //4.0f
const float M3 = 2.5f;//1.0f
//const float M2 = 4.0f; // kg //4.0f
//const float M3 = 1.0f;//1.0f
const float L_m1a = 0.3f;
const float L_m2 = 0.4f;//0.3f
const float L_m3 = 0.15f;//0.1f
//const float L_m2 = 0.3f;//0.3f
//const float L_m3 = 0.1f;//0.1f

extern Eigen::Vector3f L3se;
extern Eigen::Vector3f L4ew;
extern Eigen::Vector3f L7wt;

extern Vector7f q_home;
extern Vector7f q_home_deg;
extern Vector7f q_p_limit;
extern Vector7f q_n_limit;
extern Vector7f q_p_limit_deg;
extern Vector7f q_n_limit_deg;

extern float Psi_p_limit;
extern float Psi_n_limit;

extern float q4_U_snglr;
extern float q4_L_snglr;

extern float P0w_norm_U_limit;
extern float P0w_norm_L_limit;
//--------------------------------------

/*
	   Link([home,  d,  a, alpha, 0])
L(1) = Link([   0,  0,  0, -pi/2, 0]);
L(2) = Link([ -th,  0, d1,     0, 0]);
L(3) = Link([  th,  0, -a,  pi/2, 0]);
L(4) = Link([   0, d2,  0, -pi/2, 0]);
L(5) = Link([   0,  0,  0,  pi/2, 0]);
L(6) = Link([   0, d3,  b,     0, 0]);
       Link([home,  d,  a, alpha, 0])
L(1) = Link([   0,  0,  0, -pi/2, 0]);
L(2) = Link([   0,  0,  0,  pi/2, 0]);
L(3) = Link([   0, d1,  a, -pi/2, 0]);
L(4) = Link([   0,  0, -a,  pi/2, 0]);
L(5) = Link([   0, d2,  0, -pi/2, 0]);
L(6) = Link([   0,  0,  0,  pi/2, 0]);
L(7) = Link([   0, d3,  b,     0, 0]);
*/

//RobotLita Global Variable----
//Damped Least Squares---------
extern float JacoDampGain;
extern float JacoDampThreshold;
//-----------------------------

void Init_RobotLita();

Eigen::Matrix4f posXzx2tr( const Eigen::Vector3f& X, const Eigen::Vector3f& uz, const Eigen::Vector3f& ux );

Eigen::Matrix3f skew( const Eigen::Vector3f& V );

Eigen::Vector3f rr2delta( const Eigen::Matrix3f& R1, const Eigen::Matrix3f& R0);

Eigen::Matrix3f delta2rr( const Eigen::Vector3f& d , const Eigen::Matrix3f& R0 );

Vector6f tr2delta( const Eigen::Matrix4f& T1, const Eigen::Matrix4f& T0 );

Eigen::Matrix4f delta2tr( const Vector6f& d , const Eigen::Matrix4f& T0 );

Vector6f tr2x( const Eigen::Matrix4f& T );

Eigen::Matrix4f tr07( const Vector7f& q );

Matrix67f jacob0( const Vector7f& q );

//Matrix76f invjacob0( const Vector7f& q );
//
//Matrix6f inverse6x6M(const Matrix6f& A, float DampGain, float DampThreshold, float det, bool& sdj );

Vector7f gravity_term( const Vector7f& q, const Eigen::Vector3f& g, float load, bool normal = 1);

#endif