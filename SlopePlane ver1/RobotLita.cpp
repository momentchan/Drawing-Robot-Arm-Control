#include "RobotLita.h"
#include <iostream>
using namespace std;
using namespace Eigen;

//RobotLita Global Variable----
//end-effector DH parameter----
//float d3 = 0.31f;
float d3 = 0.0f;
float d3_0 = 0.32f;//0.26f;
float d3_1;
float d3_d;
float  b = 0.0f;
float d1a;
float d1a2;
float d2a;
float d2a2;
float B1;
float B2;
float B3;

float JacoDampGain = 0.1f;
float JacoDampThreshold = 0.000001f;

Vector3f L3se;
Vector3f L4ew;
Vector3f L7wt;

Vector7f q_home;
Vector7f q_home_deg;

Vector7f q_p_limit;
Vector7f q_n_limit;
Vector7f q_p_limit_deg;
Vector7f q_n_limit_deg;

float Psi_p_limit;
float Psi_n_limit;

float q4_U_snglr;
float q4_L_snglr;
float P0w_norm_U_limit;
float P0w_norm_L_limit;

//-----------------------------

//RobotLita KIN functions
//qqvei: it is a simple limit setting
void Init_RobotLita()
{
	d3 = d3_0;
	d3_1 = d3_0;
	d3_d = d3_0;
	d1a2 = d1*d1 + a*a;
	d2a2 = d2*d2 + a*a;
	d1a = sqrt(d1a2);
	d2a = sqrt(d2a2);

	B1 = atanf(d1/a);
	B2 = atanf(d2/a);
	B3 = 2*pi-B1-B2;

	L3se <<  a,  -d1, 0.0f;
	L4ew << -a, 0.0f,   d2;
	L7wt <<  b, 0.0f,   d3;

	    q_home << 0.0f, -pi/2.0f, 0.0f,     pi, 0.0f, 0.0f, 0.0f;
	q_home_deg << 0.0f,   -90.0f, 0.0f, 180.0f, 0.0f, 0.0f, 0.0f;

	    q_p_limit <<      pi,deg2rad*100.0f,      pi,       pi,      pi,  pi/2.0f,  2.0f*pi;
	    q_n_limit <<     -pi,      -pi/2.0f,     -pi, -pi/4.0f,     -pi, -pi/2.0f, -2.0f*pi;
	q_p_limit_deg <<  180.0f,        100.0f,  180.0f,   180.0f,  180.0f,    90.0f,   360.0f;
	q_n_limit_deg << -180.0f,        -90.0f, -180.0f,   -45.0f, -180.0f,   -90.0f,  -360.0f;

	Psi_p_limit = deg2rad*88.0f;
	Psi_n_limit = -Psi_p_limit;

	q4_U_snglr = B3 - pi + deg2rad*10.0f;
	q4_L_snglr = pi - deg2rad*10.0f;
	P0w_norm_U_limit = sqrt( d1a2 + d2a2 + 2.0f*d1a*d2a*cosf(deg2rad*10.0f) );
	P0w_norm_L_limit = sqrt( d1a2 + d2a2 - 2.0f*d1a*d2a*cosf(B3 - pi + deg2rad*10.0f) );
}

Matrix4f posXzx2tr( const Vector3f& X, const Vector3f& uz, const Vector3f& ux )
{
	Matrix4f T;
	Vector3f uy;
	uy = uz.cross(ux);
	for ( int i=0; i<3; i++)
	{
		T(i,0) = ux(i,0);
		T(i,1) = uy(i,0);
		T(i,2) = uz(i,0);
		T(i,3) = X(i,0);
		T(3,i) = 0.0f;
	}
	T(3,3) = 1.0f;
	return T;
}

Matrix3f skew( const Vector3f& V )
{
	Matrix3f S;
	S << 0.0f, -V(2,0), V(1,0), V(2,0), 0.0f, -V(0,0), -V(1,0), V(0,0), 0.0f;
	/*
	S = skew(v)
    S = [  0   -v(3)  v(2)
          v(3)   0   -v(1)
         -v(2)  v(1)   0  ];
	*/
	return S;
}

Vector3f rr2delta( const Matrix3f& R1, const Matrix3f& R0)
{
	Vector3f delta;
	Matrix3f S;
	//Matrix3f R0t;

	//R0t = R0.transpose();
	S = R1*R0.transpose();
	S(0,0) = S(0,0) - 1.0f;
	S(1,1) = S(1,1) - 1.0f;
	S(2,2) = S(2,2) - 1.0f;
	delta(0,0) = 0.5f*(S(2,1)-S(1,2));
	delta(1,0) = 0.5f*(S(0,2)-S(2,0));
	delta(2,0) = 0.5f*(S(1,0)-S(0,1));

	return delta;
}

Matrix3f delta2rr( const Vector3f& d , const Matrix3f& R0 )
{
	Matrix3f R;
	Matrix3f R1;

	R << 1.0f, -d(2,0), d(1,0), d(2,0), 1.0f, -d(0,0), -d(1,0), d(0,0), 1.0f;

	/*R(0,0) =  1.0f;
	R(1,0) =  d(2,0);
	R(2,0) = -d(1,0);
	R(0,1) = -d(2,0);
	R(1,1) =  1.0f;
	R(2,1) =  d(0,0);
	R(0,2) =  d(1,0);
	R(1,2) = -d(0,0);
	R(2,2) =  1.0f;*/

	R1 = R*R0;

	return R1;
}

Vector6f tr2delta( const Matrix4f& T1, const Matrix4f& T0 )
{
	Vector6f delta;
	Matrix3f S;
	Matrix3f R1;
	Matrix3f R0t;
	//Matrix3f I3;
	//I3 << 1,0,0,0,1,0,0,0,1;
	delta.block(0,0,3,1) = T1.block(0,3,3,1) - T0.block(0,3,3,1);
	/*for( int i=0; i < 3; i++)
	{
		for( int j=0; j < 3; j++)
		{
			S(i,j) = T1(0,j)*T0(0,i) + T1(1,j)*T0(1,i) + T1(2,j)*T0(2,i) - eye3[i][j];
		}
	}*/
	R1 = T1.block(0,0,3,3);
	R0t = T0.block(0,0,3,3).transpose();
	//S = (T1.block(0,0,3,3))*(T0.block(0,0,3,3).transpose());
	S = R1*R0t;
	S(0,0) = S(0,0) - 1.0f;
	S(1,1) = S(1,1) - 1.0f;
	S(2,2) = S(2,2) - 1.0f;
	delta(3,0) = 0.5f*(S(2,1)-S(1,2));
	delta(4,0) = 0.5f*(S(0,2)-S(2,0));
	delta(5,0) = 0.5f*(S(1,0)-S(0,1));
	return delta;
}

Matrix4f delta2tr( const Vector6f& d , const Matrix4f& T0 )
{
	Matrix3f R;
	Matrix4f T1;

	for( int i=0; i<3; i++)
	{
		R(i,i) = 1.0f;
		T1(3,i) = 0.0f;
	}
	R(1,0) =  d(5,0);
	R(2,0) = -d(4,0);
	R(0,1) = -d(5,0);
	R(2,1) =  d(3,0);
	R(0,2) =  d(4,0);
	R(1,2) = -d(3,0);

	T1.block(0,0,3,3) = R*T0.block(0,0,3,3);

	T1.block(0,3,3,1) = T0.block(0,3,3,1) + d.block(0,0,3,1);
	
	T1(3,3) = 1.0f;

	return T1;
	
	/*
    delta = eye(4,4) + [skew(d(4:6)) d(1:3); 0 0 0 0];
	
	S = skew(v)
    S = [  0   -v(3)  v(2)
          v(3)   0   -v(1)
         -v(2)  v(1)   0  ];
	*/
}

Vector6f tr2x( const Matrix4f& T )
{
	Vector6f X;
	X.block(0,0,3,1) = T.block(0,3,3,1);
	
	Vector3f O;
	float beta, cb;
	beta = atan2( -T(2,0), sqrt( T(0,0)*T(0,0) + T(1,0)*T(1,0) ) );
	if( beta >= 89.99f && beta <= 90.01f )
		O << atan2( T(0,1), T(1,1) ), beta, 0.0f;
	else if( beta <= -89.99f && beta >= -90.01f )
		O << -atan2( T(0,1), T(1,1) ), beta, 0.0f;
	else
	{
		cb = cosf(beta);
		O << atan2( T(2,1)/cb, T(2,2)/cb ), beta, atan2( T(1,0)/cb, T(0,0)/cb );
	}
	X.block(3,0,3,1) = O;
	return X;
}

Matrix4f tr07( const Vector7f& q )
{
	float c1 = cosf( q(0,0) );
	float s1 = sinf( q(0,0) );
	float c2 = cosf( q(1,0) );
	float s2 = sinf( q(1,0) );
	float c3 = cosf( q(2,0) );
	float s3 = sinf( q(2,0) );
	float c4 = cosf( q(3,0) );
	float s4 = sinf( q(3,0) );
	float c5 = cosf( q(4,0) );
	float s5 = sinf( q(4,0) );
	float c6 = cosf( q(5,0) );
	float s6 = sinf( q(5,0) );
	float c7 = cosf( q(6,0) );
	float s7 = sinf( q(6,0) );

	Matrix4f T;

	T(0,0) =   c7*(s6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + s7*(s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c5*(c3*s1 + c1*c2*s3));
	T(0,1) =   c7*(s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - s7*(s6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)));
	T(0,2) = - c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3));
	T(0,3) =   d1*c1*s2 - d3*(c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) + s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) - d2*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - a*s1*s3 + b*c7*(s6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + b*s7*(s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + a*c4*(s1*s3 - c1*c2*c3) + a*c1*c2*c3 + a*c1*s2*s4;
	T(1,0) = - c7*(s6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) - c6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - s7*(s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) - c5*(c1*c3 - c2*s1*s3));
	T(1,1) =   s7*(s6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) - c6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - c7*(s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) - c5*(c1*c3 - c2*s1*s3));
	T(1,2) =   c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3));
	T(1,3) =   d2*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) + d3*(c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) + a*c1*s3 + d1*s1*s2 - b*c7*(s6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) - c6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - b*s7*(s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) - a*c4*(c1*s3 + c2*c3*s1) + a*c2*c3*s1 + a*s1*s2*s4;
	T(2,0) =   s7*(s5*(c2*s4 + c3*c4*s2) + c5*s2*s3) - c7*(c6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) + s6*(c2*c4 - c3*s2*s4));
	T(2,1) =   c7*(s5*(c2*s4 + c3*c4*s2) + c5*s2*s3) + s7*(c6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) + s6*(c2*c4 - c3*s2*s4));
	T(2,2) =   c6*(c2*c4 - c3*s2*s4) - s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5);
	T(2,3) =   d2*(c2*c4 - c3*s2*s4) - d3*(s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) - c6*(c2*c4 - c3*s2*s4)) + d1*c2 - a*c3*s2 + a*c2*s4 + b*s7*(s5*(c2*s4 + c3*c4*s2) + c5*s2*s3) - b*c7*(c6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) + s6*(c2*c4 - c3*s2*s4)) + a*c3*c4*s2;
	T(3,0) =   0.0f;
	T(3,1) =   0.0f;
	T(3,2) =   0.0f;
	T(3,3) =   1.0f;

/*
T =
[   cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) + sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))), cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) - sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))), - cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))), d1*cos(q1)*sin(q2) - d3*(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - d2*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - a*sin(q1)*sin(q3) + b*cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) + b*sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) + a*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + a*cos(q1)*cos(q2)*cos(q3) + a*cos(q1)*sin(q2)*sin(q4)]
[ - cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))), sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))),   cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))), d2*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) + d3*(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) + a*cos(q1)*sin(q3) + d1*sin(q1)*sin(q2) - b*cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - b*sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - a*cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + a*cos(q2)*cos(q3)*sin(q1) + a*sin(q1)*sin(q2)*sin(q4)]
[                                                                                                                                                                       sin(q7)*(sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4))),                                                                                                                                                                     cos(q7)*(sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) + sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4))),                                                                                                       cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)) - sin(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)),                                                                                                                                                                                                                                                                                                                                                                                     d2*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)) - d3*(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4))) + d1*cos(q2) - a*cos(q3)*sin(q2) + a*cos(q2)*sin(q4) + b*sin(q7)*(sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - b*cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4))) + a*cos(q3)*cos(q4)*sin(q2)]
[                                                                                                                                                                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                1]
*/
	return T;
}

Matrix67f jacob0( const Vector7f& q )
{
	float c1 = cos( q(0,0) );
	float s1 = sin( q(0,0) );
	float c2 = cos( q(1,0) );
	float s2 = sin( q(1,0) );
	float c3 = cos( q(2,0) );
	float s3 = sin( q(2,0) );
	float c4 = cos( q(3,0) );
	float s4 = sin( q(3,0) );
	float c5 = cos( q(4,0) );
	float s5 = sin( q(4,0) );
	float c6 = cos( q(5,0) );
	float s6 = sin( q(5,0) );
	float c7 = cos( q(6,0) );
	float s7 = sin( q(6,0) );

	Matrix67f J;

	J(0,0) =   b*c7*(s6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) - c6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - d3*(c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - a*c1*s3 - d1*s1*s2 - d2*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) + b*s7*(s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) + a*c4*(c1*s3 + c2*c3*s1) - a*c2*c3*s1 - a*s1*s2*s4;
	J(0,1) =   d3*(c6*(c1*c2*c4 - c1*c3*s2*s4) - s6*(c5*(c1*c2*s4 + c1*c3*c4*s2) - c1*s2*s3*s5)) + d2*(c1*c2*c4 - c1*c3*s2*s4) - b*c7*(s6*(c1*c2*c4 - c1*c3*s2*s4) + c6*(c5*(c1*c2*s4 + c1*c3*c4*s2) - c1*s2*s3*s5)) + b*s7*(s5*(c1*c2*s4 + c1*c3*c4*s2) + c1*c5*s2*s3) + d1*c1*c2 - a*c1*c3*s2 + a*c1*c2*s4 + a*c1*c3*c4*s2;
	J(0,2) =   d3*(s6*(s5*(s1*s3 - c1*c2*c3) - c4*c5*(c3*s1 + c1*c2*s3)) - c6*s4*(c3*s1 + c1*c2*s3)) - d2*s4*(c3*s1 + c1*c2*s3) + b*s7*(c5*(s1*s3 - c1*c2*c3) + c4*s5*(c3*s1 + c1*c2*s3)) + b*c7*(c6*(s5*(s1*s3 - c1*c2*c3) - c4*c5*(c3*s1 + c1*c2*s3)) + s4*s6*(c3*s1 + c1*c2*s3)) - a*c3*s1 + a*c4*(c3*s1 + c1*c2*s3) - a*c1*c2*s3;
	J(0,3) =   b*c7*(s6*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + c5*c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2)) - d2*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - a*s4*(s1*s3 - c1*c2*c3) - d3*(c6*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c5*s6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2)) + a*c1*c4*s2 - b*s5*s7*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2);
	J(0,4) =   b*s7*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) + d3*s6*(s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + b*c6*c7*(s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c5*(c3*s1 + c1*c2*s3));
	J(0,5) =   d3*(s6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + b*c7*(c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) + s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)));
	J(0,6) =   b*c7*(s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - b*s7*(s6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)));
	J(1,0) =   d1*c1*s2 - d3*(c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) + s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) - d2*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - a*s1*s3 + b*c7*(s6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + b*s7*(s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + a*c4*(s1*s3 - c1*c2*c3) + a*c1*c2*c3 + a*c1*s2*s4;
	J(1,1) =   d3*(c6*(c2*c4*s1 - c3*s1*s2*s4) - s6*(c5*(c2*s1*s4 + c3*c4*s1*s2) - s1*s2*s3*s5)) + d2*(c2*c4*s1 - c3*s1*s2*s4) - b*c7*(s6*(c2*c4*s1 - c3*s1*s2*s4) + c6*(c5*(c2*s1*s4 + c3*c4*s1*s2) - s1*s2*s3*s5)) + b*s7*(s5*(c2*s1*s4 + c3*c4*s1*s2) + c5*s1*s2*s3) + d1*c2*s1 - a*c3*s1*s2 + a*c2*s1*s4 + a*c3*c4*s1*s2;
	J(1,2) =   d2*s4*(c1*c3 - c2*s1*s3) - d3*(s6*(s5*(c1*s3 + c2*c3*s1) - c4*c5*(c1*c3 - c2*s1*s3)) - c6*s4*(c1*c3 - c2*s1*s3)) - b*s7*(c5*(c1*s3 + c2*c3*s1) + c4*s5*(c1*c3 - c2*s1*s3)) - b*c7*(c6*(s5*(c1*s3 + c2*c3*s1) - c4*c5*(c1*c3 - c2*s1*s3)) + s4*s6*(c1*c3 - c2*s1*s3)) + a*c1*c3 - a*c4*(c1*c3 - c2*s1*s3) - a*c2*s1*s3;
	J(1,3) =   d3*(c6*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) - c5*s6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2)) + d2*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + a*s4*(c1*s3 + c2*c3*s1) - b*c7*(s6*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + c5*c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2)) + a*c4*s1*s2 + b*s5*s7*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2);
	J(1,4) = - b*s7*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)) - d3*s6*(s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) - b*c6*c7*(s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) - c5*(c1*c3 - c2*s1*s3));
	J(1,5) = - d3*(s6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) - c6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - b*c7*(c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)));
	J(1,6) =   b*s7*(s6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) - c6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - b*c7*(s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) - c5*(c1*c3 - c2*s1*s3));
	J(2,0) =   0.0f;
	J(2,1) =   d3*(s6*(c5*(s2*s4 - c2*c3*c4) + c2*s3*s5) - c6*(c4*s2 + c2*c3*s4)) - d1*s2 - d2*(c4*s2 + c2*c3*s4) - a*c2*c3 - a*s2*s4 - b*s7*(s5*(s2*s4 - c2*c3*c4) - c2*c5*s3) + b*c7*(c6*(c5*(s2*s4 - c2*c3*c4) + c2*s3*s5) + s6*(c4*s2 + c2*c3*s4)) + a*c2*c3*c4;
	J(2,2) =   d3*(s6*(c3*s2*s5 + c4*c5*s2*s3) + c6*s2*s3*s4) + b*c7*(c6*(c3*s2*s5 + c4*c5*s2*s3) - s2*s3*s4*s6) + a*s2*s3 + b*s7*(c3*c5*s2 - c4*s2*s3*s5) - a*c4*s2*s3 + d2*s2*s3*s4;
	J(2,3) =   b*c7*(s6*(c2*s4 + c3*c4*s2) - c5*c6*(c2*c4 - c3*s2*s4)) - d3*(c6*(c2*s4 + c3*c4*s2) + c5*s6*(c2*c4 - c3*s2*s4)) - d2*(c2*s4 + c3*c4*s2) + a*c2*c4 - a*c3*s2*s4 + b*s5*s7*(c2*c4 - c3*s2*s4);
	J(2,4) =   b*s7*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) + d3*s6*(s5*(c2*s4 + c3*c4*s2) + c5*s2*s3) + b*c6*c7*(s5*(c2*s4 + c3*c4*s2) + c5*s2*s3);
	J(2,5) =   b*c7*(s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) - c6*(c2*c4 - c3*s2*s4)) - d3*(c6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) + s6*(c2*c4 - c3*s2*s4));
	J(2,6) =   b*c7*(s5*(c2*s4 + c3*c4*s2) + c5*s2*s3) + b*s7*(c6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) + s6*(c2*c4 - c3*s2*s4));
	J(3,0) =   0.0f;
	J(3,1) = - s1;
	J(3,2) =   c1*s2;
	J(3,3) = - c3*s1 - c1*c2*s3;
	J(3,4) =   c1*c4*s2 - s4*(s1*s3 - c1*c2*c3);
	J(3,5) =   s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c5*(c3*s1 + c1*c2*s3);
	J(3,6) = - c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3));
	J(4,0) =   0.0f;
	J(4,1) =   c1;
	J(4,2) =   s1*s2;
	J(4,3) =   c1*c3 - c2*s1*s3;
	J(4,4) =   s4*(c1*s3 + c2*c3*s1) + c4*s1*s2;
	J(4,5) =   c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4);
	J(4,6) =   c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3));
	J(5,0) =   1.0f;
	J(5,1) =   0.0f;
	J(5,2) =   c2;
	J(5,3) =   s2*s3;
	J(5,4) =   c2*c4 - c3*s2*s4;
	J(5,5) =   s5*(c2*s4 + c3*c4*s2) + c5*s2*s3;                          
	J(5,6) =   c6*(c2*c4 - c3*s2*s4) - s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5);
	
/*
J = 
[ b*cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - d3*(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - a*cos(q1)*sin(q3) - d1*sin(q1)*sin(q2) - d2*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) + b*sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) + a*cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - a*cos(q2)*cos(q3)*sin(q1) - a*sin(q1)*sin(q2)*sin(q4), d3*(cos(q6)*(cos(q1)*cos(q2)*cos(q4) - cos(q1)*cos(q3)*sin(q2)*sin(q4)) - sin(q6)*(cos(q5)*(cos(q1)*cos(q2)*sin(q4) + cos(q1)*cos(q3)*cos(q4)*sin(q2)) - cos(q1)*sin(q2)*sin(q3)*sin(q5))) + d2*(cos(q1)*cos(q2)*cos(q4) - cos(q1)*cos(q3)*sin(q2)*sin(q4)) - b*cos(q7)*(sin(q6)*(cos(q1)*cos(q2)*cos(q4) - cos(q1)*cos(q3)*sin(q2)*sin(q4)) + cos(q6)*(cos(q5)*(cos(q1)*cos(q2)*sin(q4) + cos(q1)*cos(q3)*cos(q4)*sin(q2)) - cos(q1)*sin(q2)*sin(q3)*sin(q5))) + b*sin(q7)*(sin(q5)*(cos(q1)*cos(q2)*sin(q4) + cos(q1)*cos(q3)*cos(q4)*sin(q2)) + cos(q1)*cos(q5)*sin(q2)*sin(q3)) + d1*cos(q1)*cos(q2) - a*cos(q1)*cos(q3)*sin(q2) + a*cos(q1)*cos(q2)*sin(q4) + a*cos(q1)*cos(q3)*cos(q4)*sin(q2), d3*(sin(q6)*(sin(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q4)*cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) - cos(q6)*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) - d2*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)) + b*sin(q7)*(cos(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q4)*sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) + b*cos(q7)*(cos(q6)*(sin(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q4)*cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) + sin(q4)*sin(q6)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) - a*cos(q3)*sin(q1) + a*cos(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)) - a*cos(q1)*cos(q2)*sin(q3), b*cos(q7)*(sin(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + cos(q5)*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2))) - d2*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - a*sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - d3*(cos(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2))) + a*cos(q1)*cos(q4)*sin(q2) - b*sin(q5)*sin(q7)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)),   b*sin(q7)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) + d3*sin(q6)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) + b*cos(q6)*cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))),   d3*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) + b*cos(q7)*(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))), b*cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) - b*sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))]
[ d1*cos(q1)*sin(q2) - d3*(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - d2*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - a*sin(q1)*sin(q3) + b*cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) + b*sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) + a*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + a*cos(q1)*cos(q2)*cos(q3) + a*cos(q1)*sin(q2)*sin(q4), d3*(cos(q6)*(cos(q2)*cos(q4)*sin(q1) - cos(q3)*sin(q1)*sin(q2)*sin(q4)) - sin(q6)*(cos(q5)*(cos(q2)*sin(q1)*sin(q4) + cos(q3)*cos(q4)*sin(q1)*sin(q2)) - sin(q1)*sin(q2)*sin(q3)*sin(q5))) + d2*(cos(q2)*cos(q4)*sin(q1) - cos(q3)*sin(q1)*sin(q2)*sin(q4)) - b*cos(q7)*(sin(q6)*(cos(q2)*cos(q4)*sin(q1) - cos(q3)*sin(q1)*sin(q2)*sin(q4)) + cos(q6)*(cos(q5)*(cos(q2)*sin(q1)*sin(q4) + cos(q3)*cos(q4)*sin(q1)*sin(q2)) - sin(q1)*sin(q2)*sin(q3)*sin(q5))) + b*sin(q7)*(sin(q5)*(cos(q2)*sin(q1)*sin(q4) + cos(q3)*cos(q4)*sin(q1)*sin(q2)) + cos(q5)*sin(q1)*sin(q2)*sin(q3)) + d1*cos(q2)*sin(q1) - a*cos(q3)*sin(q1)*sin(q2) + a*cos(q2)*sin(q1)*sin(q4) + a*cos(q3)*cos(q4)*sin(q1)*sin(q2), d2*sin(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)) - d3*(sin(q6)*(sin(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - cos(q6)*sin(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - b*sin(q7)*(cos(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - b*cos(q7)*(cos(q6)*(sin(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) + sin(q4)*sin(q6)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) + a*cos(q1)*cos(q3) - a*cos(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)) - a*cos(q2)*sin(q1)*sin(q3), d3*(cos(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2))) + d2*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + a*sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - b*cos(q7)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + cos(q5)*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2))) + a*cos(q4)*sin(q1)*sin(q2) + b*sin(q5)*sin(q7)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)), - b*sin(q7)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - d3*sin(q6)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - b*cos(q6)*cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))), - d3*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - b*cos(q7)*(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))), b*sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - b*cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))]
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                0,                                                                                                                                                         d3*(sin(q6)*(cos(q5)*(sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4)) + cos(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4))) - d1*sin(q2) - d2*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) - a*cos(q2)*cos(q3) - a*sin(q2)*sin(q4) - b*sin(q7)*(sin(q5)*(sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4)) - cos(q2)*cos(q5)*sin(q3)) + b*cos(q7)*(cos(q6)*(cos(q5)*(sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4)) + cos(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4))) + a*cos(q2)*cos(q3)*cos(q4),                                                                                                                                                                                                                                                                                                                     d3*(sin(q6)*(cos(q3)*sin(q2)*sin(q5) + cos(q4)*cos(q5)*sin(q2)*sin(q3)) + cos(q6)*sin(q2)*sin(q3)*sin(q4)) + b*cos(q7)*(cos(q6)*(cos(q3)*sin(q2)*sin(q5) + cos(q4)*cos(q5)*sin(q2)*sin(q3)) - sin(q2)*sin(q3)*sin(q4)*sin(q6)) + a*sin(q2)*sin(q3) + b*sin(q7)*(cos(q3)*cos(q5)*sin(q2) - cos(q4)*sin(q2)*sin(q3)*sin(q5)) - a*cos(q4)*sin(q2)*sin(q3) + d2*sin(q2)*sin(q3)*sin(q4),                                                                                                                                                                                                                                                             b*cos(q7)*(sin(q6)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - cos(q5)*cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4))) - d3*(cos(q6)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4))) - d2*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + a*cos(q2)*cos(q4) - a*cos(q3)*sin(q2)*sin(q4) + b*sin(q5)*sin(q7)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)),                                                                                                                                                                                                   b*sin(q7)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + d3*sin(q6)*(sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) + b*cos(q6)*cos(q7)*(sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)),                                                                                                                                                                                                           b*cos(q7)*(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4))) - d3*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4))),                                                                                                                                                                     b*cos(q7)*(sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) + b*sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)))]
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             -sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         cos(q1)*sin(q2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         - cos(q3)*sin(q1) - cos(q1)*cos(q2)*sin(q3),                                                                                                                                                                                                                                                                                                                                                                                                                cos(q1)*cos(q4)*sin(q2) - sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)),                                                                                                                                                                                                                                                                                                                                                                            sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)),                                                                                                                                                                       - cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))]
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         sin(q1)*sin(q2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3),                                                                                                                                                                                                                                                                                                                                                                                                                sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2),                                                                                                                                                                                                                                                                                                                                                                            cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)) - sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)),                                                                                                                                                                         cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))]
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 cos(q2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     sin(q2)*sin(q3),                                                                                                                                                                                                                                                                                                                                                                                                                                                    cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4),                                                                                                                                                                                                                                                                                                                                                                                                                                            sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3),                                                                                                                                                                                                                                                                             cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)) - sin(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5))]
*/
	return J;
}

//Matrix76f invjacob0( const Vector7f& q )
//{
//	Matrix67f J;
//	Matrix76f JT;
//	Matrix7f JTJ;
//	J = jacob0( q );
//	JT = J.transpose();
//	JTJ = JT*J;
//	float h = abs(JTJ.determinant());
//	if (h < JacoDampThreshold )
//	{
//		for( int i=0; i<7; i++)
//			JTJ(i,i) = JTJ(i,i) + JacoDampGain*(1-h/JacoDampThreshold);
//		//cout << "J0(" << q.transpose()<< ")=" << endl << J << endl;
//		return JTJ.inverse()*JT;
//	}
//	return JTJ.inverse()*JT;
//}

//Matrix6f inverse6x6M(const Matrix6f& A, float DampGain, float DampThreshold, float &det, bool &sdj )
//{
//	det = A.determinant();
//	if( det < DampThreshold )
//	{
//		sdj = true;
//		Matrix6f Ap;
//		Ap = A;
//		for( int i=0; i<6; i++ )
//			Ap(i,i) = Ap(i,i) + DampGain*( 1 - det/DampThreshold );
//
//		return Ap.inverse();
//	}
//	sdj = false;
//	return A.inverse();
//}

Vector7f gravity_term( const Vector7f& q, const Vector3f& g, float load, bool normal)
{

	float c1 = cos( q(0,0) );
	float s1 = sin( q(0,0) );
	float c2 = cos( q(1,0) );
	float s2 = sin( q(1,0) );
	float c3 = cos( q(2,0) );
	float s3 = sin( q(2,0) );
	float c4 = cos( q(3,0) );
	float s4 = sin( q(3,0) );
	float c5 = cos( q(4,0) );
	float s5 = sin( q(4,0) );
	float c6 = cos( q(5,0) );
	float s6 = sin( q(5,0) );
	float c7 = cos( q(6,0) );
	float s7 = sin( q(6,0) );
	
	float L1;
	float L2;
	float L3;
	L1 = L_m1a;
	L2 = (M2*L_m2+load*(L_m2+0.5f*d3))/(M2+load);


	Vector7f TqG;
	TqG << 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f;

	if( normal == true)// g0 = "-"[0,0,-9.81] = [0,0,9.81] => g0z = 9.81~
	{
		float g0z = g0_z;
		
		TqG(0,0) = 0.0f;
		TqG(1,0) = - M2*g0z*(L2*(c4*s2 + c2*c3*s4) + s2*d1 + c2*c3*a + s2*s4*a - c2*c3*c4*a) - M1b*g0z*(s2*d1 + c2*c3*a) - M1a*g0z*s2*L1;
		TqG(2,0) =   M2*g0z*(s2*s3*a + s2*s3*s4*L2 - c4*s2*s3*a) + M1b*g0z*s2*s3*a;
		TqG(3,0) = - M2*g0z*(L2*(c2*s4 + c3*c4*s2) - c2*c4*a + c3*s2*s4*a);
 
		TqG(4,0) =   M3*g0z*s6*L3*(s5*(c2*s4 + c3*c4*s2) + c5*s2*s3);
		TqG(5,0) = - M3*g0z*L3*(c6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) + s6*(c2*c4 - c3*s2*s4)); 

	}
	else
	{
		float g0x; g0x = -g(0,0);
		float g0y; g0y = -g(1,0);
		float g0z; g0z = -g(2,0);

		TqG(0,0) = M1b*g0y*(c1*s2*d1 - s1*s3*a + c1*c2*c3*a) - M1b*g0x*(c1*s3*a + s1*s2*d1 + c2*c3*s1*a) + M2*g0y*(c1*s2*d1 - L2*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) + c4*a*(s1*s3 - c1*c2*c3) - s1*s3*a + c1*c2*c3*a + c1*s2*s4*a) - M2*g0x*(L2*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) + c1*s3*a - c4*a*(c1*s3 + c2*c3*s1) + s1*s2*d1 + c2*c3*s1*a + s1*s2*s4*a) + M1a*g0y*c1*s2*L1 - M1a*g0x*s1*s2*L1;
		TqG(1,0) = M1b*g0y*(c2*s1*d1 - c3*s1*s2*a) + M2*g0y*(L2*(c2*c4*s1 - c3*s1*s2*s4) + c2*s1*d1 - c3*s1*s2*a + c2*s1*s4*a + c3*c4*s1*s2*a) - M2*g0z*(L2*(c4*s2 + c2*c3*s4) + s2*d1 + c2*c3*a + s2*s4*a - c2*c3*c4*a) - M1b*g0z*(s2*d1 + c2*c3*a) + M2*g0x*(L2*(c1*c2*c4 - c1*c3*s2*s4) + c1*c2*d1 - c1*c3*s2*a + c1*c2*s4*a + c1*c3*c4*s2*a) + M1b*g0x*(c1*c2*d1 - c1*c3*s2*a) - M1a*g0z*s2*L1 + M1a*g0x*c1*c2*L1 + M1a*g0y*c2*s1*L1;
		TqG(2,0) =  M2*g0y*(c1*c3*a + s4*L2*(c1*c3 - c2*s1*s3) - c4*a*(c1*c3 - c2*s1*s3) - c2*s1*s3*a) - M2*g0x*(c3*s1*a + s4*L2*(c3*s1 + c1*c2*s3) - c4*a*(c3*s1 + c1*c2*s3) + c1*c2*s3*a) - M1b*g0x*(c3*s1*a + c1*c2*s3*a) + M1b*g0y*(c1*c3*a - c2*s1*s3*a) + M2*g0z*(s2*s3*a + s2*s3*s4*L2 - c4*s2*s3*a) + M1b*g0z*s2*s3*a;
		TqG(3,0) =  M2*g0y*(L2*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s4*a*(c1*s3 + c2*c3*s1) + c4*s1*s2*a) - M2*g0x*(L2*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s4*a*(s1*s3 - c1*c2*c3) - c1*c4*s2*a) - M2*g0z*(L2*(c2*s4 + c3*c4*s2) - c2*c4*a + c3*s2*s4*a); 

	}
	return TqG;
}