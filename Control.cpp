#include "Control.h"
#include <stdio.h>

extern float joint_pos[2][6];//兩臂之六軸馬達編碼值
extern float end_effector_pos[2][3];//兩臂之[x,y,z]


void Gravity_Compensator( int ARM, float Gcomp[4] ) // 重量補償器
{
	float s0 = sin( joint_pos[ARM][0] );
	float s1 = sin( joint_pos[ARM][1] );
	float s2 = sin( joint_pos[ARM][2] );
	float s3 = sin( joint_pos[ARM][3] );
	float s4 = sin( joint_pos[ARM][4] );
	float s5 = sin( joint_pos[ARM][5] );
	float c0 = cos( joint_pos[ARM][0] );
	float c1 = cos( joint_pos[ARM][1] );
	float c2 = cos( joint_pos[ARM][2] );
	float c3 = cos( joint_pos[ARM][3] );
	float c4 = cos( joint_pos[ARM][4] );
	float c5 = cos( joint_pos[ARM][5] );

	float r1 = L1/5.0;
	float r2 = L2/3.0;

	//printf("%f\n",joint_pos[1][0] );

	Gcomp[0] = Mg1*c1*s0*r1 + Mg2*(0.3*c1*s0 - r2*(s3*(c0*c2 - s0*s1*s2) - c1*c3*s0));
	Gcomp[1] = Mg1*c0*s1*r1 + Mg2*(0.3*c0*s1 + r2*(c0*c3*s1 - c0*c1*s2*s3));
	Gcomp[2] = r2*Mg2*s3*(s0*s2 - c0*c2*s1);
	Gcomp[3] = -r2*Mg2*(c3*(c2*s0 + c0*s1*s2) - c0*c1*s3);

	//printf("%f\n",Gcomp[0] );
}

void Forward_Kinematic( int ARM, float end_effector_pos[3] ) // 正向運動學
{
	float s0 = sin( joint_pos[ARM][0] );
	float s1 = sin( joint_pos[ARM][1] );
	float s2 = sin( joint_pos[ARM][2] );
	float s3 = sin( joint_pos[ARM][3] );
	float s4 = sin( joint_pos[ARM][4] );
	float s5 = sin( joint_pos[ARM][5] );
	float c0 = cos( joint_pos[ARM][0] );
	float c1 = cos( joint_pos[ARM][1] );
	float c2 = cos( joint_pos[ARM][2] );
	float c3 = cos( joint_pos[ARM][3] );
	float c4 = cos( joint_pos[ARM][4] );
	float c5 = cos( joint_pos[ARM][5] );

	end_effector_pos[0] = L1*c0*c1 - L2*c0*c1*s3 - L2*c2*c3*s0 + L2*c0*c3*s1*s2;
	end_effector_pos[1] = L1*c1*s0 + L2*c0*c2*c3 - L2*c1*s0*s3 + L2*c3*s0*s1*s2;
	end_effector_pos[2] = L1*s1 - L2*s1*s3 - L2*c1*c3*s2;
}
