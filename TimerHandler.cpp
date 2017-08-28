#include "TimerHandler.h"

extern boost::shared_ptr< rl::kin::Kinematics > kin[2];//share_ptr:共用一份記憶體空間
extern Tcp6D tcp[2];
long lEncoder[2][6] = {0};
float joint_pos[2][6] = {0.0};
float last_joint_pos[2][6] = {0.0};
// last position and angular position
Eigen::MatrixXd OTG_X_Last = Eigen::MatrixXd::Zero(2, 6);
// last velocity and angular velocity
Eigen::MatrixXd OTG_V_Last = Eigen::MatrixXd::Zero(2, 6);

float Gcomp[2][4] = {0.0};
float updateTorque[2][6] = {0.0};
float Integator[2][6]={0}; //積分項

// RTX periodic timer handler function
void RTFCNDCL TimerHandler( PVOID context ) // 1(ms)執行一次
{
	//-----初始狀態設定-----
	static int initial_count = 1;
	if( initial_count)
	{
		for( int j = 0; j < 2; j++) // j = 0, 1 表示左右手分別設定
		{
			OTG_X_Last(j, 0) = 0.65;
			OTG_X_Last(j, 1) = 0;
			OTG_X_Last(j, 2) = 0;
			OTG_X_Last(j, 3) = -90*rl::math::DEG2RAD;
			OTG_X_Last(j, 4) = 0;
			OTG_X_Last(j, 5) = 180*rl::math::DEG2RAD;

			for( int i = 0; i < 6; i++) //單隻手臂的六軸速度設定
				OTG_V_Last(j, i) = 0;
		}
		initial_count = 0;
	}
	//-----初始狀態設定-----

	
	// read the current state of the motor angle (encoder counter)
	Read_Encoder( joint_pos, lEncoder , ARM_L);
	Read_Encoder( joint_pos, lEncoder , ARM_R);

	
	// forward kinematics of current state
	// velocity and angular velocity
	Eigen::MatrixXd x(2, 6), v(2,6);
	
	
	// 0: current state,  1: on-line generated state,  2: target state
	Eigen::Vector3d t0[2], t1[2], t2[2];
	Eigen::Matrix3d r0[2], r1[2], r2[2];
	
	
	for( int j = 0; j < 2 ; j++) // j = 0, 1 代表左右手
	{
		Eigen::Vector3d dx, dth;
		// get current transformation matrix
		rl::math::Vector q(kin[j]->getDof());
		
		for ( int i = 0; i < kin[j]->getDof(); i++)
			q(i) = joint_pos[j][i];

		kin[j]->setPosition(q);
		kin[j]->updateFrames();
		rl::math::Transform T0 = kin[j]->forwardPosition();
		r0[j] = T0.rotation();
		t0[j] = T0.translation();
		
		static int qqq = 1;
		if( qqq++ % 1001 == 0)
		{
			std::cout<<t0[0]<<std::endl;
			std::cout<<" "<<std::endl;
		}
		
		// calculate the target transformation matrix
		r2[j] = Eigen::AngleAxisd( tcp[j].a*rl::math::DEG2RAD, Eigen::Vector3d::UnitZ())
			   *Eigen::AngleAxisd( tcp[j].b*rl::math::DEG2RAD, Eigen::Vector3d::UnitY())
			   *Eigen::AngleAxisd( tcp[j].c*rl::math::DEG2RAD, Eigen::Vector3d::UnitX());
		t2[j] = Eigen::Vector3d( tcp[j].x, tcp[j].y, tcp[j].z);

		// calculate the on-line generated transformation matrix
		r1[j] = Eigen::AngleAxisd( OTG_X_Last(j, 3), Eigen::Vector3d::UnitZ())
			   *Eigen::AngleAxisd( OTG_X_Last(j, 4), Eigen::Vector3d::UnitY())
			   *Eigen::AngleAxisd( OTG_X_Last(j, 5), Eigen::Vector3d::UnitX());
		t1[j] = Eigen::Vector3d( OTG_X_Last(j, 0), OTG_X_Last(j, 1), OTG_X_Last(j, 2));

		dx = t2[j] - t1[j];

		
		x(j, 0) = t2[j](0) - t1[j](0);
		x(j, 1) = t2[j](1) - t1[j](1);
		x(j, 2) = t2[j](2) - t1[j](2);
		x(j, 3) = tcp[j].a*rl::math::DEG2RAD - OTG_X_Last(j, 3);
		x(j, 4) = tcp[j].b*rl::math::DEG2RAD - OTG_X_Last(j, 4);
		x(j, 5) = tcp[j].c*rl::math::DEG2RAD - OTG_X_Last(j, 5);
	}
	v = x/0.001;

	//static int ccc = 1;

	//if( ccc++ % 100 == 0)
	//{
	//	std::cout<<x<<std::endl;
	//	std::cout<<std::endl;
	//}

	// 最大速度與最大角速度
	//const double vmax[6] = {0.5, 1, 0.5, 1, 1, 1};   // m/s
	const double vmax[6] = {1, 2, 1, 1, 1, 1};   // m/s
	
	// 最大加速度與最大角加速度
	//const double amax[6] = {1, 2, 1, 2, 2, 2};
	const double amax[6] = {2, 2, 1, 2, 2, 2};
	
	// cartesian force
	rl::math::Vector torque[2];			
	rl::math::Vector TCP_V[2];			// current cartesian velocity

	// OTG here, three dimensional- x,y,z
	for( int j = 0; j < 2; j++)
	{
		rl::math::Vector OTG_X(6);
		rl::math::Vector OTG_V(6);			

		kin[j]->updateJacobian();
		rl::math::Matrix J = kin[j]->getJacobian();

		// current cartesian velocity
		rl::math::Vector dq(6);
		for( int i = 0; i < 6; i++)
			dq(i) = (joint_pos[j][i] - last_joint_pos[j][i])/0.001;		
		TCP_V[j] = J*dq;

		for( int i = 0; i < 6; i++)
		{
			OTG_X(i) = x(j, i);
			OTG_V(i) = v(j, i);
			if( fabs(OTG_X(i)) < 0.000001)
				OTG_X(i) = 0;
			double vel_dec  = sqrt( 2*amax[i]*fabs(OTG_X(i)));	// deceleration part of the velocity profile

			if( OTG_V(i) >= 0)
			{
				if( OTG_V(i) > vmax[i])
					OTG_V(i) = vmax[i];
				if( OTG_V(i) > vel_dec)
					OTG_V(i) = vel_dec;
				if( OTG_V(i) > OTG_V_Last(j,i) + amax[i]*0.001)
					OTG_V(i) = OTG_V_Last(j,i) + amax[i]*0.001;

			}
			else
			{
				if( OTG_V(i) < -vmax[i])
					OTG_V(i) = -vmax[i];
				if( OTG_V(i) < -vel_dec)
					OTG_V(i) = -vel_dec;
				if( OTG_V(i) < OTG_V_Last(j,i) - amax[i]*0.001)
					OTG_V(i) = OTG_V_Last(j,i) - amax[i]*0.001;
			}

			// update
			OTG_X(i) = OTG_X_Last(j,i) + 0.5*(OTG_V(i)+OTG_V_Last(j,i))*0.001;
			// record the last cartesian position and velocity
			OTG_X_Last(j,i) = OTG_X(i);
			OTG_V_Last(j,i) = OTG_V(i);
		}
		
		//static int qqq = 1;
		//if( qqq++ % 200 == 0)
		//{
		//	std::cout<<OTG_X<<std::endl;
		//	std::cout<<std::endl;
		//}

		// update on-line generated state
		Eigen::Vector3d dx, dth;
		Eigen::Vector3d tcmd( OTG_X(0), OTG_X(1), OTG_X(2));
		// calcuate the target transformation matrix
		Eigen::Matrix3d rcmd;
		rcmd = Eigen::AngleAxisd( OTG_X(3), Eigen::Vector3d::UnitZ())
			   *Eigen::AngleAxisd( OTG_X(4), Eigen::Vector3d::UnitY())
			   *Eigen::AngleAxisd( OTG_X(5), Eigen::Vector3d::UnitX());

		Eigen::Matrix3d skew = rcmd*r0[j].transpose() - Eigen::Matrix3d::Identity();
		dth = Eigen::Vector3d( 0.5*(skew(2,1)-skew(1,2)), 0.5*(skew(0,2)-skew(2,0)), 0.5*(skew(1,0)-skew(0,1)));
		dx = tcmd - t0[j];

		rl::math::Vector delta_x(6);
		for( int i = 0; i < 3; i++)
		{
			delta_x(i) = dx(i);
			delta_x(i+3) = 0;
		}

		rl::math::Vector delta_v(6);
		for( int i = 0; i < 6; i++)
			delta_v(i) = OTG_V(i) - TCP_V[j](i);

		rl::math::Vector k(6), kx(6);
		const float d = 0.0;




		kx(0) = 400;  //cartesian gain   f=kx
		kx(1) = 400;
		kx(2) = 400;
		kx(3) = 100;
		kx(4) = 100;
		kx(5) = 100;
		
		for( int i = 0; i < 6; i++)
			delta_x(i) *= kx(i);

		//for( int i = 0; i < 6; i++)  //積分
		//{
		//	Integator[j][i] +=delta_x(i);
		//	delta_x(i) = delta_x(i)+0.01*Integator[j][i];
		//}
		//---- cartesian space----

		torque[j] = J.transpose()*( delta_x);

		//---- joint space----

		k(0) = 1.5;
		k(1) = 1.5;
		k(2) = 1.5;
		k(3) = 1.5;
		k(4) = 1;
		k(5) = 1;
		
		for( int i = 0; i < 6; i++)
			torque[j][i] = torque[j][i]*k(i); // torque[j][i] *= k(i);

	}

	// Gravity_Compensation
	Gravity_Compensator( ARM_L, Gcomp[0] );
	Gravity_Compensator( ARM_R, Gcomp[1] );

	//---------------------------------------------------------------------------------------------------------------------------------------------------
	for(int i=0; i<4; i++)
	{
		updateTorque[0][i] = Gcomp[0][i];
		updateTorque[1][i] = Gcomp[1][i];
	}

	static float aa=0;
	static float lock=joint_pos[0][0];

	aa=aa+(lock-joint_pos[0][0]);
	updateTorque[0][0]=updateTorque[0][0]+0.01*aa;


	if(0)
	{

		for( int j = 0; j < 2; j++)
			for( int i = 0; i < 6; i++)
				updateTorque[j][i] += torque[j](i);//torque:軌跡規劃完所要算出的torque

	}



	static int count = 1;

	if( count++ % 100 == 0)
	{
		//std::cout<<torque[1]<<std::endl;
		//std::cout<<std::endl;
	}
	
	Output_Voltage( updateTorque, ARM_L );
	Output_Voltage( updateTorque, ARM_R );
	//--------------------------------------------------------------------------------

	// record the last position data of joint
	for( int i = 0; i < 6; i++)
	{
		last_joint_pos[ARM_L][i] = joint_pos[ARM_L][i];		
		last_joint_pos[ARM_R][i] = joint_pos[ARM_R][i];
	}

} //end of void RTFCNDCL TimerHandler( PVOID context )

