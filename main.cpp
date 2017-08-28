//#include "System.h"
#include "TimerHandler.h"
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>	//kbhit
#include <string.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

// OpenNI Header
#include <XnCppWrapper.h>

// OpenNI Library
#pragma comment( lib, "OpenNI.lib" )

// OpenCV Header
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

// OpenCV Library
#pragma comment( lib, "cv210.lib" )
#pragma comment( lib, "cxcore210.lib" )
#pragma comment( lib, "highgui210.lib" )

// Robotics Library
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <rl/kin/Kinematics.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/math/Cubic.h>

#include "Tcp6D.h"



const int ESC_KEY = 27;
char cKey;
boost::shared_ptr< rl::kin::Kinematics > kin[2];
Tcp6D tcp[2];

using namespace std;
void _cdecl wmain( int argc, wchar_t **argv, wchar_t **envp )
{
	//------------------------------------------------------------------------------------------------------------

	// for periodic timer code
    LARGE_INTEGER  liPeriod;   // timer period
    HANDLE         hTimer;     // timer handle

    //  RTX periodic timer code:
    //  TO DO: Set default timer period to your desired time.
    //  The period needs to be an even multiple of the HAL period found in the control panel.

    liPeriod.QuadPart = SERVOLOOP_TIME * 10000;  // 10000 = 1ms

	Initial_IMPCard( ARM_L );
	//Initial_IMPCard( ARM_R );
		
	// Create a periodic timer
	if (! (hTimer = RtCreateTimer(
									NULL,            // security
									0,               // stack size - 0 uses default
									TimerHandler,    // timer handler
									NULL,            // NULL context (argument to handler)
									RT_PRIORITY_MAX, // priority
									CLOCK_FASTEST) ))      // RTX HAL timer
	{
		// TO DO:  exception code here
		// RtWprintf(L"RtCreateTimer error = %d\n",GetLastError());
		ExitProcess(1);
	}

	//讀取手臂組態.xml檔
	boost::shared_ptr< rl::kin::Kinematics > kinematics_L(rl::kin::Kinematics::create("massage.xml"));
	boost::shared_ptr< rl::kin::Kinematics > kinematics_R(rl::kin::Kinematics::create("massage.xml"));
	kin[0] = kinematics_L;
	kin[1] = kinematics_R;

	// initial tool center point
	tcp[0].input( 0.65, 0, 0, -90, 0, 180);
	tcp[1].input( 0.65, 0, 0, -90, 0, 180);


	RtSetTimerRelative( hTimer, &liPeriod, &liPeriod);
//---------------------------------------------------------------------------------

	
	
	while(true)
	{  
		if(_kbhit())
		{
			cKey = _getch();
			if (cKey == ESC_KEY){break;}
		}
		tcp[0].input( 0, 0.35, 0, 0, 0, 180); 
		Sleep(4000);
		tcp[0].input( 0.5, 0.25, 0, 0, 0, 180);
		Sleep(4000);

		//if (_kbhit())
		//{
		//	cKey = _getch();
		//	if (cKey == ESC_KEY)
		//	{
		//		break;
		//	}
		//	if (cKey == 'y')
		//	{
		//		tcp[0].input( 0, 0.35, 0, 0, 0, 180); //[x,y,z, alpha, beta, gamma] , alpha / beta / gamma 不用調，沒有用
		//		tcp[1].input( 0, 0.35, 0, 0, 0, 180);
		//	}
		//	if (cKey == 'u')
		//	{
		//		tcp[0].input( 0.5, 0.25, 0, 0, 0, 180);
		//		tcp[1].input( 0.5, 0.25, 0, 0, 0, 180);
		//	}
			//if (cKey == 't')
			//{
			//	tcp[0].input( 0.3, 0.35, 0, 0, 0, 180);
			//	tcp[1].input( 0.3, 0.35, 0, 0, 0, 180);
			//}
			//if (cKey == 'r')
			//{
			//	tcp[0].input( 0.65, 0, 0, -90, 0, 180);
			//	tcp[1].input( 0.65, 0, 0, -90, 0, 180);
			//}
			//if (cKey == 'f')
			//{
			//	tcp[0].input( 0.3, 0.25, 0, 0, 0, 180);
			//	tcp[1].input( 0.3, 0.25, 0, 0, 0, 180);
			//}
			//if (cKey == 'g')
			//{
			//	tcp[0].input( 0.3, 0.45, 0, 0, 0, 180);
			//	tcp[1].input( 0.3, 0.45, 0, 0, 0, 180);
			//}
			//if (cKey == 'h')
			//{
			//	tcp[0].input( 0.2, 0.35, 0, 0, 0, 180);
			//	tcp[1].input( 0.2, 0.35, 0, 0, 0, 180);
			//}
			//if (cKey == 'j')
			//{
			//	tcp[0].input( 0.4, 0.35, 0, 0, 0, 180);
			//	tcp[1].input( 0.4, 0.35, 0, 0, 0, 180);
			//}
			//if (cKey == 'c')
			//{
			//	tcp[0].input( 0.3, 0.35, 0, 0, 0, 150);
			//	tcp[1].input( 0.3, 0.35, 0, 0, 0, 150);
			//}
			//if (cKey == 'v')
			//{
			//	tcp[0].input( 0.3, 0.35, 0, 0, 0, 210);
			//	tcp[1].input( 0.3, 0.35, 0, 0, 0, 210);
			//}
			//if (cKey == 'b')
			//{
			//	tcp[0].input( 0.3, 0.35, 0, 30, 0, 180);
			//	tcp[1].input( 0.3, 0.35, 0, 30, 0, 180);
			//}
			//if (cKey == 'n')
			//{
			//	tcp[0].input( 0.3, 0.35, 0, -30, 0, 180);
			//	tcp[1].input( 0.3, 0.35, 0, -30, 0, 180);
			//}
			//if (cKey == 'z')
			//{
			//	tcp[0].input( 0.3, 0.35, 0, 0, 30, 180);
			//	tcp[1].input( 0.3, 0.35, 0, 0, 30, 180);
			//}
			//if (cKey == 'x')
			//{
			//	tcp[0].input( 0.3, 0.35, 0, 0, -30, 180);
			//	tcp[1].input( 0.3, 0.35, 0, 0, -30, 180);
			//}
		//} // endif

	}//end while( true )

//----------------------------------------------------------------------------------------------
	Close_IMPCard( ARM_L );
	Close_IMPCard( ARM_R );

	RtDeleteTimer( hTimer );
	printf("Close System\n");


	ExitProcess(0);
}
