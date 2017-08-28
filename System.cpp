#include "System.h"

void Initial_IMPCard( int Card_Index )
{
	IMC_OpenDevice(0, Card_Index);					//  初始化IMP Series運動控制平台
	IMC_GLB_ResetModule(RESET_ALL, Card_Index);		//  Reset 所選定的 IMP 模組
	Initial_Encoder( Card_Index );	
	Initial_DAC( Card_Index );
}

void Initial_Encoder( int Card_Index )
{
	//Init Encoder
	for(int i=0; i<8; i++)
	{
	    IMC_ENC_EnableInAInverse(i, 0, Card_Index);	// Inverse -> 0 : No-Inv, 1 : Inv
	    IMC_ENC_EnableInBInverse(i, 0, Card_Index);	// Inverse -> 0 : No-Inv, 1 : Inv
	    IMC_ENC_EnableInCInverse(i, 0, Card_Index);	// Inverse -> 0 : No-Inv, 1 : Inv
		IMC_ENC_EnableInABSwap(i, 0, Card_Index);	// Swap -> 0 : No-Inv, 1 : Inv
		IMC_ENC_SetInputFormat(i, ENC_FMT_AB, Card_Index);	// 設定各 Encoder counter 輸入訊號型態	// Format -> 0: Disable, 1 : P/D, 2 : CW/CCW, 3 : A/B
		IMC_ENC_SetInputRate(i, ENC_RATE_X4, Card_Index);	// 設定各 Encoder counter 訊號解碼倍率	// Rate -> 0/3 : X4, 1 : X1, 2 : X2
		IMC_ENC_ClearCounter(i, 1, Card_Index);		// 開啟清除 Encoder counter value 功能			// Claer -> 0: No-Clear, 1 : Clear
		IMC_ENC_ClearCounter(i, 0, Card_Index);		// 關閉清除 Encoder counter value 功能			// Claer -> 0: No-Clear, 1 : Clear
		IMC_ENC_StartCounter(i, 1, Card_Index);		// 啟動 encoder 記錄功能
	}
}

void Initial_DAC( int Card_Index ) //初始化DAC 進行數位/類比電壓輸出轉換
{
	for(int i=0; i<8; i++)
	{
	 	IMC_DAC_EnableChannel(i, 1, Card_Index);	// 開啟 DAC channel 數位轉類比電壓輸出功能
	 	IMC_DAC_SelectSource(i, 0, Card_Index);		// 0:Source from DAC output buffer   1:Soruce from PCL error counter
		IMC_LIO_SetServoOn(i, Card_Index);			// 開啟指定軸之伺服驅動接點功能
	}
	IMC_DAC_StartConverter(1, Card_Index);			// 啟動 DAC 進行數位/類比電壓輸出轉換
}

void Read_Encoder( float joint_pos[2][6], long lEncoder[2][6], int Card_Index )
{
	for(int i=0; i<6; i++) 
	{
		IMC_ENC_ReadCounter(i, &lEncoder[Card_Index][i], Card_Index);
		joint_pos[Card_Index][i] = lEncoder[Card_Index][i] * ENC2RAD_FACTOR[Card_Index][i];
	}
}

void Output_Voltage( float updateTorque[2][6], int Card_Index )
{
	float outputVoltage[6]={0.0};
	for(int i=0; i<6; i++) 
	{
		outputVoltage[i] = updateTorque[Card_Index][i] * OUTPUT_TORQUE_FACTOR[Card_Index][i];
		
		if(outputVoltage[i] > MAX_OUTPUT_VOLTAGE) 
			outputVoltage[i] = MAX_OUTPUT_VOLTAGE;
		if(outputVoltage[i] < MIN_OUTPUT_VOLTAGE) 
			outputVoltage[i] = MIN_OUTPUT_VOLTAGE;

		IMC_DAC_SetOutputVoltage(i, outputVoltage[i], Card_Index );  //  設定 DAC 輸出電壓值
	}
	//printf("%f\n", outputVoltage[1]);
}

void Close_IMPCard( int Card_Index )
{
	for(int i=0;i<8;i++)
	{	
		IMC_DAC_SetOutputVoltage(i, 0, Card_Index );// 設定 DAC channel 輸出電壓值為0
		IMC_DAC_EnableChannel(i, 0, Card_Index);	// 關閉 DAC channel 數位轉類比電壓輸出功能
		IMC_LIO_SetServoOff(i, Card_Index);			// 關閉指定軸之伺服驅動接點功能
	}
	IMC_DAC_StartConverter(0, Card_Index);			// 關閉 DAC 進行數位/類比電壓輸出轉換
	IMC_CloseIfOpen( Card_Index );					// 結束IMP模組，本函式會關閉IMP模組內所有功能
}