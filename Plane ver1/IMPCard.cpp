#include "IMPCard.h"

long qm_ENC[USAGE_CHANNELS] = {0};
long ENC7 = 0;

//long diff_qm_ENC[USAGE_CHANNELS] = {0};

float SetValue[USAGE_CHANNELS] = {0};

void Init_IMPCard(void)
{
	/*for( int i = 0; i < USAGE_CHANNELS; i++)
	{
		qm_ENC[i] = 0;
		qm_ENC_1[i] = 0;
		qm_ENC_2[i] = 0;
		qm_ENC_3[i] = 0;
		diff_qm_ENC[i] = 0;
		SetValue[i] = 0;
	}*/

	IMC_OpenDevice(0, 0);
	IMC_GLB_ResetModule(RESET_ALL);
	Init_DA();
	Init_Encoder();
}

void Init_DA(void)
{
 	for( int i = 0; i < TOTAL_CHANNELS; i++) // +1 (grasp)
 	{
 		IMC_DAC_EnableChannel(i,1);
 		IMC_DAC_StartConverter(1); 
 		IMC_LIO_SetServoOn(i);   
 	}
	for( int i = 0; i < USAGE_CHANNELS +1; i++) // +1 (grasp)
		IMC_DAC_SetOutputVoltage(i, 0);
}

void Init_Encoder(void)
{
	//Init Encoder
	for( int i = 0; i < TOTAL_CHANNELS; i++) // USAGE_CHANNELS => USAGE_CHANNELS + 1
	{
		IMC_ENC_EnableInAInverse(i, 0);		/* Inverse -> 0 : No-Inv, 1 : Inv*/
		IMC_ENC_EnableInBInverse(i, 0);		/* Inverse -> 0 : No-Inv, 1 : Inv*/
		IMC_ENC_EnableInCInverse(i, 0);		/* Inverse -> 0 : No-Inv, 1 : Inv*/
		IMC_ENC_EnableInABSwap(i, 0);		/* Swap -> 0 : No-Inv, 1 : Inv*/	
		
	}
    	/* Swap -> 0 : No-Inv, 1 : Inv*/
	    IMC_ENC_EnableInABSwap(0, 0); //MT0_ENC_POLAR   1
		IMC_ENC_EnableInABSwap(1, 1); //MT1_ENC_POLAR  -1
		IMC_ENC_EnableInABSwap(2, 0); //MT2_ENC_POLAR   1
		IMC_ENC_EnableInABSwap(3, 1); //MT3_ENC_POLAR  -1
		IMC_ENC_EnableInABSwap(4, 0); //MT4_ENC_POLAR   1
		IMC_ENC_EnableInABSwap(5, 1); //MT5_ENC_POLAR  -1
		IMC_ENC_EnableInABSwap(6, 0); //MT6_ENC_POLAR   1

		IMC_ENC_EnableInABSwap(7, 0); //grasp

	for( int i = 0; i < TOTAL_CHANNELS; i++) // +1 (grasp)
	{	
		IMC_ENC_SetInputRate(i, 3);			/* Rate -> 0/3 : X4, 1 : X1, 2 : X2*/
		IMC_ENC_SetInputFormat(i, 3);		/* Format -> 0: Disable, 1 : P/D, 2 : CW/CCW, 3 : A/B*/
		IMC_ENC_ClearCounter(i, 1);			/* Claer -> 0: No-Clear, 1 : Clear*/
		IMC_ENC_ClearCounter(i, 0);			/* Claer -> 0: No-Clear, 1 : Clear*/
		IMC_ENC_StartCounter(i, 1);
	}
}

void Set_ENC_Home()	
{
}

void Close_IMPCard()
{
	for( int i = 0; i < TOTAL_CHANNELS; i++)
		IMC_DAC_SetOutputVoltage(i, 0);
	for( int i = 0; i < TOTAL_CHANNELS; i++)
	{
		IMC_DAC_EnableChannel(i,0);
		IMC_DAC_StartConverter(0);			//  stop D/A converter
		IMC_LIO_SetServoOff(i);
	}
	IMC_CloseIfOpen();
}

void ReadEncoder()
{
	for( int i = 0; i < USAGE_CHANNELS; i++)
	{
		IMC_ENC_ReadCounter(i, qm_ENC+i); //&qm_ENC[i]
	}
}

void SetOutputDA()
{
	for( int i = 0; i < USAGE_CHANNELS; i++)
		IMC_DAC_SetOutputVoltage(i, SetValue[i]);
}

