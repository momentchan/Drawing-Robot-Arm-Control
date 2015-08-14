
/* The following ifdef block is the standard way of creating macros which make exporting */
/* from a DLL simpler. All files within this DLL are compiled with the IMC_LIB_EXPORTS*/
/* symbol defined on the command line. this symbol should not be defined on any project*/
/* that uses this DLL. This way any other project whose source files include this file see*/ 
/* IMC_LIB_API functions as being imported from a DLL, wheras this DLL sees symbols*/
/* defined with this macro as being exported.*/
#ifndef _IMC_LIB_H_
#define _IMC_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef WIN32
	#include <windows.h>

    #ifdef IMC_LIB_EXPORTS
    #define IMC_LIB_API __declspec(dllexport)
    #else
    #define IMC_LIB_API __declspec(dllimport)
    #endif
    #define IMC_LIB_CALL _stdcall

#else /* WIN32*/

    #define IMC_LIB_API  /*IMPORT*/
    #define IMC_LIB_CALL

    #ifndef _DEFINED_BOOL
        typedef int                 BOOL;
    #endif		
        typedef unsigned char       BYTE;
        typedef unsigned short      WORD;
        typedef unsigned long       DWORD;
#endif /* !WIN32*/

typedef struct _PGE_INT
{
	BYTE FIFO0;
	BYTE FIFO1;
	BYTE FIFO2;
	BYTE FIFO3;
	BYTE FIFO4;
	BYTE FIFO5;
	BYTE FIFO6;
	BYTE FIFO7;
	BYTE CYCLE;
} PGEINT;

typedef struct _ENC_INT
{
	BYTE INDEX0;
	BYTE INDEX1;
	BYTE INDEX2;
	BYTE INDEX3;
	BYTE INDEX4;
	BYTE INDEX5;
	BYTE INDEX6;
	BYTE INDEX7;
	BYTE COMP0;
	BYTE COMP1;
	BYTE COMP2;
	BYTE COMP3;
	BYTE COMP4;
	BYTE COMP5;
	BYTE COMP6;
	BYTE COMP7;
} ENCINT;

typedef struct _TMR_INT
{
	BYTE TIMER;
} TMRINT;

typedef struct _PCL_INT
{
	BYTE OVP0;
	BYTE OVP1;
	BYTE OVP2;
	BYTE OVP3;
	BYTE OVP4;
	BYTE OVP5;
	BYTE OVP6;
	BYTE OVP7;
	BYTE OVN0;
	BYTE OVN1;
	BYTE OVN2;
	BYTE OVN3;
	BYTE OVN4;
	BYTE OVN5;
	BYTE OVN6;
	BYTE OVN7;
} PCLINT;

typedef struct _LIO_INT
{
	BYTE OTP0;
	BYTE OTP1;
	BYTE OTP2;
	BYTE OTP3;
	BYTE OTP4;
	BYTE OTP5;
	BYTE OTP6;
	BYTE OTP7;
	BYTE OTN0;
	BYTE OTN1;
	BYTE OTN2;
	BYTE OTN3;
	BYTE OTN4;
	BYTE OTN5;
	BYTE OTN6;
	BYTE OTN7;
	BYTE HOME0;
	BYTE HOME1;
	BYTE HOME2;
	BYTE HOME3;
	BYTE HOME4;
	BYTE HOME5;
	BYTE HOME6;
	BYTE HOME7;
} LIOINT;

typedef struct _ADC_INT
{
	BYTE COMP0;
	BYTE COMP1;
	BYTE COMP2;
	BYTE COMP3;
	BYTE COMP4;
	BYTE COMP5;
	BYTE COMP6;
	BYTE COMP7;
} ADCINT;

typedef struct _RIO_INT
{
	BYTE S0DI0;
	BYTE S0DI1;
	BYTE S0DI2;
	BYTE S0DI3;
	BYTE S1DI0;
	BYTE S1DI1;
	BYTE S1DI2;
	BYTE S1DI3;
	BYTE S2DI0;
	BYTE S2DI1;
	BYTE S2DI2;
	BYTE S2DI3;
	BYTE FAIL;
} RIOINT;

typedef struct _GSB_INT
{
	BYTE STATE;
} GSBINT;

typedef struct _IMC_INT
{
	BYTE PGEINT;
	BYTE ENCINT;
	BYTE LDIOINT;
	BYTE TIMERINT;
	BYTE ADCINT;
	BYTE PCLINT;
	BYTE GSBINT;
} IMCINT;

typedef void(IMC_LIB_CALL *PGEISR)(PGEINT *p);
typedef void(IMC_LIB_CALL *ENCISR)(ENCINT *p);
typedef void(IMC_LIB_CALL *TMRISR)(TMRINT *p);
typedef void(IMC_LIB_CALL *PCLISR)(PCLINT *p);
typedef void(IMC_LIB_CALL *LIOISR)(LIOINT *p);
typedef void(IMC_LIB_CALL *RIOISR)(RIOINT *p);
typedef void(IMC_LIB_CALL *ADCISR)(ADCINT *p);
typedef void(IMC_LIB_CALL *GSBISR)(GSBINT *p);


IMC_LIB_API int     IMC_LIB_CALL IMC_GetInterruptCount(); 
                    
IMC_LIB_API BOOL    IMC_LIB_CALL IMC_OpenDevice(int nMode, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_CloseIfOpen(WORD wCardIndex = 0);
                    
IMC_LIB_API DWORD   IMC_LIB_CALL IMC_GLB_GetDeviceID(WORD wCardIndex = 0);
IMC_LIB_API DWORD   IMC_LIB_CALL IMC_GLB_GetHandShake(WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_GLB_SetHandShake(DWORD HandShake, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_GLB_SetInterruptHostCPU(int Host, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_GLB_SetInterruptMode(int Mode, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_GLB_SetInterruptMask(WORD MaskBit, WORD wCardIndex = 0);
IMC_LIB_API long    IMC_LIB_CALL IMC_GLB_GetInterruptSource(IMCINT *Source, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_GLB_ResetModule(WORD ModuleID, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_GLB_ResetPPC(WORD wCardIndex = 0);

IMC_LIB_API void    IMC_LIB_CALL IMC_GLB_SetInterruptPeriod(WORD Period, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_GLB_SetInterruptEnable(int IRQNumber, WORD wCardIndex = 0);

IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_Start(WORD Start, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_SetClockDivider(DWORD Divider, WORD wCardIndex = 0);/*DDA Clock period */
IMC_LIB_API DWORD   IMC_LIB_CALL IMC_PGE_GetClockDivider(WORD wCardIndex = 0);/*DDA Clock period */
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_SetClockNumber(DWORD Number, WORD wCardIndex = 0);/* DDA Clock number per DDA Cycle*/
IMC_LIB_API DWORD   IMC_LIB_CALL IMC_PGE_GetClockNumber(WORD wCardIndex = 0);/* DDA Clock number per DDA Cycle*/
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_SetOutputFormat(WORD Channel, WORD Format, WORD wCardIndex = 0);/* Format -> 0: Disable, 1 : P/D, 2 : CW/CCW, 3 : A/B*/
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_EnableOutAInverse(WORD Channel, WORD Inverse, WORD wCardIndex = 0);/* Inverse -> 0 : No-Inv, 1 : Inv*/
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_EnableOutBInverse(WORD Channel, WORD Inverse, WORD wCardIndex = 0);/* Inverse -> 0 : No-Inv, 1 : Inv*/
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_EnableOutABSwap(WORD Channel, WORD Swap, WORD wCardIndex = 0);/* Inverse -> 0 : No-Inv, 1 : Inv*/
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_SendPulse(WORD Channel, long Command, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_GetCurrentCommand(WORD Channel, long *lCommand, WORD wCardIndex = 0); 
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_CheckFifoEmpty(WORD Channel, WORD* wFlag, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_CheckFifoFull(WORD Channel, WORD* wFlag, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_EnableCycleInterrupt(WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_EnableStockInterrupt(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API DWORD   IMC_LIB_CALL IMC_PGE_GetInterruptSource(PGEINT *Source, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_GetPulseCounter(WORD Channel, long *lPulse, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_EnablePulseCounter(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_ClearPulseCounter(WORD Channel, WORD Clear, WORD wCardIndex = 0) ;
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_SetIPOTime(double ipotime, WORD wCardIndex = 0);
IMC_LIB_API double  IMC_LIB_CALL IMC_PGE_GetIPOTime(WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_SetISRFunction(PGEISR  myPGE_ISR, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_GetStockCount(WORD Channel, WORD *wStock, WORD wCardIndex = 0); 
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_SetStockThreshold(WORD wThreshold, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PGE_EraseFIFOCmd(WORD Channel, WORD EraseNumber, WORD wCardIndex = 0);


IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_SetCounter(WORD Channel, long value, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_SetComparator(WORD Channel, long value, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_SetCounterLatchMode(WORD Channel, WORD Mode, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_SetIndexLatchSource(WORD Channel, WORD Source, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_SetExternalLatchSource(WORD Channel, WORD Source, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_SetInputFormat(WORD Channel, WORD Format, WORD wCardIndex = 0);/* Format -> 0: Disable, 1 : P/D, 2 : CW/CCW, 3 : A/B*/
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_EnableInAInverse(WORD Channel, WORD Inverse, WORD wCardIndex = 0);/* Inverse -> 0 : No-Inv, 1 : Inv*/
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_EnableInBInverse(WORD Channel, WORD Inverse, WORD wCardIndex = 0);/* Inverse -> 0 : No-Inv, 1 : Inv*/
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_EnableInCInverse(WORD Channel, WORD Inverse, WORD wCardIndex = 0);/* Inverse -> 0 : No-Inv, 1 : Inv*/
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_EnableInABSwap(WORD Channel, WORD Swap, WORD wCardIndex = 0);/* Swap -> 0 : No-Inv, 1 : Inv*/
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_SetInputRate(WORD Channel, WORD Rate, WORD wCardIndex = 0);/* Rate -> 0/3 : X4, 1 : X1, 2 : X2*/
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_ClearCounter(WORD Channel, WORD Clear, WORD wCardIndex = 0);/* Claer -> 0: No-Clear, 1 : Clear*/
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_ClearLatchCounter(WORD Channel, WORD Clear, WORD wCardIndex = 0);/* Claer -> 0: No-Clear, 1 : Clear*/
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_ReadCounter(WORD Channel, long* lCounter, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_ReadLatchCounter(WORD Channel, long* lCounter, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_StartCounter(WORD Channel, WORD Start, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_EnableIndexInterrupt(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_EnableComparatorInterrupt(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API long    IMC_LIB_CALL IMC_ENC_GetInterruptSource(ENCINT *Source, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_SetISRFunction(ENCISR  myENC_ISR , WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ENC_GetIndexStatus(WORD Channel, WORD* wStatus, WORD wCardIndex = 0);
                    
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_SetPGain(WORD Channel, WORD Gain, WORD wCardIndex = 0);/* Gain -> 0 ~ 0xFF*/
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_SetIGain(WORD Channel, WORD Gain, WORD wCardIndex = 0);/* Gain -> 0 ~ 0xFF*/
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_SetDGain(WORD Channel, WORD Gain, WORD wCardIndex = 0);/* Gain -> 0 ~ 0xFF*/
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_SetFGain(WORD Channel, WORD Gain, WORD wCardIndex = 0);/* Gain -> 0 ~ 0xFF*/
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_ClearErrorCounter(WORD Channel, WORD Clear, WORD wCardIndex = 0);/*Clear -> 0: No-Clear, 1 : Clear*/
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_SetFeedbackMode(WORD Channel, WORD Mode, WORD wCardIndex = 0);/*Mode -> 0: Encoder, 1 : ADC*/
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_SetErrorCounterMode(WORD Channel, WORD Mode, WORD wCardIndex = 0);/*Mode -> 0: S/W, 1 : H/W*/
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_EnableCloseLoop(WORD Channel, WORD Enable, WORD wCardIndex = 0);/*Enable -> 0: Disable, 1 : Enable*/
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_ReadErrorCounter(WORD Channel, short* nError, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_ReadErrorVoltage(WORD Channel, short* Voltage, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_SetIClockDivider(DWORD Divider, WORD wCardIndex = 0);
IMC_LIB_API DWORD   IMC_LIB_CALL IMC_PCL_GetIClockDivider(WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_SetDClockDivider(DWORD Divider, WORD wCardIndex = 0);
IMC_LIB_API DWORD   IMC_LIB_CALL IMC_PCL_GetDClockDivider(WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_SetTemperatureCommand(WORD Channel, float Voltage, WORD wCardIndex = 0);
IMC_LIB_API float   IMC_LIB_CALL IMC_PCL_GetTemperatureCommand(WORD Channel, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_SetFrictionCompensation(WORD Channel, WORD Compensation, WORD wCardIndex = 0);
IMC_LIB_API WORD    IMC_LIB_CALL IMC_PCL_GetFrictionCompensation(WORD Channel, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_SetErrorThreshold(WORD Channel, int PlusThreshold, int MinusThreshold, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_EnablePlusOverflowInterrupt(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_EnableMinusOverflowInterrupt(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API long    IMC_LIB_CALL IMC_PCL_GetInterruptSource(PCLINT *Source, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_PCL_SetISRFunction(PCLISR  myPCL_ISR, WORD wCardIndex = 0);

IMC_LIB_API void    IMC_LIB_CALL IMC_DAC_EnableChannel(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_DAC_SelectSource(WORD Channel, WORD Source, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_DAC_GetOutputVoltage(WORD Channel, float* fVoltage, WORD wCardIndex = 0);/*0 : DAC, 1 : PCL*/
IMC_LIB_API void    IMC_LIB_CALL IMC_DAC_SetOutputVoltage(WORD Channel, float Voltage, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_DAC_SetTriggerVoltage(WORD Channel, float Voltage, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_DAC_SetTriggerSource(WORD Channel, DWORD Source, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_DAC_StartConverter(WORD Start, WORD wCardIndex = 0);
                    
IMC_LIB_API void    IMC_LIB_CALL IMC_ADC_EnableChannel(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ADC_GetInputVoltage(WORD Channel, float *pdfVoltage, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ADC_SetCompareVoltage(WORD Channel, float Voltage, WORD wCardIndex = 0);
IMC_LIB_API float   IMC_LIB_CALL IMC_ADC_GetCompareVoltage(WORD Channel, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ADC_SetCompareMode(WORD Channel, WORD Mode, WORD wCardIndex = 0);
IMC_LIB_API WORD    IMC_LIB_CALL IMC_ADC_GetCompareMode(WORD Channel, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ADC_StartConverter(WORD Start, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ADC_SetConverterMode(WORD Mode, WORD wCardIndex = 0); // Mode 0 : BIP/Differential, 1 : UNI/Differential, 2 : BIP/Single End, 3 : UNI/Single End
IMC_LIB_API long    IMC_LIB_CALL IMC_ADC_GetInterruptSource(ADCINT *Source, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_ADC_SetISRFunction(ADCISR myADC_ISR, WORD wCardIndex = 0);
                    
IMC_LIB_API long    IMC_LIB_CALL IMC_LIO_GetInterruptSource(LIOINT *Source, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetServoOn(WORD Channel, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetServoOff(WORD Channel, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_EnableServoOnOff(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetServoTriggerMode(WORD Channel, WORD Mode, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_GetPlusLimitLDIInput(DWORD *LDIState, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_GetPlusLimitStatus(WORD Channel, WORD *State, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_EnablePlusLimit(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetPlusLimitTriggerMode(WORD Channel, WORD Mode, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_EnablePlusLimitInterrupt(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_GetMinusLimitLDIInput(DWORD *LDIState, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_GetMinusLimitStatus(WORD Channel, WORD *State, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_EnableMinusLimit(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetMinusLimitTriggerMode(WORD Channel, WORD Mode, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_EnableMinusLimitInterrupt(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_GetHomeSensorLDIInput(DWORD *LDIState, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_GetHomeSensorStatus(WORD Channel, WORD *State, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_EnableHomeSensor(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetHomeSensorTriggerMode(WORD Channel, WORD Mode, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_EnableHomeSensorInterrupt(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetLedLightOn(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_EnableLedLight(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_GetLedLightOutput(DWORD *dwLDIState, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_GetLedLightStatus(WORD Channel, WORD *State, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetLedTriggerSource(WORD Channel, WORD Source, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetLedTriggerPeriod(WORD Period, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_EnableLedTrigger(WORD Channel, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetLedTriggerValue(WORD Channel, WORD Value, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_EnablePrdy(WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetMotionEnable(WORD OnOff, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_GetEmgcStopStatus(WORD *pwStatus, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetISRFunction(LIOISR myLIO_ISR, WORD wCardIndex = 0);
                    
IMC_LIB_API void    IMC_LIB_CALL IMC_TMR_SetISRFunction(TMRISR myTMR_ISR, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_TMR_SetTimerEnable(WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API WORD    IMC_LIB_CALL IMC_TMR_GetTimerEnable(WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_TMR_SetTimerIntEnable(WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API WORD    IMC_LIB_CALL IMC_TMR_GetTimerIntEnable(WORD wCardIndex = 0);
IMC_LIB_API long    IMC_LIB_CALL IMC_TMR_GetInterruptSource(TMRINT *Source, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_TMR_SetTimerClock(DWORD Clock, WORD wCardIndex = 0);
IMC_LIB_API DWORD   IMC_LIB_CALL IMC_TMR_ReadTimerClock(WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_TMR_SetTimer(float Period, WORD wCardIndex = 0);
IMC_LIB_API DWORD   IMC_LIB_CALL IMC_TMR_ReadTimerCount(WORD wCardIndex = 0);

IMC_LIB_API void    IMC_LIB_CALL IMC_WDG_EnableTimer(WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_WDG_SetTimerClock(DWORD clock, WORD wCardIndex = 0);
IMC_LIB_API DWORD   IMC_LIB_CALL IMC_WDG_ReadTimerClock(WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_WDG_SetTimer(DWORD Period, WORD wCardIndex = 0);//unit in micro-second
IMC_LIB_API void    IMC_LIB_CALL IMC_WDG_SetResetPeriod(DWORD clock, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_WDG_RefreshTimer(WORD wCardIndex = 0);

//  Remote Digital IO functions
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_GetInterruptSource(RIOINT *Source, WORD wCardIndex = 0);
                        
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_GetInputValue(WORD Set, WORD Slave, WORD Port, DWORD *Input, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_GetMasterStatus(WORD Set, WORD *Status, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_GetSlaveStatus(WORD Set, DWORD *Status, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_GetSlaveFailChannel(WORD Set, DWORD *Channel, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_GetTransStatus(WORD Set, WORD *Status, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_SetClockDivider(WORD Set, WORD Divider, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_SetInterruptType(WORD Set, WORD Slave, WORD Point, WORD Type, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_SetOutputValue(WORD Set, WORD Slave, WORD Port, DWORD value, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_SetTransErrorTimes(WORD Set, WORD Times, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_EnableInputInterrupt(WORD Set, WORD Slave, WORD Point, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_EnableTransInterrupt(WORD Set, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_EnableSlaveControl(WORD Set, WORD Slave, WORD Enable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_RIO_EnableSetControl(WORD Set, WORD Enable, WORD wCardIndex = 0);

IMC_LIB_API void    IMC_LIB_CALL IMC_DPR_ReadCommand(void *pData, int Size, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_DPR_WriteCommand(void *pData, int Size, WORD wCardIndex = 0);
IMC_LIB_API int     IMC_LIB_CALL IMC_DPR_ReadAck(WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_DPR_WriteAck(WORD wCardIndex = 0);
IMC_LIB_API int     IMC_LIB_CALL IMC_DPR_ReadReq(WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_DPR_WriteReq(WORD wCardIndex = 0);
IMC_LIB_API int     IMC_LIB_CALL IMC_DPR_ReadErrorCode(WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_DPR_WriteErrorCode(int nErrorCode, WORD wCardIndex = 0);

//For IMP TestPlatform
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetPlusLimitLDOOutput(DWORD dwLDOState, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetMinusLimitLDOOutput(DWORD dwLDOState, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetHomeSensorLDOOutput(DWORD dwLDOState, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetPrdy(WORD OnOff, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_EnableEmgcStop(WORD wEnable, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_SetEmgcStop(WORD OnOff, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_LIO_EnableMotionEnable(WORD Enable, WORD wCardIndex = 0);

IMC_LIB_API void    IMC_LIB_CALL IMC_GSB_SetISRFunction(GSBISR  myGSB_ISR, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_GSB_SendPulse(WORD Channel, long Pulse, WORD wCardIndex = 0);
IMC_LIB_API int     IMC_LIB_CALL IMC_GSB_InitialSetting(WORD MaxAxes, WORD wCardIndex = 0);
IMC_LIB_API int     IMC_LIB_CALL IMC_GSB_ReadErrorStatus(WORD MaxAxes, WORD wCardIndex = 0);
IMC_LIB_API int     IMC_LIB_CALL IMC_GSB_Initialize(WORD MaxAxes, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_GSB_BankSwitching(WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_GSB_SetServoOn(WORD Channel, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_GSB_SetServoOff(WORD Channel, WORD wCardIndex = 0);

IMC_LIB_API void    IMC_LIB_CALL IMC_GSB_SetControlMode(WORD Channel, WORD Mode, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_GSB_SpeedCommand(WORD Channel, long RPM, WORD wCardIndex = 0);
IMC_LIB_API long    IMC_LIB_CALL IMC_GSB_ReadPostionFeedback(WORD Channel, long *encoder, WORD wCardIndex = 0);
IMC_LIB_API void    IMC_LIB_CALL IMC_GSB_SetMaxAxis(WORD wMaxAxis, WORD wCardIndex = 0);

#ifdef __cplusplus
}
#endif


#endif /*_IMC_LIB_H_*/
