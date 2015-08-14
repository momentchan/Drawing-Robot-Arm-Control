#ifndef _IMPCARD_H
#define _IMPCARD_H

#include "IMCDefine.h"
#include "IMCDriver.h"
#include "MotorDefine.h"

#define TOTAL_CHANNELS 8
#define USAGE_CHANNELS 7

extern long qm_ENC[USAGE_CHANNELS];
//extern long diff_qm_ENC[USAGE_CHANNELS];

extern long ENC7;

extern float SetValue[USAGE_CHANNELS];

void Init_IMPCard();
void Init_DA();
void Init_Encoder();
void Set_ENC_Home();
void Close_IMPCard();

void ReadEncoder();
void SetOutputDA();

#endif