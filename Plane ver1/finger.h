#include <iostream>
#include <windows.h>
#include "C:\Twincat\AdsApi\TcAdsDll\Include\TcAdsDef.h"
#include "C:\Twincat\AdsApi\TcAdsDll\Include\TcAdsApi.h"

class Finger
{
public:
    Finger();
    void init();
    int initConnection();
    void initGripper();
    void setSpeed(unsigned short speed_);
    void setForce(unsigned short force_);
    void setMode(char mode_);
    void move(unsigned short pos_);
    void reset();
    void activate();
    void close();
    void open();
	short force;
private:
    AmsAddr* pAddr;
    unsigned long Rxadd0;
    unsigned long Txadd0;
    unsigned short port;
    unsigned long add1,add2;
    DWORD dwData;
    short pos, speed;
    char mode;
};