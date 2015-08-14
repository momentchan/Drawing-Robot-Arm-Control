#include "finger.h"

Finger::Finger()
{
    pAddr = new AmsAddr;
    Rxadd0 = 0x11004;//need to check the adderess in TwinCAT
    Txadd0 = 0x12004;//need to check the adderess in TwinCAT
    //Rxadd0 = 0x3040010;
	//Txadd0 = 0x3040010;
	port = 300;
    pos, speed, force = 255;
    mode = 'b';
}
void Finger::init()
{
    initConnection();
    initGripper();
}
int Finger::initConnection()
{
    int nErr = 0;
    AdsPortOpen();
    nErr = AdsGetLocalAddress(pAddr);
    pAddr->port = port;
    
    return nErr;
}
void Finger::initGripper()
{
    reset();//Reset the gripper    
    activate();//Activate the gripper
    
    Sleep(4000);//need about 4 sec to initialize
    
    setMode('b');//switch to basic mode
    setSpeed(255);//use the max speed
    setForce(255);//use the max force    
    close();//close gripper
}

void Finger::setSpeed(unsigned short speed_)
{
    speed = speed_;
    
    add1 = 0x2B; add2 = 0x01;// byte 4
    dwData = speed;
    AdsSyncWriteReq(pAddr,Rxadd0,add1,add2,&dwData);
}
void Finger::setForce(unsigned short force_)
{
    force = force_;
    
    add1 = 0x2C; add2 = 0x01;// byte 5
    dwData = force;
    AdsSyncWriteReq(pAddr,Rxadd0,add1,add2,&dwData);
}
void Finger::setMode(char mode_)
{
    mode = mode_;
    switch(mode)
    {
	case 'b':
	    add1 = 0x27; add2 = 0x01;//byte 0
	    dwData = 0x09;
	    AdsSyncWriteReq(pAddr,Rxadd0,add1,add2,&dwData);
	    break;
	case 'p':
	    add1 = 0x27; add2 = 0x01;//byte 0
	    dwData = 0x0B;
	    AdsSyncWriteReq(pAddr,Rxadd0,add1,add2,&dwData);
	    break;
	case 'w':
	    add1 = 0x27; add2 = 0x01;//byte 0
	    dwData = 0x0D;
	    AdsSyncWriteReq(pAddr,Rxadd0,add1,add2,&dwData);
	    break;
	case 's':
	    add1 = 0x27; add2 = 0x01;//byte 0
	    dwData = 0x0F;
	    AdsSyncWriteReq(pAddr,Rxadd0,add1,add2,&dwData);
	    break;
    }
}
void Finger::move(unsigned short pos_)
{
    pos = pos_;
    
    add1 = 0x2A; add2 = 0x01;// byte 3
    dwData = pos;
    AdsSyncWriteReq(pAddr,Rxadd0,add1,add2,&dwData);
}
void Finger::reset()
{
    add1 = 0x27; add2 = 0x01;//byte 0
    dwData = 0x00;//Reset gripper
    AdsSyncWriteReq(pAddr,Rxadd0,add1,add2,&dwData);
}
void Finger::activate()
{
    add1 = 0x27; add2 = 0x01;//byte 0
    dwData = 0x01;//Activate gripper
    AdsSyncWriteReq(pAddr,Rxadd0,add1,add2,&dwData);
}
void Finger::close()
{
    move(255);
}
void Finger::open()
{
    move(0);
}