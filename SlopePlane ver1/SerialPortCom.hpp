#ifndef _SERIALPORTCOM_HPP
#define _SERIALPORTCOM_HPP
//#include "stdafx.h"
#include <cstring>
#include <string>
#include <iostream>
#include <windows.h>
#include<time.h>

using namespace std;

/**
 * SerialPortCom:
 *   In the case of RenBot, the two wheel 
 *   motors are controlled by only one serial port 
 */

class SerialPortCom
{
public:
	SerialPortCom()
	{
	}
	SerialPortCom(string port_name, int baud_rate)
	{
		openPort(port_name, baud_rate);
	}
public:
	bool openPort(string port_name, int baud_rate);
	bool writePort(string str);
	string readPort();
protected:
	HANDLE  _hComm;
};
#endif