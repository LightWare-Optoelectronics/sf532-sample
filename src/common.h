#pragma once

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <vector>

#include "lwSerialPort.h"

struct lwSerialPortInfo {
	char deviceName[256];
	char vendorName[256];
	char serial[256];
	char vendorId[256];
	char productId[256];
};

//----------------------------------------------------------------------------------------------------------------------------------
// Platform specific functions.
//----------------------------------------------------------------------------------------------------------------------------------
#ifdef __linux__
	#include "./linux/platformLinux.h"
#else
	#include "./win32/platformWin32.h"
#endif

//----------------------------------------------------------------------------------------------------------------------------------
// Helper utilities.
//----------------------------------------------------------------------------------------------------------------------------------
void printHexDebug(uint8_t* Data, uint32_t Size);
