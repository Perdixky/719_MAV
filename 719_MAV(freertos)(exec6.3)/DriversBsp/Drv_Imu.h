#ifndef __DRV_IMU_H
#define __DRV_IMU_H

#include "stdio.h"
#include "stdint.h"
#include "719_Interface.h"
#include "Drv_wit_c_sdk.h"
#include "iic.h"

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

static void Delayms(uint16_t ucMs);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void AutoScanSensor(void);
static void ShowHelp(void);
void Get_ImuData_Init(void);
void Get_ImuData_Server(void);
#endif
