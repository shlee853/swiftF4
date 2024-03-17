/*
 * sensors_icm20602.h
 *
 *  Created on: Mar 9, 2024
 *      Author: swift
 */

#ifndef INC_DRIVERS_SENSORS_ICM20602_H_
#define INC_DRIVERS_SENSORS_ICM20602_H_

#pragma once


#include "sensors.h"
void sensorsIcm20602Init_SPI(void);

void sensorsIcm20602Init(void);
bool sensorsIcm20602Test(void);
bool sensorsIcm20602AreCalibrated(void);
bool sensorsIcm20602ManufacturingTest(void);
void sensorsIcm20602Acquire(sensorData_t *sensors);
void sensorsIcm20602WaitDataReady(void);
bool sensorsIcm20602ReadGyro(Axis3f *gyro);
bool sensorsIcm20602ReadAcc(Axis3f *acc);
bool sensorsIcm20602ReadMag(Axis3f *mag);
bool sensorsIcm20602ReadBaro(baro_t *baro);
void sensorsIcm20602SetAccMode(accModes accMode);
void sensorsIcm20602DataAvailableCallback(void);

#endif /* INC_DRIVERS_SENSORS_ICM20602_H_ */


