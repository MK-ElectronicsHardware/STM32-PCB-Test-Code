/// @file PressureSense.h
///
/// @brief Header file for PressureSense.c
///
/// @par COPYRIGHT NOTICE: (c) 2019 Analox Sensor Technology. All rights reserved.


#ifndef PRESSURE_SENSE_H
#define PRESSURE_SENSE_H

#include "stm32f1xx_hal.h"

#define PRESSURE_READ_ERROR_VALUE __UINT32_MAX__

extern I2C_HandleTypeDef hi2c1;
//I2C_HandleTypeDef hi2c2;

ErrorStatus PressureSense_Initialise(void);
int32_t PressureSense_ReadPressure(void);

#endif
