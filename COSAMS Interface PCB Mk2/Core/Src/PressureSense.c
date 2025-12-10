/// @file PressureSense.c
///
/// @brief Communicate with Pressure Sense device and calculate pressure accordingly
///
/// @par COPYRIGHT NOTICE: (c) 2021 Analox Group Ltd. All rights reserved.

// NOTE: The device used is the MS5803-02BA. See the device datasheet for a full explanation of its operation.
//       Calculations in this module are derived from the calculations given in the datasheet

#include "PressureSense.h"
#include "main.h"
//#include "fault.h"
//#include "TraceAssert.h"
//#include "cmsis_os.h"
//
#ifdef __cplusplus
extern "C"
#endif
	
//MODULE_ID(PRESSURE_SENSE_MODULE_ID)
	
#define STRLEN 15
	
// These defines allow the variable names from the device datasheet to map to the correct array indices
#define C1 1
#define C2 2
#define C3 3
#define C4 4
#define C5 5
#define C6 6
	
I2C_HandleTypeDef hi2c1;

const uint16_t PressureSenseAddress = 0x0077U;	// This is the logical address of the Pressure Sensor on the bus

// The pressure sensor uses 7 bit addressing. The HAL can do 7 bit addressing, but does so by chopping off
// the LSB. So the address needs to shift one bit to the left 
const uint16_t PressureSenseAddress_7Bit = PressureSenseAddress << 0x01U; 

static uint16_t CalibrationCoefficients[8] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };

static ErrorStatus ReadPressureDeviceCoefficients();
static ErrorStatus ReadRawData(uint8_t command, uint32_t* const rawResult);
static ErrorStatus ReadRawPressure(uint32_t* const rawPressureResult);
static ErrorStatus ReadRawTemperature(uint32_t* const rawTemperatureResult);
static int32_t CalculateTemperatureOffset(uint32_t const rawTemperature);
static int32_t CalculateCompensatedPressure(int32_t rawPressure, int32_t temperatureOffset);

/// <summary>
///  Initialise the Pressure Sense device
/// </summary>
/// <Return> ErrorStatus - SUCCESS if device is successfully initialised, otherwise ERROR <\Return>
ErrorStatus PressureSense_Initialise()
{
	ErrorStatus PressureSuccess = ERROR;
	// Making this command code constant throws a warning as the const qualifier will be discarded, but
	//  I still think it's the right thing to do.
	uint8_t CMD_Reset[1] = { 0x1E };
	
	// Reset the device and wait for it to reinitialise
	if(HAL_OK == HAL_I2C_Master_Transmit(&hi2c1, PressureSenseAddress_7Bit, CMD_Reset, 1, 30000))
	{
		//vTaskDelay(500u);
		HAL_Delay(1);//delay 1ms;
		// Attempt to read the calibration coefficients back from the device. These will be used
		//  to calculate pressure readings later
		PressureSuccess = ReadPressureDeviceCoefficients();
	}
	
	return (PressureSuccess);
}

/// <summary>
/// Get the current pressure measured in 0.01 mb (hundredths of a millibar) 
/// </summary>
/// <Return> int32_t - The pressure in 0.01mB. Not sure why it is signed, but that is what is provided in the device datasheet <\Return>
int32_t PressureSense_ReadPressure()
{
	ErrorStatus pressureSuccess = ERROR;
	
	uint32_t rawPressureReading = 0u;
	uint32_t rawTemperatureReading = 0u;
	int32_t temperatureOffset = 0;
	int32_t compensatedPressure = 0;
	
	// Attempt to read raw pressure and temperature from device
	pressureSuccess  = ReadRawPressure(&rawPressureReading);
	g_raw_pressure_sensor =  rawPressureReading;
	
	if (SUCCESS == pressureSuccess)
	{
		pressureSuccess  = ReadRawTemperature(&rawTemperatureReading);
		g_raw_temperature_sensor = rawTemperatureReading;
	}

	// If read was successful, then calculate the pressure in accordance with the device datasheet
	if(SUCCESS == pressureSuccess)
	{
		temperatureOffset = CalculateTemperatureOffset(rawTemperatureReading);
		compensatedPressure = CalculateCompensatedPressure(rawPressureReading, temperatureOffset);
	}
	else
	{
		compensatedPressure = PRESSURE_READ_ERROR_VALUE;
	}

	// Round to integer value
	compensatedPressure /= 100u;

	return (compensatedPressure);
}

/// <summary>
/// Get the factory set Calibration Coefficients from PROM on the device
/// </summary>
/// <Param = pCoefficients> pointer to memory where coefficients should be stored </Param>
/// <Return> ErrorStatus - SUCCESS if coefficients are successfully read, otherwise ERROR <\Return>
ErrorStatus ReadPressureDeviceCoefficients()
{
	HAL_StatusTypeDef HALStatus = HAL_OK;
	
	// Addresses in the device PROM where calibration coefficients are held.
	// Making this command set constant throws a warning as the const qualifier will be discarded, but
	//  I still think it's the right thing to do.
	static uint8_t PROM_Read[8] = { 0xA0U, 0xA2U, 0xA4U, 0xA6U, 0xA8U, 0xAAU, 0xACU, 0xAEU };
	
	uint8_t buffer[16] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };
	
	// Read PROM memory from device, 1 byte at a time
	for (uint8_t loop = 0; loop < 8; loop++)
	{
		HALStatus = HAL_I2C_Master_Transmit(&hi2c1, PressureSenseAddress_7Bit, &PROM_Read[loop], 1, 30000);
		if (HAL_OK == HALStatus)
		{
			HALStatus = HAL_I2C_Master_Receive(&hi2c1, PressureSenseAddress_7Bit, &buffer[loop * 2], 2, 30000);
		}
		if (HAL_OK != HALStatus)
		{
			break;
		}
	}
	
	if (HAL_OK == HALStatus)
	{
		// Reassemble the recieved bytes into 16 bit values
		for(uint8_t loop = 0 ; loop < 8 ; loop++)
		{
			uint8_t index = loop * 2u;
			
			// High byte
			uint16_t highLowByte = buffer[index];
			highLowByte <<= 8u;
			
			// Low byte
			highLowByte += buffer[index + 1u];
			
			CalibrationCoefficients[loop] = highLowByte;
		}
	}
	
	return ((HAL_OK == HALStatus) ? SUCCESS : ERROR);
}

/// <summary>
/// Read the raw temperature value from the device 
/// </summary>
/// <Param = rawTemperatureResult> Pointer to memory where value should be stored </Param>
/// <Return> ErrorStatus - SUCCESS if value is successfully read, otherwise ERROR <\Return>
ErrorStatus ReadRawTemperature(uint32_t* const rawTemperatureResult)
{
	const uint8_t readTemperatureCommand = 0x58U;
	
	ErrorStatus readSuccess = ReadRawData(readTemperatureCommand, rawTemperatureResult);
		
	return (readSuccess);
}	

/// <summary>
/// Read the raw pressure value from the device 
/// </summary>
/// <Param = rawPressureResult> Pointer to memory where value should be stored </Param>
/// <Return> ErrorStatus - SUCCESS if value is successfully read, otherwise ERROR <\Return>
ErrorStatus ReadRawPressure(uint32_t* const rawPressureResult)
{
	const uint8_t readTemperatureCommand = 0x48U;
	
	ErrorStatus readSuccess = ReadRawData(readTemperatureCommand, rawPressureResult);
		
	return (readSuccess);
}	

/// <summary>
/// Read a raw value from the ADC on the device
/// </summary>
/// <Param = command> Read command to be sent to the device </Param>
/// <Param = rawResult> pointer to memory where the value should be stored </Param>
/// <Return> ErrorStatus - SUCCESS if value is successfully read, otherwise ERROR <\Return>
ErrorStatus ReadRawData(uint8_t command, uint32_t* const rawResult)
{
	//trace_assert((command == 0x48U) || (command == 0x58U)); //only high accuracy pressure read or temperature read commands
	
	// Command code to read from ADC
	// Making this command set constant throws a warning as the const qualifier will be discarded, but
	//  I still think it's the right thing to do.
	static uint8_t CMD_ADC_Read = 0x00;
	
	uint8_t buffer[3] = { 0U, 0U, 0U };
	ErrorStatus readSuccess = SUCCESS;
	HAL_StatusTypeDef HalStatus = HAL_OK;
	
	// Set the Sensor to the correct conversion mode
	HalStatus = HAL_I2C_Master_Transmit(&hi2c1, PressureSenseAddress_7Bit, &command, 1, 30000);
	
	HAL_Delay(100); //give the conversion time to complete
	
	// Set the Sensor to ADC read mode
	if (HalStatus == HAL_OK)
	{
		HAL_I2C_Master_Transmit(&hi2c1, PressureSenseAddress_7Bit, &CMD_ADC_Read, 1, 30000);
	}
	
	// Read the data from the ADC
	if (HalStatus == HAL_OK)
	{
		HalStatus = HAL_I2C_Master_Receive(&hi2c1, PressureSenseAddress_7Bit, buffer, 3, 30000);
	}
	
	// If everything was successful, assemble the reading from the buffer
	if (HalStatus == HAL_OK)
	{
		*rawResult = (buffer[0] << 16u);
		*rawResult += (buffer[1] << 8u);
		*rawResult += buffer[2];
		
		readSuccess = SUCCESS;
	}
	else
	{
		*rawResult = 0U;
		readSuccess = ERROR;
	}
	
	return (readSuccess);
}

/// <summary>
/// Calculate the temperature offset for the pressure sensor. 
/// This will be needed to calculate the adjusted pressure
/// - The calculation here is taken from the device datasheet
/// </summary>
/// <Param = rawTemperature> the raw temperature value from which to calculate the offset </Param>
/// <Return> int32_t - the temperature offset value. Not sure why it's signed, but that is what is given on the device datasheet <\Return>
int32_t CalculateTemperatureOffset(uint32_t rawTemperature)
{
	const int32_t referenceMultiplier = 1u << 8u; // 2^8 taken from datasheet

	// First step in the calculation of pressure, is the measure the ambient temperature of the pressure sense device	
	int32_t referenceTemperature = (int32_t)CalibrationCoefficients[C5];	// referred to as 'C5' or 'Tref' in the datasheet
	int32_t temperatureOffset = rawTemperature - (referenceTemperature * referenceMultiplier);	// referred to as 'dT' in the device datasheet
	
	return (temperatureOffset);
}

/// <summary>
/// Calculate the temperature compensated pressure
/// - The calculation here is taken from the device datasheet
/// </summary>
/// <Param = rawPressure> the raw pressure value from the device</Param>
/// <Param = temperatureOffset> the temperature offset value, as calculated in accordance with the device datasheet</Param>
/// <Return> int32_t - the compensated temperature value. Not sure why it's signed, but that is what is given on the device datasheet <\Return>
int32_t CalculateCompensatedPressure(int32_t rawPressure, int32_t temperatureOffset)
{
	const int64_t pressureOffsetMultiplier = 1u << 17u;			// 2^17 taken from datasheet
	const int64_t coefficientOfPressureDivider = 1u << 6u;		// 2^6 taken from datasheet
	const int64_t pressureSensitivityMultiplier = 1u << 16u;	// 2^16 taken from datasheet
	const int64_t coefficientOfSensitivityDivider = 1u << 7u;	// 2^7 taken from datasheet
	const int64_t sensitivityDivider = 1u << 21u;				// 2^21 taken from datasheet
	const int64_t finalPressureDivider = 1u << 15u;				// 2^15 taken from datasheet
	
	
	int64_t pressureOffsetAtTemperature = (int64_t)CalibrationCoefficients[C2] * pressureOffsetMultiplier + (((int64_t)CalibrationCoefficients[C4] * (int64_t)temperatureOffset) / coefficientOfPressureDivider);             // referred to as 'OFF' on datasheet		
	int64_t pressureSensitivityAtTemperature = (int64_t)CalibrationCoefficients[C1] * pressureSensitivityMultiplier + (((int64_t)CalibrationCoefficients[C3] * (int64_t)temperatureOffset) / coefficientOfSensitivityDivider);        // referred to as 'SENS' on the datasheet
	int64_t temperatureCompensatedPressure = (((int64_t)rawPressure * (pressureSensitivityAtTemperature / sensitivityDivider)) - pressureOffsetAtTemperature) / finalPressureDivider;
	
	//trace_assert(temperatureCompensatedPressure < INT32_MAX);
	
	int32_t actualPressure = (int32_t)temperatureCompensatedPressure;
	
	return (actualPressure);
}
