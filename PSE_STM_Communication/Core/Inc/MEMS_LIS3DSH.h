
#ifndef MEMS_LIS3DSH_H_
#define MEMS_LIS3DSH_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* LIS3DSH Addresses ---------------------------------------------------------*/
#define LIS3DSH_WHO_AM_I_ADDR				0x0F
#define LIS3DSH_OFF_X_ADDR   				0x10
#define LIS3DSH_OFF_Y_ADDR   				0x11
#define LIS3DSH_OFF_Z_ADDR   				0x12

#define LIS3DSH_CTRL_REG1_ADDR				0x21
#define LIS3DSH_CTRL_REG2_ADDR				0x22
#define LIS3DSH_CTRL_REG3_ADDR				0x23
#define LIS3DSH_CTRL_REG4_ADDR				0x20
#define LIS3DSH_CTRL_REG5_ADDR				0x24
#define LIS3DSH_CTRL_REG6_ADDR				0x25

#define LIS3DSH_STATUS_ADDR					0x27

#define LIS3DSH_OUT_X_L_ADDR				0x28
#define LIS3DSH_OUT_X_H_ADDR				0x29
#define LIS3DSH_OUT_Y_L_ADDR				0x2A
#define LIS3DSH_OUT_Y_H_ADDR				0x2B
#define LIS3DSH_OUT_Z_L_ADDR				0x2C
#define LIS3DSH_OUT_Z_H_ADDR				0x2D

/* Controle Register 4 -----------------------------------------------------------*/
	// Enable Axis
#define LIS3DSH_AXES_MASK					((uint8_t)0x07)
#define LIS3DSH_X_ENABLE  					((uint8_t)0x01)
#define LIS3DSH_Y_ENABLE  					((uint8_t)0x02)
#define LIS3DSH_Z_ENABLE  					((uint8_t)0x04)
#define LIS3DSH_XYZ_ENABLE					((uint8_t)0x07)

	// Output Data Rate (ODR) Options
#define LIS3DSH_ODR_MASK					((uint8_t)0xF0)
#define LIS3DSH_ODR_POWERDOWN				((uint8_t)0x00)  /* Power Down Mode*/
#define LIS3DSH_ODR_3_125    				((uint8_t)0x10)  /* ODR 3.125 Hz */
#define LIS3DSH_ODR_6_25     				((uint8_t)0x20)  /* ODR 6.25  Hz */
#define LIS3DSH_ODR_12_5     				((uint8_t)0x30)  /* ODR 12.5  Hz */
#define LIS3DSH_ODR_25       				((uint8_t)0x40)  /* ODR 25    Hz */
#define LIS3DSH_ODR_50       				((uint8_t)0x50)  /* ODR 50    Hz */
#define LIS3DSH_ODR_100      				((uint8_t)0x60)  /* ODR 100   Hz */
#define LIS3DSH_ODR_400      				((uint8_t)0x70)  /* ODR 400   Hz */
#define LIS3DSH_ODR_800      				((uint8_t)0x80)  /* ODR 800   Hz */
#define LIS3DSH_ODR_1600     				((uint8_t)0x90)  /* ODR 1600  Hz */

/* Controle Register 5 -----------------------------------------------------------*/
	// Full scale
#define LIS3DSH_FSCALE_MASK					((uint8_t)0x38)
#define LIS3DSH_FSCALE_2 					((uint8_t)0x00)  /* ±2  g  */
#define LIS3DSH_FSCALE_4 					((uint8_t)0x08)  /* ±4  g  */
#define LIS3DSH_FSCALE_6 					((uint8_t)0x10)  /* ±6  g  */
#define LIS3DSH_FSCALE_8 					((uint8_t)0x18)  /* ±8  g  */
#define LIS3DSH_FSCALE_16					((uint8_t)0x20)  /* ±16 g  */
	
	// Sensitivity values (Related with Full scale)
#define LIS3DSH_SENSITIVITY_0_06G            0.06  /* 0.06 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_12G            0.12  /* 0.12 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_18G            0.18  /* 0.18 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_24G            0.24  /* 0.24 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_73G            0.73  /* 0.73 mg/digit*/

#define GRAVITY_VALUE						 9.80665

/* Typedef Structs -----------------------------------------------------------*/

// Config Struct
typedef struct 
{
	uint8_t dataRate;
	uint8_t fullScale;
	uint8_t enableAxes;
}MEMS_Config_T;

// Data Struct
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}MEMS_DataRaw_T;

typedef struct
{
	float x;
	float y;
	float z;
}MEMS_DataScaled_T;

// Calibration Struct
typedef struct 
{
	float Bias;
	float Scale;
}MEMS_AxesCalibration_T;

typedef struct 
{
	MEMS_AxesCalibration_T x;
	MEMS_AxesCalibration_T y;
	MEMS_AxesCalibration_T z;
}MEMS_Calibration_T;

/* Function prototypes -----------------------------------------------*/
void MEMS_Init(SPI_HandleTypeDef *extSPI_Handle, MEMS_Config_T *memsConfig);
void MEMS_WriteReg(uint8_t addr, uint8_t *data, uint8_t size);
void MEMS_ReadReg(uint8_t addr, uint8_t *data, uint8_t size);
void selectSensitivity(uint8_t fullScale);

int16_t MEMS_GetAxesData(uint8_t addr);
MEMS_DataRaw_T MEMS_GetDataRaw(void);
MEMS_DataScaled_T MEMS_GetDataScaled(void);
MEMS_DataScaled_T MEMS_GetDataMS2(void);

void MEMS_X_calibrate(float x_min, float x_max);
void MEMS_Y_calibrate(float y_min, float y_max);
void MEMS_Z_calibrate(float z_min, float z_max);

#endif

