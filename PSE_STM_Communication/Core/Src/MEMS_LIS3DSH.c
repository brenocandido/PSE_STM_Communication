
/* Includes ------------------------------------------------------------------*/
#include "MEMS_LIS3DSH.h"

/* Private define ------------------------------------------------------------*/
	
	// SPI Chip Select
#define MEMS_CS_ENABLE		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
#define MEMS_CS_DISABLE		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

/* Private variables ---------------------------------------------------------*/
	
	// SPI Handle
static SPI_HandleTypeDef SPI_Handle;

	// Sensitivity
static float mems_Sensitivity;

	// Bias variables
MEMS_Calibration_T calibration;

/* Private functions ---------------------------------------------------------*/

void MEMS_WriteReg(uint8_t addr, uint8_t *data, uint8_t size)
{
	uint8_t spiReg = addr;

	MEMS_CS_ENABLE;
	HAL_SPI_Transmit(&SPI_Handle, &spiReg, 1, 10);
	HAL_SPI_Transmit(&SPI_Handle, data, size, 10);
	MEMS_CS_DISABLE;
}

void MEMS_ReadReg(uint8_t addr, uint8_t *data, uint8_t size)
{
	uint8_t spiBuf[4];
	spiBuf[0] = addr | 0x80;

	MEMS_CS_ENABLE;
	HAL_SPI_Transmit(&SPI_Handle, spiBuf, 1, 10);
	HAL_SPI_Receive(&SPI_Handle, spiBuf, size, 10);
	MEMS_CS_DISABLE;
	
	for(uint8_t i=0; i<(size&0x3); i++)
	{
		data[i] = spiBuf[i];
	}
}

void selectSensitivity(uint8_t fullScale)
{
	switch(fullScale)
	{
		case LIS3DSH_FSCALE_2:
			mems_Sensitivity = LIS3DSH_SENSITIVITY_0_06G;
			break;
		
		case LIS3DSH_FSCALE_4:
			mems_Sensitivity = LIS3DSH_SENSITIVITY_0_12G;
			break;
		
		case LIS3DSH_FSCALE_6:
			mems_Sensitivity = LIS3DSH_SENSITIVITY_0_18G;
			break;
		
		case LIS3DSH_FSCALE_8:
			mems_Sensitivity = LIS3DSH_SENSITIVITY_0_24G;
			break;
		
		case LIS3DSH_FSCALE_16:
			mems_Sensitivity = LIS3DSH_SENSITIVITY_0_73G;
			break;
	}
}

void MEMS_Init(SPI_HandleTypeDef *extSPI_Handle, MEMS_Config_T *memsConfig)
{
	uint8_t spiData = 0;
	
	// Copy Handle from principal application
	memcpy(&SPI_Handle, extSPI_Handle, sizeof(*extSPI_Handle));

	// Init Configuration (Control Register 4)
		// Enable Axes
	spiData |= (memsConfig->enableAxes & LIS3DSH_AXES_MASK);
		// Output Data Rate
	spiData |= (memsConfig->dataRate & LIS3DSH_ODR_MASK);
		//Write to accelerometer
	MEMS_WriteReg(LIS3DSH_CTRL_REG4_ADDR, &spiData, 1);
	
	// Init Configuration (Control Register 5)
	spiData = 0;
		// Full Scale
	spiData |= (memsConfig->fullScale & LIS3DSH_FSCALE_MASK);
		//Write to accelerometer
	MEMS_WriteReg(LIS3DSH_CTRL_REG5_ADDR, &spiData, 1);
		// Select Sensitivity based in Full Scale
	selectSensitivity(memsConfig->fullScale);

	// Default Calibration
	calibration.x.Scale = 1.0f;
	calibration.y.Scale = 1.0f;
	calibration.z.Scale = 1.0f;

	calibration.x.Bias = 0.0f;
	calibration.x.Bias = 0.0f;
	calibration.x.Bias = 0.0f;
}

int16_t MEMS_GetAxesData(uint8_t addr)
{
	uint8_t spiBuf[2]; // Measure have MSB and LSB
	int16_t DataRaw;

	MEMS_ReadReg(addr, spiBuf, 2);
	DataRaw = ((spiBuf[1] << 8) + spiBuf[0]);

	return DataRaw;
}

MEMS_DataRaw_T MEMS_GetDataRaw(void)
{
	MEMS_DataRaw_T tempDataRaw;
	
	// Get XYZ data
	tempDataRaw.x = MEMS_GetAxesData(LIS3DSH_OUT_X_L_ADDR);
	tempDataRaw.y = MEMS_GetAxesData(LIS3DSH_OUT_Y_L_ADDR);
	tempDataRaw.z = MEMS_GetAxesData(LIS3DSH_OUT_Z_L_ADDR);

	return tempDataRaw;
}

MEMS_DataScaled_T MEMS_GetDataScaled(void)
{
	// Read raw data
	MEMS_DataRaw_T tempRawData = MEMS_GetDataRaw();
	
	// Scale
	MEMS_DataScaled_T tempScaledData;
	tempScaledData.x = (tempRawData.x * mems_Sensitivity * calibration.x.Scale) + 0.0f - calibration.x.Bias;
	tempScaledData.y = (tempRawData.y * mems_Sensitivity * calibration.y.Scale) + 0.0f - calibration.y.Bias;
	tempScaledData.z = (tempRawData.z * mems_Sensitivity * calibration.z.Scale) + 0.0f - calibration.z.Bias;
	
	return tempScaledData;
}

void MEMS_X_calibrate(float x_min, float x_max)
{
	calibration.x.Bias	= (x_max+x_min)/2.0f;
	calibration.x.Scale = (2*1000)/(x_max - x_min);
}

void MEMS_Y_calibrate(float y_min, float y_max)
{
	calibration.y.Bias	= (y_max+y_min)/2.0f;
	calibration.y.Scale = (2*1000)/(y_max - y_min);
}

void MEMS_Z_calibrate(float z_min, float z_max)
{
	calibration.z.Bias	= (z_max+z_min)/2.0f;
	calibration.z.Scale = (2*1000)/(z_max - z_min);
}

