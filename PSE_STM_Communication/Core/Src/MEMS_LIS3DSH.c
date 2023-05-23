
/* Includes ------------------------------------------------------------------*/
#include "MEMS_LIS3DSH.h"

/* Private define ------------------------------------------------------------*/
	
	// SPI Chip Select
#define MEMS_CS_ENABLE		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
#define MEMS_CS_DISABLE		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

/* Private variables ---------------------------------------------------------*/
	
/* Private functions ---------------------------------------------------------*/

void MEMS_WriteReg(SPI_HandleTypeDef *pSPI, uint8_t addr, uint8_t *data, uint8_t size)
{
	uint8_t spiReg = addr;

	MEMS_CS_ENABLE;
	HAL_SPI_Transmit(pSPI, &spiReg, 1, 10);
	HAL_SPI_Transmit(pSPI, data, size, 10);
	MEMS_CS_DISABLE;
}

void MEMS_ReadReg(SPI_HandleTypeDef *pSPI, uint8_t addr, uint8_t *data, uint8_t size)
{
	uint8_t spiBuf[4];
	spiBuf[0] = addr | 0x80;

	MEMS_CS_ENABLE;
	HAL_SPI_Transmit(pSPI, spiBuf, 1, 10);
	HAL_SPI_Receive(pSPI, spiBuf, size, 10);
	MEMS_CS_DISABLE;
	
	for(uint8_t i=0; i<(size&0x3); i++)
	{
		data[i] = spiBuf[i];
	}
}

void selectSensitivity(MEMS_Config_t *pConfig)
{
	switch(pConfig->fullScale)
	{
		case LIS3DSH_FSCALE_2:
			pConfig->calibration.Sensitivity = LIS3DSH_SENSITIVITY_0_06G;
			break;
		
		case LIS3DSH_FSCALE_4:
			pConfig->calibration.Sensitivity = LIS3DSH_SENSITIVITY_0_12G;
			break;
		
		case LIS3DSH_FSCALE_6:
			pConfig->calibration.Sensitivity = LIS3DSH_SENSITIVITY_0_18G;
			break;
		
		case LIS3DSH_FSCALE_8:
			pConfig->calibration.Sensitivity = LIS3DSH_SENSITIVITY_0_24G;
			break;
		
		case LIS3DSH_FSCALE_16:
			pConfig->calibration.Sensitivity = LIS3DSH_SENSITIVITY_0_73G;
			break;
	}
}

void MEMS_Init(MEMSHandler_t *pHandler, SPI_HandleTypeDef *pSPI_ext, MEMS_Config_t pConfig)
{
	uint8_t spiData = 0;

	// Set SPI
	pHandler->pSPI = pSPI_ext;

	// Set Config
	pHandler->config = pConfig;

	// Init Configuration (Control Register 4)
		// Enable Axes
	spiData |= (pHandler->config.enableAxes & LIS3DSH_AXES_MASK);
		// Output Data Rate
	spiData |= (pHandler->config.dataRate & LIS3DSH_ODR_MASK);
		//Write to accelerometer
	MEMS_WriteReg(pHandler->pSPI, LIS3DSH_CTRL_REG4_ADDR, &spiData, 1);
	
	// Init Configuration (Control Register 5)
	spiData = 0;
		// Full Scale
	spiData |= (pHandler->config.fullScale & LIS3DSH_FSCALE_MASK);
		//Write to accelerometer
	MEMS_WriteReg(pHandler->pSPI,LIS3DSH_CTRL_REG5_ADDR, &spiData, 1);
		// Select Sensitivity based in Full Scale
	selectSensitivity(&(pHandler->config));

	// Default Calibration
	pHandler->config.calibration.x.Scale = 1.0f;
	pHandler->config.calibration.y.Scale = 1.0f;
	pHandler->config.calibration.z.Scale = 1.0f;

	pHandler->config.calibration.x.Bias = 0.0f;
	pHandler->config.calibration.y.Bias = 0.0f;
	pHandler->config.calibration.z.Bias = 0.0f;
}

int16_t MEMS_GetAxesData(SPI_HandleTypeDef *pSPI, uint8_t addr)
{
	uint8_t spiBuf[2]; // Measure have MSB and LSB
	int16_t DataRaw;

	MEMS_ReadReg(pSPI, addr, spiBuf, 2);
	DataRaw = ((spiBuf[1] << 8) + spiBuf[0]);

	return DataRaw;
}

MEMS_DataRaw_t MEMS_GetDataRaw(MEMSHandler_t *pHandler)
{
	MEMS_DataRaw_t tempDataRaw;
	
	// Get XYZ data
	tempDataRaw.x = MEMS_GetAxesData(pHandler->pSPI,LIS3DSH_OUT_X_L_ADDR);
	tempDataRaw.y = MEMS_GetAxesData(pHandler->pSPI,LIS3DSH_OUT_Y_L_ADDR);
	tempDataRaw.z = MEMS_GetAxesData(pHandler->pSPI,LIS3DSH_OUT_Z_L_ADDR);

	return tempDataRaw;
}

MEMS_DataScaled_t MEMS_GetDataScaled(MEMSHandler_t *pHandler)
{
	// Read raw data
	MEMS_DataRaw_t tempRawData = MEMS_GetDataRaw(pHandler);
	
	// Scale
	MEMS_DataScaled_t tempScaledData;
	tempScaledData.x = (tempRawData.x * pHandler->config.calibration.Sensitivity  * pHandler->config.calibration.x.Scale) +
						0.0f - pHandler->config.calibration.x.Bias;
	tempScaledData.y = (tempRawData.y * pHandler->config.calibration.Sensitivity  * pHandler->config.calibration.y.Scale) +
						0.0f - pHandler->config.calibration.y.Bias;
	tempScaledData.z = (tempRawData.z * pHandler->config.calibration.Sensitivity  * pHandler->config.calibration.z.Scale) +
						0.0f - pHandler->config.calibration.z.Bias;
	
	return tempScaledData;
}

MEMS_DataScaled_t MEMS_GetDataMS2(MEMSHandler_t *pHandler)
{
	// Read Scaled data
	MEMS_DataScaled_t tempData = MEMS_GetDataScaled(pHandler);
	
	// Scale
	tempData.x = (tempData.x * GRAVITY_VALUE) / 1000.0f;
	tempData.y = (tempData.y * GRAVITY_VALUE) / 1000.0f;
	tempData.z = (tempData.z * GRAVITY_VALUE) / 1000.0f;
	
	return tempData;
}

void MEMS_X_calibrate(MEMSHandler_t *pHandler, float x_min, float x_max)
{
	pHandler->config.calibration.x.Bias	= (x_max+x_min)/2.0f;
	pHandler->config.calibration.x.Scale = (2*1000)/(x_max - x_min);
}

void MEMS_Y_calibrate(MEMSHandler_t *pHandler, float y_min, float y_max)
{
	pHandler->config.calibration.y.Bias	= (y_max+y_min)/2.0f;
	pHandler->config.calibration.y.Scale = (2*1000)/(y_max - y_min);
}

void MEMS_Z_calibrate(MEMSHandler_t *pHandler, float z_min, float z_max)
{
	pHandler->config.calibration.z.Bias	= (z_max+z_min)/2.0f;
	pHandler->config.calibration.z.Scale = (2*1000)/(z_max - z_min);
}

