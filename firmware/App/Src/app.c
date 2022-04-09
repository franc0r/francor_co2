/**
 * @file app.cpp
 * @author Martin Bauernschmitt <martin.bauernschmitt@francor.de>
 * @brief Main application functions of CO2 sensor
 *
 * @version 1.0
 * @date 2022-04-09
 *
 * @copyright Copyright (c) 2022 - BSD-3-clause - FRANCOR e.V.
 *
 */

#include <string.h>

#include "app.h"
#include "bme680.h"
#include "stm32f1xx_hal.h"

/* Private macros ------------------------------------------------------------*/

/* Extern variables ----------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;

/* Private variables ---------------------------------------------------------*/
static struct bme680_dev hsensor;

/* Private function prototypes -----------------------------------------------*/

static void initSensor(void);
static void configSensor(void);

int8_t readSensorRegisters(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t size);
int8_t writeSensorRegisters(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t size);

/* Public functions ----------------------------------------------------------*/

void initApp(void)
{
   initSensor();
   configSensor();
}

void runApp(void)
{

}

/* Private functions ---------------------------------------------------------*/

static void initSensor(void)
{
   hsensor.dev_id = BME680_I2C_ADDR_PRIMARY;
   hsensor.intf = BME680_I2C_INTF;
   hsensor.read = readSensorRegisters;
   hsensor.write = writeSensorRegisters;
   hsensor.delay_ms = HAL_Delay;
   hsensor.amb_temp = APP_DFT_AMBIENT_TEMP;

   // TODO Init sensor
}

static void configSensor(void)
{

}

/* BME680 i2c interface functions --------------------------------------------*/

int8_t readSensorRegisters(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t size)
{
   HAL_StatusTypeDef result;

   const uint8_t i2c_address = (dev_id << 1);

   result = HAL_I2C_Master_Transmit(&hi2c1, i2c_address, &reg_addr, 1U, APP_I2C_TIMEOUT_MS);

   if(HAL_OK == result)
   {
      result = HAL_I2C_Master_Receive(&hi2c1, i2c_address, &reg_data[0U], size, APP_I2C_TIMEOUT_MS);
   }

   return result;
}

int8_t writeSensorRegisters(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t size)
{
   HAL_StatusTypeDef result;
   uint8_t tx_buffer[APP_I2C_MAX_BUFFER_SIZE];

   const uint8_t i2c_address = (dev_id << 1);

   tx_buffer[0U] = reg_addr;
   memcpy(&tx_buffer[1U], &reg_data[0U], size);

   result = HAL_I2C_Master_Transmit(&hi2c1, i2c_address, &tx_buffer[0U], (size+1U), APP_I2C_TIMEOUT_MS);

   return result;
}
