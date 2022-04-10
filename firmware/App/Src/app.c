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

#include "main.h"
#include "app.h"
#include "usbd_cdc_if.h"
#include "bme680.h"


/* Private macros ------------------------------------------------------------*/
#define APP_ERROR_HANDLER(DESC)   runErrorHandler(DESC);


/* Extern variables ----------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;

/* Private variables ---------------------------------------------------------*/
static struct bme680_dev hsensor;

/* Private function prototypes -----------------------------------------------*/

static eAppResult_t runErrorHandler(const char* desc);

static void resetVariables(void);
static void setLEDOn(void);
static void setLEDOff(void);
static void toggleLED(void);
static void showInitLEDSequence(void);
static eAppResult_t initSensor(void);
static eAppResult_t configSensor(void);

eAppResult_t readSensorRegisters(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t size);
eAppResult_t writeSensorRegisters(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t size);

/* Public functions ----------------------------------------------------------*/

eAppResult_t initApp(void)
{
   resetVariables();
   showInitLEDSequence();

   if(APP_OK != initSensor())
   {
      return APP_ERR;
   }

   if(APP_OK != configSensor())
   {
      return APP_ERR;
   }

   return APP_OK;
}

eAppResult_t runApp(void)
{
   uint32_t update_timestamp = HAL_GetTick();
   for (;;)
   {
      const uint32_t delta_time_ms = HAL_GetTick() - update_timestamp;

      if (delta_time_ms > APP_SENSOR_UPDATE_TIME_MS)
      {
         struct bme680_field_data data;

         update_timestamp = HAL_GetTick();

         if (BME680_OK != bme680_get_sensor_data(&data, &hsensor))
         {
            return APP_ERROR_HANDLER(APP_ERR_BME680_GET_DATA);
         }

         if (BME680_OK != bme680_set_sensor_mode(&hsensor))
         {
            return APP_ERROR_HANDLER(APP_ERR_BME680_SET_SENSOR_MODE);
         }

         char tx_buffer[APP_VCP_TX_BUFFER];
         sprintf(&tx_buffer[0U], "d|%i|%i|%i|%li|%li|%i|%li\n\r", data.meas_index, data.status, data.temperature,
                 data.pressure, data.humidity, data.gas_index, data.gas_resistance);


         CDC_Transmit_FS((uint8_t*)&tx_buffer[0U], strlen(&tx_buffer[0U]));

         toggleLED();
      }

      __WFI();
   }

   /* Shall never get here */
   return APP_ERR;
}

/* Private functions ---------------------------------------------------------*/

static eAppResult_t runErrorHandler(const char* desc)
{
   /* Transmit error message */
   char tx_buffer[APP_VCP_TX_BUFFER];
   sprintf(&tx_buffer[0U], "e|%s|\n\re|%s|\n\r", desc, APP_ERR_RESET_MESSAGE);
   CDC_Transmit_FS((uint8_t*)&tx_buffer[0U], strlen(&tx_buffer[0U]));

   /* Wait for message tx */
   HAL_Delay(100);
   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
   HAL_Delay(APP_ERR_RESET_DELAY_MS);

   return APP_ERR;
}

static void resetVariables(void)
{
   memset(&hsensor, 0, sizeof(hsensor));
}

static void setLEDOn(void)
{
   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

static void setLEDOff(void)
{
   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

static void toggleLED(void)
{
   HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

static void showInitLEDSequence(void)
{
   for(uint32_t cnt = 0U; cnt < 3U; cnt++)
   {
      setLEDOn();
      HAL_Delay(100);
      setLEDOff();
      HAL_Delay(50);
   }

   setLEDOn();
}

static eAppResult_t initSensor(void)
{
   hsensor.dev_id = BME680_I2C_ADDR_PRIMARY;
   hsensor.intf = BME680_I2C_INTF;
   hsensor.read = readSensorRegisters;
   hsensor.write = writeSensorRegisters;
   hsensor.delay_ms = HAL_Delay;
   hsensor.amb_temp = APP_DFT_AMBIENT_TEMP;

   if(BME680_OK != bme680_init(&hsensor))
   {
      return APP_ERROR_HANDLER(APP_ERR_BME680_INIT_SENSOR);
   }

   return APP_OK;
}

static eAppResult_t configSensor(void)
{
   hsensor.tph_sett.os_hum = BME680_OS_2X;
   hsensor.tph_sett.os_pres = BME680_OS_1X;
   hsensor.tph_sett.os_temp = BME680_OS_1X;
   hsensor.tph_sett.filter = BME680_FILTER_SIZE_0;
   hsensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
   hsensor.gas_sett.heatr_temp = 300;
   hsensor.gas_sett.heatr_dur = 150;

   hsensor.power_mode = BME680_FORCED_MODE;

   const uint8_t settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;

   if(BME680_OK != bme680_set_sensor_settings(settings, &hsensor))
   {
      return APP_ERROR_HANDLER(APP_ERR_BME680_SET_SETTINGS);
   }

   if (BME680_OK != bme680_set_sensor_mode(&hsensor))
   {
      return APP_ERROR_HANDLER(APP_ERR_BME680_SET_SENSOR_MODE);
   }

   return APP_OK;
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
