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
extern TIM_HandleTypeDef htim4;

/* Private variables ---------------------------------------------------------*/

static struct     bme680_dev hsensor;
static struct     bme680_field_data sensor_data;

static uint32_t sensor_humidity_lst[APP_NUM_AVG_VALUES];
static uint32_t sensor_avg_humidity;
static uint32_t sensor_gas_resistance_lst[APP_NUM_AVG_VALUES];
static uint32_t sensor_avg_gas_resistance;
static uint32_t sensor_lst_pos;

static uint32_t co2_level;
static uint32_t humidity_level;
/* Private function prototypes -----------------------------------------------*/

static eAppResult_t runErrorHandler(const char* desc);

static void resetVariables(void);
static eAppResult_t initLED(void);
static void setLEDOn(void);
static void setLEDOff(void);
static void toggleLED(void);
static void showInitLEDSequence(void);
static eAppResult_t initSensor(void);
static eAppResult_t configSensor(void);

static void updateLEDTask(void);
static eAppResult_t updateSensorTask(void);


static eAppResult_t updateSensor(void);
static eAppResult_t readSensorData(void);
static void processSensorData(void);
static void sendMeasurementDataToHost(void);


eAppResult_t readSensorRegisters(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t size);
eAppResult_t writeSensorRegisters(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t size);

/* Public functions ----------------------------------------------------------*/

eAppResult_t initApp(void)
{
   resetVariables();

   if(APP_OK != initLED())
   {
      return APP_ERR;
   }

   showInitLEDSequence();

   if(APP_OK != initSensor())
   {
      return APP_ERR;
   }

   if(APP_OK != configSensor())
   {
      return APP_ERR;
   }

   HAL_Delay(APP_SENSOR_UPDATE_TIME_MS);

   return APP_OK;
}

eAppResult_t runApp(void)
{
   for (;;)
   {
      updateLEDTask();
      if (APP_OK != updateSensorTask())
      {
         return APP_ERR;
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
   memset(&sensor_humidity_lst[0U], 0, sizeof(sensor_humidity_lst));
   memset(&sensor_gas_resistance_lst[0U], 0, sizeof(sensor_gas_resistance_lst));

   sensor_lst_pos = 0U;
   sensor_avg_humidity = 0U;
   sensor_avg_gas_resistance = 0U;
   co2_level = 0U;
   humidity_level = 0U;
}

static eAppResult_t initLED(void)
{
   HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
   HAL_TIM_Base_Stop(&htim4);

   if(HAL_OK != HAL_TIM_Base_Start(&htim4))
   {
      return APP_ERROR_HANDLER(APP_ERR_LED_INIT_TIM_FAILURE);
   }

   if(HAL_OK != HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4))
   {
      return APP_ERROR_HANDLER(APP_ERR_LED_START_PWM_FAILURE);
   }

   return APP_OK;
}

static void setLEDOn(void)
{
   htim4.Instance->CCR4 = APP_LED_PWM_MAX_VALUE;
}

static void setLEDOff(void)
{
   htim4.Instance->CCR4 = 0;
}

static void toggleLED(void)
{
   if(htim4.Instance->CCR4 == APP_LED_PWM_MAX_VALUE)
   {
      htim4.Instance->CCR4 = 0;
   }
   else
   {
      htim4.Instance->CCR4 = APP_LED_PWM_MAX_VALUE;
   }
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

static void updateLEDTask(void)
{
   static uint32_t update_timestamp = 0U;

   const uint32_t delta_time_ms = HAL_GetTick() - update_timestamp;
   if(delta_time_ms > APP_LED_UPDATE_TIME_MS)
   {
      update_timestamp = HAL_GetTick();
      if(co2_level > APP_CO2_LED_THRESHOLD)
      {
         htim4.Instance->CCR4 = co2_level;
      }
      else
      {
         htim4.Instance->CCR4 = 0U;
      }
   }
}

static eAppResult_t updateSensorTask(void)
{
   static uint32_t update_timestamp = 0U;

   const uint32_t delta_time_ms = HAL_GetTick() - update_timestamp;
   if (delta_time_ms > APP_SENSOR_UPDATE_TIME_MS)
   {
      update_timestamp = HAL_GetTick();
      if(APP_OK != updateSensor())
      {
         return APP_ERR;
      }
   }

   return APP_OK;
}

static eAppResult_t updateSensor(void)
{
   eAppResult_t result = APP_ERR;

   if(APP_OK == readSensorData())
   {
      processSensorData();
      sendMeasurementDataToHost();
      result = APP_OK;
   }

   return result;
}

static eAppResult_t readSensorData(void)
{
   if (BME680_OK != bme680_get_sensor_data(&sensor_data, &hsensor))
   {
      return APP_ERROR_HANDLER(APP_ERR_BME680_GET_DATA);
   }

   if (BME680_OK != bme680_set_sensor_mode(&hsensor))
   {
      return APP_ERROR_HANDLER(APP_ERR_BME680_SET_SENSOR_MODE);
   }

   return APP_OK;
}

static void processSensorData(void)
{
   /* Calculate average values */
   sensor_gas_resistance_lst[sensor_lst_pos] = sensor_data.gas_resistance;
   sensor_humidity_lst[sensor_lst_pos] = sensor_data.humidity;

   for (uint32_t idx = 0U; idx < APP_NUM_AVG_VALUES; idx++)
   {
      sensor_avg_humidity += sensor_humidity_lst[idx];
      sensor_avg_gas_resistance += sensor_gas_resistance_lst[idx];
   }
   sensor_avg_humidity /= (APP_NUM_AVG_VALUES+1U);
   sensor_avg_gas_resistance /= (APP_NUM_AVG_VALUES+1U);

   sensor_lst_pos += 1;
   if (sensor_lst_pos >= APP_NUM_AVG_VALUES)
   {
      sensor_lst_pos = 0U;
   }

   /* Process data to see if CO2 or humidity changes due to breath detection */
   co2_level = (uint32_t)(abs((int32_t)(sensor_data.gas_resistance) - (int32_t)(sensor_avg_gas_resistance)) / 10);

   if(co2_level > APP_CO2_LEVEL_MAX)
   {
      co2_level = APP_CO2_LEVEL_MAX;
   }
}

static void sendMeasurementDataToHost(void)
{
   char tx_buffer[APP_VCP_TX_BUFFER ];
   sprintf(&tx_buffer[0U], "d|%i|%i|%i|%li|%li|%i|%li|%li|%li\n", sensor_data.meas_index, sensor_data.status,
           sensor_data.temperature, sensor_data.pressure, sensor_data.humidity, sensor_data.gas_index,
           sensor_data.gas_resistance, sensor_avg_humidity, co2_level);

   CDC_Transmit_FS((uint8_t*) &tx_buffer[0U], strlen(&tx_buffer[0U]));
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
