/**
 * @file app.h
 * @author Martin Bauernschmitt <martin.bauernschmitt@francor.de>
 * @brief Main application header of CO2 sensor
 *
 * @version 1.0
 * @date 2022-04-09
 *
 * @copyright Copyright (c) 2022 - BSD-3-clause - FRANCOR e.V.
 *
 */

#ifndef __APP_H
#define __APP_H

/* Defines -------------------------------------------------------------------*/
#define APP_DFT_AMBIENT_TEMP     (int8_t)(22)
#define APP_I2C_TIMEOUT_MS       (uint32_t)(100U)
#define APP_I2C_MAX_BUFFER_SIZE  (uint32_t)(256U)
#define APP_VCP_TX_BUFFER        (uint32_t)(256U)

#define APP_ERR_RESET_DELAY_MS   (uint32_t)(1000U)

#define APP_SENSOR_UPDATE_TIME_MS   (uint32_t)(500U)

/* Error Defines -------------------------------------------------------------*/
#define APP_ERR_RESET_MESSAGE          "Reset board after 1 seconds!"
#define APP_ERR_BME680_FAILURE         "Cannot communicate with BME680! Is the sensor connected?"
#define APP_ERR_BME680_INIT_SENSOR     "Initialization of BME680 sensor failed!"
#define APP_ERR_BME680_SET_SETTINGS    "Configuration of BME680 sensor failed!"
#define APP_ERR_BME680_SET_SENSOR_MODE "Cannot set sensor mode of BME680!"
#define APP_ERR_BME680_GET_DATA        "Failed to read measurement data from BME680!"

/* Public enumerations -------------------------------------------------------*/

typedef enum
{
   APP_OK   =  0,
   APP_ERR  = -1,
}eAppResult_t;

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initializes the CO2 firmware
 */
eAppResult_t initApp(void);

/**
 * @brief Enter main loop of application (no return)
 */
eAppResult_t runApp(void);

#endif /* __APP_H */
