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

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initializes the CO2 firmware
 */
void initApp(void);

/**
 * @brief Enter main loop of application (no return)
 */
void runApp(void);

#endif /* __APP_H */
