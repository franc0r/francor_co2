# FRANCOR CO2 Sensor

This package contains the interface software for the CO2 or "breath" detecting sensor of FRANCOR e.V.

The top layer contains the ROS2 package to communicate with the sensor board.

In the folder 'fw' is the STM32CubeIDE project which contains the firmware for the [BluePill Board](https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html). The sensor is a [BME680](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/) environmental sensor from BOSCH.

## Sensorboard


### Hardware
- [STM32 BluePill](https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html) based on the [STM32F103C8T6 MCU](https://www.st.com/en/microcontrollers-microprocessors/stm32f103c8.html)
- [DFRobot Breakout Board of Bosch BME680](https://www.dfrobot.com/product-2143.html) connected to the BluePill via I2C
- Communication with host via USB virtual com port
- STLink Debugger to flash the firmware

### IDE
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) is necessary to build the firmware

### PINOUT

The connection between BluePill and the breakout board is shown in the table below

| BluePill-Pin | BME680 Module Pin | Function |
|-|-|-|
|3.3V|I2C-VCC|Power supply|
|GND|I2C-GND|Ground|
|B6|I2C-SCL|I2C clock line|
|B7|I2C-SDA|I2C data line|

The breakout board support two interface types: I2C and SPI. Because I2C needs only two data lines we choose I2C over SPI bus.

![Wiring](https://github.com/franc0r/francor_co2/blob/devel/docs/img/francor_co2_wiring.png "Wiring")

### Firmware

To build the firmware you have to import the project in the firmware directory into the STM32CubeIDE. You have to use the code generator of CubeMX to generate the driver files, because thei are not checked into this git repository.

Please make sure you have pulled the submodules with:
```
git submodule update --init --recursive
```
This is necessary, because the BME680 sensors needs Drivers which are imported as submodule from the official [Bosch Sensortec repository](git@github.com:BoschSensortec/BME680_driver.git).