/*
 * arduino_co2.ino
 *
 *  Created on: 25.04.2018
 *      Author: feesmrt
 *
 * BSD 3-Clause License
 * Copyright (c) 2018, FRANC0R - Franconian Open Robotics
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "Arduino.h"

#include "mcp2515.h"
#include "defaults.h"
#include "mcp2515_defs.h"
#include "global.h"

#include "CO2DataMsg.h"

/**
 * Definitions
 */
/*
 * CAN Speed Definitions
 */
constexpr uint8_t CAN_SPEED_100k  = 9;
constexpr uint8_t CAN_SPEED_125k  = 7;
constexpr uint8_t CAN_SPEED_250k  = 3;
constexpr uint8_t CAN_SPEED_500k  = 1;
constexpr uint8_t CAN_SPEED_1M    = 0;

constexpr uint16_t CAN_TX_ID      = 0x701;
constexpr uint8_t  ADC_CO2_PIN    = A0;
constexpr uint32_t CAN_TX_TIME_MS = 100;

/**
 * Global Variables
 */
francor::CO2DataMsg g_can_co2_data(CAN_TX_ID);


/**
 * @brief Sends determined CO2 value via CAN
 * 
 */
void sendCO2DataOnCan() {
  tCAN txMsg;

  // Header
  txMsg.id = g_can_co2_data._can_id;
  txMsg.header.ide = 0;
  txMsg.header.rtr = 0;
  txMsg.header.length = 8;

  memcpy(txMsg.data, g_can_co2_data._raw_data, 8);

  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  mcp2515_send_message(&txMsg);
}

void setup() {
  // init CAN network
  mcp2515_init(CAN_SPEED_500k);

  // Config pin as input
  pinMode(ADC_CO2_PIN, INPUT);

  
}

void loop() {
  static uint32_t sys_tick_cnt = 0;

  if((millis() - sys_tick_cnt) > CAN_TX_TIME_MS) {
    sys_tick_cnt = millis();

    // read CO2 voltage
    // reference voltage 5 V, ADC resolution 1024 -> 5V / 1025 = 0.005 V = 5 mV
    const uint16_t co2_adc_voltage = analogRead(ADC_CO2_PIN) * 5;

    // calculate ppm from voltage
    // 0.4 - 2.0V correlates with 0 - 5000 ppm 

    // range voltage
    // minimum voltage 0.4 V = 400 mV
    if(co2_adc_voltage < 400) {
      g_can_co2_data._co2_ppm = 0;
    }
    // maximum voltage 2.0 V = 2000 mV
    else if(co2_adc_voltage > 2000) {
      g_can_co2_data._co2_ppm = 5000;
    }
    else {
      // calculate value
      // 0.4 V - 2.0 V correlates 0 - 5000 ppm
      // 1.6 V linear range: 5000 / 1.6 V = 5000 / 1600 mV = 3 ppm/mV
      g_can_co2_data._co2_ppm = (co2_adc_voltage-400) * 3;
    }

    // send data on CAN
    sendCO2DataOnCan(); 
  }
}
