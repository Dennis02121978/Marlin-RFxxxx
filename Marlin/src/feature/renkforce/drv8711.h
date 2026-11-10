/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * TI DRV8711 stepper drivers of the Renkforce RF1000 / RF2000 board.
 *
 * Ported from the RF Repetier firmware (RF.cpp: drv8711Init, setMotorCurrent,
 * drv8711adjustMicroSteps, readMotorStatus). The drivers sit on a software SPI bus
 * (SCLK PD5, SDATI PD6, SDATO PD4) with one high active chip select per driver.
 *
 * Motor current: TORQUE register (0..255). RF Repetier: 126 = ~2 A.
 *   I = TORQUE * 2.75 V / (256 * ISGAIN 20 * Rsense 0.033 Ohm)  ->  TORQUE = I[A] * 61.44
 */

#include "../../inc/MarlinConfigPre.h"

#define RF_DRV_COUNT 5   // X, Y, Z, E0, E1 (RF Repetier DRV8711_NUM_CHANNELS)

enum RFDriver : uint8_t { RF_DRV_X, RF_DRV_Y, RF_DRV_Z, RF_DRV_E0, RF_DRV_E1 };

class RF_DRV8711 {
public:
  static bool initialized;

  static void init();                                         // reset + write all registers
  static void refresh();                                      // rewrite mode + current (no reset)
  static void set_current(const uint8_t d, const uint16_t mA);
  static void set_microsteps(const uint8_t d, const uint16_t ms);
  static uint16_t read_status(const uint8_t d);               // register 7, bits 0..7
  static void clear_status(const uint8_t d);
  static void report_status(const uint8_t d);
  static bool fault();                                        // FAULTn pin (any driver)

  static constexpr uint8_t mA_to_torque(const uint16_t mA) {
    return uint8_t(_MIN(255UL, (uint32_t(mA) * 6144UL + 50000UL) / 100000UL));
  }
  static constexpr uint16_t torque_to_mA(const uint8_t t) {
    return uint16_t((uint32_t(t) * 100000UL + 3072UL) / 6144UL);
  }
  static bool valid_microsteps(const uint16_t ms) { return ms >= 1 && ms <= 256 && !(ms & (ms - 1)); }

  static void write_reg(const uint8_t d, const uint8_t reg, const uint16_t data);
  static uint16_t read_reg(const uint8_t d, const uint8_t reg);

private:
  static void select(const uint8_t d, const bool on);
  static uint16_t transfer(const uint16_t out);
  static uint16_t reg0(const uint8_t d);
};

extern RF_DRV8711 rf_drv;
