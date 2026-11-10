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
 * Strain gauge (load cell) under the hotend of the Renkforce RF1000 / RF2000:
 * ADS1100 16 bit ADC on I2C address 0x49, continuous conversion.
 *
 * All values are "digits" on the scale of the RF Repetier firmware (8 SPS, gain 1),
 * so limits like the emergency pause (+-11000) or the probe delta keep their meaning.
 * Faster sample rates use a higher gain with the same scale (16 SPS x2, 32 SPS x4).
 * Klipper (ads1100 + rf1000_load_cell_probe) uses 16 SPS, gain 2, force = digits * 0.52.
 */

#include "../../inc/MarlinConfigPre.h"

class RFLoadCell {
public:
  static int16_t raw;              // last sample, digits
  static int16_t tare_offset;      // raw value at the last tare
  static bool ok;                  // sensor answers on I2C
  static uint8_t seq;              // increments with every new sample
  static volatile bool triggered;  // probe state, read by the endstop ISR

  static void init();
  static bool update();            // call often (idle), true when a new sample was read
  static int16_t value() { return raw - tare_offset; }  // tared value
  static bool tare();              // wait for samples, set the zero point; true = error
  static bool wait_samples(const uint8_t n, int32_t &sum);  // blocking, keeps Marlin running
  static uint16_t period_ms();

private:
  static millis_t next_ms;
  static uint8_t errors;
  static bool configure();
  static void update_trigger();
};

extern RFLoadCell rf_loadcell;
