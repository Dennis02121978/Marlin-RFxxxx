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

#include "../../inc/MarlinConfig.h"

#if ENABLED(RENKFORCE_RF)

#include <Wire.h>
#include "loadcell.h"
#include "renkforce.h"
#include "../../MarlinCore.h"
#include "../../lcd/marlinui.h"

RFLoadCell rf_loadcell;

int16_t RFLoadCell::raw, RFLoadCell::tare_offset;
bool RFLoadCell::ok;
uint8_t RFLoadCell::seq, RFLoadCell::errors;
volatile bool RFLoadCell::triggered;
millis_t RFLoadCell::next_ms;

// ADS1100 config byte: bit 4 SC = 0 (continuous), bits 3..2 data rate, bits 1..0 PGA
#if RF_LOADCELL_SPS == 8
  #define ADS_DR 3
  #define ADS_PGA 0   // x1
#elif RF_LOADCELL_SPS == 16
  #define ADS_DR 2
  #define ADS_PGA 1   // x2
#elif RF_LOADCELL_SPS == 32
  #define ADS_DR 1
  #define ADS_PGA 2   // x4
#else
  #error "RF_LOADCELL_SPS must be 8, 16 or 32."
#endif

uint16_t RFLoadCell::period_ms() { return 1000 / (RF_LOADCELL_SPS) + 2; }

bool RFLoadCell::configure() {
  Wire.beginTransmission(uint8_t(RF_I2C_ADDRESS_LOADCELL));
  Wire.write(uint8_t((ADS_DR << 2) | ADS_PGA));
  return Wire.endTransmission() == 0;
}

void RFLoadCell::init() {
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true);     // never hang Marlin on a stuck bus (3 ms, reset TWI)
  ok = configure();
  errors = ok ? 0 : 3;
  triggered = !ok;
  next_ms = millis() + 300;            // first conversion
  if (!ok) SERIAL_ECHOLNPGM("DMS (ADS1100) antwortet nicht");
}

void RFLoadCell::update_trigger() {
  // Fail safe: without a valid reading the probe counts as triggered
  const int16_t v = value();
  triggered = !ok || ABS(v) >= rf.settings.probe_trigger || ABS(raw) >= RF_LOADCELL_IDLE_MAX + 2000;
}

bool RFLoadCell::update() {
  const millis_t ms = millis();
  if (PENDING(ms, next_ms)) return false;
  next_ms = ms + period_ms();

  bool good = false;
  if (Wire.requestFrom(uint8_t(RF_I2C_ADDRESS_LOADCELL), uint8_t(3)) >= 2) {
    const uint8_t hi = Wire.read(), lo = Wire.read();
    while (Wire.available()) (void)Wire.read();   // config byte
    raw = int16_t((uint16_t(hi) << 8) | lo);
    good = true;
  }

  if (good) {
    if (!ok) configure();
    ok = true;
    errors = 0;
    ++seq;
  }
  else if (errors < 3 && ++errors == 3) {
    ok = false;
    SERIAL_ECHOLNPGM("DMS (ADS1100) Lesefehler");
  }
  update_trigger();
  return good;
}

bool RFLoadCell::wait_samples(const uint8_t n, int32_t &sum) {
  sum = 0;
  uint8_t got = 0, last = seq;
  const millis_t timeout = millis() + uint32_t(n + 3) * period_ms() + 500;
  while (got < n) {
    idle();                      // runs update() via rf.idle()
    if (seq != last) { last = seq; sum += raw; ++got; }
    if (ELAPSED(millis(), timeout)) return true;
  }
  return false;
}

bool RFLoadCell::tare() {
  // Let the bed and the hotend settle (RF Repetier HEAT_BED_SCAN_IDLE_DELAY_MS)
  const millis_t settle = millis() + RF_LOADCELL_SETTLE_MS;
  while (PENDING(millis(), settle)) idle();

  int32_t sum;
  if (wait_samples(RF_LOADCELL_TARE_SAMPLES, sum) || !ok) {
    SERIAL_ERROR_MSG("DMS: keine Messwerte");
    LCD_ALERTMESSAGE_F("DMS: kein Messwert");
    triggered = true;
    return true;
  }
  const int16_t avg = int16_t(sum / (RF_LOADCELL_TARE_SAMPLES));
  if (ABS(avg) > RF_LOADCELL_IDLE_MAX) {
    // RF Repetier HEAT_BED_SCAN_IDLE_PRESSURE_MIN/MAX: hotend touches something or sensor broken
    SERIAL_ERROR_MSG("DMS: Ruhewert ", avg, " ausserhalb +-", RF_LOADCELL_IDLE_MAX);
    LCD_ALERTMESSAGE_F("DMS: Ruhewert falsch");
    triggered = true;
    return true;
  }
  tare_offset = avg;
  update_trigger();
  return false;
}

#endif // RENKFORCE_RF
