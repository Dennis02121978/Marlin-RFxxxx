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

#include "drv8711.h"
#include "renkforce.h"
#include "../../HAL/shared/Delay.h"

RF_DRV8711 rf_drv;
bool RF_DRV8711::initialized; // = false

/**
 * Register values (address in bits 14..12, data in bits 11..0).
 * Original Conrad values, Klipper drv8711.py uses the same defaults.
 *
 * RF_DRV8711_LESS_NOISE = RF Repetier FEATURE_STEPPER_LESS_NOISE (shorter off time and
 * auto mixed decay). Note: RF Repetier writes 0xB2D7 for register 3 in this mode. Bit 15 set
 * makes that a READ command, so the drivers kept their reset value. Here register 3 always
 * gets the original value 0x1D7 (TBLANK 4.3 us, adaptive blanking on).
 */
#define DRV_REG0_BASE  0x0E01   // DTIME 850 ns, ISGAIN 20, EXSTALL 0, MODE (bits 6..3), RSTEP 0, RDIR 0, ENBL 1
#define DRV_REG1_BASE  0x0100   // SMPLTH 100 us, TORQUE in bits 7..0
#if ENABLED(RF_DRV8711_LESS_NOISE)
  #define DRV_REG2     0x0020   // TOFF 16.5 us, PWMMODE 0
  #define DRV_REG4     0x0504   // TDECAY 2 us, DECMOD 101 auto mixed decay
#else
  #define DRV_REG2     0x0097   // TOFF 76 us, PWMMODE 0
  #define DRV_REG4     0x0430   // TDECAY 24 us, DECMOD 100 mixed decay
#endif
#define DRV_REG3       0x01D7   // TBLANK 4.3 us, ABT 1
#define DRV_REG5       0x083C   // SDTHR 0x3C, SDCNT 00, VDIV 10
#define DRV_REG6       0x00F0   // OCPTH 250 mV, OCPDEG 1 us, TDRIVEN/P 2 us, IDRIVEN/P lowest
#define DRV_REG7       0x0000   // clear all status flags

static const char drv_name[RF_DRV_COUNT][3] PROGMEM = { "X", "Y", "Z", "E0", "E1" };

void RF_DRV8711::select(const uint8_t d, const bool on) {
  // SCS is high active
  switch (d) {
    case RF_DRV_X:  WRITE(RF_X_CS_PIN, on); break;
    case RF_DRV_Y:  WRITE(RF_Y_CS_PIN, on); break;
    case RF_DRV_Z:  WRITE(RF_Z_CS_PIN, on); break;
    case RF_DRV_E0: WRITE(RF_E0_CS_PIN, on); break;
    case RF_DRV_E1: WRITE(RF_E1_CS_PIN, on); break;
  }
  DELAY_US(1);
}

// MSB first. SDATI is latched by the driver on the rising edge, SDATO changes on the falling edge.
uint16_t RF_DRV8711::transfer(const uint16_t out) {
  uint16_t in = 0;
  for (int8_t i = 15; i >= 0; --i) {
    WRITE(DRV8711_SDATI_PIN, TEST(out, i));
    DELAY_US(1);
    if (READ(DRV8711_SDATO_PIN)) SBI(in, i);
    WRITE(DRV8711_SCLK_PIN, HIGH);
    DELAY_US(5);
    WRITE(DRV8711_SCLK_PIN, LOW);
  }
  WRITE(DRV8711_SDATI_PIN, LOW);
  return in;
}

void RF_DRV8711::write_reg(const uint8_t d, const uint8_t reg, const uint16_t data) {
  select(d, true);
  transfer((uint16_t(reg & 0x07) << 12) | (data & 0x0FFF));
  select(d, false);
}

uint16_t RF_DRV8711::read_reg(const uint8_t d, const uint8_t reg) {
  select(d, true);
  const uint16_t in = transfer(0x8000 | (uint16_t(reg & 0x07) << 12));
  select(d, false);
  return in & 0x0FFF;
}

uint16_t RF_DRV8711::reg0(const uint8_t d) {
  uint16_t ms = rf.settings.drv_microsteps[d];
  uint8_t mode = 0;
  while (ms > 1 && mode < 8) { ms >>= 1; ++mode; }   // 1 -> 0, 2 -> 1 ... 256 -> 8
  return DRV_REG0_BASE | (uint16_t(mode) << 3);
}

bool RF_DRV8711::fault() { return !READ(DRV8711_FAULT_PIN); }

void RF_DRV8711::init() {
  OUT_WRITE(DRV8711_RESET_PIN, LOW);
  OUT_WRITE(DRV8711_SCLK_PIN, LOW);
  OUT_WRITE(DRV8711_SDATI_PIN, LOW);
  SET_INPUT_PULLUP(DRV8711_SDATO_PIN);
  SET_INPUT_PULLUP(DRV8711_FAULT_PIN);
  SET_INPUT_PULLUP(RF_X_STALL_PIN);
  SET_INPUT_PULLUP(RF_Y_STALL_PIN);
  SET_INPUT_PULLUP(RF_Z_STALL_PIN);
  SET_INPUT_PULLUP(RF_E0_STALL_PIN);
  SET_INPUT_PULLUP(RF_E1_STALL_PIN);
  OUT_WRITE(RF_X_CS_PIN, LOW);
  OUT_WRITE(RF_Y_CS_PIN, LOW);
  OUT_WRITE(RF_Z_CS_PIN, LOW);
  OUT_WRITE(RF_E0_CS_PIN, LOW);
  OUT_WRITE(RF_E1_CS_PIN, LOW);

  // Reset all DRV8711 (RESET is high active)
  WRITE(DRV8711_RESET_PIN, HIGH);
  delay(5);
  WRITE(DRV8711_RESET_PIN, LOW);
  delay(5);

  // All five drivers are configured, also E1 on a single extruder machine (as in RF Repetier)
  for (uint8_t d = 0; d < RF_DRV_COUNT; ++d) {
    write_reg(d, 0, reg0(d));
    write_reg(d, 1, DRV_REG1_BASE | mA_to_torque(rf.settings.drv_current[d]));
    write_reg(d, 2, DRV_REG2);
    write_reg(d, 3, DRV_REG3);
    write_reg(d, 4, DRV_REG4);
    write_reg(d, 5, DRV_REG5);
    write_reg(d, 6, DRV_REG6);
    write_reg(d, 7, DRV_REG7);
  }
  initialized = true;
}

void RF_DRV8711::refresh() {
  if (!initialized) return;
  for (uint8_t d = 0; d < RF_DRV_COUNT; ++d) {
    write_reg(d, 0, reg0(d));
    write_reg(d, 1, DRV_REG1_BASE | mA_to_torque(rf.settings.drv_current[d]));
  }
}

void RF_DRV8711::set_current(const uint8_t d, const uint16_t mA) {
  if (d >= RF_DRV_COUNT) return;
  rf.settings.drv_current[d] = _MIN(mA, uint16_t(RF_DRV8711_CURRENT_LIMIT));
  if (initialized) write_reg(d, 1, DRV_REG1_BASE | mA_to_torque(rf.settings.drv_current[d]));
}

void RF_DRV8711::set_microsteps(const uint8_t d, const uint16_t ms) {
  if (d >= RF_DRV_COUNT || !valid_microsteps(ms)) return;
  rf.settings.drv_microsteps[d] = ms;
  if (initialized) write_reg(d, 0, reg0(d));
}

uint16_t RF_DRV8711::read_status(const uint8_t d) { return read_reg(d, 7) & 0xFF; }

void RF_DRV8711::clear_status(const uint8_t d) { write_reg(d, 7, DRV_REG7); }

void RF_DRV8711::report_status(const uint8_t d) {
  static const char flag_names[] PROGMEM = "OTS\0AOCP\0BOCP\0APDF\0BPDF\0UVLO\0STD\0STDLAT";
  const uint16_t st = read_status(d);
  SERIAL_ECHOPGM("DRV8711 ");
  SERIAL_ECHOPGM_P(drv_name[d]);
  SERIAL_ECHOPGM(": ", rf.settings.drv_current[d], "mA (TORQUE ", mA_to_torque(rf.settings.drv_current[d]),
                 ") 1/", rf.settings.drv_microsteps[d], " status 0x");
  SERIAL_PRINT(st, PrintBase::Hex);
  if (st) {
    const char *p = flag_names;
    for (uint8_t b = 0; b < 8; ++b) {
      if (TEST(st, b)) { SERIAL_CHAR(' '); SERIAL_ECHOPGM_P(p); }
      p += strlen_P(p) + 1;
    }
  }
  else
    SERIAL_ECHOPGM(" ok");
  SERIAL_EOL();
}

#endif // RENKFORCE_RF
