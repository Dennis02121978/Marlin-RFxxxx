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

#if ALL(RENKFORCE_RF, AUTO_BED_LEVELING_BILINEAR)

#include <Wire.h>
#include "zmatrix.h"
#include "renkforce.h"
#include "../bedlevel/bedlevel.h"
#include "../../module/planner.h"
#include "../../module/probe.h"
#include "../../lcd/marlinui.h"
#include "../../MarlinCore.h"

RFZMatrix rf_zmatrix;
uint8_t RFZMatrix::active = 1;
bool RFZMatrix::present;

// RF Repetier Configuration.h
#define ZM_I2C            RF_I2C_ADDRESS_EXT_EEPROM
#define ZM_FORMAT         7
#define ZM_SECTOR         1536U
#define ZM_HDR_FORMAT     0
#define ZM_HDR_ACTIVE_BED 4
#define ZM_SEC_FORMAT     0
#define ZM_SEC_DIM_X      2
#define ZM_SEC_DIM_Y      4
#define ZM_SEC_MICROSTEPS 6
#define ZM_SEC_X_START   16
#define ZM_SEC_Y_START   18
#define ZM_SEC_X_STEP    20
#define ZM_SEC_Y_STEP    22
#define ZM_SEC_X_END     24
#define ZM_SEC_Y_END     26
#define ZM_SEC_MATRIX    28
#define ZM_MAX_DIM       40          // highest index we accept (RAM for the header row/column)

// ---- 24C256 access ------------------------------------------------------------------------

static bool ee_wait_ready() {
  // a write cycle takes up to 5 ms, the chip does not ACK while busy
  for (uint8_t i = 0; i < 20; ++i) {
    Wire.beginTransmission(uint8_t(ZM_I2C));
    if (Wire.endTransmission() == 0) return true;
    delay(1);
  }
  return false;
}

static bool ee_read_word(const uint16_t addr, int16_t &val) {
  Wire.beginTransmission(uint8_t(ZM_I2C));
  Wire.write(uint8_t(addr >> 8));
  Wire.write(uint8_t(addr & 0xFF));
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(uint8_t(ZM_I2C), uint8_t(2)) != 2) return false;
  const uint8_t hi = Wire.read(), lo = Wire.read();
  val = int16_t((uint16_t(hi) << 8) | lo);
  return true;
}

static bool ee_write_word(const uint16_t addr, const int16_t val) {
  int16_t old;
  if (ee_read_word(addr, old) && old == val) return true;   // spare the EEPROM
  // addr is even and pages are 64 bytes, so a word never crosses a page
  Wire.beginTransmission(uint8_t(ZM_I2C));
  Wire.write(uint8_t(addr >> 8));
  Wire.write(uint8_t(addr & 0xFF));
  Wire.write(uint8_t(uint16_t(val) >> 8));
  Wire.write(uint8_t(val & 0xFF));
  if (Wire.endTransmission() != 0) return false;
  return ee_wait_ready();
}

static inline uint16_t sector(const uint8_t slot) { return ZM_SECTOR * slot; }
static inline uint16_t cell(const uint8_t slot, const uint8_t x, const uint8_t y, const uint8_t dy) {
  return sector(slot) + ZM_SEC_MATRIX + (uint16_t(x) * (dy + 1) + y) * 2;
}

// ---- Marlin grid <-> RF Repetier columns ---------------------------------------------------

// Positions of the matrix columns (or rows) in RF Repetier mm and the Marlin grid index for each.
// Like RF Repetier prepareCompensationMatrix(): a border at 0 and at the axis length.
static uint8_t build_axis(const float start, const float spacing, const uint8_t n, const int16_t offset,
                          const int16_t length, int16_t pos[], uint8_t src[]) {
  uint8_t k = 0;
  const int16_t first = int16_t(LROUND(start)) + offset;
  if (first > 0) { pos[k] = 0; src[k] = 0; ++k; }
  for (uint8_t i = 0; i < n; ++i) { pos[k] = int16_t(LROUND(start + i * spacing)) + offset; src[k] = i; ++k; }
  if (pos[k - 1] < length) { pos[k] = length; src[k] = n - 1; ++k; }
  return k;
}

static constexpr int16_t y_offset = RF_Y_HOME;                       // RF Repetier Y = Marlin Y + this
static constexpr int16_t x_length = X_MAX_POS, y_length = Y_MAX_POS + RF_Y_HOME;

static float z_steps_per_mm() { return planner.settings.axis_steps_per_mm[Z_AXIS]; }

// ---- public --------------------------------------------------------------------------------

void RFZMatrix::init() {
  int16_t fmt, act;
  present = ee_read_word(ZM_HDR_FORMAT, fmt);
  if (!present) { SERIAL_ECHOLNPGM("Z-Matrix-EEPROM (0x50) antwortet nicht"); return; }
  if (fmt == ZM_FORMAT && ee_read_word(ZM_HDR_ACTIVE_BED, act) && WITHIN(act, 1, RF_ZMATRIX_SLOTS)) active = act;
  if (load(active, true)) SERIAL_ECHOLNPGM("Z-Matrix ", active, " aus dem EEPROM geladen");
}

bool RFZMatrix::select(const uint8_t slot) {
  if (!WITHIN(slot, 1, RF_ZMATRIX_SLOTS) || !present) return false;
  active = slot;
  int16_t fmt;
  if (!ee_read_word(ZM_HDR_FORMAT, fmt) || fmt != ZM_FORMAT) ee_write_word(ZM_HDR_FORMAT, ZM_FORMAT);
  ee_write_word(ZM_HDR_ACTIVE_BED, slot);
  if (load(slot)) return true;
  // empty slot: no compensation (RF Repetier switchActiveHeatBed)
  set_bed_leveling_enabled(false);
  bedlevel.reset();
  return false;
}

bool RFZMatrix::load(const uint8_t slot, const bool quiet/*=false*/) {
  if (!WITHIN(slot, 1, RF_ZMATRIX_SLOTS) || !present) return false;
  const uint16_t base = sector(slot);
  int16_t hfmt, sfmt, dx, dy, ms;
  if (!ee_read_word(ZM_HDR_FORMAT, hfmt) || !ee_read_word(base + ZM_SEC_FORMAT, sfmt)
      || !ee_read_word(base + ZM_SEC_DIM_X, dx) || !ee_read_word(base + ZM_SEC_DIM_Y, dy)
      || !ee_read_word(base + ZM_SEC_MICROSTEPS, ms)) return false;
  if (hfmt != ZM_FORMAT || sfmt != ZM_FORMAT || !WITHIN(dx, 2, ZM_MAX_DIM) || !WITHIN(dy, 2, ZM_MAX_DIM)
      || ZM_SEC_MATRIX + uint16_t(dx + 1) * (dy + 1) * 2 > ZM_SECTOR) {
    if (!quiet) SERIAL_ECHOLNPGM("Z-Matrix ", slot, " ist leer oder ungueltig");
    return false;
  }

  // header row (X) and column (Y), RF Repetier mm
  int16_t xs[ZM_MAX_DIM + 1], ys[ZM_MAX_DIM + 1];
  for (uint8_t x = 1; x <= dx; ++x) if (!ee_read_word(cell(slot, x, 0, dy), xs[x])) return false;
  for (uint8_t y = 1; y <= dy; ++y) if (!ee_read_word(cell(slot, 0, y, dy), ys[y])) return false;

  // Z steps were stored with "ms" micro steps
  const uint16_t ms_now = rf.settings.drv_microsteps[RF_DRV_Z];
  const float spm = z_steps_per_mm() * (ms > 0 ? float(ms) / ms_now : 1.0f);

  // Marlin grid = grid of the RF Repetier heat bed scan
  bedlevel.set_grid(rf.scan_grid_spacing(), rf.scan_grid_start());

  auto find = [](const int16_t *p, const uint8_t n, const float v, uint8_t &i, float &t) {
    if (v <= p[1]) { i = 1; t = 0; return; }
    if (v >= p[n]) { i = n - 1; t = 1; return; }
    i = 1;
    while (i < n - 1 && v > p[i + 1]) ++i;
    const int16_t d = p[i + 1] - p[i];
    t = d > 0 ? (v - p[i]) / d : 0;
  };

  bed_mesh_t z;
  for (uint8_t gx = 0; gx < GRID_MAX_POINTS_X; ++gx) {
    uint8_t ix; float tx;
    find(xs, dx, bedlevel.get_mesh_x(gx), ix, tx);
    for (uint8_t gy = 0; gy < GRID_MAX_POINTS_Y; ++gy) {
      uint8_t iy; float ty;
      find(ys, dy, bedlevel.get_mesh_y(gy) + y_offset, iy, ty);
      int16_t a, b, c, d;
      if (!ee_read_word(cell(slot, ix, iy, dy), a) || !ee_read_word(cell(slot, ix + 1, iy, dy), b)
          || !ee_read_word(cell(slot, ix, iy + 1, dy), c) || !ee_read_word(cell(slot, ix + 1, iy + 1, dy), d)) return false;
      const float v = (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
      z[gx][gy] = v / spm;
    }
  }

  const bool was_on = planner.leveling_active;
  if (was_on) set_bed_leveling_enabled(false);
  COPY(bedlevel.z_values, z);
  bedlevel.refresh_bed_level();
  if (was_on) set_bed_leveling_enabled(true);
  active = slot;
  if (!quiet) SERIAL_ECHOLNPGM("Z-Matrix ", slot, " geladen (", dx, "x", dy, ", ", ms, " Mikroschritte)");
  return true;
}

bool RFZMatrix::save(const uint8_t slot) {
  if (!WITHIN(slot, 1, RF_ZMATRIX_SLOTS) || !present || !bedlevel.has_mesh()) return false;

  int16_t xs[GRID_MAX_POINTS_X + 2], ys[GRID_MAX_POINTS_Y + 2];
  uint8_t sx[GRID_MAX_POINTS_X + 2], sy[GRID_MAX_POINTS_Y + 2];
  const uint8_t dx = build_axis(bedlevel.grid_start.x, bedlevel.grid_spacing.x, GRID_MAX_POINTS_X, 0, x_length, xs, sx),
                dy = build_axis(bedlevel.grid_start.y, bedlevel.grid_spacing.y, GRID_MAX_POINTS_Y, y_offset, y_length, ys, sy);
  const float spm = z_steps_per_mm();
  const uint16_t base = sector(slot);

  ui.set_status(F("Z-Matrix speichern"));
  bool ok = ee_write_word(ZM_HDR_FORMAT, ZM_FORMAT)
         && ee_write_word(ZM_HDR_ACTIVE_BED, slot)
         && ee_write_word(base + ZM_SEC_FORMAT, ZM_FORMAT)
         && ee_write_word(base + ZM_SEC_DIM_X, dx)
         && ee_write_word(base + ZM_SEC_DIM_Y, dy)
         && ee_write_word(base + ZM_SEC_MICROSTEPS, rf.settings.drv_microsteps[RF_DRV_Z])
         && ee_write_word(base + ZM_SEC_X_START, xs[0]) && ee_write_word(base + ZM_SEC_Y_START, ys[0])
         && ee_write_word(base + ZM_SEC_X_STEP, int16_t(LROUND(bedlevel.grid_spacing.x)))
         && ee_write_word(base + ZM_SEC_Y_STEP, int16_t(LROUND(bedlevel.grid_spacing.y)))
         && ee_write_word(base + ZM_SEC_X_END, xs[dx - 1]) && ee_write_word(base + ZM_SEC_Y_END, ys[dy - 1]);

  // matrix: index 0 = header, data from index 1
  for (uint8_t x = 0; ok && x <= dx; ++x) {
    for (uint8_t y = 0; ok && y <= dy; ++y) {
      int16_t v;
      if (x == 0 && y == 0) v = ZM_FORMAT;
      else if (y == 0) v = xs[x - 1];
      else if (x == 0) v = ys[y - 1];
      else {
        const float zmm = bedlevel.z_values[sx[x - 1]][sy[y - 1]];
        v = isnan(zmm) ? 0 : int16_t(LROUND(zmm * spm));
      }
      ok = ee_write_word(cell(slot, x, y, dy), v);
    }
    idle();
  }

  active = slot;
  if (ok) {
    SERIAL_ECHOLNPGM("Z-Matrix ", slot, " im EEPROM gespeichert (", dx, "x", dy, ")");
    ui.set_status(F("Z-Matrix gespeichert"));
  }
  else {
    SERIAL_ERROR_MSG("Z-Matrix ", slot, ": Schreibfehler im EEPROM");
    LCD_ALERTMESSAGE_F("Z-Matrix Fehler");
  }
  return ok;
}

void RFZMatrix::clear(const uint8_t slot) {
  if (!WITHIN(slot, 1, RF_ZMATRIX_SLOTS) || !present) return;
  // RF Repetier clearCompensationMatrix: sector format and dimensions 0 = no matrix
  const uint16_t base = sector(slot);
  ee_write_word(ZM_HDR_FORMAT, ZM_FORMAT);
  ee_write_word(ZM_HDR_ACTIVE_BED, slot);
  for (uint8_t o = ZM_SEC_FORMAT; o < ZM_SEC_MATRIX; o += 2) ee_write_word(base + o, 0);
  active = slot;
  set_bed_leveling_enabled(false);
  bedlevel.reset();
  SERIAL_ECHOLNPGM("Z-Matrix ", slot, " geloescht");
}

//
// M3013 [S<slot>] [P1]: stored matrix of the slot like RF Repetier outputCompensationMatrix()
// (front left first, header row X [mm], header column Y [mm], values in steps or with P1 in mm)
//
bool RFZMatrix::report_slot(const uint8_t slot, const bool in_mm) {
  if (!WITHIN(slot, 1, RF_ZMATRIX_SLOTS) || !present) return false;
  const uint16_t base = sector(slot);
  int16_t hfmt, sfmt, dx, dy, ms;
  if (!ee_read_word(ZM_HDR_FORMAT, hfmt) || !ee_read_word(base + ZM_SEC_FORMAT, sfmt)
      || !ee_read_word(base + ZM_SEC_DIM_X, dx) || !ee_read_word(base + ZM_SEC_DIM_Y, dy)
      || !ee_read_word(base + ZM_SEC_MICROSTEPS, ms)) return false;
  if (hfmt != ZM_FORMAT || sfmt != ZM_FORMAT || !WITHIN(dx, 2, ZM_MAX_DIM) || !WITHIN(dy, 2, ZM_MAX_DIM)) {
    SERIAL_ECHOLNPGM("M3013: Z-Matrix ", slot, " ist leer oder ungueltig");
    return true;
  }
  // steps of the matrix were stored with "ms" Z micro steps
  const uint16_t ms_now = rf.settings.drv_microsteps[RF_DRV_Z];
  const float spm = z_steps_per_mm() * (ms > 0 ? float(ms) / ms_now : 1.0f);
  int16_t vmin = 32767, vmax = -32768;
  SERIAL_ECHOLNPGM("front left ... front right");
  SERIAL_ECHOLNPGM("...        ...         ...");
  SERIAL_ECHOLNPGM("back left  ...  back right");
  for (uint8_t y = 0; y <= dy; ++y) {
    for (uint8_t x = 0; x <= dx; ++x) {
      int16_t v;
      if (!ee_read_word(cell(slot, x, y, dy), v)) { SERIAL_EOL(); return false; }
      SERIAL_CHAR(';');
      if (x == 0 || y == 0 || !in_mm) SERIAL_ECHO(v);
      else SERIAL_ECHO_F(v / spm, 3);
      if (x && y) { NOMORE(vmin, v); NOLESS(vmax, v); }
    }
    SERIAL_ECHOLNPGM(" ");
    idle();
  }
  SERIAL_ECHOPGM("offset = ", vmax, " [steps] (= ");
  SERIAL_ECHO_F(vmax / spm, 3);
  SERIAL_ECHOLNPGM(" [mm])");
  SERIAL_ECHOPGM("warpage = ", vmin - vmax, " [steps] (= ");
  SERIAL_ECHO_F((vmin - vmax) / spm, 3);
  SERIAL_ECHOLNPGM(" [mm])");
  SERIAL_ECHOLNPGM("g_uZMatrixMax[X_AXIS] = ", dx);
  SERIAL_ECHOLNPGM("g_uZMatrixMax[Y_AXIS] = ", dy);
  SERIAL_ECHOLNPGM("Z-Mikroschritte der Matrix = ", ms, " (jetzt ", ms_now, ")");
  SERIAL_ECHOLNPGM("g_nActiveHeatBed = ", active, ", Kompensation ", planner.leveling_active ? 1 : 0);
  return true;
}

void RFZMatrix::report(const bool in_mm) {
  // Output of the current grid in the layout of RF Repetier M3013 (front left first)
  if (!bedlevel.has_mesh()) { SERIAL_ECHOLNPGM("keine Z-Matrix"); return; }
  int16_t xs[GRID_MAX_POINTS_X + 2], ys[GRID_MAX_POINTS_Y + 2];
  uint8_t sx[GRID_MAX_POINTS_X + 2], sy[GRID_MAX_POINTS_Y + 2];
  const uint8_t dx = build_axis(bedlevel.grid_start.x, bedlevel.grid_spacing.x, GRID_MAX_POINTS_X, 0, x_length, xs, sx),
                dy = build_axis(bedlevel.grid_start.y, bedlevel.grid_spacing.y, GRID_MAX_POINTS_Y, y_offset, y_length, ys, sy);
  const float spm = z_steps_per_mm();
  SERIAL_ECHOLNPGM("front left ... front right");
  SERIAL_ECHOLNPGM("...        ...         ...");
  SERIAL_ECHOLNPGM("back left  ...  back right");
  for (uint8_t y = 0; y <= dy; ++y) {
    for (uint8_t x = 0; x <= dx; ++x) {
      SERIAL_CHAR(';');
      if (x == 0 && y == 0) SERIAL_ECHO(ZM_FORMAT);
      else if (y == 0) SERIAL_ECHO(xs[x - 1]);
      else if (x == 0) SERIAL_ECHO(ys[y - 1]);
      else {
        const float zmm = bedlevel.z_values[sx[x - 1]][sy[y - 1]];
        if (in_mm) SERIAL_ECHO_F(zmm, 3); else SERIAL_ECHO(int16_t(LROUND(zmm * spm)));
      }
    }
    SERIAL_EOL();
  }
  SERIAL_ECHOLNPGM("aktive Heizbett-Matrix = ", active, ", Kompensation ", planner.leveling_active ? 1 : 0);
}

#endif // RENKFORCE_RF && AUTO_BED_LEVELING_BILINEAR
