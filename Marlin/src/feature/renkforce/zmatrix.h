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
 * Heat bed Z matrix in the external I2C EEPROM (24C256, 0x50) in the format of the
 * RF Repetier firmware, so both firmwares can use the same matrices.
 *
 * Layout (RF Repetier Configuration.h, EEPROM_FORMAT 7), all values 16 bit big endian:
 *   0     header format (7)
 *   2     active work part matrix (not used here)
 *   4     active heat bed matrix 1..9
 *   1536 * n (n = 1..9) heat bed matrix n:
 *     +0 sector format (7), +2 x dimension, +4 y dimension (= highest index), +6 Z micro steps,
 *     +16..+26 scan area (only used for work parts), +28 matrix [x][y], x outer loop
 *   matrix[0][0] = format, matrix[x][0] = X position [mm], matrix[0][y] = Y position [mm]
 *   (RF Repetier coordinates: Y=0 at the Y home switch), matrix[x][y] = Z in steps from Z home.
 *
 * Marlin uses a regular bilinear grid. Saving writes the grid like a prepared RF Repetier
 * matrix (border columns at 0 and at the axis length). Loading interpolates any RF Repetier
 * matrix onto the Marlin grid.
 */

#include "../../inc/MarlinConfigPre.h"

#define RF_ZMATRIX_SLOTS 9

class RFZMatrix {
public:
  static uint8_t active;        // 1..9 (RF Repetier g_nActiveHeatBed)
  static bool present;          // EEPROM answers

  static void init();           // read the active slot, load its matrix
  static bool select(const uint8_t slot);   // M3009 S: store as active and load
  static bool load(const uint8_t slot, const bool quiet=false);
  static bool save(const uint8_t slot);
  static bool save_active() { return save(active); }   // after G29
  static void clear(const uint8_t slot);   // M3011
  static void report(const bool in_mm);    // current Marlin grid (fallback)
  static bool report_slot(const uint8_t slot, const bool in_mm);   // M3013 [S] [P]: stored matrix like RF Repetier
};

extern RFZMatrix rf_zmatrix;
