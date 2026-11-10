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
 * RGB light of the RF2000 / RF2000 v2 like the RF Repetier firmware (FEATURE_RGB_LIGHT_EFFECTS):
 *
 *  Modes (M3308 P):  0 Aus, 1 Weiss, 2 Auto (status colors), 3 Manuell
 *  Auto:             heating, printing (target reached), cooling, idle;
 *                    after 30 s idle a slow color change (red, yellow, green, cyan, blue, magenta)
 *  Colors:           M3303 heating, M3304 printing, M3305 cooling, M3306 idle, M3307 manual (P1..3 = R/G/B, S0..255)
 *  Back key on the status screen toggles "always white".
 *  All colors fade by one step every 10 ms (Timer 4 PWM, 100 Hz, as in Repetier).
 *  After power on the light starts dark and fades into the color of the stored mode.
 */

#include "../../inc/MarlinConfigPre.h"

#define RF_RGB_OFF     0
#define RF_RGB_WHITE   1
#define RF_RGB_AUTO    2
#define RF_RGB_MANUAL  3

// index of the color sets in rf_settings_t.rgb
#define RF_RGB_C_HEATING   0
#define RF_RGB_C_PRINTING  1
#define RF_RGB_C_COOLING   2
#define RF_RGB_C_IDLE      3
#define RF_RGB_C_MANUAL    4
#define RF_RGB_COLORS      5

class RFRgbLight {
public:
  static bool force_white;              // back key on the status screen

  static void init();                   // Timer 4 PWM, start dark, fade into the mode color
  static void idle(const millis_t ms);
  static void set_mode(const uint8_t m);
  static void colors_changed();         // M3303..M3307 / menu: show the new color at once
  static void toggle_white();
  static FSTR_P mode_name();

private:
  static uint8_t status;
  static uint8_t cur[3], tgt[3];
  static millis_t fade_ms, status_ms, idle_start_ms;
  static void target(const uint8_t r, const uint8_t g, const uint8_t b) { tgt[0] = r; tgt[1] = g; tgt[2] = b; }
  static void target(const uint8_t c);
  static void output();
  static void update_status(const millis_t ms);
};

extern RFRgbLight rf_rgb;
