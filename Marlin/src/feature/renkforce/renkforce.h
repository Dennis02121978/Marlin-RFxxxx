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
 * Renkforce RF1000 / RF2000 / RF2000 v2: features of the RF Repetier firmware
 *
 *  - DRV8711 stepper drivers: current (M906), microsteps (M350), status (M122)
 *  - Load cell (ADS1100) as Z probe for G29 (RF heat bed scan)
 *  - Emergency pause when the load cell force is too high (M3075)
 *  - Sense offset: lift Z in the first layer when the nozzle presses the bed (M3909)
 *  - Digit flow compensation: flow / speed depending on the extrusion pressure (M3911)
 *  - Z keys, E keys, Pause and Continue keys next to the display
 *  - Case fan (M3120 / M3121), 24V FET outputs (M3300), 230V output (M3301)
 *
 * G-codes are dispatched by rf_process_mcode() (see gcode.cpp).
 */

#include "../../inc/MarlinConfigPre.h"
#include "drv8711.h"
#include "loadcell.h"
#include "zmatrix.h"
#include "rgblight.h"

typedef struct {
  uint16_t drv_current[RF_DRV_COUNT];     // mA            M906
  uint16_t drv_microsteps[RF_DRV_COUNT];  // 1..256        M350
  int16_t  probe_trigger;                 // digits        M3050
  int16_t  pause_min, pause_max;          // digits, 0/0 = off  M3075 L H
  int16_t  pause_pos[3];                  // mm, X Y Z of the pause position (Z = min. height)
  bool     sense_auto;                    // sense offset on at every print start
  int16_t  sense_threshold;               // digits        M3909 P
  uint16_t sense_max_um;                  // um            M3909 S
  int16_t  flow_min, flow_max;            // digits        M3911 L H
  int8_t   flow_percent, feed_percent;    // %, 0 = off    M3911 S F
  uint8_t  case_fan_mode;                 // 0 auto, 1 on, 2 off  M3120 / M3121
  uint8_t  rgb_mode;                      // 0 off, 1 white, 2 auto, 3 manual  M3308
  uint8_t  rgb[5][3];                     // heating, printing, cooling, idle, manual  M3303..M3307
  bool     case_light;                    // X19 light at power on  M355
} rf_settings_t;

class Renkforce {
public:
  static rf_settings_t settings;
  static bool ready;                   // init() done

  // Sense offset (first layer)
  static bool sense_enabled;
  static float sense_offset;           // mm, applied as Z babysteps

  // Digit flow compensation
  static float dyn_flow, dyn_feed;     // current factors (1.0 = off)

  static void reset_settings();
  static void before_save();           // M500: take over runtime states (X19 light)
  static void apply_settings();        // after M501 / boot
  static void report_settings(const bool forReplay);

  static void init();                  // after stepper.init()
  static void idle();                  // from idle()
  static void on_homing();             // G28 start

  static void pause_print(const bool emergency);
  static void continue_print();

  static void set_case_fan_mode(const uint8_t m);
  static void set_output(const uint8_t which, const bool on);   // 1..3 = FET1..3, 0 = 230V

  // Microsteps changed: keep mm, rescale steps/mm (RF Repetier FEATURE_ADJUSTABLE_MICROSTEPS)
  static void change_microsteps(const uint8_t d, const uint16_t ms);

  static void sense_enable(const int16_t threshold, const uint16_t max_um);
  static uint8_t status_page;          // RF info page on the status screen
  static void status_page_step(const int16_t dir);
  static void draw_status();           // HD44780 status screen (RF Repetier pages)
  static void z_offset_scan();         // M3900 (hbscan.cpp)
  static void heatbed_scan(const uint8_t mode);   // M3010 / G29: RF Repetier heat bed scan, 0 normal, 1 PLA, 2 ABS
  static xy_pos_t scan_grid_start();   // Marlin coordinates of the first scan point
  static xy_pos_t scan_grid_spacing();
  static void sense_disable();

private:
  static void handle_keys(const millis_t ms);
  static void handle_sample();
  static void handle_case_fan(const millis_t ms);
  static void handle_flow(const millis_t ms);
  static void sense_sample(const int16_t v);
  static void emergency_sample(const int16_t v);
  static void z_drive_start(const int8_t dir);
  static void z_drive_stop();
  static bool jog(const AxisEnum axis, const float dist, const feedRate_t fr_mm_s);
};

extern Renkforce rf;

bool rf_process_mcode(const uint16_t code);   // true when the M-code was handled

#if HAS_MARLINUI_MENU
  void menu_renkforce();
#endif
