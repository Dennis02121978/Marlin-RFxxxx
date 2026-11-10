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
 * Renkforce (Conrad) RF1000 / RF2000 / RF2000 v2 mainboard, ATmega2560
 *
 * Pin assignment taken from the RF Repetier firmware (pins.h, DEVICE_TYPE_RF1000 /
 * DEVICE_TYPE_RF2000 / DEVICE_TYPE_RF2000v2) and cross-checked with the Klipper
 * configs that run on real RF2000 v2 / RF2000 printers.
 *
 * Special hardware on this board (handled in src/feature/renkforce/):
 *  - 5x TI DRV8711 stepper drivers, configured by software SPI (no MS pins, no VREF pot).
 *    Microsteps and motor current are register values -> M350 / M906 / M122.
 *  - ADS1100 16 bit ADC (I2C 0x49) on the strain gauge (load cell) under the hotend.
 *    Used as Z probe, emergency pause, sense offset and flow compensation.
 *  - Z/E keys and Pause/Continue keys next to the display.
 *  - RF2000: RGB lights, 230V output, 3x 24V FET outputs.
 *  - External watchdog on WDI (RF2000: PA7, RF1000: PC0). It only becomes active once
 *    the pin is driven as output, so it is left alone here (same as with Klipper).
 *
 * RF2000 and RF2000 v2 use identical pins (Repetier pins.h, only mechanics differ).
 */

#include "env_validate.h"

#if MB(RENKFORCE_RF1000)
  #define RF_BOARD_RF1000
  #ifndef BOARD_INFO_NAME
    #define BOARD_INFO_NAME "Renkforce RF1000"
  #endif
#else
  #ifndef BOARD_INFO_NAME
    #define BOARD_INFO_NAME "Renkforce RF2000"
  #endif
#endif

//
// Limit Switches
// Endstops are low active, the board has its own pull-ups (Klipper: "!PC2" without "^").
// Z-Min is the light barrier used for homing, Z-Max (RF2000) is the lower end of the table.
//
#ifdef RF_BOARD_RF1000
  #define X_MIN_PIN                            3  // PE5
  #define Y_MIN_PIN                            2  // PE4
#else
  #define X_MIN_PIN                           35  // PC2
  #define Y_MIN_PIN                           37  // PC0
  #define Z_MAX_PIN                           16  // PH1
#endif
#define Z_MIN_PIN                             31  // PC6

//
// Z probe = strain gauge (load cell). There is no real probe pin; endstops.cpp reads
// the trigger state of the load cell instead of this pin (see RF_LOADCELL_PROBE).
// PJ0 is a spare pin (Repetier DEBUG_TEST_J0_PIN), it is only configured as input.
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     15  // PJ0 (dummy)
#endif

//
// Steppers (DRV8711)
//
#define X_STEP_PIN                            54  // PF0
#define X_DIR_PIN                             55  // PF1
#define X_ENABLE_PIN                          38  // PD7
#define RF_X_CS_PIN                              49  // PL0  (SCS, high active)

#define Y_STEP_PIN                            60  // PF6
#define Y_DIR_PIN                             61  // PF7
#define Y_ENABLE_PIN                          56  // PF2
#define RF_Y_CS_PIN                              47  // PL2

#ifdef RF_BOARD_RF1000
  #define Z_STEP_PIN                          46  // PL3
  #define RF_Z_CS_PIN                            45  // PL4
#else
  #define Z_STEP_PIN                          57  // PF3
  #define RF_Z_CS_PIN                            12  // PB6
#endif
#define Z_DIR_PIN                             48  // PL1
#define Z_ENABLE_PIN                          62  // PK0

#define E0_STEP_PIN                           26  // PA4
#define E0_DIR_PIN                            28  // PA6
#define E0_ENABLE_PIN                         24  // PA2
#define RF_E0_CS_PIN                             43  // PL6

#define E1_STEP_PIN                           36  // PC1
#define E1_DIR_PIN                            34  // PC3
#define E1_ENABLE_PIN                         30  // PC7
#define RF_E1_CS_PIN                             74  // PJ7

//
// DRV8711 bus (software SPI, shared by all five drivers)
//
#define DRV8711_SCLK_PIN                      82  // PD5  SCLK
#define DRV8711_SDATO_PIN                     81  // PD4  SDATO (driver -> MCU)
#define DRV8711_SDATI_PIN                     83  // PD6  SDATI (MCU -> driver)
#define DRV8711_RESET_PIN                     80  // PE7  RESET_DRV (high active)
#define DRV8711_FAULT_PIN                     79  // PE6  FAULTn (low active)

#define RF_X_STALL_PIN                           41  // PG0  STALLn_1
#define RF_Y_STALL_PIN                           40  // PG1  STALLn_2
#define RF_Z_STALL_PIN                           39  // PG2  STALLn_3
#define RF_E0_STALL_PIN                          71  // PG3  STALLn_4
#define RF_E1_STALL_PIN                          70  // PG4  STALLn_5

//
// Temperature Sensors (analog channel numbers)
//
#define TEMP_0_PIN                            13  // PK5  TH1
#define TEMP_1_PIN                            14  // PK6  TH2
#define TEMP_BED_PIN                          15  // PK7  TH3
#ifndef RF_BOARD_RF1000
  #define RF_RESERVE_TEMP_PIN                 12  // PK4  X35 (optional sensor, RF2000 only)
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                          10  // PB4  HZ1
#define HEATER_1_PIN                           9  // PH6  HZ2
#ifdef RF_BOARD_RF1000
  #define HEATER_BED_PIN                       8  // PH5
#else
  #define HEATER_BED_PIN                      78  // PE2  HZ3
#endif

#ifndef FAN0_PIN
  #define FAN0_PIN                            27  // PA5  OUT2 (part cooling fan)
#endif

// Case fan (Repetier FEATURE_CASE_FAN, M3120/M3121), switched by src/feature/renkforce
#ifndef RF_CASE_FAN_PIN
  #ifdef RF_BOARD_RF1000
    #if EXTRUDERS < 2
      #define RF_CASE_FAN_PIN                  9  // PH6 (= heater 2 on a dual RF1000)
    #endif
  #else
    #define RF_CASE_FAN_PIN                   58  // PF4  OUT5 / FET3
  #endif
#endif

//
// Misc. Functions
//
#define SDSS                                  53  // PB0
#ifndef RF_BOARD_RF1000
  #define SD_DETECT_PIN                       19  // PD2 (low = card inserted)
#endif

#ifdef RF_BOARD_RF1000
  #define BEEPER_PIN                          23  // PA1
#else
  #define BEEPER_PIN                           5  // PE3
#endif

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                      25  // PA3  OUT1
#endif

#ifndef RF_BOARD_RF1000
  // RGB lights (Repetier FEATURE_RGB_LIGHT_EFFECTS)
  #if ENABLED(RF_RGB_LIGHT)
  #define RF_RGB_R_PIN                         6  // PH3  L_RT
  #define RF_RGB_G_PIN                         7  // PH4  L_GN
  #define RF_RGB_B_PIN                         8  // PH5  L_BL
  #endif

  // 230V output and 24V FET outputs (Repetier M3300 / M3301)
  #define RF_OUTPUT_230V_PIN                   3  // PE5  P230
  #define RF_FET1_PIN                         33  // PC4  OUT3 X42
  #define RF_FET2_PIN                         32  // PC5  OUT4 X44
  #define RF_FET3_PIN                         58  // PF4  OUT5 X45 (= case fan)
#endif

#define RF_WATCHDOG_PIN                       29  // PA7 (RF2000) - not used
#define RF_I2C_ADDRESS_LOADCELL             0x49  // ADS1100 on the strain gauge
#define RF_I2C_ADDRESS_EXT_EEPROM           0x50  // 24C256 with the Repetier Z matrix (not used)

//
// Keys next to the display (all low active)
//
#define RF_KEY_Z_UP_PIN                       72  // PJ2  TAST_E1  (bed up   = Z smaller)
#define RF_KEY_Z_DOWN_PIN                     75  // PJ4  TAST_E2  (bed down = Z bigger)
#define RF_KEY_E_REV_PIN                      76  // PJ5  TAST_E3  (retract)
#define RF_KEY_E_FWD_PIN                      77  // PJ6  TAST_E4  (extrude)
#define RF_KEY_CONTINUE_PIN                   85  // PH7  TAST_E5
#define RF_KEY_PAUSE_PIN                      84  // PH2  TAST_E6

//
// LCD / Controller: HD44780 20x4 (RF1000: 16x4), 4 bit, five keys
//
#if HAS_WIRED_LCD
  #define LCD_PINS_RS                         63  // PK1
  #define LCD_PINS_EN                         65  // PK3
  #define LCD_PINS_D4                         59  // PF5
  #define LCD_PINS_D5                         64  // PK2
  #ifdef RF_BOARD_RF1000
    #define LCD_PINS_D6                       44  // PL5
    #define LCD_PINS_D7                       66  // PK4
  #else
    #define LCD_PINS_D6                       23  // PA1
    #define LCD_PINS_D7                       73  // PJ3
  #endif

  #if IS_NEWPANEL
    #define BTN_ENC                            4  // PG5  TAST1 "OK"
    #define BTN_BACK                          11  // PB5  TAST4 "links / zurueck"
    #define BTN_UP                            42  // PL7  TAST5
    #ifdef RF_BOARD_RF1000
      #define BTN_DOWN                         5  // PE3  TAST2
      #define RF_KEY_RIGHT_PIN                 6  // PH3  TAST3
    #else
      #define BTN_DOWN                        13  // PB7  TAST2
      #define RF_KEY_RIGHT_PIN                17  // PH0  TAST3
    #endif
  #endif
#endif
