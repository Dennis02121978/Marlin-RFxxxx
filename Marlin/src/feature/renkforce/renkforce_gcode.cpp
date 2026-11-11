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

/**
 * G-codes of the Renkforce RF port. Numbers follow the RF Repetier firmware where it has them.
 *
 *  M122 [S1]                 DRV8711 status of all drivers (S1: clear the flags)
 *  M350 [X] [Y] [Z] [E] [S]  microsteps 1..256 (steps/mm are rescaled, position stays)
 *  M906 [X] [Y] [Z] [E] [T]  motor current in mA (RF Repetier: menu "Motor current")
 *  M3000 / M3001             Z compensation (bed leveling) off / on
 *  M3002 [T1]                read the load cell (T1: set the zero point now)
 *  M3010 [S]                 heat bed scan like RF Repetier (S1 PLA, S2 ABS), also G29
 *  M3013 [S] [P1]            print matrix S (default active) like RF Repetier, P1 = in mm
 *  M3050 [S]                 probe: contact pressure delta in digits
 *  M3060                     device type and firmware version
 *  M3070                     pause the print like the Pause key
 *  M3075 [S]                 emergency pause limit in digits, S0 = off
 *  M3079                     output the printed object (bed down and to the front)
 *  M3120 [S0]                case fan on (S0: automatic)
 *  M3121                     case fan off
 *  M3300 P<1..3> S<0|1>      24V FET outputs (FET3 = case fan)
 *  M3301 S<0|1>              230V output
 *  M3909 [P] [S] [D]         sense offset: P threshold digits, S max offset um. P0 = off.
 *                            Switches on for the running print unless D1 (only store the values).
 *  M3911 [S] [F] [L] [H]     digit flow compensation: S flow %, F speed %, L/H digits range
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(RENKFORCE_RF)

#include <Wire.h>
#include "renkforce.h"
#include "../../gcode/gcode.h"
#include "../../gcode/queue.h"
#include "../../module/planner.h"
#include "../../module/motion.h"
#include "../../MarlinCore.h"
#if HAS_LEVELING
  #include "../bedlevel/bedlevel.h"
#endif

static const char axis_drv[] PROGMEM = "XYZ";

static void drv_from_params(void (*apply)(const uint8_t d, const uint16_t v)) {
  for (uint8_t i = 0; i < 3; ++i) {
    const char c = pgm_read_byte(&axis_drv[i]);
    if (parser.seenval(c)) apply(i, parser.value_ushort());
  }
  if (parser.seenval('E')) {
    const uint16_t v = parser.value_ushort();
    if (parser.seen('T')) {
      const uint8_t t = parser.value_byte();
      if (t <= 1) apply(RF_DRV_E0 + t, v);
    }
    else {
      apply(RF_DRV_E0, v);
      apply(RF_DRV_E1, v);
    }
  }
}

static void report_drivers(const bool clear) {
  for (uint8_t d = 0; d < RF_DRV_COUNT; ++d) {
    if (d == RF_DRV_E1 && DISABLED(HAS_MULTI_EXTRUDER)) continue;
    rf_drv.report_status(d);
    if (clear) rf_drv.clear_status(d);
  }
  SERIAL_ECHOPGM("DRV8711 FAULT: ");
  SERIAL_ECHOLNF(rf_drv.fault() ? F("aktiv") : F("nein"));
}

bool rf_process_mcode(const uint16_t code) {
  switch (code) {

    case 122: report_drivers(parser.boolval('S')); break;

    case 350: {
      if (parser.seenval('S')) {
        const uint16_t ms = parser.value_ushort();
        for (uint8_t d = 0; d < RF_DRV_COUNT; ++d) rf.change_microsteps(d, ms);
      }
      drv_from_params([](const uint8_t d, const uint16_t v) { rf.change_microsteps(d, v); });
      SERIAL_ECHOLNPGM("Mikroschritte X", rf.settings.drv_microsteps[RF_DRV_X], " Y", rf.settings.drv_microsteps[RF_DRV_Y],
                       " Z", rf.settings.drv_microsteps[RF_DRV_Z], " E0 ", rf.settings.drv_microsteps[RF_DRV_E0],
                       " E1 ", rf.settings.drv_microsteps[RF_DRV_E1]);
    } break;

    case 906: {
      const bool any = parser.seen("XYZE");
      if (any) drv_from_params([](const uint8_t d, const uint16_t v) { rf_drv.set_current(d, v); });
      SERIAL_ECHOLNPGM("Motorstrom mA X", rf.settings.drv_current[RF_DRV_X], " Y", rf.settings.drv_current[RF_DRV_Y],
                       " Z", rf.settings.drv_current[RF_DRV_Z], " E0 ", rf.settings.drv_current[RF_DRV_E0],
                       " E1 ", rf.settings.drv_current[RF_DRV_E1]);
    } break;

    #if HAS_LEVELING
      case 3000: set_bed_leveling_enabled(false); SERIAL_ECHOLNPGM("Z-Kompensation aus"); break;
      case 3001:
        set_bed_leveling_enabled(true);
        SERIAL_ECHOPGM("Z-Kompensation ");
        SERIAL_ECHOLNF(planner.leveling_active ? F("an") : F("nicht moeglich (kein Scan)"));
        break;
      case 3010: rf.heatbed_scan(parser.byteval('S')); break;   // RF Repetier heat bed scan, result in the active matrix
    #endif

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      case 3009:   // M3009 [S] - get/choose the active heat bed z-compensation matrix
        if (parser.seenval('S')) {
          if (printingIsActive()) { SERIAL_ECHOLNPGM("M3009: nicht waehrend eines Drucks"); break; }
          const uint8_t s = parser.value_byte();
          if (!WITHIN(s, 1, RF_ZMATRIX_SLOTS)) { SERIAL_ECHOLNPGM("M3009: ungueltige Matrix (1..", RF_ZMATRIX_SLOTS, ")"); break; }
          rf_zmatrix.select(s);
        }
        SERIAL_ECHOLNPGM("M3009: aktive Heizbett-Matrix: ", rf_zmatrix.active);
        break;
      case 3011: {   // M3011 [S] - clear the specified (or active) z-compensation matrix
        if (printingIsActive()) { SERIAL_ECHOLNPGM("M3011: nicht waehrend eines Drucks"); break; }
        const uint8_t s = parser.seenval('S') ? parser.value_byte() : rf_zmatrix.active;
        if (!WITHIN(s, 1, RF_ZMATRIX_SLOTS)) { SERIAL_ECHOLNPGM("M3011: ungueltige Matrix (1..", RF_ZMATRIX_SLOTS, ")"); break; }
        rf_zmatrix.clear(s);
      } break;
      case 3013: {   // M3013 [S<1..9>] [P1] - like RF Repetier: S = matrix slot (default active), P1 = values in mm
        const uint8_t slot = parser.byteval('S', rf_zmatrix.active);
        const bool mm = parser.boolval('P');
        if (!rf_zmatrix.report_slot(slot, mm)) {
          if (!parser.seen('S')) rf_zmatrix.report(mm);   // no EEPROM: current grid
          else SERIAL_ECHOLNPGM("M3013: ungueltige Matrix ", slot);
        }
      } break;
      case 3014: rf_zmatrix.save(parser.seenval('S') ? parser.value_byte() : rf_zmatrix.active); break;   // store the current grid
    #endif

    case 3002:
      if (parser.boolval('T') && rf_loadcell.ok) rf_loadcell.tare_offset = rf_loadcell.raw;
      SERIAL_ECHOPGM("DMS roh ", rf_loadcell.raw, " genullt ", rf_loadcell.value(), " Null ", rf_loadcell.tare_offset);
      SERIAL_ECHOF(rf_loadcell.ok ? F(" ok") : F(" FEHLER"));
      SERIAL_ECHOLNF(rf_loadcell.triggered ? F(" (ausgeloest)") : F(""));
      break;

    case 3050:
      if (parser.seenval('S')) rf.settings.probe_trigger = constrain(parser.value_int(), 5, 5000);
      SERIAL_ECHOLNPGM("Sonde: Kontakt-Delta ", rf.settings.probe_trigger, " digits");
      break;

    case 3060: {
      SERIAL_ECHOLNPGM("Device: " RF_MACHINE_NAME ", Firmware: Marlin " SHORT_BUILD_VERSION " RF-Port");
      // I2C devices of the RF board: load cell, Z matrix EEPROM (Repetier), type EEPROM
      static const uint8_t addr[] PROGMEM = { RF_I2C_ADDRESS_LOADCELL, RF_I2C_ADDRESS_EXT_EEPROM, 0x51 };
      static const char n0[] PROGMEM = "DMS ADS1100", n1[] PROGMEM = "EEPROM Z-Matrix 24C256", n2[] PROGMEM = "EEPROM Geraetetyp";
      static PGM_P const names[] PROGMEM = { n0, n1, n2 };
      for (uint8_t i = 0; i < COUNT(addr); ++i) {
        const uint8_t a = pgm_read_byte(&addr[i]);
        Wire.beginTransmission(a);
        const bool found = Wire.endTransmission() == 0;
        SERIAL_ECHOPGM("I2C 0x");
        SERIAL_PRINT(a, PrintBase::Hex);
        SERIAL_CHAR(' ');
        SERIAL_ECHOPGM_P((PGM_P)pgm_read_ptr(&names[i]));
        SERIAL_ECHOLNF(found ? F(": vorhanden") : F(": keine Antwort"));
      }
    } break;

    case 3070: rf.pause_print(false); break;

    case 3075:
      // S<n>: +-n (S0 = off), L<min> H<max> separately (RF Repetier EMERGENCY_PAUSE_DIGITS_MIN/MAX)
      if (parser.seenval('S')) { const int16_t v = constrain(parser.value_int(), 0, 32000); rf.settings.pause_min = -v; rf.settings.pause_max = v; }
      if (parser.seenval('L')) rf.settings.pause_min = constrain(parser.value_int(), -32000, 0);
      if (parser.seenval('H')) rf.settings.pause_max = constrain(parser.value_int(), 0, 32000);
      SERIAL_ECHOLNPGM("Notfall-Pause: ", rf.settings.pause_min, " .. ", rf.settings.pause_max, " digits (0/0 = aus)");
      break;

    case 3079:   // output the object like RF Repetier OUTPUT_OBJECT_SCRIPT_PRINT: bed down, then Y 245
      if (all_axes_trusted()) {
        do_blocking_move_to_z(_MAX(current_position.z, float(RF_OUTPUT_OBJECT_Z)), homing_feedrate(Z_AXIS));
        do_blocking_move_to_xy(current_position.x, float(RF_OUTPUT_OBJECT_Y), homing_feedrate(Y_AXIS));
      }
      break;

    case 3105:   // M3105 [X] [Y] [Z] - pause position (mm, Z = minimum height)
      if (parser.seenval('X')) rf.settings.pause_pos[0] = constrain(parser.value_int(), int16_t(X_MIN_POS), int16_t(X_MAX_POS));
      if (parser.seenval('Y')) rf.settings.pause_pos[1] = constrain(parser.value_int(), int16_t(Y_MIN_POS), int16_t(Y_MAX_POS));
      if (parser.seenval('Z')) rf.settings.pause_pos[2] = constrain(parser.value_int(), 0, int16_t(Z_MAX_POS));
      SERIAL_ECHOLNPGM("Pause Position X", rf.settings.pause_pos[0], " Y", rf.settings.pause_pos[1], " Z", rf.settings.pause_pos[2]);
      break;

    case 3900: rf.z_offset_scan(); break;   // M3900 - Z offset scan: measure one point, shift the matrix

    case 3120: rf.set_case_fan_mode(parser.seen('S') && !parser.value_bool() ? 0 : 1); break;
    case 3121: rf.set_case_fan_mode(2); break;

    case 3300:
      if (parser.seenval('P')) rf.set_output(constrain(parser.value_byte(), 1, 3), parser.boolval('S'));
      break;
    case 3301: rf.set_output(0, parser.boolval('S')); break;

    case 3909: {
      const int16_t thr = parser.seenval('P') ? constrain(parser.value_int(), 0, 32000) : rf.settings.sense_threshold;
      const uint16_t um = parser.seenval('S') ? constrain(parser.value_int(), 0, 1000) : rf.settings.sense_max_um;
      if (parser.boolval('D')) { rf.settings.sense_threshold = thr; rf.settings.sense_max_um = um; }
      else if (thr == 0) { rf.settings.sense_threshold = 0; rf.sense_disable(); }
      else rf.sense_enable(thr, um);
      if (parser.seen('A')) rf.settings.sense_auto = parser.value_bool();
      SERIAL_ECHOPGM("Sense Offset ");
      SERIAL_ECHOF(rf.sense_enabled ? F("an") : F("aus"));
      SERIAL_ECHOLNPGM(": Schwelle ", rf.settings.sense_threshold,
                       " digits, max ", rf.settings.sense_max_um, " um, aktuell ", int16_t(rf.sense_offset * 1000), " um");
    } break;

    case 3911:
      if (parser.seenval('S')) rf.settings.flow_percent = constrain(parser.value_int(), -99, 99);
      if (parser.seenval('F')) rf.settings.feed_percent = constrain(parser.value_int(), -90, 99);
      if (parser.seenval('L')) rf.settings.flow_min = constrain(parser.value_int(), 0, 32000);
      if (parser.seenval('H')) rf.settings.flow_max = constrain(parser.value_int(), 0, 32000);
      SERIAL_ECHOLNPGM("Fluss-Kompensation: Fluss ", rf.settings.flow_percent, "% Tempo ", rf.settings.feed_percent,
                       "% von ", rf.settings.flow_min, " bis ", rf.settings.flow_max, " digits");
      break;

    #if PIN_EXISTS(RF_RGB_R)
      // M3303 heating, M3304 printing, M3305 cooling, M3306 idle, M3307 manual colors
      //   RF Repetier: P1..3 = red/green/blue, S0..255   or   R U B (like M150)
      case 3303: case 3304: case 3305: case 3306: case 3307: {
        uint8_t * const c = rf.settings.rgb[code - 3303];
        if (parser.seenval('P')) {
          const uint8_t p = parser.value_byte();
          if (WITHIN(p, 1, 3) && parser.seenval('S')) c[p - 1] = constrain(parser.value_int(), 0, 255);
        }
        if (parser.seenval('R')) c[0] = constrain(parser.value_int(), 0, 255);
        if (parser.seenval('U')) c[1] = constrain(parser.value_int(), 0, 255);
        if (parser.seenval('B')) c[2] = constrain(parser.value_int(), 0, 255);
        rf_rgb.colors_changed();
        SERIAL_ECHO(int(code));
        SERIAL_ECHOLNPGM(" RGB ", c[0], " ", c[1], " ", c[2]);
      } break;

      case 3308:   // M3308 P - RGB light mode 0 off, 1 white, 2 auto, 3 manual
        if (parser.seenval('P')) rf_rgb.set_mode(parser.value_byte());
        SERIAL_ECHOPGM("RGB Licht: ");
        SERIAL_ECHOLNF(rf_rgb.mode_name());
        break;

      case 150:    // M150 R U B - set the manual color and switch to manual mode
        rf.settings.rgb[RF_RGB_C_MANUAL][0] = parser.seen('R') ? (parser.has_value() ? constrain(parser.value_int(), 0, 255) : 255) : 0;
        rf.settings.rgb[RF_RGB_C_MANUAL][1] = parser.seen('U') ? (parser.has_value() ? constrain(parser.value_int(), 0, 255) : 255) : 0;
        rf.settings.rgb[RF_RGB_C_MANUAL][2] = parser.seen('B') ? (parser.has_value() ? constrain(parser.value_int(), 0, 255) : 255) : 0;
        rf_rgb.set_mode(RF_RGB_MANUAL);
        break;
    #endif

    default: return false;
  }
  return true;
}

#endif // RENKFORCE_RF
