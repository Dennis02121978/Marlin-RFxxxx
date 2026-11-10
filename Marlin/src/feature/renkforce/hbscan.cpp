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
 * Heat bed scan and Z offset scan with the algorithm of the RF Repetier firmware
 * (RF.cpp scanHeatBed, searchZOScan, moveZMinusUpFast/Slow, moveZPlusDownFast/Slow,
 *  readIdlePressure, readAveragePressure, prepareCompensationMatrix, adjustCompensationMatrix).
 *
 * Every point:
 *   - read the idle pressure of the load cell (average of several reads, spread < 15 digits)
 *   - move the bed up in 0.02 mm steps until the pressure leaves idle +-10 digits  (contact)
 *   - move down in 0.004 mm steps until the pressure is back within idle +-5 digits
 *   - move up in 0.004 mm steps until contact again
 *   - move down in single steps until idle +-5 digits again  ->  this Z is the value of the point
 *   - move the bed down 0.1 mm and go to the next point (serpentine, X outer, Y inner)
 *   Errors (Z out of range, bed too uneven, no constant pressure) retry the point up to 5 times:
 *   lift, home Y, home Z, return to the point, read the idle pressure again.
 *
 * Grid: X from RF_SCAN_X_START to the axis end - RF_SCAN_X_END, Y (RF Repetier coordinates, from the
 * Y switch) from 30 to the axis end - 5, both every 20 mm. The Marlin grid is exactly this grid.
 * The result is stored like RF Repetier (border columns at 0 and axis length) in the active matrix
 * of the 24C256 and used as Marlin bilinear mesh.
 *
 * Modes (M3010 S / G29 S): 0 normal, 1 PLA (bed 60, hotend 100, 10 min warm up, after the scan
 * hotend 210 for 10 min and correction at the calibration point), 2 ABS (bed 100, hotend 240).
 */

#include "../../inc/MarlinConfig.h"

#if ALL(RENKFORCE_RF, AUTO_BED_LEVELING_BILINEAR)

#include "renkforce.h"
#include "../../MarlinCore.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/stepper.h"
#include "../../module/temperature.h"
#include "../../gcode/gcode.h"
#include "../../lcd/marlinui.h"
#include "../bedlevel/bedlevel.h"
#include "../../libs/buzzer.h"

// ---- RF Repetier RF2000.h / RF1000.h / RF2000v2.h ----------------------------------------------
#define HBS_Z_START_MM          1.0f     // HEAT_BED_SCAN_Z_START_MM
#define HBS_UP_FAST_MM          0.02f    // HEAT_BED_SCAN_UP_FAST_MM
#define HBS_DOWN_FAST_MM        0.1f     // HEAT_BED_SCAN_DOWN_FAST_MM
#define HBS_UP_SLOW_MM          0.004f   // HEAT_BED_SCAN_UP_SLOW_MM
#define HBS_DOWN_SLOW_MM        0.004f   // HEAT_BED_SCAN_DOWN_SLOW_MM
#define HBS_SLOW_STEP_DELAY_MS  100      // HEAT_BED_SCAN_SLOW_STEP_DELAY_MS
#define HBS_IDLE_DELAY_MS       250      // HEAT_BED_SCAN_IDLE_DELAY_MS
#define HBS_DELAY_MS            1000     // HEAT_BED_SCAN_DELAY
#define HBS_CONTACT_DELTA       10       // HEAT_BED_SCAN_CONTACT_PRESSURE_DELTA
#define HBS_RETRY_DELTA         5        // HEAT_BED_SCAN_RETRY_PRESSURE_DELTA
#define HBS_IDLE_MIN            -7500    // HEAT_BED_SCAN_IDLE_PRESSURE_MIN
#define HBS_IDLE_MAX            7500     // HEAT_BED_SCAN_IDLE_PRESSURE_MAX
#define HBS_RETRIES             5        // HEAT_BED_SCAN_RETRIES
#define HBS_PRESSURE_TOLERANCE  15       // HEAT_BED_SCAN_PRESSURE_TOLERANCE
// RF Repetier reads 15 times every 15 ms (225 ms). Here the ADS1100 delivers RF_LOADCELL_SPS samples/s,
// 3 samples at 16 SPS = ~190 ms, so one step is not slower than in RF Repetier.
#define HBS_PRESSURE_READS      _MAX(2, (RF_LOADCELL_SPS) * 3 / 16)
#define HBS_WARMUP_S            600      // PRECISE_HEAT_BED_SCAN_WARMUP_DELAY
#define HBS_CALIBRATION_S       600      // PRECISE_HEAT_BED_SCAN_CALIBRATION_DELAY
#define HBS_CAL_X_MM            100      // HEAT_BED_SCAN_X_CALIBRATION_POINT_MM
#define HBS_CAL_Y_MM            100      // HEAT_BED_SCAN_Y_CALIBRATION_POINT_MM (RF Repetier Y)
#define ZOS_CONTACT_DELTA       33       // SEARCH_HEAT_BED_OFFSET_CONTACT_PRESSURE_DELTA
#define ZOS_RETRY_DELTA         30       // SEARCH_HEAT_BED_OFFSET_RETRY_PRESSURE_DELTA
#define ZOS_RETRIES             20
#define ZOS_RAND_MM             3        // SEARCH_HEAT_BED_OFFSET_SCAN_POSITION_RAND_MM

namespace {

  enum Result : uint8_t { R_OK, R_RETRY, R_ABORT };

  int32_t zsteps;                       // Z position in steps from the Z switch (g_nAxisScanPosition)
  int32_t last_z;                       // g_nLastZScanZPosition, 0 = none
  bool    has_last;
  int16_t idle_p, c_min, c_max, r_min, r_max;
  uint8_t retries;
  FSTR_P  reason;
  bool    aborted;

  float   spm()               { return planner.settings.axis_steps_per_mm[Z_AXIS]; }
  int32_t mm_steps(const float mm) { return int32_t(mm * spm()); }   // like RF Repetier: truncated

  void limits(const int16_t idle, const int16_t contact, const int16_t retry) {
    idle_p = idle;
    c_min = idle - contact; c_max = idle + contact;
    r_min = idle - retry;   r_max = idle + retry;
  }

  // Back key held for 1 s: cancel (RF Repetier: start the scan again)
  bool user_abort() {
    #if PIN_EXISTS(BTN_BACK)
      static millis_t down_ms;
      if (READ(BTN_BACK)) down_ms = 0;
      else if (!down_ms) down_ms = millis();
      else if (ELAPSED(millis(), down_ms + 1000)) { reason = F("abgebrochen"); aborted = true; }
    #endif
    return aborted;
  }

  void wait_ms(const millis_t ms) {
    const millis_t end = millis() + ms;
    while (PENDING(millis(), end) && !user_abort()) { gcode.reset_stepper_timeout(); idle(); }
  }

  void zmove(const int32_t steps) {
    if (!steps) return;
    zsteps += steps;
    current_position.z = zsteps / spm();
    line_to_current_position(homing_feedrate(Z_AXIS));
    // RF Repetier steps the motor at once. Marlin holds the first block of an empty queue back
    // for 100 ms (to merge moves) - not wanted for the many small scan steps.
    planner.delay_before_delivering = 0;
    planner.synchronize();
  }

  void zmove_to(const int32_t steps) { zmove(steps - zsteps); }

  void xy_to(const float x, const float y) {
    do_blocking_move_to_xy(x, y, _MIN(homing_feedrate(X_AXIS), homing_feedrate(Y_AXIS)));
  }

  // readAveragePressure
  bool read_avg(int16_t &out) {
    for (uint8_t attempt = 0; attempt < 5; ++attempt) {
      int32_t sum = 0;
      int16_t mn = 32767, mx = -32768;
      for (uint8_t i = 0; i < HBS_PRESSURE_READS; ++i) {
        const uint8_t s = rf_loadcell.seq;
        const millis_t end = millis() + 1000;
        while (rf_loadcell.seq == s) {
          if (ELAPSED(millis(), end) || !rf_loadcell.ok) { reason = F("DMS antwortet nicht"); return false; }
          idle();
        }
        const int16_t v = rf_loadcell.raw;
        sum += v; NOMORE(mn, v); NOLESS(mx, v);
      }
      if (mx - mn < HBS_PRESSURE_TOLERANCE) { out = int16_t(sum / HBS_PRESSURE_READS); return true; }
      wait_ms(100);
      if (aborted) return false;
    }
    reason = F("Druck nicht konstant");
    return false;
  }

  // readIdlePressure
  bool read_idle(int16_t &out) {
    if (!read_avg(out)) return false;
    int16_t prev = 0;
    for (uint8_t n = 0; ABS(prev - out) > 5; ) {
      if (++n >= 5) { reason = F("Leerlaufdruck schwankt"); return false; }
      prev = out;
      if (!read_avg(out)) return false;
      wait_ms(500);
    }
    if (!WITHIN(out, HBS_IDLE_MIN, HBS_IDLE_MAX)) { reason = F("Leerlaufdruck ausserhalb"); return false; }
    return true;
  }

  inline bool contact(const int16_t p) { return p > c_max || p < c_min; }
  inline bool released(const int16_t p) { return p < r_max && p > r_min; }

  Result fail(FSTR_P why) { reason = why; return retries ? R_RETRY : R_ABORT; }

  int32_t max_comp_steps() { return mm_steps(RF_SCAN_Z_DRIVE_OVER); }

  // moveZMinusUpFast: bed up until contact
  Result up_fast() {
    for (;;) {
      if (zsteps <= -max_comp_steps()) return fail(F("Z ausserhalb (max. Kompensation)"));
      int16_t p;
      if (!read_avg(p)) return aborted ? R_ABORT : fail(reason);
      if (contact(p)) return R_OK;
      zmove(-mm_steps(HBS_UP_FAST_MM));
      if (user_abort()) return R_ABORT;
    }
  }

  // moveZMinusUpSlow
  Result up_slow(const uint8_t acuteness) {
    const int32_t st = _MAX(1L, mm_steps(HBS_UP_SLOW_MM) / acuteness);
    for (;;) {
      if (zsteps <= -max_comp_steps()) return fail(F("Z ausserhalb (max. Kompensation)"));
      wait_ms(HBS_SLOW_STEP_DELAY_MS);
      int16_t p;
      if (!read_avg(p)) return aborted ? R_ABORT : fail(reason);
      if (contact(p)) return R_OK;
      zmove(-st);
      if (user_abort()) return R_ABORT;
    }
  }

  // moveZPlusDownSlow
  Result down_slow(const uint8_t acuteness) {
    const int32_t start = zsteps, st = _MAX(1L, mm_steps(HBS_DOWN_SLOW_MM) / acuteness),
                  fast = mm_steps(HBS_DOWN_FAST_MM);
    for (;;) {
      wait_ms(HBS_SLOW_STEP_DELAY_MS);
      int16_t p;
      if (!read_avg(p)) return aborted ? R_ABORT : fail(reason);
      if (released(p)) return R_OK;
      zmove(st);
      if (user_abort()) return R_ABORT;
      const uint8_t used = HBS_RETRIES - retries;   // allow a bit more after retries (curved ceramics)
      if (zsteps > mm_steps(HBS_Z_START_MM)) return fail(F("Z ueber Startposition"));
      if (has_last && ABS(zsteps - last_z) > fast * (2 + (retries < HBS_RETRIES ? _MIN(used, 2) : 0)))
        return fail(F("Heizbett zu uneben"));
      if (ABS(start - zsteps) > fast * 2 / acuteness) return fail(F("Z-Schritt ausserhalb"));
    }
  }

  // moveZPlusDownFast
  bool down_fast() {
    last_z = zsteps; has_last = true;
    zmove(mm_steps(HBS_DOWN_FAST_MM));
    int16_t p;
    return read_avg(p);
  }

  // scanHeatBed states 50..54: one point, result in zsteps
  Result measure_point() {
    wait_ms(HBS_IDLE_DELAY_MS);
    int16_t p;
    if (!read_avg(p)) return aborted ? R_ABORT : fail(reason);   // testIdlePressure
    limits(p, HBS_CONTACT_DELTA, HBS_RETRY_DELTA);
    Result r;
    if ((r = up_fast()) != R_OK) return r;
    if ((r = down_slow(1)) != R_OK) return r;
    if ((r = up_slow(1)) != R_OK) return r;
    if ((r = down_slow(8)) != R_OK) return r;
    return R_OK;
  }

  void home(FSTR_P axes) {
    gcode.process_subcommands_now(axes);
    zsteps = LROUND(current_position.z * spm());
  }

  // scanHeatBed states 45..48: lift, home Y, home Z, back to the point, idle pressure again
  bool retry_point(const float x, const float y) {
    SERIAL_ECHOPGM("HBS: Wiederholung, noch ", retries, " - ");
    SERIAL_ECHOLNF(reason);
    --retries;
    zmove(mm_steps(HBS_Z_START_MM));
    home(F("G28 Y"));
    home(F("G28 Z"));
    zmove_to(mm_steps(HBS_Z_START_MM));
    has_last = false;
    xy_to(x, y);
    wait_ms(HBS_DELAY_MS);
    int16_t p;
    if (!read_idle(p)) return false;
    limits(p, HBS_CONTACT_DELTA, HBS_RETRY_DELTA);
    return true;
  }

  // one point with retries (scanHeatBed 49..54 / 45..48)
  bool scan_point(const float x, const float y, const uint8_t n_retries) {
    retries = n_retries;
    for (;;) {
      const Result r = measure_point();
      if (r == R_OK) return true;
      if (r == R_ABORT || aborted || !retries) return false;
      if (!retry_point(x, y)) return false;
    }
  }

  void status(FSTR_P s) { ui.set_status(s); SERIAL_ECHOLNF(s); }

  // heat up and wait (PLA / ABS scan)
  bool heat_wait(const celsius_t bed, const celsius_t hotend, const uint16_t secs) {
    #if HAS_HEATED_BED
      if (bed) thermalManager.setTargetBed(bed);
    #endif
    HOTEND_LOOP() thermalManager.setTargetHotend(hotend, e);
    status(F("Heizen..."));
    #if HAS_HEATED_BED
      if (bed && !thermalManager.wait_for_bed(true)) { reason = F("Heizen abgebrochen"); return false; }
    #endif
    HOTEND_LOOP() if (!thermalManager.wait_for_hotend(e, true)) { reason = F("Heizen abgebrochen"); return false; }
    // RF Repetier: let the temperature settle (status "Heizen ... [s]")
    for (uint16_t s = secs; s; --s) {
      char msg[24];
      sprintf_P(msg, PSTR("Heizen %3u[s]"), s);
      ui.set_status(msg);
      wait_ms(1000);
      if (aborted) return false;
    }
    return true;
  }

  void heaters_off() {
    thermalManager.disable_all_heaters();
  }

  // Z of the matrix at the current position in steps (getZMatrixDepth_CurrentXY)
  int32_t matrix_steps_here() {
    return LROUND(bedlevel.get_z_correction(current_position) * spm());
  }

  void end_moves() {
    zmove(mm_steps(5));                  // do not crash during homing
    home(F("G28"));
  }

  struct SoftEndstopsOff {
    #if HAS_SOFTWARE_ENDSTOPS
      const bool was = soft_endstop._enabled;
      SoftEndstopsOff() { soft_endstop._enabled = false; }
      ~SoftEndstopsOff() { soft_endstop._enabled = was; }
    #endif
  };

} // namespace

xy_pos_t Renkforce::scan_grid_start() {
  return { float(RF_SCAN_X_START), float(RF_SCAN_Y_START - RF_Y_HOME) };   // Marlin Y = RF Repetier Y - RF_Y_HOME
}
xy_pos_t Renkforce::scan_grid_spacing() { return { float(RF_SCAN_STEP), float(RF_SCAN_STEP) }; }

//
// M3010 [S] / G29 [S] - heat bed scan
//
void Renkforce::heatbed_scan(const uint8_t mode) {
  if (!rf_loadcell.ok) { SERIAL_ERROR_MSG("HBS: DMS (0x49) antwortet nicht"); return; }
  KEEPALIVE_STATE(IN_PROCESS);
  aborted = false; reason = nullptr; has_last = false;
  const millis_t t0 = millis();
  SERIAL_ECHOLNPGM("HBS: Heizbett-Scan (RF Repetier), Modus ", mode, ", Matrix ", rf_zmatrix.active);
  sense_disable();
  set_bed_leveling_enabled(false);

  bool ok = true;

  // states 10..22: precise scan, heat up
  if (mode == 1 || mode == 2) {
    ok = heat_wait(mode == 1 ? 60 : 100, 100, HBS_WARMUP_S);
  }

  SoftEndstopsOff soft_off;

  if (ok) {
    status(F("Heizbett Scan"));
    home(F("G28"));                                           // state 22
    zmove(mm_steps(HBS_Z_START_MM));
    xy_to(scan_grid_start().x, scan_grid_start().y);          // state 25
    wait_ms(HBS_DELAY_MS);                                    // state 30
    int16_t p;
    ok = read_idle(p);
    if (ok) limits(p, HBS_CONTACT_DELTA, HBS_RETRY_DELTA);
  }

  // states 35..55: serpentine, X outer, Y inner
  if (ok) {
    bedlevel.reset();
    bedlevel.set_grid(scan_grid_spacing(), scan_grid_start());
    for (uint8_t ix = 0; ok && ix < GRID_MAX_POINTS_X; ++ix) {
      const bool fwd = !(ix & 1);
      for (uint8_t n = 0; ok && n < GRID_MAX_POINTS_Y; ++n) {
        const uint8_t iy = fwd ? n : GRID_MAX_POINTS_Y - 1 - n;
        const float x = bedlevel.get_mesh_x(ix), y = bedlevel.get_mesh_y(iy);
        xy_to(x, y);
        char msg[24];
        sprintf_P(msg, PSTR("HBS %u/%u"), unsigned(ix * GRID_MAX_POINTS_Y + n + 1), unsigned(GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y));
        ui.set_status(msg);
        ok = scan_point(x, y, HBS_RETRIES);
        if (!ok) break;
        bedlevel.z_values[ix][iy] = zsteps / spm();
        SERIAL_ECHOLNPGM("HBS: X", x, " Y", y, " Z", zsteps, " Schritte = ", bedlevel.z_values[ix][iy], " mm");
        if (!down_fast()) { ok = false; break; }
      }
    }
  }

  if (ok) {
    end_moves();                                              // state 60
    if (!mode) heaters_off();
    bedlevel.refresh_bed_level();
  }

  // states 130..148: precise scan, correction with the hotend at print temperature
  if (ok && (mode == 1 || mode == 2)) {
    xy_to(HBS_CAL_X_MM, HBS_CAL_Y_MM - RF_Y_HOME);
    zmove(mm_steps(10));
    ok = heat_wait(0, mode == 1 ? 210 : 240, HBS_CALIBRATION_S);
    if (ok) {
      zmove_to(mm_steps(HBS_Z_START_MM));
      has_last = false;
      wait_ms(HBS_DELAY_MS);
      int16_t p;
      ok = read_idle(p);
      if (ok) {
        limits(p, HBS_CONTACT_DELTA, HBS_RETRY_DELTA);
        ok = scan_point(current_position.x, current_position.y, HBS_RETRIES);
      }
    }
    if (ok) {
      // adjustCompensationMatrix
      const float delta = (zsteps - matrix_steps_here()) / spm();
      for (uint8_t x = 0; x < GRID_MAX_POINTS_X; ++x)
        for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; ++y) bedlevel.z_values[x][y] += delta;
      bedlevel.refresh_bed_level();
      SERIAL_ECHOLNPGM("HBS: Matrix fuer die Drucktemperatur um ", int16_t(LROUND(delta * 1000)), " um angepasst");
      end_moves();                                            // state 149
    }
  }

  if (!ok) {
    // abort: like RF Repetier, reload the stored matrix
    if (!reason) reason = F("Fehler");
    SERIAL_ECHOPGM("HBS: Abbruch - ");
    SERIAL_ECHOLNF(reason);
    zmove(mm_steps(5));
    home(F("G28"));
    stepper.disable_all_steppers();
    heaters_off();
    bedlevel.reset();
    rf_zmatrix.load(rf_zmatrix.active, true);
    ui.set_status(F("Heizbett Scan Abbruch"));
    BUZZ(500, 1000);
    return;
  }

  // states 150..160: motors and heaters off, save to the active matrix
  stepper.disable_all_steppers();
  heaters_off();
  rf_zmatrix.save_active();
  set_bed_leveling_enabled(true);
  rf_zmatrix.report(false);
  SERIAL_ECHOLNPGM("HBS: fertig in ", (millis() - t0) / 60000UL, " min");
  ui.set_status(F("Heizbett Scan fertig"));
  BUZZ(100, 2000);
}

//
// M3900 - Z offset scan (RF Repetier searchZOScan, without auto matrix leveling):
// measure the middle point of the matrix and move the whole matrix to it.
//
void Renkforce::z_offset_scan() {
  if (!bedlevel.has_mesh()) { SERIAL_ECHOLNPGM("M3900: keine Z-Matrix, zuerst Heizbett-Scan"); return; }
  if (!rf_loadcell.ok) { SERIAL_ERROR_MSG("M3900: DMS (0x49) antwortet nicht"); return; }
  KEEPALIVE_STATE(IN_PROCESS);
  aborted = false; reason = nullptr; has_last = false;
  const bool was_on = planner.leveling_active;
  set_bed_leveling_enabled(false);
  SoftEndstopsOff soft_off;
  status(F("Z-Offset-Scan"));

  home(F("G28"));
  zmove(mm_steps(HBS_Z_START_MM));
  const xy_pos_t pt = {
    bedlevel.get_mesh_x((GRID_MAX_POINTS_X - 1) / 2) + random(-(ZOS_RAND_MM), ZOS_RAND_MM),
    bedlevel.get_mesh_y((GRID_MAX_POINTS_Y - 1) / 2) + random(-(ZOS_RAND_MM), ZOS_RAND_MM)
  };
  xy_to(pt.x, pt.y);

  bool ok = false;
  int32_t z_min = mm_steps(HBS_Z_START_MM);
  retries = ZOS_RETRIES;
  for (;;) {
    int16_t p;
    if (!read_idle(p)) { if (aborted || !retries--) break; wait_ms(HBS_DELAY_MS); zmove(mm_steps(HBS_DOWN_FAST_MM)); continue; }
    limits(p, ZOS_CONTACT_DELTA, ZOS_RETRY_DELTA);
    // state 10: fast up, back up to 2 fast steps, the pressure must change
    const int32_t old_z = zsteps;
    Result r = up_fast();
    int16_t p_contact = 0, p_back = 0;
    if (r == R_OK) {
      wait_ms(HBS_SLOW_STEP_DELAY_MS);
      if (!read_avg(p_contact)) r = R_RETRY;
    }
    if (r == R_OK) {
      const int32_t did = ABS(zsteps - old_z), fast = ABS(mm_steps(HBS_UP_FAST_MM));
      zmove(did > 2 * fast ? 2 * fast : did);
      wait_ms(HBS_SLOW_STEP_DELAY_MS);
      if (!read_avg(p_back) || ABS(p_back - p_contact) < ZOS_CONTACT_DELTA) r = R_RETRY;
    }
    // state 20: slow up (acuteness 2), slow down (acuteness 8)
    if (r == R_OK) {
      zmove(ABS(mm_steps(HBS_UP_SLOW_MM)));
      has_last = false;
      if ((r = up_slow(2)) == R_OK) r = down_slow(8);
    }
    if (r == R_OK) { NOMORE(z_min, zsteps); ok = true; break; }
    if (aborted || r == R_ABORT || !retries) break;
    --retries;
    wait_ms(HBS_DELAY_MS);   // state 7
    down_fast();
  }

  float delta = 0;
  if (ok) {
    delta = (z_min - matrix_steps_here()) / spm();
    for (uint8_t x = 0; x < GRID_MAX_POINTS_X; ++x)
      for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; ++y)
        if (!isnan(bedlevel.z_values[x][y])) bedlevel.z_values[x][y] += delta;
    bedlevel.refresh_bed_level();
  }

  // state 99: lift, home Y, back to Z = 0
  zmove(mm_steps(1));
  home(F("G28 Y"));
  zmove_to(0);
  set_bed_leveling_enabled(was_on);

  if (!ok) {
    SERIAL_ECHOPGM("M3900: Abbruch - ");
    SERIAL_ECHOLNF(reason ? reason : F("Fehler"));
    ui.set_status(F("Z-Offset-Scan Abbruch"));
    BUZZ(500, 1000);
    return;
  }
  SERIAL_ECHOLNPGM("M3900: Matrix um ", int16_t(LROUND(delta * 1000)), " um verschoben");
  rf_zmatrix.save_active();
  ui.set_status(F("Z-Offset-Scan fertig"));
  BUZZ(100, 2000);
}

#endif // RENKFORCE_RF && AUTO_BED_LEVELING_BILINEAR
