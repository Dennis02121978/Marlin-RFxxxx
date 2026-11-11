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

#include "renkforce.h"
#include "../../MarlinCore.h"
#include "../../module/planner.h"
#include "../../module/stepper.h"
#include "../../module/motion.h"
#include "../../module/temperature.h"
#include "../../gcode/gcode.h"
#include "../../gcode/queue.h"
#include "../../lcd/marlinui.h"
#include "../babystep.h"
#include "../../module/endstops.h"
#include "../../module/settings.h"
#include "../../libs/buzzer.h"
#if ENABLED(CASE_LIGHT_ENABLE)
  #include "../caselight.h"
#endif
#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "../pause.h"
#endif
#if HAS_MEDIA
  #include "../../sd/cardreader.h"
#endif
#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../host_actions.h"
#endif

Renkforce rf;

rf_settings_t Renkforce::settings;
bool Renkforce::ready; // = false
bool Renkforce::sense_enabled;
float Renkforce::sense_offset;
float Renkforce::dyn_flow = 1.0f, Renkforce::dyn_feed = 1.0f;

namespace {
  // ---- state that does not need to be visible outside -------------------------------------
  bool was_printing;

  // keys
  enum : uint8_t { K_Z_UP, K_Z_DOWN, K_E_REV, K_E_FWD, K_PAUSE, K_CONT, K_COUNT };
  uint8_t key_state, key_raw_last;            // debounced state / last raw read
  millis_t key_next_ms, key_down_ms[K_COUNT];
  bool key_holding[K_COUNT];
  bool jogging;                               // our own moves are in the planner
  bool z_driving, z_endstops;                 // Z key held: bed moves until the key is released
  bool zo_dirty;                              // Z offset changed by the keys, save after the print

  // sense offset
  int32_t sense_steps;                        // babysteps applied so far
  float sense_sum, sense_n, sense_last, sense_reported;
  uint16_t sense_samples, sense_over;
  int16_t sense_force_max;
  bool sense_done, sense_max_reported;

  // emergency pause
  int16_t ep_buf[RF_EMERGENCY_PAUSE_SAMPLES];
  uint8_t ep_count, ep_index;
  bool ep_pending;
  millis_t ep_pending_ms;

  // flow compensation
  millis_t flow_next_ms;
  int16_t flow_buf[4];
  uint8_t flow_count, flow_index;
  int16_t flow_set_pct = 100, feed_set_pct = 100;   // what we wrote last
  int16_t user_flow_pct = 100, user_feed_pct = 100;

  // case fan
  millis_t fan_off_ms;
  bool fan_on;

  bool printing_now() {
    return printingIsActive() && !printingIsPaused() && !TERN0(ADVANCED_PAUSE_FEATURE, did_pause_print);
  }

  bool all_homed_xyz() { return !axes_should_home(); }

  uint8_t read_keys() {
    uint8_t k = 0;
    if (!READ(RF_KEY_Z_UP_PIN))     SBI(k, K_Z_UP);
    if (!READ(RF_KEY_Z_DOWN_PIN))   SBI(k, K_Z_DOWN);
    if (!READ(RF_KEY_E_REV_PIN))    SBI(k, K_E_REV);
    if (!READ(RF_KEY_E_FWD_PIN))    SBI(k, K_E_FWD);
    if (!READ(RF_KEY_PAUSE_PIN))    SBI(k, K_PAUSE);
    if (!READ(RF_KEY_CONTINUE_PIN)) SBI(k, K_CONT);
    return k;
  }
}

//
// Settings (EEPROM)
//
void Renkforce::reset_settings() {
  constexpr uint16_t cur[] = RF_DRV8711_CURRENT, mst[] = RF_DRV8711_MICROSTEPS;
  static_assert(COUNT(cur) == RF_DRV_COUNT && COUNT(mst) == RF_DRV_COUNT, "RF_DRV8711_CURRENT / _MICROSTEPS need 5 values.");
  for (uint8_t d = 0; d < RF_DRV_COUNT; ++d) {
    settings.drv_current[d] = cur[d];
    settings.drv_microsteps[d] = mst[d];
  }
  settings.probe_trigger   = RF_LOADCELL_TRIGGER;
  settings.pause_min       = RF_EMERGENCY_PAUSE_MIN;
  settings.pause_max       = RF_EMERGENCY_PAUSE_MAX;
  settings.pause_pos[0]    = RF_PAUSE_POS_X;
  settings.pause_pos[1]    = RF_PAUSE_POS_Y;
  settings.pause_pos[2]    = RF_PAUSE_POS_Z;
  settings.sense_auto      = false;
  settings.sense_threshold = RF_SENSE_OFFSET_DIGITS;
  settings.sense_max_um    = RF_SENSE_OFFSET_MAX_UM;
  settings.flow_min        = RF_FLOW_COMP_MIN;
  settings.flow_max        = RF_FLOW_COMP_MAX;
  settings.flow_percent    = RF_FLOW_COMP_FLOW_PERCENT;
  settings.feed_percent    = RF_FLOW_COMP_FEED_PERCENT;
  settings.case_fan_mode   = 0;
  // RGB light like RF Repetier (RF2000.h RGB_*, Eeprom.cpp default mode automatic)
  constexpr uint8_t rgb_def[RF_RGB_COLORS][3] = {
    { 255,   0,   0 },   // heating
    { 255, 255, 255 },   // printing
    {   0,   0, 255 },   // cooling
    { 255, 255, 255 },   // idle
    { 128, 128, 255 }    // manual
  };
  settings.rgb_mode = RF_RGB_AUTO;
  for (uint8_t c = 0; c < RF_RGB_COLORS; ++c) for (uint8_t i = 0; i < 3; ++i) settings.rgb[c][i] = rgb_def[c][i];
  settings.case_light = ENABLED(CASE_LIGHT_DEFAULT_ON);
  apply_settings();
}

void Renkforce::apply_settings() {
  for (uint8_t d = 0; d < RF_DRV_COUNT; ++d) {
    if (!RF_DRV8711::valid_microsteps(settings.drv_microsteps[d])) settings.drv_microsteps[d] = 32;
    NOMORE(settings.drv_current[d], uint16_t(RF_DRV8711_CURRENT_LIMIT));
  }
  if (settings.probe_trigger < 5) settings.probe_trigger = RF_LOADCELL_TRIGGER;
  if (settings.rgb_mode > RF_RGB_MANUAL) settings.rgb_mode = RF_RGB_AUTO;
  TERN_(CASE_LIGHT_ENABLE, caselight.on = settings.case_light);   // caselight.init() follows in setup
  #if PIN_EXISTS(RF_RGB_R)
    if (ready) rf_rgb.set_mode(settings.rgb_mode);
  #endif
  rf_drv.refresh();   // no reset at runtime, the drivers would lose their microstep position
}

void Renkforce::before_save() {
  TERN_(CASE_LIGHT_ENABLE, settings.case_light = caselight.on);
}

void Renkforce::report_settings(const bool forReplay) {
  #define _RF_ECHO_CMD(S) do{ if (!forReplay) SERIAL_ECHO_START(); SERIAL_ECHOPGM(S); }while(0)
  if (!forReplay) { SERIAL_ECHO_MSG("; Renkforce DRV8711 current (mA), microsteps:"); }
  _RF_ECHO_CMD("  M906");
  SERIAL_ECHOLNPGM(" X", settings.drv_current[RF_DRV_X], " Y", settings.drv_current[RF_DRV_Y],
                   " Z", settings.drv_current[RF_DRV_Z], " T0 E", settings.drv_current[RF_DRV_E0]);
  #if HAS_MULTI_EXTRUDER
    _RF_ECHO_CMD("  M906");
    SERIAL_ECHOLNPGM(" T1 E", settings.drv_current[RF_DRV_E1]);
  #endif
  _RF_ECHO_CMD("  M350");
  SERIAL_ECHOLNPGM(" X", settings.drv_microsteps[RF_DRV_X], " Y", settings.drv_microsteps[RF_DRV_Y],
                   " Z", settings.drv_microsteps[RF_DRV_Z], " E", settings.drv_microsteps[RF_DRV_E0]);
  if (!forReplay) { SERIAL_ECHO_MSG("; Renkforce load cell: probe delta, emergency pause, sense offset, flow compensation:"); }
  _RF_ECHO_CMD("  M3050");
  SERIAL_ECHOLNPGM(" S", settings.probe_trigger);
  _RF_ECHO_CMD("  M3075");
  SERIAL_ECHOLNPGM(" L", settings.pause_min, " H", settings.pause_max);
  _RF_ECHO_CMD("  M3105");
  SERIAL_ECHOLNPGM(" X", settings.pause_pos[0], " Y", settings.pause_pos[1], " Z", settings.pause_pos[2]);
  _RF_ECHO_CMD("  M3909");
  SERIAL_ECHOLNPGM(" P", settings.sense_threshold, " S", settings.sense_max_um, " A", settings.sense_auto, " D1");
  _RF_ECHO_CMD("  M3911");
  SERIAL_ECHOLNPGM(" S", settings.flow_percent, " F", settings.feed_percent, " L", settings.flow_min, " H", settings.flow_max);
  #if PIN_EXISTS(RF_RGB_R)
    if (!forReplay) { SERIAL_ECHO_MSG("; Renkforce RGB light: mode, colors heating/printing/cooling/idle/manual:"); }
    _RF_ECHO_CMD("  M3308");
    SERIAL_ECHOLNPGM(" P", settings.rgb_mode);
    for (uint8_t c = 0; c < RF_RGB_COLORS; ++c) {
      _RF_ECHO_CMD("  M");
      SERIAL_ECHO(int(3303 + c));
      SERIAL_ECHOLNPGM(" R", settings.rgb[c][0], " U", settings.rgb[c][1], " B", settings.rgb[c][2]);
    }
  #endif
}

//
// Setup
//
void Renkforce::init() {
  rf_drv.init();
  rf_loadcell.init();
  TERN_(AUTO_BED_LEVELING_BILINEAR, rf_zmatrix.init());   // active heat bed matrix from the 24C256

  SET_INPUT_PULLUP(RF_KEY_Z_UP_PIN);
  SET_INPUT_PULLUP(RF_KEY_Z_DOWN_PIN);
  SET_INPUT_PULLUP(RF_KEY_E_REV_PIN);
  SET_INPUT_PULLUP(RF_KEY_E_FWD_PIN);
  SET_INPUT_PULLUP(RF_KEY_PAUSE_PIN);
  SET_INPUT_PULLUP(RF_KEY_CONTINUE_PIN);

  #if PIN_EXISTS(RF_CASE_FAN)
    OUT_WRITE(RF_CASE_FAN_PIN, LOW);
  #endif
  #if PIN_EXISTS(RF_OUTPUT_230V)
    OUT_WRITE(RF_OUTPUT_230V_PIN, LOW);
  #endif
  #if PIN_EXISTS(RF_FET1)
    OUT_WRITE(RF_FET1_PIN, LOW);
    OUT_WRITE(RF_FET2_PIN, LOW);
  #endif

  ready = true;
  #if PIN_EXISTS(RF_RGB_R)
    rf_rgb.init();   // starts dark and fades into the color of the stored mode
  #endif
}

void Renkforce::on_homing() {
  // Homing ends the sense offset of the last print (as in Klipper z_sense_offset)
  sense_enabled = false;
  sense_offset = 0;
  sense_steps = 0;
}

//
// Called from idle()
//
void Renkforce::idle() {
  if (!ready) return;   // idle() also runs during setup, before Wire and the pins are set up
  const millis_t ms = millis();

  if (rf_loadcell.update()) handle_sample();

  // print start / end
  const bool printing = printingIsActive();
  if (printing != was_printing) {
    was_printing = printing;
    if (printing) {
      // RF Repetier FEATURE_ZERO_DIGITS: zero the load cell at the start of a print
      if (rf_loadcell.ok && ABS(rf_loadcell.raw) < RF_LOADCELL_IDLE_MAX) rf_loadcell.tare_offset = rf_loadcell.raw;
      ep_count = ep_index = 0;
      flow_count = flow_index = 0;
      // RF Repetier: sense offset "AutoStart"
      if (settings.sense_auto && settings.sense_threshold > 0) sense_enable(settings.sense_threshold, settings.sense_max_um);
    }
    else {
      // print finished or aborted: switch the compensations off
      sense_enabled = false;
      dyn_flow = dyn_feed = 1.0f;
      if (planner.flow_percentage[active_extruder] == flow_set_pct) {
        planner.set_flow(active_extruder, user_flow_pct);
      }
      if (feedrate_percentage == feed_set_pct) feedrate_percentage = user_feed_pct;
      flow_set_pct = user_flow_pct = planner.flow_percentage[active_extruder];
      feed_set_pct = user_feed_pct = feedrate_percentage;
    }
  }

  if (ELAPSED(ms, key_next_ms)) {
    key_next_ms = ms + 10;
    handle_keys(ms);
  }

  handle_flow(ms);
  handle_case_fan(ms);
  #if PIN_EXISTS(RF_RGB_R)
    #if ALL(HAS_MARLINUI_MENU, HAS_MARLINUI_HD44780) && PIN_EXISTS(BTN_BACK)
      // RF Repetier backAction: the back key on the status screen toggles the white light.
      // Only when the status screen was already shown before the key went down (not the key that left a menu).
      static millis_t status_since_ms;
      static bool back_last;
      if (!ui.on_status_screen()) status_since_ms = 0;
      else if (!status_since_ms) status_since_ms = ms ? ms : 1;
      const bool back = !READ(BTN_BACK);
      if (back != back_last) {
        back_last = back;
        if (back && status_since_ms && ms - status_since_ms > 300) rf_rgb.toggle_white();
        status_since_ms = status_since_ms ? ms : 0;   // debounce: next toggle earliest 300 ms later
      }
    #endif
    rf_rgb.idle(ms);
  #endif
}

void Renkforce::handle_sample() {
  const int16_t v = rf_loadcell.value();
  emergency_sample(v);
  if (sense_enabled) sense_sample(v);
  flow_buf[flow_index] = v;
  flow_index = (flow_index + 1) % COUNT(flow_buf);
  if (flow_count < COUNT(flow_buf)) ++flow_count;
}

//
// Emergency pause (RF Repetier FEATURE_EMERGENCY_PAUSE, M3075)
//
void Renkforce::emergency_sample(const int16_t v) {
  // a requested pause is done once the print runs again (resumed by key, display or host)
  if (ep_pending && printing_now() && ELAPSED(millis(), ep_pending_ms + 10000UL)) ep_pending = false;
  if ((!settings.pause_min && !settings.pause_max) || ep_pending || !printing_now() || !all_homed_xyz()) { ep_count = 0; return; }
  ep_buf[ep_index] = v;
  ep_index = (ep_index + 1) % (RF_EMERGENCY_PAUSE_SAMPLES);
  if (ep_count < RF_EMERGENCY_PAUSE_SAMPLES) { ++ep_count; return; }
  int32_t sum = 0;
  for (uint8_t i = 0; i < RF_EMERGENCY_PAUSE_SAMPLES; ++i) sum += ep_buf[i];
  const int16_t avg = int16_t(sum / (RF_EMERGENCY_PAUSE_SAMPLES));
  if ((settings.pause_min && avg < settings.pause_min) || (settings.pause_max && avg > settings.pause_max)) {
    ep_count = 0;
    SERIAL_ECHOLNPGM("RF: Notfall-Pause, DMS ", avg, " ausserhalb ", settings.pause_min, "..", settings.pause_max);
    pause_print(true);
  }
}

void Renkforce::pause_print(const bool emergency) {
  if (!printingIsActive() || printingIsPaused() || TERN0(ADVANCED_PAUSE_FEATURE, did_pause_print)) return;
  if (emergency) LCD_ALERTMESSAGE_F("Notfall-Pause (DMS)");
  ep_pending = true;
  ep_pending_ms = millis();
  #if ENABLED(PARK_HEAD_ON_PAUSE)
    // park at the RF pause position (RF Repetier M3105), works for SD and host prints
    static char cmd[40];
    sprintf_P(cmd, PSTR("M125 X%i Y%i Z%i"), settings.pause_pos[0], settings.pause_pos[1], settings.pause_pos[2]);
    queue.inject(cmd);
  #elif HAS_MEDIA
    queue.inject(F("M25"));
  #elif ENABLED(HOST_ACTION_COMMANDS)
    hostui.pause();
  #endif
}

void Renkforce::continue_print() {
  ep_pending = false;
  #if HAS_RESUME_CONTINUE
    if (wait_for_user) { wait_for_user = false; return; }   // same as a click on the display
  #endif
  #if HAS_MEDIA
    if (card.isPaused()) { queue.inject(F("M24")); return; }
  #endif
  #if ENABLED(HOST_ACTION_COMMANDS)
    if (printingIsPaused()) hostui.resume();
  #endif
}

//
// Sense offset (RF Repetier FEATURE_SENSIBLE_PRESSURE, Klipper z_sense_offset)
// While Z <= RF_SENSE_OFFSET_MAX_Z: when the averaged force is over the threshold the
// nozzle is lifted in small steps, at most sense_max_um. The offset stays for the print.
//
void Renkforce::sense_enable(const int16_t threshold, const uint16_t max_um) {
  settings.sense_threshold = threshold;
  settings.sense_max_um = max_um;
  sense_enabled = threshold > 0 && max_um > 0;
  sense_sum = sense_n = sense_last = sense_reported = 0;
  sense_samples = sense_over = 0;
  sense_force_max = 0;
  sense_done = sense_max_reported = false;
}

void Renkforce::sense_disable() { sense_enabled = false; }

void Renkforce::sense_sample(const int16_t v) {
  if (!printing_now()) return;
  const float max_off = settings.sense_max_um * 0.001f;
  if (current_position.z > RF_SENSE_OFFSET_MAX_Z) {
    if (sense_samples && !sense_done) {
      sense_done = true;
      SERIAL_ECHOLNPGM("RF Sense Offset erster Layer: ", int16_t(sense_offset * 1000), " um, Kraft max ", sense_force_max,
                       ", ueber Schwelle ", sense_over, " von ", sense_samples);
    }
    return;
  }

  constexpr uint8_t n_avg = 10;          // Klipper n_average_force
  constexpr float smoothing = 0.5f, rel_tol = 0.1f, acuteness = 24.0f, max_steps = 48.0f, step_mm = 0.001f;
  sense_sum += v;
  sense_n += 1;
  if (sense_n < n_avg) return;
  const float f = ABS(sense_sum) / n_avg;
  sense_sum *= smoothing;
  sense_n *= smoothing;

  ++sense_samples;
  NOLESS(sense_force_max, int16_t(f));
  const float thr = settings.sense_threshold;
  if (f < thr) { sense_last = f; return; }
  ++sense_over;

  const bool exceed = f > thr * (1 + rel_tol);
  float n_steps = 1;
  if (exceed && f >= sense_last) {
    n_steps += _MIN((f / thr - 1) * acuteness, max_steps);
    n_steps -= (n_steps - 1) * (sense_offset / max_off);
  }
  if (!exceed && f < sense_last) n_steps = 0;
  sense_last = f;

  sense_offset = _MIN(sense_offset + n_steps * step_mm, max_off);

  #if ENABLED(BABYSTEPPING)
    const int32_t target = LROUND(sense_offset * planner.settings.axis_steps_per_mm[Z_AXIS]);
    if (target != sense_steps) {
      babystep.add_steps(Z_AXIS, int16_t(target - sense_steps));   // + = nozzle away from the bed
      sense_steps = target;
    }
  #endif

  if (sense_offset >= max_off && !sense_max_reported) {
    sense_max_reported = true;
    SERIAL_ECHOLNPGM("RF Sense Offset: Obergrenze ", int16_t(sense_offset * 1000), " um erreicht");
  }
  else if (sense_offset >= sense_reported + 0.02f) {
    sense_reported = sense_offset;
    SERIAL_ECHOLNPGM("RF Sense Offset: ", int16_t(sense_offset * 1000), " um (Kraft ", int16_t(f), ")");
  }
}

//
// Digit flow compensation (RF Repetier FEATURE_DIGIT_FLOW_COMPENSATION, Klipper rf1000_pressure_guard)
// Between flow_min and flow_max digits the flow / speed change linearly up to flow_percent / feed_percent.
//
void Renkforce::handle_flow(const millis_t ms) {
  if (PENDING(ms, flow_next_ms)) return;
  flow_next_ms = ms + 250;

  const uint8_t e = active_extruder;
  // keep changes made by the user (M220 / M221 / display)
  if (planner.flow_percentage[e] != flow_set_pct)
    user_flow_pct = dyn_flow > 0.01f ? int16_t(planner.flow_percentage[e] / dyn_flow + 0.5f) : planner.flow_percentage[e];
  if (feedrate_percentage != feed_set_pct)
    user_feed_pct = dyn_feed > 0.01f ? int16_t(feedrate_percentage / dyn_feed + 0.5f) : feedrate_percentage;

  float goal_flow = 1.0f, goal_feed = 1.0f;
  const bool on = (settings.flow_percent || settings.feed_percent) && settings.flow_max > settings.flow_min;
  if (on && flow_count && printing_now() && current_position.z > RF_FLOW_COMP_MIN_Z) {
    int32_t sum = 0;
    for (uint8_t i = 0; i < flow_count; ++i) sum += flow_buf[i];
    const float d = ABS(float(sum) / flow_count);
    const float share = d <= settings.flow_min ? 0.0f : d >= settings.flow_max ? 1.0f
                      : (d - settings.flow_min) / float(settings.flow_max - settings.flow_min);
    goal_flow = 1.0f + 0.01f * settings.flow_percent * share;
    goal_feed = 1.0f + 0.01f * settings.feed_percent * share;
  }

  // flow follows at once, the speed is ramped by 1 % per update (as in the firmware)
  dyn_flow = goal_flow;
  if (ABS(goal_feed - dyn_feed) <= 0.01f) dyn_feed = goal_feed;
  else dyn_feed += goal_feed > dyn_feed ? 0.01f : -0.01f;

  const int16_t new_flow = int16_t(user_flow_pct * dyn_flow + 0.5f),
                new_feed = int16_t(user_feed_pct * dyn_feed + 0.5f);
  if (new_flow != planner.flow_percentage[e]) planner.set_flow(e, new_flow);
  if (new_feed != feedrate_percentage) feedrate_percentage = new_feed;
  flow_set_pct = new_flow;
  feed_set_pct = new_feed;
}

//
// Case fan (RF Repetier FEATURE_CASE_FAN): on while motors or heaters are on, off 60 s later
//
void Renkforce::set_case_fan_mode(const uint8_t m) { settings.case_fan_mode = _MIN(m, 2); }

void Renkforce::handle_case_fan(const millis_t ms) {
  #if PIN_EXISTS(RF_CASE_FAN)
    bool want;
    switch (settings.case_fan_mode) {
      case 1: want = true; break;
      case 2: want = false; break;
      default: {
        bool active = stepper.axis_enabled.bits || printingIsActive() || thermalManager.degTargetBed() > 0;
        HOTEND_LOOP() if (thermalManager.degTargetHotend(e) > 0) active = true;
        if (active) fan_off_ms = ms + SEC_TO_MS(RF_CASE_FAN_OFF_DELAY);
        want = active || PENDING(ms, fan_off_ms);
      }
    }
    if (want != fan_on) { fan_on = want; WRITE(RF_CASE_FAN_PIN, want); }
  #else
    UNUSED(ms);
  #endif
}

void Renkforce::set_output(const uint8_t which, const bool on) {
  switch (which) {
    #if PIN_EXISTS(RF_OUTPUT_230V)
      case 0: WRITE(RF_OUTPUT_230V_PIN, on); break;
    #endif
    #if PIN_EXISTS(RF_FET1)
      case 1: WRITE(RF_FET1_PIN, on); break;
      case 2: WRITE(RF_FET2_PIN, on); break;
      case 3: set_case_fan_mode(on ? 1 : 2); break;   // FET3 = case fan
    #endif
    default: UNUSED(on); break;
  }
}

//
// Microsteps: the axis keeps its position in mm, steps/mm are rescaled
//
void Renkforce::change_microsteps(const uint8_t d, const uint16_t ms) {
  if (!RF_DRV8711::valid_microsteps(ms) || ms == settings.drv_microsteps[d]) return;
  const float f = float(ms) / settings.drv_microsteps[d];
  planner.synchronize();
  if (d <= RF_DRV_Z)
    planner.settings.axis_steps_per_mm[d] *= f;
  else {
    #if ENABLED(DISTINCT_E_FACTORS)
      planner.settings.axis_steps_per_mm[E_AXIS + d - RF_DRV_E0] *= f;
    #else
      // both extruders share one steps/mm value: always change both drivers
      planner.settings.axis_steps_per_mm[E_AXIS] *= f;
      rf_drv.set_microsteps(d == RF_DRV_E0 ? RF_DRV_E1 : RF_DRV_E0, ms);
    #endif
  }
  rf_drv.set_microsteps(d, ms);
  planner.refresh_positioning();
}

//
// Keys next to the display
//
//
// Z key held (RF Repetier MOVE_MODE_SINGLE_MOVE): one long move, stopped when the key is released
//
void Renkforce::z_drive_start(const int8_t dir) {
  #if ENABLED(HOST_KEEPALIVE_FEATURE)
    if (gcode.busy_state != GcodeSuite::NOT_BUSY && gcode.busy_state != GcodeSuite::PAUSED_FOR_USER) return;
  #endif
  if (planner.has_blocks_queued()) return;
  float target;
  if (axis_is_trusted(Z_AXIS)) {
    if (dir < 0) {
      // up to Z 0 (switch); at the switch single steps down to the overtravel limit
      target = current_position.z > 0.0f ? 0.0f : _MAX(current_position.z - (RF_KEY_Z_BABYSTEP), -float(RF_SCAN_Z_DRIVE_OVER));
    }
    else
      target = float(Z_MAX_POS);
    z_endstops = false;
  }
  else {
    // not homed: drive until the key is released; upwards the Z switch stops the bed
    target = current_position.z + dir * float(Z_MAX_POS);
    z_endstops = dir < 0;
    if (z_endstops) endstops.enable(true);
  }
  if (ABS(target - current_position.z) < 0.0005f) return;
  current_position.z = target;
  line_to_current_position(homing_feedrate(Z_AXIS));
  z_driving = jogging = true;
}

void Renkforce::z_drive_stop() {
  planner.quick_stop();
  planner.synchronize();
  set_current_from_steppers_for_axis(ALL_AXES_ENUM);
  sync_plan_position();
  if (z_endstops) { endstops.hit_on_purpose(); endstops.not_homing(); z_endstops = false; }
  z_driving = jogging = false;
}

bool Renkforce::jog(const AxisEnum axis, const float dist, const feedRate_t fr_mm_s) {
  // Same rule as the manual move of the LCD: only between G-codes (or while a pause
  // waits for the user) and only when the planner holds nothing but our own moves.
  #if ENABLED(HOST_KEEPALIVE_FEATURE)
    if (gcode.busy_state != GcodeSuite::NOT_BUSY && gcode.busy_state != GcodeSuite::PAUSED_FOR_USER) return false;
  #endif
  if (planner.has_blocks_queued() && !jogging) return false;

  if (axis == Z_AXIS) {
    if (!axis_is_trusted(Z_AXIS)) { LCD_MESSAGE_F("Zuerst Z referenzieren"); return false; }
    float target = current_position.z + dist;
    LIMIT(target, -float(RF_SCAN_Z_DRIVE_OVER), float(Z_MAX_POS) - 0.5f);   // RF Repetier: keys may go below Z=0
    if (ABS(target - current_position.z) < 0.0005f) return false;
    current_position.z = target;
  }
  else {
    if (thermalManager.tooColdToExtrude(active_extruder)) { LCD_MESSAGE_F("Duese zu kalt"); return false; }
    current_position.e += dist;
  }
  jogging = true;
  line_to_current_position(fr_mm_s);
  return true;
}

void Renkforce::handle_keys(const millis_t ms) {
  // debounce: a change must be seen twice in a row (10 ms apart)
  const uint8_t raw = read_keys();
  const uint8_t stable = (raw == key_raw_last) ? raw : key_state;
  key_raw_last = raw;
  const uint8_t pressed = stable & ~key_state, released = key_state & ~stable;
  key_state = stable;

  if (jogging && !planner.has_blocks_queued()) jogging = false;

  const bool printing = printing_now();

  for (uint8_t k = 0; k < K_COUNT; ++k) {
    if (TEST(pressed, k)) { key_down_ms[k] = ms; key_holding[k] = false; }
  }

  // Z keys like RF Repetier (moveZAction): Z_UP moves the bed up (Z smaller), Z_DOWN moves it down.
  //  - mod page of the status screen: Z offset -/+ 5 um (zOffsetMinusAction / zOffsetPlusAction)
  //  - printing: single steps (babystep 0.01 mm)
  //  - otherwise: the bed moves as long as the key is held (MOVE_MODE_SINGLE_MOVE), also without homing
  for (uint8_t i = 0; i < 2; ++i) {
    const uint8_t k = i ? K_Z_DOWN : K_Z_UP;
    const int8_t dir = i ? 1 : -1;
    if (TEST(pressed, k)) {
      #if HAS_MARLINUI_HD44780
        const bool mod_page = ui.on_status_screen() && status_page == 1;
      #else
        constexpr bool mod_page = false;
      #endif
      if (mod_page) {
        #if HAS_M206_COMMAND
          // RF Repetier Z offset: Z_DOWN = bed lower = zO bigger = home offset smaller
          float zo = -home_offset.z * 1000.0f + dir * (RF_KEY_Z_OFFSET_UM);
          if (ABS(zo) < (RF_KEY_Z_OFFSET_UM)) zo = 0;   // pass through 0 exactly
          LIMIT(zo, -10000.0f, 10000.0f);
          set_home_offset(Z_AXIS, -zo / 1000.0f);
          zo_dirty = true;
          BUZZ(10, 4000);
        #endif
      }
      else if (printing) {
        #if ENABLED(BABYSTEPPING)
          babystep.add_mm(Z_AXIS, dir * (RF_KEY_Z_BABYSTEP));
        #endif
      }
      else
        z_drive_start(dir);
    }
    if (TEST(released, k) && z_driving) z_drive_stop();
  }
  if (zo_dirty && !printingIsActive()) { zo_dirty = false; ::settings.save(); }

  // E keys: short press 2 mm, held: extrude until released (needs a hot nozzle)
  for (uint8_t i = 0; i < 2; ++i) {
    const uint8_t k = i ? K_E_FWD : K_E_REV;
    const int8_t dir = i ? 1 : -1;
    if (printing) continue;
    if (TEST(pressed, k))
      jog(E_AXIS, dir * (RF_KEY_E_STEP), RF_KEY_E_SPEED);
    else if (TEST(stable, k) && ELAPSED(ms, key_down_ms[k] + RF_KEY_E_HOLD_MS)) {
      key_holding[k] = true;
      if (planner.movesplanned() < 3) jog(E_AXIS, dir * RF_KEY_E_SPEED * 0.25f, RF_KEY_E_SPEED);
    }
    if (TEST(released, k) && key_holding[k] && jogging) {
      planner.quick_stop();
      set_current_from_steppers_for_axis(ALL_AXES_ENUM);
      sync_plan_position();
      jogging = false;
    }
  }

  if (TEST(pressed, K_PAUSE)) pause_print(false);
  if (TEST(pressed, K_CONT)) continue_print();
}

#endif // RENKFORCE_RF
