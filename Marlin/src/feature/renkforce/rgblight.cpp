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
 * RGB light like the RF Repetier firmware, see rgblight.h
 * (RF.cpp setRGBLEDs / updateRGBLightStatus, HAL.cpp Timer 4 setup and 10 ms fading)
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(RENKFORCE_RF) && PIN_EXISTS(RF_RGB_R)

#include "rgblight.h"
#include "renkforce.h"
#include "../../MarlinCore.h"
#include "../../module/temperature.h"

RFRgbLight rf_rgb;

bool     RFRgbLight::force_white;
uint8_t  RFRgbLight::status;
uint8_t  RFRgbLight::cur[3], RFRgbLight::tgt[3];
millis_t RFRgbLight::fade_ms, RFRgbLight::status_ms, RFRgbLight::idle_start_ms;

// Repetier RGB_STATUS_*
enum : uint8_t { ST_NOT_AUTO = 0, ST_AUTO = 1, ST_PRINTING = 11, ST_HEATING = 12, ST_COOLING = 13, ST_IDLE = 14, ST_COLOR_CHANGE = 15 };

#define RF_RGB_TEMP_TOLERANCE   5.0f   // [°C] RGB_LIGHT_TEMP_TOLERANCE
#define RF_RGB_ROOM_TEMP        40     // [°C] MAX_ROOM_TEMPERATURE
#define RF_RGB_COOLDOWN         40     // [°C] COOLDOWN_THRESHOLD
#define RF_RGB_CHANGE_DELAY_S   30     // [s]  RGB_LIGHT_COLOR_CHANGE_DELAY

void RFRgbLight::init() {
  // Timer 4: fast PWM, TOP = ICR4 = 20480 -> 100 Hz, OC4A/B/C = PH3/PH4/PH5 (Repetier HAL.cpp)
  TCCR4A = 0; TCCR4B = 0;
  ICR4 = 20480;
  TCCR4A |= _BV(WGM41);
  TCCR4B |= _BV(WGM42) | _BV(WGM43) | _BV(CS41);
  OUT_WRITE(RF_RGB_R_PIN, LOW);
  OUT_WRITE(RF_RGB_G_PIN, LOW);
  OUT_WRITE(RF_RGB_B_PIN, LOW);
  OCR4A = OCR4B = OCR4C = 0;

  // power on: dark, then fade into the color of the stored mode (Repetier Printer::setup)
  cur[0] = cur[1] = cur[2] = 0;
  output();
  force_white = false;
  set_mode(rf.settings.rgb_mode);
  fade_ms = millis() + 10;
}

void RFRgbLight::output() {
  #define _RGB_OUT(I, OCR, COM) do{ if (cur[I]) { TCCR4A |= _BV(COM); OCR = uint16_t(cur[I]) * 80; } else TCCR4A &= ~_BV(COM); }while(0)
  _RGB_OUT(0, OCR4A, COM4A1);
  _RGB_OUT(1, OCR4B, COM4B1);
  _RGB_OUT(2, OCR4C, COM4C1);
}

void RFRgbLight::target(const uint8_t c) {
  const uint8_t * const v = rf.settings.rgb[c];
  target(v[0], v[1], v[2]);
}

void RFRgbLight::set_mode(const uint8_t m) {
  rf.settings.rgb_mode = m > RF_RGB_MANUAL ? RF_RGB_AUTO : m;
  force_white = false;
  switch (rf.settings.rgb_mode) {
    case RF_RGB_OFF:    status = ST_NOT_AUTO; target(0, 0, 0); break;
    case RF_RGB_WHITE:  status = ST_NOT_AUTO; target(255, 255, 255); break;
    case RF_RGB_MANUAL: status = ST_NOT_AUTO; target(RF_RGB_C_MANUAL); break;
    default:            status = ST_AUTO; idle_start_ms = 0; target(RF_RGB_C_IDLE); break;
  }
}

FSTR_P RFRgbLight::mode_name() {
  if (force_white) return F("Weiss");
  switch (rf.settings.rgb_mode) {
    case RF_RGB_OFF:    return F("Aus");
    case RF_RGB_WHITE:  return F("Weiss");
    case RF_RGB_MANUAL: return F("Manuell");
    default:            return F("Auto");
  }
}

void RFRgbLight::colors_changed() {
  if (force_white) return;
  if (rf.settings.rgb_mode == RF_RGB_MANUAL) target(RF_RGB_C_MANUAL);
  else if (rf.settings.rgb_mode == RF_RGB_AUTO) status = ST_AUTO;   // the next update sets the color of the status
}

// Repetier backAction on the status screen: toggle the white light
void RFRgbLight::toggle_white() {
  force_white = !force_white;
  if (force_white) target(255, 255, 255);
  else switch (rf.settings.rgb_mode) {
    case RF_RGB_OFF:    target(0, 0, 0); break;
    case RF_RGB_WHITE:  target(255, 255, 255); break;
    case RF_RGB_MANUAL: target(RF_RGB_C_MANUAL); break;
    default:            status = ST_AUTO; break;
  }
}

void RFRgbLight::update_status(const millis_t ms) {
  if (force_white || status == ST_NOT_AUTO) return;

  uint8_t ns = ST_IDLE;
  HOTEND_LOOP() {
    const celsius_t t = thermalManager.degTargetHotend(e);
    const celsius_float_t c = thermalManager.degHotend(e);
    if (t > RF_RGB_ROOM_TEMP) {
      if (ABS(t - c) < RF_RGB_TEMP_TOLERANCE) ns = ST_PRINTING;
      else if (t > c) ns = ST_HEATING;
    }
  }
  #if HAS_HEATED_BED
    {
      const celsius_t t = thermalManager.degTargetBed();
      const celsius_float_t c = thermalManager.degBed();
      if (t > RF_RGB_ROOM_TEMP) {
        if (ABS(t - c) < RF_RGB_TEMP_TOLERANCE) { if (ns == ST_IDLE) ns = ST_PRINTING; }
        else if (t > c) ns = ST_HEATING;
      }
    }
  #endif
  HOTEND_LOOP()
    if (thermalManager.degHotend(e) - thermalManager.degTargetHotend(e) > RF_RGB_COOLDOWN && ns == ST_IDLE) ns = ST_COOLING;
  #if HAS_HEATED_BED
    if (thermalManager.degBed() - thermalManager.degTargetBed() > RF_RGB_COOLDOWN && ns == ST_IDLE) ns = ST_COOLING;
  #endif

  if (ns != status && !(ns == ST_IDLE && status == ST_COLOR_CHANGE)) {
    status = ns;
    switch (status) {
      case ST_PRINTING: target(RF_RGB_C_PRINTING); break;
      case ST_HEATING:  target(RF_RGB_C_HEATING); break;
      case ST_COOLING:  target(RF_RGB_C_COOLING); break;
      default:
        status = ST_IDLE;
        idle_start_ms = ms ? ms : 1;
        target(RF_RGB_C_IDLE);
        break;
    }
  }

  // idle for 30 s: color change
  if (status == ST_IDLE && idle_start_ms && ms - idle_start_ms > SEC_TO_MS(RF_RGB_CHANGE_DELAY_S)) {
    status = ST_COLOR_CHANGE;
    idle_start_ms = 0;
    target(255, 0, 0);
  }

  if (status == ST_COLOR_CHANGE && tgt[0] == cur[0] && tgt[1] == cur[1] && tgt[2] == cur[2]) {
    uint8_t &r = tgt[0], &g = tgt[1], &b = tgt[2];
    if      (r == 255 && g ==   0 && b ==   0) g = 255;   // red     -> yellow
    else if (r == 255 && g == 255 && b ==   0) r = 0;     // yellow  -> green
    else if (r ==   0 && g == 255 && b ==   0) b = 255;   // green   -> cyan
    else if (r ==   0 && g == 255 && b == 255) g = 0;     // cyan    -> blue
    else if (r ==   0 && g ==   0 && b == 255) r = 255;   // blue    -> magenta
    else if (r == 255 && g ==   0 && b == 255) b = 0;     // magenta -> red
    else target(255, 0, 0);
  }
}

void RFRgbLight::idle(const millis_t ms) {
  if (ELAPSED(ms, status_ms)) { status_ms = ms + 100; update_status(ms); }

  // fade one step per 10 ms (Repetier PWM ISR), catch up after a long blocking call
  if (!ELAPSED(ms, fade_ms)) return;
  uint8_t steps = _MIN((ms - fade_ms) / 10 + 1, 255UL);
  fade_ms = ms + 10;
  bool change = false;
  while (steps--) {
    bool step = false;
    for (uint8_t i = 0; i < 3; ++i) {
      if (tgt[i] > cur[i]) { ++cur[i]; step = true; }
      else if (tgt[i] < cur[i]) { --cur[i]; step = true; }
    }
    if (!step) break;
    change = true;
  }
  if (change) output();
}

#endif // RENKFORCE_RF && RF_RGB_R_PIN
