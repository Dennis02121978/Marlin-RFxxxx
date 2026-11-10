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
 * Status screen like the RF Repetier firmware (uimenu.h UI_PAGES) and the Klipper
 * rf_display.cfg: page 1, mod page, page 2, page 3, page 4. Keys up / down change the
 * page, "back" returns to page 1, OK opens the menu.
 *
 *  Page 1     |T 215/215° B 60/ 60°|   Page 2  |X: 100.00 mm        |
 *             |Z:  0.25  -0.31 Kmp |           |Y:  50.00 mm        |
 *             |P:   -12 L: 100%    |           |Z:  0.25  -0.31 Kmp |
 *             |Drucker bereit.     |           |F: 100mm/s 100%     |
 *  Mod page   |P:   -12     V: 100%|   Page 3  |E0:215/215°C-> 45%  |
 *             |zO:    0um zMat: An |           |                    |
 *             |sO:  12um @ 1000    |           |B:  60/ 60°C-> 30%  |
 *             |Z:  0.25  -0.31 Kmp |           |Drucker bereit.     |
 *  Page 4     |file name           |
 *             |Zeit:   01:23:45    |
 *             |Fortschritt:  42%   |
 *             |Drucker bereit.     |
 */

#include "../../inc/MarlinConfig.h"

#if ALL(RENKFORCE_RF, HAS_MARLINUI_HD44780)

#include "renkforce.h"
#include "../../MarlinCore.h"
#include "../../lcd/marlinui.h"
#include "../../lcd/lcdprint.h"
#include "../../module/temperature.h"
#include "../../module/planner.h"
#include "../../module/motion.h"
#include "../../module/probe.h"
#include "../../module/printcounter.h"
#include "../../gcode/gcode.h"
#if HAS_LEVELING
  #include "../bedlevel/bedlevel.h"
#endif
#if HAS_MEDIA
  #include "../../sd/cardreader.h"
#endif
#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "../pause.h"
#endif

#define RF_PAGES 5   // page 1, mod page, page 2, page 3, page 4

uint8_t Renkforce::status_page; // = 0

void Renkforce::status_page_step(const int16_t dir) {
  if (dir > 0) status_page = (status_page + 1) % RF_PAGES;
  else if (dir < 0) status_page = (status_page + RF_PAGES - 1) % RF_PAGES;
}

static char line[LCD_WIDTH + 8];

// write the line buffer, padded with spaces to the display width
static void out(const uint8_t row) {
  uint8_t n = strlen(line);
  while (n < LCD_WIDTH) line[n++] = ' ';
  line[LCD_WIDTH] = '\0';
  lcd_moveto(0, row);
  lcd_put_u8str(line);
}

static char *fmt_f(char *p, const float v, const int8_t w, const uint8_t prec) { return dtostrf(v, w, prec, p) + strlen(p); }

static void z_line(const uint8_t row) {
  char *p = line;
  *p++ = 'Z';
  *p++ = axis_is_trusted(Z_AXIS) ? ':' : '?';
  p = fmt_f(p, current_position.z, 6, 2);
  *p++ = ' ';
  float corr = 0;
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    if (planner.leveling_active && bedlevel.has_mesh()) corr = bedlevel.get_z_correction(current_position);
  #endif
  p = fmt_f(p, corr, 6, 2);
  strcpy_P(p, planner.leveling_active ? PSTR(" Kmp") : PSTR(""));
  out(row);
}

static void status_line(const uint8_t row) {
  if (ui.status_message[0]) {
    strncpy(line, ui.status_message, LCD_WIDTH);
    line[LCD_WIDTH] = '\0';
  }
  else if (printingIsPaused() || TERN0(ADVANCED_PAUSE_FEATURE, did_pause_print))
    strcpy_P(line, PSTR("Pause"));
  else if (printingIsActive())
    sprintf_P(line, PSTR("Druck %3u%%"), ui.get_progress_percent());
  else
    strcpy_P(line, PSTR("Drucker bereit."));
  out(row);
}

static uint8_t heater_pct(const heater_id_t h) { return uint8_t(int32_t(thermalManager.getHeaterPower(h)) * 100 / 255); }

static void hotend_line(const uint8_t row, const uint8_t e) {
  sprintf_P(line, PSTR("E%u:%3i/%3i" LCD_STR_DEGREE "C->%3u%%"), e,
            int(thermalManager.degHotend(e) + 0.5f), thermalManager.degTargetHotend(e), heater_pct((heater_id_t)e));
  out(row);
}

static void axis_line(const uint8_t row, const AxisEnum a, const char c) {
  char *p = line;
  *p++ = c;
  *p++ = axis_is_trusted(a) ? ':' : '?';
  p = fmt_f(p, current_position[a], 7, 2);
  strcpy_P(p, PSTR(" mm"));
  out(row);
}

void Renkforce::draw_status() {
  // "back" key on the status screen: page 1 (Klipper rf_display: zurueck springt auf Seite 1)
  #if PIN_EXISTS(BTN_BACK)
    if (!READ(BTN_BACK)) status_page = 0;
  #endif

  const int16_t dms = rf_loadcell.ok ? rf_loadcell.value() : 0;

  switch (status_page) {
    default:
    case 0: {   // page 1
      sprintf_P(line, PSTR(LCD_STR_THERMOMETER "%3i/%3i" LCD_STR_DEGREE " B%3i/%3i" LCD_STR_DEGREE),
                int(thermalManager.degHotend(active_extruder) + 0.5f), thermalManager.degTargetHotend(active_extruder),
                int(thermalManager.degBed() + 0.5f), thermalManager.degTargetBed());
      out(0);
      z_line(1);
      sprintf_P(line, PSTR("P:%6i L: %3u%%"), dms, thermalManager.fanSpeedPercent(0));
      out(2);
      status_line(3);
    } break;

    case 1: {   // mod page (RF Repetier ui_page_mod)
      sprintf_P(line, PSTR("P:%6i     V: %3i%%"), dms, feedrate_percentage);
      out(0);
      #if HAS_HOME_OFFSET
        const int16_t zo = int16_t(LROUND(-home_offset.z * 1000));
      #else
        const int16_t zo = 0;
      #endif
      sprintf_P(line, PSTR("zO:%5ium zMat: %S"), zo, planner.leveling_active ? PSTR("An") : PSTR("Aus"));
      out(1);
      const int16_t so = int16_t(LROUND(sense_offset * 1000));
      if (sense_enabled)       sprintf_P(line, PSTR("sO:%4ium @%5i"), so, settings.sense_threshold);
      else if (so > 0)         sprintf_P(line, PSTR("sO:%4ium ^%5i"), so, settings.sense_threshold);
      else                     sprintf_P(line, PSTR("sO:%4ium Aus"), so);
      out(2);
      z_line(3);
    } break;

    case 2: {   // page 2
      axis_line(0, X_AXIS, 'X');
      axis_line(1, Y_AXIS, 'Y');
      z_line(2);
      sprintf_P(line, PSTR("F:%4imm/s %3i%%"), int(feedrate_mm_s + 0.5f), feedrate_percentage);
      out(3);
    } break;

    case 3: {   // page 3: heaters
      hotend_line(0, 0);
      #if HAS_MULTI_HOTEND
        hotend_line(1, 1);
      #else
        line[0] = '\0'; out(1);
      #endif
      sprintf_P(line, PSTR("B: %3i/%3i" LCD_STR_DEGREE "C->%3u%%"),
                int(thermalManager.degBed() + 0.5f), thermalManager.degTargetBed(), heater_pct(H_BED));
      out(2);
      status_line(3);
    } break;

    case 4: {   // page 4: print job
      #if HAS_MEDIA
        if (IS_SD_PRINTING() || card.isPaused()) { strncpy(line, card.longest_filename(), LCD_WIDTH); line[LCD_WIDTH] = '\0'; }
        else
      #endif
          strcpy_P(line, printingIsActive() ? PSTR("Host-Druck") : PSTR("Kein Druck"));
      out(0);
      const uint32_t t = print_job_timer.duration();
      sprintf_P(line, PSTR("Zeit:   %02u:%02u:%02u"), unsigned(t / 3600), unsigned((t / 60) % 60), unsigned(t % 60));
      out(1);
      sprintf_P(line, PSTR("Fortschritt: %3u%%"), ui.get_progress_percent());
      out(2);
      status_line(3);
    } break;
  }
}

#endif // RENKFORCE_RF && HAS_MARLINUI_HD44780
