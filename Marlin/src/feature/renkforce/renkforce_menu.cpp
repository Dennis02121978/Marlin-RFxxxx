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
 * Main menu like the RF Repetier firmware (uimenu.h UI_MENU_MAIN) and the Klipper rf_display.cfg:
 *
 *  Hauptmenü
 *   Schnelleinstellung   Druck abbrechen, Home alle, Motoren aus, Objekt ausgeben, Geschw./Fluss Mul.,
 *                        Heizbett parken, X19 Licht, RGB Licht, Neustart
 *   Position             Home alle/X/Y/Z, Achsen bewegen
 *   Pause Position       X, Y, Z
 *   Extruder             Temperaturen, aus, Vorheizen PLA/ABS, Abkühlen, Filament laden/entladen, Fluss, E-Ursprung
 *   Lüfter               Lüfter %, aus, 25/50/75/100 %
 *   SD Karte             Drucke Datei, pausieren, fortsetzen, abbrechen
 *   Konfiguration        Z Kalibrierung, DMS Features, Temperaturen (PID), Beschleunigung, Feedrate,
 *                        Motorstrom, Gehäuselüfter, 230V, Speichern, Werkseinstellungen
 */

#include "../../inc/MarlinConfig.h"

#if ALL(RENKFORCE_RF, HAS_MARLINUI_MENU)

#include "renkforce.h"
#include "../../lcd/menu/menu_item.h"
#include "../../gcode/queue.h"
#include "../../MarlinCore.h"
#include "../../module/planner.h"
#include "../../module/motion.h"
#include "../../module/temperature.h"
#include "../../module/settings.h"
#if HAS_LEVELING
  #include "../bedlevel/bedlevel.h"
#endif
#if ENABLED(CASE_LIGHT_ENABLE)
  #include "../caselight.h"
#endif
#if HAS_MEDIA
  #include "../../sd/cardreader.h"
#endif


static bool m_bool;
static uint8_t m_u8;
static float m_float;
static uint16_t m_u16, m_u16b;

#define PRE1 "M140 S" STRINGIFY(PREHEAT_1_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND)
#define PRE2 "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND)

// ---- confirmations ------------------------------------------------------------------------

static void menu_rf_abort() {
  START_MENU();
  BACK_ITEM_F(F("Nein"));
  ACTION_ITEM_F(F("Ja, abbrechen"), []{ ui.abort_print(); ui.return_to_status(); });
  END_MENU();
}

static void menu_rf_reset() {
  START_MENU();
  BACK_ITEM_F(F("Nein"));
  ACTION_ITEM_F(F("Ja, neu starten"), []{ hal.reboot(); });
  END_MENU();
}

static void menu_rf_defaults() {
  START_MENU();
  BACK_ITEM_F(F("Nein"));
  ACTION_ITEM_F(F("Ja, zurücksetzen"), []{ settings.reset(); settings.save(); ui.return_to_status(); });
  END_MENU();
}

// ---- RGB Licht (RF Repetier: "RGB Licht: Weiss/Auto/Manuell/Aus", stored at once) ------------

static void save_now() { if (!printingIsActive()) settings.save(); }   // AVR: only changed bytes are written

#if PIN_EXISTS(RF_RGB_R)
  static FSTR_P rgb_label() {
    if (rf_rgb.force_white) return F("RGB Licht: Weiss (T)");
    switch (rf.settings.rgb_mode) {
      case RF_RGB_OFF:    return F("RGB Licht: Aus");
      case RF_RGB_WHITE:  return F("RGB Licht: Weiss");
      case RF_RGB_MANUAL: return F("RGB Licht: Manuell");
      default:            return F("RGB Licht: Auto");
    }
  }
  static void rgb_next() {
    rf_rgb.set_mode(rf_rgb.force_white ? rf.settings.rgb_mode : (rf.settings.rgb_mode + 1) % 4);
    save_now();
  }

  static uint8_t rgb_set;   // color set edited in menu_rf_rgb_color
  static void menu_rf_rgb_color() {
    START_MENU();
    BACK_ITEM_F(F("RGB Farben"));
    uint8_t * const c = rf.settings.rgb[rgb_set];
    EDIT_ITEM_FAST_F(uint8, F("Rot"),   &c[0], 0, 255, rf_rgb.colors_changed);
    EDIT_ITEM_FAST_F(uint8, F("Grün"),  &c[1], 0, 255, rf_rgb.colors_changed);
    EDIT_ITEM_FAST_F(uint8, F("Blau"),  &c[2], 0, 255, rf_rgb.colors_changed);
    END_MENU();
  }

  static void menu_rf_rgb() {
    START_MENU();
    BACK_ITEM_F(F("Konfiguration"));
    ACTION_ITEM_F(rgb_label(), rgb_next);
    #define _RGB_SET(L, N) ACTION_ITEM_F(F(L), []{ rgb_set = N; ui.push_current_screen(); ui.goto_screen(menu_rf_rgb_color); })
    _RGB_SET("Farbe Manuell",  RF_RGB_C_MANUAL);
    _RGB_SET("Farbe Heizen",   RF_RGB_C_HEATING);
    _RGB_SET("Farbe Drucken",  RF_RGB_C_PRINTING);
    _RGB_SET("Farbe Kühlen",   RF_RGB_C_COOLING);
    _RGB_SET("Farbe Leerlauf", RF_RGB_C_IDLE);
    ACTION_ITEM_F(F("Farben speichern"), save_now);
    END_MENU();
  }
#endif

// ---- Schnelleinstellung ----------------------------------------------------------------------

static void menu_rf_quick() {
  START_MENU();
  BACK_ITEM_F(F("Hauptmenü"));
  if (printingIsActive()) SUBMENU_F(F("Druck abbrechen"), menu_rf_abort);
  GCODES_ITEM_F(F("Home alle"), FPSTR(G28_STR));
  GCODES_ITEM_F(F("Motoren ausschalten"), F("M84"));
  if (!printingIsActive()) GCODES_ITEM_F(F("Objekt ausgeben"), F("M3079"));
  EDIT_ITEM_FAST_F(int3, F("Geschw. Mul."), &feedrate_percentage, 10, 999);
  EDIT_ITEM_FAST_F(int3, F("Fluss Mul."), &planner.flow_percentage[active_extruder], 10, 999, []{ planner.refresh_e_factor(active_extruder); });
  if (!printingIsActive()) GCODES_ITEM_F(F("Heizbett parken"), F("G27"));
  #if ENABLED(CASE_LIGHT_ENABLE)
    EDIT_ITEM_F(bool, F("X19 Licht"), &caselight.on, []{ caselight.update_enabled(); save_now(); });
  #endif
  #if PIN_EXISTS(RF_RGB_R)
    ACTION_ITEM_F(rgb_label(), rgb_next);
  #endif
  if (!printingIsActive()) SUBMENU_F(F("Neustart"), menu_rf_reset);
  END_MENU();
}

// ---- Position ---------------------------------------------------------------------------------

static void menu_rf_position() {
  START_MENU();
  BACK_ITEM_F(F("Hauptmenü"));
  GCODES_ITEM_F(F("Home alle"), FPSTR(G28_STR));
  GCODES_ITEM_F(axis_is_trusted(X_AXIS) ? F("Home X ok") : F("Home X"), F("G28 X"));
  GCODES_ITEM_F(axis_is_trusted(Y_AXIS) ? F("Home Y ok") : F("Home Y"), F("G28 Y"));
  GCODES_ITEM_F(axis_is_trusted(Z_AXIS) ? F("Home Z ok") : F("Home Z"), F("G28 Z"));
  SUBMENU_F(F("Achsen bewegen"), menu_move);
  END_MENU();
}

static void menu_rf_pausepos() {
  START_MENU();
  BACK_ITEM_F(F("Hauptmenü"));
  EDIT_ITEM_FAST_F(int4, F("Position X"), &rf.settings.pause_pos[0], int16_t(X_MIN_POS), int16_t(X_MAX_POS));
  EDIT_ITEM_FAST_F(int4, F("Position Y"), &rf.settings.pause_pos[1], int16_t(Y_MIN_POS), int16_t(Y_MAX_POS));
  EDIT_ITEM_FAST_F(int4, F("Position Z"), &rf.settings.pause_pos[2], 0, int16_t(Z_MAX_POS));
  END_MENU();
}

// ---- Extruder ---------------------------------------------------------------------------------

static void menu_rf_extruder() {
  START_MENU();
  BACK_ITEM_F(F("Hauptmenü"));
  EDIT_ITEM_FAST_F(int3, F("Temp. Bed"), &thermalManager.temp_bed.target, 0, BED_MAX_TARGET, thermalManager.start_watching_bed);
  EDIT_ITEM_FAST_F(int3, F("Temp. E0"), &thermalManager.temp_hotend[0].target, 0, thermalManager.hotend_max_target(0), []{ thermalManager.start_watching_hotend(0); });
  #if HAS_MULTI_HOTEND
    EDIT_ITEM_FAST_F(int3, F("Temp. E1"), &thermalManager.temp_hotend[1].target, 0, thermalManager.hotend_max_target(1), []{ thermalManager.start_watching_hotend(1); });
  #endif
  ACTION_ITEM_F(F("Extruder 0 aus"), []{ thermalManager.setTargetHotend(0, 0); });
  #if HAS_MULTI_HOTEND
    ACTION_ITEM_F(F("Extruder 1 aus"), []{ thermalManager.setTargetHotend(0, 1); });
  #endif
  ACTION_ITEM_F(F("Heizbett aus"), []{ thermalManager.setTargetBed(0); });
  #if HAS_MULTI_EXTRUDER
    if (!printingIsActive()) {
      GCODES_ITEM_F(F("Aktiver Extruder 0"), F("T0"));
      GCODES_ITEM_F(F("Aktiver Extruder 1"), F("T1"));
    }
  #endif
  GCODES_ITEM_F(F("Vorheizen PLA"), F(PRE1));
  GCODES_ITEM_F(F("Vorheizen ABS"), F(PRE2));
  ACTION_ITEM_F(F("Abkühlen"), []{ thermalManager.cooldown(); });
  #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
    if (!printingIsActive()) {
      GCODES_ITEM_F(F("Filament laden"), F("M701"));
      GCODES_ITEM_F(F("Filament entladen"), F("M702"));
    }
  #endif
  EDIT_ITEM_FAST_F(int3, F("Fluss Mul."), &planner.flow_percentage[active_extruder], 10, 999, []{ planner.refresh_e_factor(active_extruder); });
  if (!printingIsActive()) GCODES_ITEM_F(F("Setze E Ursprung"), F("G92 E0"));
  END_MENU();
}

// ---- Lüfter -----------------------------------------------------------------------------------

static void menu_rf_fan() {
  START_MENU();
  BACK_ITEM_F(F("Hauptmenü"));
  m_u8 = thermalManager.fan_speed[0];
  EDIT_ITEM_FAST_F(percent, F("Lüfter"), &m_u8, 0, 255, []{ thermalManager.set_fan_speed(0, m_u8); });
  GCODES_ITEM_F(F("Lüfter aus"), F("M107"));
  GCODES_ITEM_F(F("Lüfter auf  25%"), F("M106 S64"));
  GCODES_ITEM_F(F("Lüfter auf  50%"), F("M106 S128"));
  GCODES_ITEM_F(F("Lüfter auf  75%"), F("M106 S191"));
  GCODES_ITEM_F(F("Lüfter auf 100%"), F("M106 S255"));
  END_MENU();
}

// ---- SD Karte ---------------------------------------------------------------------------------

#if HAS_MEDIA
  static void menu_rf_sd() {
    START_MENU();
    BACK_ITEM_F(F("Hauptmenü"));
    if (!printingIsActive() && card.isMounted()) SUBMENU_F(F("Drucke Datei"), menu_media);
    if (!card.isMounted()) GCODES_ITEM_F(F("Karte einlesen"), F("M21"));
    if (printingIsActive() && !printingIsPaused()) ACTION_ITEM_F(F("Druck pausieren"), ui.pause_print);
    if (printingIsPaused()) ACTION_ITEM_F(F("Druck fortsetzen"), ui.resume_print);
    if (printingIsActive()) SUBMENU_F(F("Druck abbrechen"), menu_rf_abort);
    END_MENU();
  }
#endif

// ---- Konfiguration ----------------------------------------------------------------------------

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  static void menu_rf_zcal() {
    START_MENU();
    BACK_ITEM_F(F("Konfiguration"));
    if (!printingIsActive()) {
      GCODES_ITEM_F(F("Matrix Scan"), F("M3010"));
      GCODES_ITEM_F(F("Matrix Scan PLA"), F("M3010 S1"));
      GCODES_ITEM_F(F("Matrix Scan ABS"), F("M3010 S2"));
      GCODES_ITEM_F(F("Z-Offset-Scan"), F("M3900"));
      m_u8 = rf_zmatrix.active;
      EDIT_ITEM_F(uint8, F("Heizbett-Matrix"), &m_u8, 1, RF_ZMATRIX_SLOTS, []{ rf_zmatrix.select(m_u8); });
    }
    m_bool = planner.leveling_active;
    EDIT_ITEM_F(bool, F("Z-Kompensation"), &m_bool, []{ set_bed_leveling_enabled(m_bool); });
    #if HAS_HOME_OFFSET
      // RF Repetier static z offset: positive = more distance to the bed
      m_float = -home_offset.z;
      EDIT_ITEM_FAST_F(float43, F("Z Abstand"), &m_float, -2.0f, 2.0f, []{ set_home_offset(Z_AXIS, -m_float); });
    #endif
    END_MENU();
  }
#endif

static void menu_rf_sense() {
  START_MENU();
  BACK_ITEM_F(F("DMS Features"));
  EDIT_ITEM_FAST_F(int4, F("Level digits"), &rf.settings.sense_threshold, 0, 9999);
  EDIT_ITEM_FAST_F(uint16_3, F("Max um"), &rf.settings.sense_max_um, 0, 300);
  EDIT_ITEM_F(bool, F("AutoStart"), &rf.settings.sense_auto);
  m_bool = rf.sense_enabled;
  EDIT_ITEM_F(bool, F("Jetzt aktiv"), &m_bool, []{
    if (m_bool) rf.sense_enable(rf.settings.sense_threshold, rf.settings.sense_max_um); else rf.sense_disable();
  });
  END_MENU();
}

static void menu_rf_epause() {
  START_MENU();
  BACK_ITEM_F(F("DMS Features"));
  // 5 digits (RF Repetier default +-11000); Min is shown without the minus sign
  m_u16 = uint16_t(-rf.settings.pause_min);
  EDIT_ITEM_FAST_F(uint16_5, F("Min -digits"), &m_u16, 0, 16000, []{ rf.settings.pause_min = -int16_t(m_u16); });
  m_u16b = uint16_t(rf.settings.pause_max);
  EDIT_ITEM_FAST_F(uint16_5, F("Max digits"), &m_u16b, 0, 16000, []{ rf.settings.pause_max = int16_t(m_u16b); });
  END_MENU();
}

static void menu_rf_flowcmp() {
  START_MENU();
  BACK_ITEM_F(F("DMS Features"));
  EDIT_ITEM_FAST_F(int4, F("Digit Min"), &rf.settings.flow_min, 0, 9999);
  EDIT_ITEM_FAST_F(int4, F("Digit Max"), &rf.settings.flow_max, 0, 9999);
  EDIT_ITEM_FAST_F(int8, F("dFlow %"), &rf.settings.flow_percent, -99, 99);
  EDIT_ITEM_FAST_F(int8, F("dFeed %"), &rf.settings.feed_percent, -90, 99);
  END_MENU();
}

static void menu_rf_dms() {
  START_MENU();
  BACK_ITEM_F(F("Konfiguration"));
  STATIC_ITEM_F(F(" DMS "), SS_LEFT, i16tostr4signrj(rf_loadcell.ok ? rf_loadcell.value() : 0));
  ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
  ACTION_ITEM_F(F("DMS nullen"), []{ if (rf_loadcell.ok) rf_loadcell.tare_offset = rf_loadcell.raw; });
  SUBMENU_F(F("SenseOffset"), menu_rf_sense);
  SUBMENU_F(F("Notfall Pause"), menu_rf_epause);
  SUBMENU_F(F("Digit Flow CMP"), menu_rf_flowcmp);
  EDIT_ITEM_FAST_F(int4, F("Sonde digits"), &rf.settings.probe_trigger, 5, 5000);
  END_MENU();
}

static void menu_rf_pid() {
  START_MENU();
  BACK_ITEM_F(F("Konfiguration"));
  if (!printingIsActive()) {
    GCODES_ITEM_F(F("PID Extruder 0"), F("M303 E0 S210 C8 U1\nM500"));
    #if HAS_MULTI_HOTEND
      GCODES_ITEM_F(F("PID Extruder 1"), F("M303 E1 S210 C8 U1\nM500"));
    #endif
    GCODES_ITEM_F(F("PID Heizbett"), F("M303 E-1 S60 C8 U1\nM500"));
  }
  END_MENU();
}

static void apply_current() {
  for (uint8_t d = 0; d < RF_DRV_COUNT; ++d) rf_drv.set_current(d, rf.settings.drv_current[d]);
}

static void menu_rf_current() {
  START_MENU();
  BACK_ITEM_F(F("Konfiguration"));
  #define _CUR(L, D) EDIT_ITEM_F(uint16_4, F(L), &rf.settings.drv_current[D], 300, RF_DRV8711_CURRENT_LIMIT, apply_current)
  _CUR("Strom X mA", RF_DRV_X);
  _CUR("Strom Y mA", RF_DRV_Y);
  _CUR("Strom Z mA", RF_DRV_Z);
  _CUR("Strom E0 mA", RF_DRV_E0);
  #if HAS_MULTI_EXTRUDER
    _CUR("Strom E1 mA", RF_DRV_E1);
  #endif
  END_MENU();
}

// Mikroschritte wie RF Repetier (MicroStepXY / MicroStepsZ / MicroStepsE). Schritte/mm werden mitgerechnet,
// die Position bleibt. Dauerhaft erst mit "Einst. speichern" (M500).
static void ms_set_xy(const uint16_t ms) { rf.change_microsteps(RF_DRV_X, ms); rf.change_microsteps(RF_DRV_Y, ms); ui.go_back(); }
static void ms_set_z(const uint16_t ms)  { rf.change_microsteps(RF_DRV_Z, ms); ui.go_back(); }
static void ms_set_e(const uint16_t ms)  { rf.change_microsteps(RF_DRV_E0, ms); TERN_(DISTINCT_E_FACTORS, rf.change_microsteps(RF_DRV_E1, ms)); ui.go_back(); }

static void menu_rf_ms_xy() {
  START_MENU();
  BACK_ITEM_F(F("Mikroschritte"));
  ACTION_ITEM_F(F("XY 1/8"),  []{ ms_set_xy(8); });
  ACTION_ITEM_F(F("XY 1/16"), []{ ms_set_xy(16); });
  ACTION_ITEM_F(F("XY 1/32"), []{ ms_set_xy(32); });
  ACTION_ITEM_F(F("XY 1/64"), []{ ms_set_xy(64); });
  END_MENU();
}
static void menu_rf_ms_z() {
  START_MENU();
  BACK_ITEM_F(F("Mikroschritte"));
  ACTION_ITEM_F(F("Z 1/8"),  []{ ms_set_z(8); });
  ACTION_ITEM_F(F("Z 1/16"), []{ ms_set_z(16); });
  ACTION_ITEM_F(F("Z 1/32"), []{ ms_set_z(32); });
  END_MENU();
}
static void menu_rf_ms_e() {
  START_MENU();
  BACK_ITEM_F(F("Mikroschritte"));
  ACTION_ITEM_F(F("E 1/16"),  []{ ms_set_e(16); });
  ACTION_ITEM_F(F("E 1/32"),  []{ ms_set_e(32); });
  ACTION_ITEM_F(F("E 1/64"),  []{ ms_set_e(64); });
  ACTION_ITEM_F(F("E 1/128"), []{ ms_set_e(128); });
  END_MENU();
}
static void menu_rf_microsteps() {
  START_MENU();
  BACK_ITEM_F(F("Konfiguration"));
  STATIC_ITEM_F(F("XY 1/"), SS_LEFT, i16tostr3left(rf.settings.drv_microsteps[RF_DRV_X]));
  STATIC_ITEM_F(F("Z  1/"), SS_LEFT, i16tostr3left(rf.settings.drv_microsteps[RF_DRV_Z]));
  STATIC_ITEM_F(F("E  1/"), SS_LEFT, i16tostr3left(rf.settings.drv_microsteps[RF_DRV_E0]));
  if (!printingIsActive() && !planner.has_blocks_queued()) {
    SUBMENU_F(F("XY aendern"), menu_rf_ms_xy);
    SUBMENU_F(F("Z aendern"), menu_rf_ms_z);
    SUBMENU_F(F("E aendern"), menu_rf_ms_e);
  }
  ACTION_ITEM_F(F("Einst. speichern"), []{ settings.save(); });
  END_MENU();
}

static void menu_rf_config() {
  START_MENU();
  BACK_ITEM_F(F("Hauptmenü"));
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    SUBMENU_F(F("Z Kalibrierung"), menu_rf_zcal);
  #endif
  SUBMENU_F(F("DMS Features"), menu_rf_dms);
  SUBMENU_F(F("Temperaturen PID"), menu_rf_pid);
  EDIT_ITEM_FAST_F(float5_25, F("Beschleunigung"), &planner.settings.acceleration, 25, 5000);
  m_float = planner.settings.max_feedrate_mm_s[X_AXIS];
  EDIT_ITEM_FAST_F(float3, F("Max v mm/s"), &m_float, 10, 300, []{
    planner.settings.max_feedrate_mm_s[X_AXIS] = planner.settings.max_feedrate_mm_s[Y_AXIS] = m_float;
  });
  SUBMENU_F(F("Motorstrom"), menu_rf_current);
  SUBMENU_F(F("Mikroschritte"), menu_rf_microsteps);
  #if PIN_EXISTS(RF_RGB_R)
    SUBMENU_F(F("RGB Licht"), menu_rf_rgb);
  #endif
  m_u8 = rf.settings.case_fan_mode;
  EDIT_ITEM_F(uint8, F("Gehäusel. 0A 1E 2A"), &m_u8, 0, 2, []{ rf.set_case_fan_mode(m_u8); });
  #if PIN_EXISTS(RF_OUTPUT_230V)
    m_bool = READ(RF_OUTPUT_230V_PIN);
    EDIT_ITEM_F(bool, F("230V Ausgang"), &m_bool, []{ rf.set_output(0, m_bool); });
  #endif
  ACTION_ITEM_F(F("Einst. speichern"), []{ settings.save(); });
  SUBMENU_F(F("Werkseinstellungen"), menu_rf_defaults);
  END_MENU();
}

// ---- Hauptmenü --------------------------------------------------------------------------------

void menu_renkforce() {
  START_MENU();
  BACK_ITEM_F(F("Info Seiten"));
  SUBMENU_F(F("Schnelleinstellung"), menu_rf_quick);
  SUBMENU_F(F("Position"), menu_rf_position);
  SUBMENU_F(F("Pause Position"), menu_rf_pausepos);
  SUBMENU_F(F("Extruder"), menu_rf_extruder);
  SUBMENU_F(F("Lüfter"), menu_rf_fan);
  #if HAS_MEDIA
    SUBMENU_F(F("SD Karte"), menu_rf_sd);
  #endif
  SUBMENU_F(F("Konfiguration"), menu_rf_config);
  END_MENU();
}

#endif // RENKFORCE_RF && HAS_MARLINUI_MENU
