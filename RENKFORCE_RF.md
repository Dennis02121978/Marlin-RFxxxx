# Marlin 2.1.2.9 für Renkforce RF1000 / RF2000 / RF2000 v2 – technische Beschreibung

Port der Funktionen der RF-Repetier-Firmware nach Marlin 2.1.2.9.

## Einstellungen

`Marlin/Configuration.h`, Block `BEGIN RF CONFIG CREATOR` … `END RF CONFIG CREATOR`. Diesen Block schreibt der
Renkforce Config Creator (Auswahl „Marlin“). Auskommentierte Werte nehmen den Standard des Modells.

| Wert | Bedeutung |
|---|---|
| `RF_MODEL` | `RF_MODEL_RF1000`, `RF_MODEL_RF2000`, `RF_MODEL_RF2000V2` |
| `RF_DUAL_EXTRUDER` | zwei Extruder |
| `RF_FILAMENT_DIA` | 1.75 / 2.85 / 3.00 mm |
| `RF_X_MAX`, `RF_Y_MAX`, `RF_Z_MAX` | Bauraum wie RF Repetier (`X_MAX_LENGTH_PRINT`, `Y_MAX_LENGTH`, `Z_MAX_LENGTH`), siehe „Bauraum und Home“ |
| `RF_E_STEPS` | Extruder-Schritte/mm (32 Mikroschritte) |
| `RF_T1_OFFSET_X/Y` | Versatz Düse 2 (Dual) |
| `RF_PREHEAT_HOTEND/BED` | Vorheizen 1 |
| `RF_RGB_LIGHT` | RGB-Licht (RF2000 / RF2000 v2) |
| `RF_EMERGENCY_PAUSE_MIN/MAX` | Notfall-Pause in Digits (0 = aus) |
| `RF_FLOW_COMP_*` | Digit-Flow-Kompensation |

Daraus folgen Board (`BOARD_RENKFORCE_RF1000` / `BOARD_RENKFORCE_RF2000`), Raster des Heizbett-Scans und Display
(RF1000 16x4 a00, sonst 20x4 westlich). Motorstrom, DMS-Werte und Tasten stehen am Ende von `Configuration_adv.h`.

## Bauraum und Home

Wie in der RF-Repetier-Firmware (`RF1000.h`, `RF2000.h`, `RF2000v2.h`): X=0, Y=0 und Z=0 sind die Schalter,
gehomt wird X, Y, Z nach min. G-Codes fahren nie unter Z=0; die Z-Tasten dürfen bis zum Überfahrweg darunter.

- RF1000: X 245 (Dual 200), Y 245, Z 200, Überfahrweg Z 0,8 mm, Düse 2 X 33,9 / Y 0,1
- RF2000: X 230 (Dual 200), Y 245, Z 200, Überfahrweg Z 1,3 mm, Düse 2 X 33,9 / Y 0,1
- RF2000 v2: X 230 (Dual 170), Y 290, Z 185, Überfahrweg Z 3,0 mm, Düse 2 X 33,6 / Y 0

Dual: Der X-Schalter liegt bei X = −Versatz Düse 2 (Repetier fährt T0 nach dem Homing dorthin), damit beide
Düsen X 0 … X max erreichen; T1 darf bis X max + Versatz. Homing 80/80/9 mm/s, zweites Anfahren mit 1/20
(Repetier `HOMING_FEEDRATE_*_PRINT`, `ENDSTOP_*_BACK_MOVE`, `ENDSTOP_*_RETEST_REDUCTION_FACTOR`).
Die Koordinaten der Z-Matrix im 24C256 sind damit dieselben wie in Marlin. M3079 (Objekt ausgeben) fährt wie
Repetier auf Z max und Y 245.

## Bauen und flashen

```sh
pio run -e mega2560ext     # nur diese Umgebung: erweiterte Pins (PJ3 = Display D7, PD4-6, PE6/7)
avrdude -patmega2560 -cwiring -P/dev/ttyUSB0 -b115200 -D -Uflash:w:.pio/build/mega2560ext/firmware.hex:i
```

Nach dem ersten Flashen meldet Marlin „EEPROM datasize error“ und lädt Standardwerte; dann `M500`.
Serielle Verbindung 115200 Baud. `python3 rf_tools/build_all.py` baut alle 6 Varianten nach `rf_release/`.

**Vor dem Flashen das interne EEPROM sichern**, wenn du zu Repetier zurück willst:
`avrdude -patmega2560 -cwiring -P/dev/ttyUSB0 -b115200 -Ueeprom:r:eeprom_repetier.hex:i`

## EEPROM

- **internes EEPROM (4 KB):** Marlin-Einstellungen (`M500`/`M501`).
- **externes I2C-EEPROM 24C256 (0x50):** Heizbett-Matrizen **wie bei Repetier** (Format 7, 9 Plätze à 1536 Bytes,
  aktiver Platz im Kopf). Beide Firmwares lesen die Matrizen des jeweils anderen.
  - Beim Start wird die aktive Matrix geladen, auf das Marlin-Raster umgerechnet und die Z-Kompensation
    eingeschaltet (wie Klipper mit `bed_mesh_init`, Display zeigt „Kmp“). Abschalten mit `M3000`.
  - `M3009 S<1..9>` Platz wählen und laden, `M3011 [S]` löschen, `M3013 [S<Platz>] [P1]` ausgeben wie Repetier (`P1` = mm),
    `M3014 [S]` speichern, `M3000`/`M3001` Kompensation aus/an.
- `M3060` prüft, ob 0x49 (DMS), 0x50 und 0x51 antworten.

## Heizbett-Scan wie Repetier (`M3010`, `G29`)

`src/feature/renkforce/hbscan.cpp`, Ablauf wie RF Repetier `scanHeatBed`:
- Raster ab X 15 / Y 30 (ab Y-Schalter) alle 20 mm bis 5 mm vor Achsende, Schlangenlinie.
  RF2000: 10 × 11 Punkte, RF2000 v2: 10 × 14. Das Marlin-Mesh ist genau dieses Raster.
- Je Punkt: Leerlaufdruck messen, Bett in 0,02-mm-Schritten hoch bis ±10 Digits, in 0,004-mm-Schritten zurück
  bis ±5 Digits, wieder 0,004 mm hoch bis Kontakt, in Einzelschritten zurück bis ±5 Digits → Wert. Danach 0,3 mm runter (`RF_SCAN_CLEARANCE_MM`, Repetier 0,1 mm) und zum nächsten Punkt.
- Fehler (Z außerhalb, Bett zu uneben, Druck nicht konstant): bis zu 5 Wiederholungen.
- Ergebnis im Repetier-Format in den aktiven Platz des 24C256, Kompensation an.
- `M3010 S1` PLA / `S2` ABS: Bett 60/100 °C, Düse 100 °C, 10 min warten, scannen, dann Düse 210/240 °C,
  10 min warten und Matrix am Kalibrierpunkt (100/100) korrigieren.
- Abbrechen: Zurück-Taste 1 s halten. `G29 M …` = Marlins eigenes Abtasten.
- `M3900` Z-Offset-Scan wie Repetier `searchZOScan`, verschiebt die Matrix.

## Display und Tasten wie Repetier

- Startbild (Würfel, Titel, Version, Ladebalken, Marlin-Version), Texte `RF_BOOT_*` in `Configuration_adv.h`.
- Infoseiten: Seite 1 (Temperaturen, Z mit Kompensation, DMS P, Lüfter, Status), Mod-Seite (DMS, Tempo, zO,
  Z-Matrix, Sense Offset), Seite 2 (X/Y/Z, Vorschub), Seite 3 (Heizleistung), Seite 4 (Datei, Zeit, Fortschritt).
  Hoch/runter blättert, links = Seite 1 (und RGB „immer Weiß“), OK = Menü.
- Menü wie Repetier: Schnelleinstellung, Position, Pause Position, Extruder, Lüfter, SD Karte, Konfiguration
  (Z Kalibrierung, DMS Features, PID, Beschleunigung, Max v, Motorstrom, RGB Licht, Gehäuselüfter, 230V,
  Einst. speichern, Werkseinstellungen). Das Original-Marlin-Menü ist entfernt.
- Z-Tasten: Bett fährt, solange gedrückt (auch ohne Referenzfahrt; nicht referenziert stoppt der Z-Schalter),
  im Druck 0,01 mm pro Druck, auf der Mod-Seite Z-Offset ±5 µm.
- E-Tasten: kurz 2 mm, gehalten Dauerlauf (nur mit heißer Düse). Pause / Fortsetzen.

## RGB-Licht (RF2000 / RF2000 v2)

`src/feature/renkforce/rgblight.cpp` (Timer 4, 100 Hz, Überblenden 1 Stufe / 10 ms).
- `M3308 P0..3`: Aus, Weiß, Auto, Manuell. Auto: Heizen rot, erreicht weiß, Abkühlen blau, Leerlauf weiß,
  nach 30 s Leerlauf Farbwechsel. Farben `M3303`–`M3307` (`P1..3 S0..255` oder `R U B`), `M150 R U B` = manuell.
- Beim Einschalten blendet das Licht in die Farbe des gespeicherten Modus über.

## Was übertragen ist

| Repetier | Marlin |
|---|---|
| DRV8711-Treiber (Software-SPI, CS high aktiv) | `drv8711.cpp`, `M906` (mA), `M350`, `M122`; Mikroschritte Standard X/Y/Z 16, E 32, im Menü Konfiguration → Mikroschritte änderbar (Schritte/mm werden mitgerechnet, speichern mit „Einst. speichern“) |
| DMS (ADS1100, 0x49) | `loadcell.cpp`, Werte in Repetier-Digits, `M3002` |
| Heizbett-Scan, Z-Kompensation, Z-Matrix | `hbscan.cpp`, `zmatrix.cpp` (siehe oben) |
| `FEATURE_EMERGENCY_PAUSE` | `M3075 L H` → Pause an der Pause-Position (`M3105 X Y Z`) |
| `FEATURE_SENSIBLE_PRESSURE` (Sense Offset) | `M3909 P S A` über Babysteps |
| `FEATURE_DIGIT_FLOW_COMPENSATION` | `M3911 S F L H` |
| `FEATURE_ZERO_DIGITS` | DMS wird bei Druckstart genullt |
| Gehäuselüfter `M3120/M3121`, FETs `M3300`, 230V `M3301` | wie Repetier |
| Ausgeben `M3079`, `M3060`, `M3070` | vorhanden |
| SD-Karte im Drucker | Marlin `SDSUPPORT` |

Nicht übertragen: Fräsmodus/Werkstück-Scan, `M3901`/`M3902`, `M3912`–`M3914`, `M3939`, `M3309`,
Not-Stopp Z/E, `FEATURE_DIGIT_Z_COMPENSATION`, Wartungsintervall.

## Vor dem ersten Druck prüfen

1. `M119`: Schalter frei = open, betätigt = TRIGGERED.
2. Achsen mit kleinen Wegen fahren, Richtung prüfen.
3. `M122`: alle Treiber ok.
4. `M3002`: DMS-Wert ändert sich, wenn man von unten gegen die Düse drückt.
5. `M3010` (Heizbett-Scan) mit Aufsicht, danach `M3001`.
6. Dual: Düsenversatz `M218 T1 X… Y…` mit Testdruck prüfen.

## Geänderte Marlin-Dateien

- neu: `src/feature/renkforce/*`, `src/pins/mega/pins_RENKFORCE_RF.h`, `rf_tools/`
- `Configuration.h`, `Configuration_adv.h`, `ini/features.ini`, `platformio.ini`, `core/boards.h`, `pins/pins.h`
- Einhängepunkte: `MarlinCore.cpp`, `gcode/gcode.cpp`, `gcode/calibrate/G28.cpp`, `gcode/bedlevel/abl/G29.cpp`,
  `module/endstops.cpp`, `module/probe.h/.cpp`, `module/settings.cpp`, `lcd/menu/menu_main.cpp`, `lcd/marlinui.cpp`,
  `lcd/HD44780/marlinui_HD44780.cpp`
