# Marlin RFxxxx – Marlin für Renkforce RF1000 / RF2000 / RF2000 v2

Marlin 2.1.2.9 mit den Funktionen der RF-Repetier-Firmware für das Conrad-Renkforce-Board (ATmega2560,
DRV8711-Treiber, DMS/Wägezelle). Single oder Dual Extruder.

> **Stand: Test.** Getestet bisher nur an einem RF2000 mit einem Hotend (2,85 mm). RF1000 und RF2000 v2
> kompilieren, sind aber auf echter Hardware nicht geprüft. Nutzung auf eigene Gefahr, die ersten Bewegungen
> mit der Hand am Hauptschalter beobachten.

## Was es kann (wie Repetier)

- Display und Menü wie die RF-Firmware: Startbild, Infoseiten 1–4 und Mod-Seite, Repetier-Menü auf Deutsch
- Heizbett-Scan mit dem DMS wie Repetier (`M3010`, PLA/ABS-Scan), Z-Matrix im I2C-EEPROM im **Repetier-Format**
  (9 Plätze, Repetier und Marlin lesen die Matrizen gegenseitig), Z-Offset-Scan `M3900`
- DRV8711-Motorstrom und Mikroschritte (`M906`, `M350`, `M122`, Register lesen/testen `M3922`), Mikroschritte X/Y/Z 1/16, E 1/32,
  im Menü Konfiguration → Mikroschritte änderbar
- Bauraum und Home wie Repetier (0 = Schalter); X/Y fest höchstens 150 mm/s und 1200 mm/s²
- Notfall-Pause, Sense Offset, Digit-Flow-Kompensation (`M3075`, `M3909`, `M3911`)
- Z-Tasten, E-Tasten, Pause/Fortsetzen, Gehäuselüfter, 230V- und 24V-Ausgänge
- RGB-Licht wie Repetier (Aus/Weiß/Auto/Manuell, `M3303`–`M3308`)

Technische Beschreibung: [RENKFORCE_RF.md](RENKFORCE_RF.md).

## Fertige Firmware

Unter [Releases](https://github.com/Dennis02121978/Marlin-RFxxxx/releases/latest) gibt es je Modell ein ZIP mit
fertiger `.hex`, `Configuration.h`, `Configuration_adv.h` und Anleitung:

| Drucker | Single Extruder | Dual Extruder |
|---|---|---|
| RF1000 | [Marlin_RF1000_Single.zip](https://github.com/Dennis02121978/Marlin-RFxxxx/releases/latest/download/Marlin_RF1000_Single.zip) | [Marlin_RF1000_Dual.zip](https://github.com/Dennis02121978/Marlin-RFxxxx/releases/latest/download/Marlin_RF1000_Dual.zip) |
| RF2000 | [Marlin_RF2000_Single.zip](https://github.com/Dennis02121978/Marlin-RFxxxx/releases/latest/download/Marlin_RF2000_Single.zip) | [Marlin_RF2000_Dual.zip](https://github.com/Dennis02121978/Marlin-RFxxxx/releases/latest/download/Marlin_RF2000_Dual.zip) |
| RF2000 v2 | [Marlin_RF2000v2_Single.zip](https://github.com/Dennis02121978/Marlin-RFxxxx/releases/latest/download/Marlin_RF2000v2_Single.zip) | [Marlin_RF2000v2_Dual.zip](https://github.com/Dennis02121978/Marlin-RFxxxx/releases/latest/download/Marlin_RF2000v2_Dual.zip) |

Eigene Werte (Bauraum, Extruder-Schritte, Düsenversatz, DMS-Grenzen …) stellt der **Renkforce Config Creator**
mit der Auswahl „Marlin“ ein. Er erzeugt `Configuration.h` und `Configuration_adv.h` als ZIP.

## Selbst bauen

1. Dieses Repo als ZIP laden (grüner Knopf „Code“ → „Download ZIP“) und entpacken.
2. `Configuration.h` und `Configuration_adv.h` aus dem Config Creator nach `Marlin/` kopieren (ersetzen).
3. Mit [PlatformIO](https://platformio.org/) bauen, **nur** mit der Umgebung `mega2560ext`:
   ```sh
   pio run -e mega2560ext
   ```
   Ergebnis: `.pio/build/mega2560ext/firmware.hex`. Mit `env:mega2560` bleibt das Display dunkel.

## Flashen

Drucker per USB anschließen. Läuft ein Host (Repetier-Server, OctoPrint, Klipper), diesen vorher stoppen.

```sh
# vorher das EEPROM sichern, falls du zu Repetier zurück willst
avrdude -patmega2560 -cwiring -P/dev/ttyUSB0 -b115200 -Ueeprom:r:eeprom_repetier.hex:i
# flashen
avrdude -patmega2560 -cwiring -P/dev/ttyUSB0 -b115200 -D -Uflash:w:firmware.hex:i
```

Unter Windows geht das auch mit XLoader oder Cura („Firmware aktualisieren“ → Benutzerdefiniert), Port COMx, 115200.

Nach dem ersten Start: `M500` (neues EEPROM-Layout), dann Heizbett-Scan `M3010` mit Aufsicht und `M3001`.
Verbindung zum Host: **250000 Baud** (bis v1.0.3: 115200), Firmware-Typ Marlin. Flashen bleibt bei 115200.
Wer von v1.0.2 oder älter kommt: X/Y waren auf 1/32, im Menü Konfiguration → Mikroschritte → XY 1/16 wählen und speichern.

## Slicer

- G-Code-Variante **Marlin 2**. Koordinaten wie Repetier: X 0 / Y 0 / Z 0 sind die Schalter. Am Y-Schalter steht die
  Düse vor dem Bett (Überstand). Druckbett: Größe = Bettbreite × (Y max − Überstand), Nullpunkt X 0 / Y −Überstand.
  RF1000 und RF2000 (Überstand 22 mm): Single Größe 240 × 223, Sperrbereich für die Prime Line `0x22,240x22,240x27,0x27`;
  Dual Größe 200 × 223, Sperrbereich `0x22,200x22,200x27,0x27`. Nullpunkt jeweils 0 / −22.
- Druckgeschwindigkeit am besten höchstens **130 mm/s** und **Arc fitting** an: Über USB schafft der ATmega2560 nur
  etwa 110 kurze Segmente pro Sekunde, darüber ruckeln Rundungen.
- Den passenden Start-G-Code (mit Vorextrudieren und Prime Line) und alle Orca-Werte gibt der Config Creator aus.
  Kurz:

```
G28
M3001          ; Z-Kompensation an (Matrix aus dem Heizbett-Scan)
```

Ist eine Matrix gespeichert, schaltet die Firmware die Z-Kompensation schon beim Start ein (Display: „Kmp“). `M3001` im Start-G-Code schadet nicht, `M3000` schaltet sie ab.

### Beispiel Orca: RF2000 Dual, 2,85 mm, Hotend V2, OctoPrint

Start-G-Code:

```
M140 S[bed_temperature_initial_layer_single] ; Bett heizen
M104 T[initial_tool] S{nozzle_temperature_initial_layer[initial_tool]}
G28                ; Home
M3001              ; Z-Kompensation (Heizbett-Matrix) an
M190 S[bed_temperature_initial_layer_single]
M109 T[initial_tool] S{nozzle_temperature_initial_layer[initial_tool]}
T[initial_tool]
; Vorextrudieren an der Home-Position (Duese steht vor dem Bett), dann Retract
M82
M900 K1.45    ; LinAdvance 2.85mm V2 Hotend
G92 E0
G1 Z5 F600
G1 E5 F200        ; 0.5 cm extrudieren
G1 E8 F2400        ; 2 mm Retract
; Prime Line vorne auf dem Bett (Bettkante bei Y18), Menge fuer 3.00 mm Filament
G1 X160 Y25 F5000
G1 Z0.35 F600
G1 E10 F2400       ; Retract zurueck
G1 X20 E14.1 F1000
G1 Y26 F1000
G1 X160 E18.2 F1000
G92 E0
G1 Z2 F3000
G92 E0
```

End-G-Code:

```
M104 S0
M140 S0
M107
G91
G1 E-2 F1800
G90
M3079            ; Objekt ausgeben (Bett nach unten)
M84
```

Hinweise dazu:
- `M900` (Linear Advance) ist ab v1.0.8 eingebaut, Standard K 0 = aus. K 1.45 ist für den Direktextruder des
  RF viel zu hoch (Extruder stottert); üblich sind etwa 0.02 bis 0.2. K mit dem Orca-Kalibriertest
  (Pressure Advance) bestimmen und den Wert dann in `M900 K..` eintragen.
- Mit `M82` (absolut) fördern `G1 E8` und `G1 E10` weiter vor (je 3 bzw. 2 mm), sie ziehen nicht zurück.
  Ein echter Retract nach `G1 E5` wäre `G1 E3`, das Zurückschieben `G1 E5` (so schreibt es der Config Creator).
- Dual: `M104 S0` schaltet nur die aktive Düse ab. Sicherer: `M104 S0 T0` und `M104 S0 T1`.

### Hotend wählen (ab v1.0.9)

Menü **Konfiguration → Temperaturen PID → Hotend**: V1, V2 Original, V3 NTC3950 oder E3D 104GT2 (wie im
RF-Repetier-Menü „Sens.“). Wird sofort gespeichert und setzt Thermistor und PID. Per G-Code: `M3930 S1` (V1),
`S2` (V2), `S3` (V3), `S4` (E3D), bei Dual mit `T1` für die zweite Düse, danach `M500`.

### USB-Host (OctoPrint, Repetier-Server)

OctoPrint schickt jede Zeile erst nach dem „ok“ der vorherigen. Der FTDI-Chip des RF-Boards hält Antworten
standardmäßig bis zu 16 ms zurück (`latency_timer`), dann kommen nur etwa 60 Zeilen pro Sekunde an und der Drucker
ruckelt schon bei 40 mm/s mit kurzen Segmenten. Abhilfe auf dem Pi (einmalig):

```
echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' | sudo tee /etc/udev/rules.d/99-ftdi-latency.rules
sudo udevadm control --reload
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

Pause: `M25`, Filamentwechsel: `M600`.

## Lizenz

GPLv3 wie Marlin, siehe [LICENSE](LICENSE). Das Original-README von Marlin: [README_MARLIN.md](README_MARLIN.md).
