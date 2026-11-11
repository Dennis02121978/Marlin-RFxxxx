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
- DRV8711-Motorstrom und Mikroschritte (`M906`, `M350`, `M122`)
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
Verbindung zum Host: 115200 Baud, Firmware-Typ Marlin.

## Slicer

Start-G-Code zum Beispiel:

```
G28
M3001          ; Z-Kompensation an (Matrix aus dem Heizbett-Scan)
```

Ist eine Matrix gespeichert, schaltet die Firmware die Z-Kompensation schon beim Start ein (Display: „Kmp“). `M3001` im Start-G-Code schadet nicht, `M3000` schaltet sie ab.

Pause: `M25`, Filamentwechsel: `M600`.

## Lizenz

GPLv3 wie Marlin, siehe [LICENSE](LICENSE). Das Original-README von Marlin: [README_MARLIN.md](README_MARLIN.md).
