#!/usr/bin/env python3
"""Baut die fertigen Firmware-Dateien fuer alle RF-Modelle (Single / Dual) mit den Standardwerten.

    python3 rf_tools/build_all.py   -> rf_release/Marlin_<Modell>_<Single|Dual>.hex und .zip

Das ZIP enthaelt firmware.hex, Configuration.h, Configuration_adv.h und LIESMICH.txt (Release-Asset).

Configuration.h wird dafuer kurz geaendert und danach wiederhergestellt.
"""
import os, re, shutil, subprocess, sys, zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, 'Marlin', 'Configuration.h')
OUT = os.path.join(ROOT, 'rf_release')
PIO = shutil.which('pio') or os.path.expanduser('~/.platformio/penv/bin/pio')
MODELS = [('RF1000', 'RF_MODEL_RF1000'), ('RF2000', 'RF_MODEL_RF2000'), ('RF2000v2', 'RF_MODEL_RF2000V2')]


def readme(name, dual):
    return """Marlin RFxxxx fuer %s, %s Extruder (Standardwerte)
https://github.com/Dennis02121978/Marlin-RFxxxx

Marlin_%s_%s.hex   fertige Firmware zum Flashen
Configuration.h    Einstellungen, mit denen die .hex gebaut ist
Configuration_adv.h

Flashen (Host wie Repetier-Server, OctoPrint oder Klipper vorher stoppen):
  EEPROM sichern, falls du zu Repetier zurueck willst:
    avrdude -patmega2560 -cwiring -P/dev/ttyUSB0 -b115200 -Ueeprom:r:eeprom_repetier.hex:i
  Flashen:
    avrdude -patmega2560 -cwiring -P/dev/ttyUSB0 -b115200 -D -Uflash:w:Marlin_%s_%s.hex:i
  Windows: XLoader oder Cura (Firmware aktualisieren, Benutzerdefiniert), 115200 Baud.

Nach dem ersten Start: M500, dann Heizbett-Scan M3010 mit Aufsicht und M3001.
Host-Verbindung: 250000 Baud (bis v1.0.3: 115200), Firmware Marlin.

Koordinaten wie Repetier: X=0, Y=0, Z=0 sind die Schalter. Im Slicer (Orca) G-Code Marlin 2,
Druckbett-Nullpunkt Y = -Ueberstand der Duese vor dem Bett (z. B. -22), Tiefe = Y max - Ueberstand.
Die genauen Werte je Modell zeigt der Config Creator (Auswahl Marlin, Seite Slicer).

Eigene Werte: Renkforce Config Creator, Auswahl Marlin. Die beiden Configuration-Dateien
nach Marlin/ im Quellcode kopieren und mit "pio run -e mega2560ext" bauen.

Ohne Gewaehr. Die ersten Bewegungen mit der Hand am Hauptschalter beobachten.
""" % (name, 'Dual' if dual else 'Single', name, 'Dual' if dual else 'Single', name, 'Dual' if dual else 'Single')


def main():
    orig = open(CFG, encoding='utf-8').read()
    os.makedirs(OUT, exist_ok=True)
    ok = True
    try:
        for name, macro in MODELS:
            for dual in (False, True):
                s = re.sub(r'^#define RF_MODEL .*$', '#define RF_MODEL ' + macro, orig, count=1, flags=re.M)
                s = re.sub(r'^/*#define RF_DUAL_EXTRUDER.*$', ('' if dual else '//') + '#define RF_DUAL_EXTRUDER', s, count=1, flags=re.M)
                open(CFG, 'w', encoding='utf-8').write(s)
                tag = '%s_%s' % (name, 'Dual' if dual else 'Single')
                env = {k: v for k, v in os.environ.items() if not k.startswith(('PYTHON', '__PYVENV'))}
                r = subprocess.run([PIO, 'run', '-e', 'mega2560ext'], cwd=ROOT, capture_output=True, text=True, env=env)
                lines = [l for l in r.stdout.splitlines() if 'Flash:' in l or 'RAM:' in l or 'error' in l]
                print(tag, 'OK' if r.returncode == 0 else 'FEHLER', *lines, sep='\n  ')
                if r.returncode:
                    ok = False
                    print(r.stdout[-3000:], r.stderr[-2000:])
                    continue
                hexfile = os.path.join(OUT, 'Marlin_%s.hex' % tag)
                shutil.copyfile(os.path.join(ROOT, '.pio', 'build', 'mega2560ext', 'firmware.hex'), hexfile)
                adv = open(os.path.join(ROOT, 'Marlin', 'Configuration_adv.h'), encoding='utf-8').read()
                with zipfile.ZipFile(os.path.join(OUT, 'Marlin_%s.zip' % tag), 'w', zipfile.ZIP_DEFLATED) as z:
                    z.write(hexfile, 'Marlin_%s.hex' % tag)
                    z.writestr('Configuration.h', s)
                    z.writestr('Configuration_adv.h', adv)
                    z.writestr('LIESMICH.txt', readme(name, dual).replace('\n', '\r\n'))
    finally:
        open(CFG, 'w', encoding='utf-8').write(orig)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
