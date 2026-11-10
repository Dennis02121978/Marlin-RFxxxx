#!/usr/bin/env python3
"""Baut die fertigen Firmware-Dateien fuer alle RF-Modelle (Single / Dual) mit den Standardwerten.

    python3 rf_tools/build_all.py            -> rf_release/Marlin_<Modell>_<Single|Dual>.hex

Configuration.h wird dafuer kurz geaendert und danach wiederhergestellt.
"""
import os, re, shutil, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, 'Marlin', 'Configuration.h')
OUT = os.path.join(ROOT, 'rf_release')
PIO = shutil.which('pio') or os.path.expanduser('~/.platformio/penv/bin/pio')
MODELS = [('RF1000', 'RF_MODEL_RF1000'), ('RF2000', 'RF_MODEL_RF2000'), ('RF2000v2', 'RF_MODEL_RF2000V2')]


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
                shutil.copyfile(os.path.join(ROOT, '.pio', 'build', 'mega2560ext', 'firmware.hex'),
                                os.path.join(OUT, 'Marlin_%s.hex' % tag))
    finally:
        open(CFG, 'w', encoding='utf-8').write(orig)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
