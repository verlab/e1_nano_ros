"""PlatformIO pre-build script: assemble the firmware sources into .e1_src/.

CF-Drone is used unmodified as a git submodule (external/CF-Drone). Two variants:

* Zenoh (``custom_e1_zenoh = yes``, default): copies CF-Drone, adds src_e1/, one include and five
  one-line hooks.
* Stock (``custom_e1_zenoh = no``): CF-Drone exactly as upstream (MAVLink / web RC / console).
 Every anchor is checked, so a CF-Drone update
that moves an anchor fails loudly instead of producing a silently broken firmware.
"""
import os
import shutil

import sys

try:
    Import('env')  # noqa: F821  (PlatformIO / SCons)
    PROJ = env.subst('$PROJECT_DIR')  # noqa: F821
    ZENOH = env.GetProjectOption('custom_e1_zenoh', 'yes').strip().lower() in ('yes', 'true', '1')  # noqa: F821
except NameError:   # allow running stand-alone: python3 tools/prepare_src.py [--stock]
    PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ZENOH = '--stock' not in sys.argv

CF = os.path.join(PROJ, 'external', 'CF-Drone')
OUT = os.path.join(PROJ, '.e1_src')
E1 = os.path.join(PROJ, 'src_e1')

# Forward declarations PlatformIO's .ino -> .cpp converter misses (functions returning pointers).
# Needed by the stock build too; harmless with the Arduino IDE.
PROTOTYPES = {'mavlink.ino': 'const char *getParameterName(int index);  // E1: missing prototype (parameters.ino)\n'}

PATCHES = {
    'CF-Drone.ino': [
        ('#include "board_config.h"', '#include "board_config.h"\n#include "e1_zenoh_link.h" // E1 Zenoh link'),
        ('\tsetupRC();', '\tsetupRC();\n\tsetupZenohLink(); // E1 Zenoh link'),
        ('\treadRC();', '\treadRC();\n\tzenohLinkApply(); // E1 Zenoh link: apply received commands'),
        ('\tsendMotors();', '\tsendMotors();\n\tzenohLinkSnapshot(); // E1 Zenoh link: telemetry snapshot'),
    ],
    'cli.ino': [
        ('\tcommand.toLowerCase();', '\tcommand.toLowerCase();\n\tif (zenohLinkCommand(command, arg0, arg1)) return; // E1 Zenoh link'),
        ('\tSerial.print(buf);', '\tSerial.print(buf);\n\tzenohLinkLog(buf); // E1 Zenoh link'),
    ],
}


def main():
    if not os.path.exists(os.path.join(CF, 'CF-Drone.ino')):
        raise SystemExit('CF-Drone sources missing: run  git submodule update --init --recursive')
    if os.path.isdir(OUT):
        shutil.rmtree(OUT)
    os.makedirs(OUT)
    for name in os.listdir(CF):
        src = os.path.join(CF, name)
        if name == 'drivers':
            shutil.copytree(src, os.path.join(OUT, 'drivers'))
        elif name.endswith(('.ino', '.h', '.cpp')):
            shutil.copy2(src, OUT)
    for fname, decl in PROTOTYPES.items():
        path = os.path.join(OUT, fname)
        with open(path, encoding='utf-8') as f:
            src = f.read()
        with open(path, 'w', encoding='utf-8') as f:
            f.write(decl + src)
    if not ZENOH:
        print(f'[e1] stock CF-Drone sources assembled in {OUT}')
        return
    for name in os.listdir(E1):
        shutil.copy2(os.path.join(E1, name), OUT)
    for fname, patches in PATCHES.items():
        path = os.path.join(OUT, fname)
        with open(path, encoding='utf-8') as f:
            s = f.read()
        lines = s.split('\n')
        for anchor, repl in patches:
            hits = [i for i, ln in enumerate(lines) if ln.startswith(anchor)]
            if len(hits) != 1:
                raise SystemExit(f'prepare_src: anchor {anchor!r} found {len(hits)}x in {fname} (CF-Drone changed?)')
            extra = repl.split('\n', 1)[1]          # the hook line(s) to insert after the anchor line
            lines.insert(hits[0] + 1, extra)
        s = '\n'.join(lines)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(s)
    print(f'[e1] CF-Drone + E1 Zenoh link sources assembled in {OUT}')


main()
