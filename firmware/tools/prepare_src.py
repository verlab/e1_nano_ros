"""PlatformIO pre-build script: assemble CF-Drone + the E1 Zenoh link into .e1_src/.

CF-Drone is used unmodified as a git submodule (external/CF-Drone). This script copies it,
adds src_e1/ and inserts four one-line hooks. Every anchor is checked, so a CF-Drone update
that moves an anchor fails loudly instead of producing a silently broken firmware.
"""
import os
import shutil

try:
    Import('env')  # noqa: F821  (PlatformIO / SCons)
    PROJ = env.subst('$PROJECT_DIR')  # noqa: F821
except NameError:   # allow running stand-alone: python3 tools/prepare_src.py
    PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CF = os.path.join(PROJ, 'external', 'CF-Drone')
OUT = os.path.join(PROJ, '.e1_src')
E1 = os.path.join(PROJ, 'src_e1')

PATCHES = {
    'CF-Drone.ino': [
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
    print(f'[e1] firmware sources assembled in {OUT}')


main()
