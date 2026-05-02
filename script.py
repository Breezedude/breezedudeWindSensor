Import("env")
import shutil
import tools.uf2conv as uf2
from pathlib import Path

def _get_define(name, default=None):
    for define in env.get("CPPDEFINES", []):
        if isinstance(define, (list, tuple)) and len(define) >= 2 and define[0] == name:
            return define[1]
        if define == name:
            return "1"
    return default

def _sanitize_fragment(value, default="dev"):
    text = str(value).replace('\\"', '').replace('"', '').replace("'", '').strip()
    safe = ''.join(ch for ch in text if ch.isalnum() or ch in '._-')
    return safe or default

def _ota_bin_name():
    version = _sanitize_fragment(_get_define("VERSION", "dev"), "dev")
    return f"breezedude-ota-{version}.bin"


def _get_upload_offset():
    value = env.GetProjectOption("board_upload.offset_address", default="0x4000")
    try:
        return int(str(value), 0)
    except Exception:
        return 0x4000


def export_ota_bin(target, source, env):
    src = Path(target[0].get_abspath())
    out_dir = Path(env["PROJECT_DIR"]) / "build" / "ota"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_file = out_dir / _ota_bin_name()
    shutil.copy2(src, out_file)
    print(f"Exported OTA bin: {out_file}")


def make_uf2(target, source, env):
    print(target[0].get_abspath())

    with open(target[0].get_abspath(), mode='rb') as f:
        inpbuf = f.read()
        uf2.appstartaddr = _get_upload_offset()
        outbuf = uf2.convert_to_uf2(inpbuf)
        out = env['PROJECT_DIR'] + '\\build\\'
        Path(out).mkdir(parents=True, exist_ok=True)
        uf2.write_file(out + str(env["UNIX_TIME"]) + ".uf2", outbuf)
        print(f"Created app UF2 with base address 0x{uf2.appstartaddr:08X}")

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", export_ota_bin)
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", make_uf2)

