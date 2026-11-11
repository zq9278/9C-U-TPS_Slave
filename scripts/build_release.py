"""Reproducible Boot/App build and OTA/factory delivery. Never opens a probe."""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'Tools/FirmwarePack'))
import pack_firmware as pack


def run(args, **kwargs):
    return subprocess.run(args, cwd=ROOT, check=True, **kwargs)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_info():
    commit = run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
    dirty = bool(run(['git', 'status', '--porcelain'], capture_output=True, text=True).stdout.strip())
    files = run(['git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z'], capture_output=True).stdout
    inventory = {}
    for raw in sorted(set(files.split(b'\0')) - {b''}):
        relative = raw.decode('utf-8')
        path = ROOT / relative
        if path.is_file():
            inventory[relative] = sha(path)
    encoded = json.dumps(inventory, sort_keys=True, separators=(',', ':')).encode()
    return {'commit': commit, 'dirty': dirty, 'snapshot_sha256': hashlib.sha256(encoded).hexdigest(), 'files': inventory}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--preset', choices=('Release', 'Debug'), default='Release')
    p.add_argument('--hw-rev', type=int, default=1)
    args = p.parse_args()
    if not 0 <= args.hw_rev <= 65535:
        p.error('hardware revision must be 0..65535')
    toolchain = str(ROOT / 'cmake/gcc-arm-none-eabi.cmake')
    boot = ROOT / 'build/delivery/Boot'
    app = ROOT / ('build/delivery/' + args.preset)
    for directory, is_boot in ((boot, True), (app, False)):
        run(['cmake', '-S', str(ROOT), '-B', str(directory), '-G', 'Ninja',
             '-DCMAKE_TOOLCHAIN_FILE=' + toolchain, '-DCMAKE_BUILD_TYPE=' + ('Release' if is_boot else args.preset),
             '-DTPS_BUILD_BOOT=' + ('ON' if is_boot else 'OFF'), '-DTPS_LEGACY_APP=OFF',
             '-DTPS_RTT=ON', '-DTPS_HW_REV=' + str(args.hw_rev)])
        run(['cmake', '--build', str(directory), '--parallel', '8'])
    out = ROOT / 'out' / args.preset
    out.mkdir(parents=True, exist_ok=True)
    for original, name in ((boot / 'tps_boot.bin', 'tps_boot.bin'), (boot / 'tps_boot.elf', 'tps_boot.elf'),
                           (app / 'tps_app.bin', 'tps_app.bin'), (app / '9C-TPS100-Slave.elf', 'tps_app.elf')):
        shutil.copy2(original, out / name)
    run([sys.executable, str(ROOT / 'Tools/FirmwarePack/pack_firmware.py'), str(out / 'tps_app.bin'),
         '--output', str(out / 'tps_app.tpsfw'), '--boot', str(out / 'tps_boot.bin'),
         '--factory-output', str(out / 'tps_factory.bin')])
    identity = pack.inspect_identity((out / 'tps_app.bin').read_bytes())
    pack.validate_package((out / 'tps_app.tpsfw').read_bytes())
    compiler = run(['arm-none-eabi-gcc', '--version'], capture_output=True, text=True).stdout.splitlines()[0]
    files = {}
    for name in ('tps_boot.bin', 'tps_boot.elf', 'tps_app.bin', 'tps_app.elf', 'tps_app.tpsfw', 'tps_factory.bin'):
        files[name] = {'sha256': sha(out / name), 'size': (out / name).stat().st_size}
    manifest = {'schema': 1, 'product': '9C-U-TPS Slave', 'device': 'STM32G070RB', 'preset': args.preset,
                'built_at': datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='seconds'),
                'identity': identity, 'factory_address': '0x08000000', 'app_address': '0x08004000',
                'source': source_info(), 'toolchain': compiler, 'files': files,
                'hardware_validation': {'status': 'NOT_RUN', 'note': 'No probe connection, SWD flash or real power-loss test performed by the build.'}}
    (out / 'release.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    (out / 'SHA256SUMS.txt').write_text(''.join(v['sha256'] + '  ' + k + '\n' for k, v in files.items()), encoding='ascii')
    print('DELIVERY:', out, flush=True)


if __name__ == '__main__':
    main()
