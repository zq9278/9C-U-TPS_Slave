"""Bundle the verified TPS release and Windows desktop flasher; no probe access."""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile
from build_release import ROOT, sha, source_info

APP = ROOT / 'Tools/Flasher'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--electron', type=Path, default=APP / 'node_modules/electron/dist')
    parser.add_argument('--jlink', type=Path, default=Path('C:/Program Files/SEGGER/JLink_V954'))
    parser.add_argument('--skip-build', action='store_true')
    parser.add_argument('--hw-rev', type=int, default=1)
    args = parser.parse_args()
    if not (args.electron / 'electron.exe').is_file():
        parser.error('Electron runtime missing: run npm ci in Tools/Flasher or pass --electron <dist>')
    for name in ('JLink.exe','JLinkARM.dll','JLink_x64.dll','Doc/LicenseIncGUI.txt'):
        if not (args.jlink / name).is_file():
            parser.error('Missing J-Link file: ' + name)
    if not args.skip_build:
        subprocess.run([sys.executable,str(ROOT/'scripts/build_release.py'),'--hw-rev',str(args.hw_rev)],cwd=ROOT,check=True)
    manifest = json.loads((ROOT/'out/Release/release.json').read_text(encoding='utf-8'))
    snapshot = source_info()
    if manifest['source']['snapshot_sha256'] != snapshot['snapshot_sha256']:
        parser.error('Source changed since firmware build; rebuild instead of packaging stale firmware')
    for name, info in manifest['files'].items():
        if sha(ROOT/'out/Release'/name) != info['sha256']:
            parser.error('Firmware hash mismatch: ' + name)
    validation=json.loads((ROOT/'out/validation.json').read_text(encoding='utf-8'))
    if not validation['software_pass'] or validation['source']['snapshot_sha256']!=snapshot['snapshot_sha256']:
        parser.error('Run scripts/test.py against the current source before packaging')
    # Freeze a bridge executable so the user does not need Python or pylink.
    subprocess.run([sys.executable,'-m','PyInstaller','--noconfirm','--clean','--onefile',
                    '--name','tps-rtt-bridge','--distpath',str(ROOT/'.cache/rtt-dist'),
                    '--workpath',str(ROOT/'.cache/rtt-build'),'--specpath',str(ROOT/'.cache'),
                    '--collect-all','pylink',str(APP/'rtt_bridge.py')],cwd=ROOT,check=True)
    package = json.loads((APP/'package.json').read_text(encoding='utf-8'))
    stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d-%H%M%S')
    dest=ROOT/'out/distribution'/('TPS130-Studio-'+package['version']+'-'+stamp)
    if dest.exists():
        parser.error('Output directory already exists')
    shutil.copytree(args.electron,dest)
    executable=dest/'TPS130-Studio.exe'
    (dest/'electron.exe').rename(executable)
    editor=APP/'node_modules/rcedit/bin/rcedit-x64.exe'
    if not editor.is_file():
        parser.error('Missing executable icon editor; run npm ci in Tools/Flasher')
    subprocess.run([str(editor),str(executable),'--set-icon',str(APP/'assets/tps130.ico'),
                    '--set-version-string','ProductName','TPS130 Studio',
                    '--set-version-string','FileDescription','TPS130 Device Debug Studio',
                    '--set-file-version',package['version'],'--set-product-version',package['version']],check=True)
    (dest/'resources/default_app.asar').unlink(missing_ok=True)
    application=dest/'resources/app';application.mkdir()
    for name in ('main.cjs','backend.cjs','publisher.cjs','processes.cjs','rtt.cjs','preload.cjs','index.html','renderer.js','style.css'):
        shutil.copy2(APP/name,application/name)
    for name in ('assets','devices','platform'):
        shutil.copytree(APP/name,application/name)
    (application/'package.json').write_text(json.dumps({k:package[k] for k in ('name','version','main')})+'\n',encoding='utf-8')
    payload=dest/'resources/payload'
    firmware=payload/'firmware';firmware.mkdir(parents=True)
    for name in list(manifest['files'])+['release.json','SHA256SUMS.txt']:
        shutil.copy2(ROOT/'out/Release'/name,firmware/name)
    shutil.copy2(ROOT/'out/validation.json',payload/'software-validation.json')
    shutil.copy2(ROOT/'.cache/rtt-dist/tps-rtt-bridge.exe',payload/'tps-rtt-bridge.exe')
    runtime=payload/'jlink'
    shutil.copytree(args.jlink,runtime,ignore=shutil.ignore_patterns('Uninstall.exe','*.log'))
    # STM32G070RB is built into the DLL; remove unrelated developer loader paths.
    (runtime/'JLinkDevices.xml').write_text('<DataBase />\n',encoding='utf-8')
    inventory={f.relative_to(payload).as_posix():sha(f) for f in sorted(runtime.rglob('*')) if f.is_file()}
    inventory['tps-rtt-bridge.exe']=sha(payload/'tps-rtt-bridge.exe')
    (payload/'bundle.json').write_text(json.dumps({'schema':1,'version':package['version'],'built_at':stamp,'runtime':inventory},indent=2)+'\n',encoding='utf-8')
    shutil.copy2(APP/'README.md',dest/'使用说明.txt')
    shutil.copy2(args.jlink/'Doc/LicenseIncGUI.txt',dest/'SEGGER-LICENSE.txt')
    docs=dest/'交接文档';docs.mkdir()
    for name in ('ota_requirements_acceptance.md','uart_upgrade_handoff.md','rtt_debug.md','产品OTA档案.md','debug_platform_architecture.md'):
        shutil.copy2(ROOT/'documents'/name,docs/name)
    shutil.copy2(ROOT/'Tools/FirmwarePack/protocol_vectors.json',docs/'protocol_vectors.json')
    for name,info in manifest['files'].items():
        if sha(firmware/name)!=info['sha256']:
            raise RuntimeError('Copied firmware hash mismatch')
    archive=dest.parent/(dest.name+'.zip')
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as output:
        for file in sorted(dest.rglob('*')):
            if file.is_file():
                output.write(file,file.relative_to(dest.parent))
    archive.with_suffix('.zip.sha256').write_text(sha(archive)+'  '+archive.name+'\n',encoding='ascii')
    (ROOT/'out/distribution/LATEST.txt').write_text(str(dest)+'\n',encoding='utf-8')
    print('APPLICATION:',executable,flush=True)
    print('ARCHIVE:',archive,flush=True)


if __name__ == '__main__':
    main()
