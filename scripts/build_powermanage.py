"""Build the sibling power-board project without modifying its source or connecting a probe."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess
from build_release import ROOT, sha

SOURCE=ROOT.parent/'9C-U-TPS_PowerManage'
OUT=ROOT/'out/PowerManage'

def source_info(source=SOURCE):
    def git(*args):return subprocess.check_output(['git','-C',str(source),*args])
    files={}
    for raw in sorted(set(git('ls-files','--cached','--others','--exclude-standard','-z').split(b'\0'))-{b''}):
        name=raw.decode('utf-8');p=source/name
        if p.is_file():files[name]=sha(p)
    return {'commit':git('rev-parse','HEAD').decode().strip(),'dirty':bool(git('status','--porcelain').strip()),
            'snapshot_sha256':hashlib.sha256(json.dumps(files,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'files':files}

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source',type=Path,default=SOURCE);args=parser.parse_args();source=args.source.resolve()
    if 'Mcu.Name=STM32F030F4Px' not in (source/'9C-U-TPS_PowerManage.ioc').read_text(encoding='utf-8'):parser.error('Unexpected power-board MCU')
    snapshot=source_info(source);build=ROOT/'build/powermanage/Release';OUT.mkdir(parents=True,exist_ok=True)
    for cmd in [['cmake','-S',str(source),'-B',str(build),'-G','Ninja','-DCMAKE_TOOLCHAIN_FILE='+str(source/'cmake/gcc-arm-none-eabi.cmake'),'-DCMAKE_BUILD_TYPE=Release'],
                ['cmake','--build',str(build),'--parallel','8']]:subprocess.run(cmd,check=True)
    elf=build/'9C-U-TPS_PowerManage.elf'
    if not elf.exists():elf=build/'9C-U-TPS_PowerManage'
    shutil.copy2(elf,OUT/'power_app.elf')
    subprocess.run(['arm-none-eabi-objcopy','-O','binary',str(elf),str(OUT/'power_app.bin')],check=True)
    data=(OUT/'power_app.bin').read_bytes()
    if not 8<=len(data)<=16384:parser.error('Power firmware exceeds 16KiB')
    sp,pc=struct.unpack_from('<II',data)
    if not (0x20000000<sp<=0x20001000 and sp%8==0 and pc&1 and 0x08000000<=pc-1<0x08000000+len(data)):parser.error('Invalid power-board vectors')
    if source_info(source)!=snapshot:parser.error('Power-board source changed while building')
    (OUT/'power_flash.bin').write_bytes(data+b'\xff'*(16384-len(data)))
    files={name:{'size':(OUT/name).stat().st_size,'sha256':sha(OUT/name)} for name in ['power_app.elf','power_app.bin','power_flash.bin']}
    manifest={'schema':1,'board':'powermanage','product':'9C-U-TPS PowerManage','device':'STM32F030F4','flash_address':'0x08000000','flash_size':16384,'ram_size':4096,
              'build_id':snapshot['commit'][:12]+'-'+snapshot['snapshot_sha256'][:8],'version':None,'version_note':'No firmware version interface is declared in this project; build ID identifies the source snapshot.',
              'source':snapshot,'built_at':datetime.now(timezone.utc).isoformat(),'files':files,'hardware_validation':'NOT_RUN'}
    (OUT/'release.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (OUT/'SHA256SUMS.txt').write_text(''.join(meta['sha256']+'  '+name+'\n' for name,meta in files.items()),encoding='ascii')
    print('POWER DELIVERY:',OUT,'image bytes',len(data),flush=True)

if __name__=='__main__':main()
