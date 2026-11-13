"""Software-only acceptance suite. Does not open a J-Link or serial port."""
import argparse
from datetime import datetime, timezone, timedelta
import json
import os
from pathlib import Path
import subprocess
import sys
from build_release import ROOT, source_info


def main():
    sys.stdout.reconfigure(encoding='utf-8',errors='replace')
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--wsl-distro',default='Ubuntu-24.04')
    args=p.parse_args()
    directory=ROOT/'build/validation';directory.mkdir(parents=True,exist_ok=True)
    prefix=['wsl','-d',args.wsl_distro,'--'] if os.name=='nt' else []
    flags=['gcc','-std=c11','-Wall','-Wextra','-fsanitize=address,undefined','-g']
    tests=[
        [sys.executable,'-m','unittest','discover','-s','Tools/FirmwarePack','-v'],
        [sys.executable,'-m','unittest','discover','-s','Tools/Flasher','-p','test_rtt_bridge.py','-v'],
        ['node','--test','Tools/Flasher/backend.test.cjs','Tools/Flasher/powermanage.test.cjs','Tools/Flasher/publisher.test.cjs','Tools/Flasher/platform.test.cjs'],
        prefix+flags+['-Werror','-I.','Shared/Upgrade/upgrade.c','Bootloader/boot_engine.c','Tools/FirmwarePack/test_boot.c','-o','build/validation/tps_boot_test'],
        prefix+['./build/validation/tps_boot_test'],
        prefix+['python3','Tools/FirmwarePack/make_vectors.py','--check-binary','./build/validation/tps_boot_test'],
        prefix+['./build/validation/tps_boot_test','--factory','out/Release/tps_factory.bin'],
        prefix+flags+['-DTPS_RTT=1','-ITools/Flasher/tests/stubs','-I.','-Ibuild/delivery/Release/generated','-IThirdParty/SEGGER_RTT/RTT','-IThirdParty/SEGGER_RTT/Config',
                      'Modules/Debug/debug_rtt.c','ThirdParty/SEGGER_RTT/RTT/SEGGER_RTT.c','Tools/Flasher/tests/test_debug_rtt.c','-o','build/validation/tps_rtt_test'],
        prefix+['./build/validation/tps_rtt_test'],
    ]
    results=[]
    for cmd in tests:
        print('TEST:',subprocess.list2cmdline(cmd),flush=True)
        try:
            result=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace')
            output=result.stdout+result.stderr
            code=result.returncode
        except OSError as exc:
            output=str(exc);code=127
        print(output,flush=True)
        results.append({'command':cmd,'exit_code':code,'output':output})
        if code:
            break
    okay=len(results)==len(tests) and all(x['exit_code']==0 for x in results)
    report={'software_pass':okay,'hardware_status':'NOT_RUN','source':source_info(),
            'time':datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='seconds'),'results':results}
    out=ROOT/'out';out.mkdir(exist_ok=True)
    (out/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return 0 if okay else 1


if __name__=='__main__':sys.exit(main())
