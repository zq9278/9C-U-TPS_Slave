'use strict';
const fs=require('node:fs'),path=require('node:path'),os=require('node:os');
const {Programmer,sha,validateOptions}=require('./backend.cjs');
function inspectPower(folder){
  const m=JSON.parse(fs.readFileSync(path.join(folder,'release.json'),'utf8'));
  if(m.schema!==1||m.board!=='powermanage'||m.device!=='STM32F030F4'||m.flash_address!=='0x08000000'||m.flash_size!==16384||m.ram_size!==4096||typeof m.build_id!=='string')throw Error('电源板固件目标或容量不匹配');
  for(const name of ['power_app.elf','power_app.bin','power_flash.bin']){
    const meta=m.files?.[name],file=path.join(folder,name);
    if(!meta||fs.statSync(file).size!==meta.size||sha(file)!==meta.sha256)throw Error('电源板固件哈希不匹配：'+name);
  }
  const app=fs.readFileSync(path.join(folder,'power_app.bin')),full=fs.readFileSync(path.join(folder,'power_flash.bin'));
  if(app.length<8||app.length>16384||full.length!==16384||!full.subarray(0,app.length).equals(app)||full.subarray(app.length).some(b=>b!==255))throw Error('电源板镜像长度或填充错误');
  const sp=app.readUInt32LE(0),pc=app.readUInt32LE(4);
  if(sp<=0x20000000||sp>0x20001000||sp%8||!(pc&1)||(pc&0xFFFFFFFE)<0x08000000||(pc&0xFFFFFFFE)>=0x08000000+app.length)throw Error('电源板向量表错误');
  return m;
}
function powerScript(verify=false){return ['ExitOnError 1','exec DisableAutoUpdateFW','connect','r','h',...(verify?[]:['loadfile "firmware.bin" 0x08000000']),
  'verifybin "firmware.bin" 0x08000000','savebin "readback.bin" 0x08000000 0x4000','r','g','q',''].join('\n');}
class PowerProgrammer extends Programmer {
  folder(){return this.payload?path.join(this.payload,'firmware-powermanage'):path.join(this.root,'out/PowerManage');}
  inspect(){return inspectPower(this.folder());}
  async flash(options,log=()=>{},progress=()=>{},verifyOnly=false){
    validateOptions(options);if(this.busy)throw Error('电源板操作进行中');this.busy=true;let folder;
    try{
      this.verifyRuntime();const m=this.inspect(),exe=this.runtime(options);if(!exe||!fs.existsSync(exe))throw Error('未找到 JLink.exe');
      folder=fs.mkdtempSync(path.join(os.tmpdir(),'tps-power-'));fs.copyFileSync(path.join(this.folder(),'power_flash.bin'),path.join(folder,'firmware.bin'));
      if(sha(path.join(folder,'firmware.bin'))!==m.files['power_flash.bin'].sha256)throw Error('电源板固件在校验后发生变化');
      fs.writeFileSync(path.join(folder,'program.jlink'),powerScript(verifyOnly),'ascii');progress(1);log(`${verifyOnly?'校验':'烧录'} PowerManage / STM32F030F4 / ${m.build_id}\n`);
      const args=['-NoGui','1','-ExitOnError','1','-AutoConnect','0','-Device','STM32F030F4','-If','SWD','-Speed',String(options.speed),...(options.serial?['-USB',options.serial]:[]),'-CommandFile',path.join(folder,'program.jlink')];
      const output=await this.runner(exe,args,{cwd:folder,log,timeout:120000});progress(2);const readback=path.join(folder,'readback.bin');
      if(/(?:^|\n)\s*(?:ERROR\s*[:!]|Cannot connect|Failed to)/i.test(output)||!fs.existsSync(readback)||fs.statSync(readback).size!==16384||sha(readback)!==m.files['power_flash.bin'].sha256)throw Error('电源板完整16KiB回读不匹配，不能确认成功');
      progress(3);return {board:'powermanage',build_id:m.build_id,sha256:m.files['power_flash.bin'].sha256,verified:true,verifyOnly};
    }finally{this.busy=false;if(folder){for(const name of ['program.jlink','firmware.bin','readback.bin'])fs.rmSync(path.join(folder,name),{force:true});try{fs.rmdirSync(folder);}catch{}}}
  }
}
module.exports={PowerProgrammer,inspectPower,powerScript};
