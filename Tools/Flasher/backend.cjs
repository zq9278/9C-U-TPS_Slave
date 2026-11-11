'use strict';
const fs=require('node:fs'), path=require('node:path'), os=require('node:os');
const crypto=require('node:crypto');
const {spawn}=require('./processes.cjs');
const ROOT=path.resolve(__dirname,'../..');
const sha=file=>crypto.createHash('sha256').update(fs.readFileSync(file)).digest('hex');
function crc32(bytes) {
  let c=0xFFFFFFFF;
  for(const b of bytes) {c^=b;for(let i=0;i<8;i++) c=(c>>>1)^((c&1)?0xEDB88320:0);}
  return (c^0xFFFFFFFF)>>>0;
}
function safePath(root,relative) {
  const file=path.resolve(root,relative);
  if(typeof relative!=='string'||!file.startsWith(path.resolve(root)+path.sep)) throw Error('无效的软件包路径');
  return file;
}
function validateOptions(o) {
  if(!o||typeof o.serial!=='string'||!/^\d{0,12}$/.test(o.serial)||!['1000','4000','8000'].includes(String(o.speed))) throw Error('探针设置无效');
}
function inspect(folder) {
  const m=JSON.parse(fs.readFileSync(path.join(folder,'release.json'),'utf8'));
  if(m.schema!==1 || m.device!=='STM32G070RB' || m.factory_address!=='0x08000000' || m.app_address!=='0x08004000') throw Error('固件目标或分区错误');
  for(const name of ['tps_boot.bin','tps_boot.elf','tps_app.bin','tps_app.elf','tps_app.tpsfw','tps_factory.bin']) {
    const f=path.join(folder,name), info=m.files?.[name];
    if(!info || fs.statSync(f).size!==info.size || sha(f)!==info.sha256) throw Error('固件校验失败：'+name);
  }
  const pkg=fs.readFileSync(path.join(folder,'tps_app.tpsfw')), h=pkg.subarray(0,64), image=pkg.subarray(64);
  if(pkg.length<72 || h.subarray(0,8).toString()!=='TPSFW001' || h.readUInt16LE(8)!==64 || h.readUInt16LE(10)!==1 ||
     h.readUInt32LE(12)!==0x9C01 || h.readUInt32LE(20)!==0x08004000 || image.length>110592 ||
     h.readUInt32LE(24)!==image.length || h.readUInt32LE(28)!==crc32(image) || h.readUInt32LE(60)!==crc32(h.subarray(0,60)) ||
     h.subarray(40,60).some(b=>b!==0)) throw Error('OTA 容器格式或 CRC 错误');
  const identity=m.identity;
  if(!identity || identity.target!==0x9C01 || identity.version_code!==h.readUInt32LE(32) ||
     identity.min_boot_version!==h.readUInt32LE(36) || h.readUInt16LE(16)>identity.hardware || h.readUInt16LE(18)<identity.hardware) throw Error('版本或硬件身份不匹配');
  const id=image.indexOf(Buffer.from('TPSID001'));
  if(id<0 || id+64>image.length || image.indexOf(Buffer.from('TPSID001'),id+1)!==-1 ||
     image.readUInt32LE(id+8)!==identity.version_code || image.readUInt16LE(id+12)!==identity.hardware ||
     image.readUInt32LE(id+16)!==0x9C01 || image.readUInt32LE(id+20)!==0x08004000 ||
     image.readUInt32LE(id+24)!==identity.min_boot_version || image.readUInt32LE(id+28)!==1 ||
     image.subarray(id+32,id+64).toString().split('\0')[0]!==identity.version || !identity.rtt) throw Error('编译身份与交付清单不一致或未启用 RTT');
  const sp=image.readUInt32LE(0), pc=image.readUInt32LE(4);
  if(sp<=0x20000000 || sp>0x20008FE0 || sp%8 || !(pc&1) || (pc&0xFFFFFFFE)<0x08004000 || (pc&0xFFFFFFFE)>=0x08004000+image.length) throw Error('App 向量表错误');
  if(!image.equals(fs.readFileSync(path.join(folder,'tps_app.bin')))) throw Error('App 与 OTA 容器内容不同');
  const factory=fs.readFileSync(path.join(folder,'tps_factory.bin')), boot=fs.readFileSync(path.join(folder,'tps_boot.bin'));
  if(factory.length!==131072 || boot.length>16384 || !factory.subarray(0,boot.length).equals(boot) ||
     !factory.subarray(0x4000,0x4000+image.length).equals(image)) throw Error('完整烧录镜像与 Boot/App 不匹配');
  const expected=Buffer.alloc(131072,255); boot.copy(expected);image.copy(expected,0x4000);
  const record=Buffer.alloc(64);record.write('TPSM');record.writeUInt16LE(1,4);record.writeUInt16LE(64,6);record.writeUInt32LE(1,8);
  h.copy(record,12,12,16);h.copy(record,16,20,36);h.copy(record,32,16,20);record.writeUInt32LE(crc32(record.subarray(0,52)),52);record.write('TPSVALID',56);
  record.copy(expected,0x1F000);record.copy(expected,0x1F800);
  if(!expected.equals(factory)) throw Error('完整镜像元数据或填充区不匹配');
  return m;
}
function findJLink() {
  const root=path.join(process.env.ProgramFiles||'C:/Program Files','SEGGER');
  const candidates=[process.env.JLINK_COMMANDER];
  if(fs.existsSync(root)) for(const folder of fs.readdirSync(root).sort().reverse()) candidates.push(path.join(root,folder,'JLink.exe'));
  return candidates.find(p=>p&&fs.existsSync(p))||'';
}
function run(command,args,{cwd=ROOT,log=()=>{},timeout=120000}={}) {
  return new Promise((resolve,reject)=>{
    const child=spawn(command,args,{cwd,windowsHide:true,shell:false,stdio:['ignore','pipe','pipe']});
    let output='',expired=false;
    const timer=setTimeout(()=>{expired=true;child.kill();},timeout);
    const read=bytes=>{const t=bytes.toString();output=(output+t).slice(-300000);log(t);};
    child.stdout.on('data',read);child.stderr.on('data',read);
    child.once('error',e=>{clearTimeout(timer);reject(e);});
    child.once('close',code=>{clearTimeout(timer);expired?reject(Error('操作超时，设备状态未确认')):
      code!==0?reject(Error(`工具退出码 ${code}\n${output.slice(-1200)}`)):resolve(output);});
  });
}
function script(verifyOnly=false) {
  return ['ExitOnError 1','exec DisableAutoUpdateFW','connect','r','h',
    ...(verifyOnly?[]:['loadfile "firmware.bin" 0x08000000']),
    'verifybin "firmware.bin" 0x08000000','savebin "readback.bin" 0x08000000 0x20000','r','g','q',''].join('\n');
}
class Programmer {
  constructor({root=ROOT,payload=null,runner=run}={}) {this.root=root;this.payload=payload;this.runner=runner;this.busy=false;}
  folder() {return this.payload?path.join(this.payload,'firmware'):path.join(this.root,'out/Release');}
  inspect() {return inspect(this.folder());}
  runtime(options={}) {return this.payload?path.join(this.payload,'jlink/JLink.exe'):options.jlink||findJLink();}
  verifyRuntime() {
    if(!this.payload) return;
    const bundle=JSON.parse(fs.readFileSync(path.join(this.payload,'bundle.json'),'utf8'));
    if(!bundle.runtime?.['jlink/JLink.exe'] || !bundle.runtime?.['tps-rtt-bridge.exe']) throw Error('软件包运行时清单缺失');
    for(const [file,hash] of Object.entries(bundle.runtime)) if(sha(safePath(this.payload,file))!==hash) throw Error('软件包文件损坏：'+file);
  }
  async build(log) {
    if(this.payload) throw Error('分发版使用内置固件；开发版才可编译');
    if(this.busy) throw Error('操作进行中');
    this.busy=true;
    try {await this.runner(process.env.PYTHON||'python',['scripts/build_release.py'],{cwd:this.root,log,timeout:900000});return this.inspect();}
    finally {this.busy=false;}
  }
  async flash(options,log=()=>{},progress=()=>{},verifyOnly=false) {
    validateOptions(options);
    if(this.busy) throw Error('操作进行中');this.busy=true;
    let folder;
    try {
      this.verifyRuntime();const m=this.inspect();const exe=this.runtime(options);
      if(!exe||!fs.existsSync(exe)) throw Error('请选择已安装的 JLink.exe');
      folder=fs.mkdtempSync(path.join(os.tmpdir(),'tps-program-'));
      fs.copyFileSync(path.join(this.folder(),'tps_factory.bin'),path.join(folder,'firmware.bin'));
      if(sha(path.join(folder,'firmware.bin'))!==m.files['tps_factory.bin'].sha256) throw Error('固件在检查后发生变化');
      fs.writeFileSync(path.join(folder,'program.jlink'),script(verifyOnly),'ascii');
      progress(1);log(`${verifyOnly?'校验':'烧录'} STM32G070RB / ${m.identity.version}\nSHA256 ${m.files['tps_factory.bin'].sha256}\n`);
      const args=['-NoGui','1','-ExitOnError','1','-AutoConnect','0','-Device','STM32G070RB','-If','SWD','-Speed',String(options.speed),
                  ...(options.serial?['-USB',options.serial]:[]),'-CommandFile',path.join(folder,'program.jlink')];
      const output=await this.runner(exe,args,{cwd:folder,log,timeout:120000});
      progress(2);
      const readback=path.join(folder,'readback.bin');
      if(/(?:^|\n)\s*(?:ERROR\s*[:!]|Cannot connect|Failed to)/i.test(output)||!fs.existsSync(readback)||fs.statSync(readback).size!==131072||sha(readback)!==m.files['tps_factory.bin'].sha256)
        throw Error('FLASH 回读 SHA256 不匹配或未收到完整回读，不能确认成功');
      progress(3);return {version:m.identity.version,sha256:m.files['tps_factory.bin'].sha256,verified:true,verifyOnly};
    } finally {
      this.busy=false;
      if(folder) {for(const name of ['program.jlink','firmware.bin','readback.bin']) fs.rmSync(path.join(folder,name),{force:true});try{fs.rmdirSync(folder);}catch{}}
    }
  }
}
module.exports={ROOT,Programmer,inspect,crc32,sha,safePath,script,validateOptions,findJLink,run};
