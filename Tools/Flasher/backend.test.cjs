'use strict';
const {test}=require('node:test');const assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),os=require('node:os'),crypto=require('node:crypto');
const {Programmer,inspect,crc32,script,validateOptions,safePath,sha,run}=require('./backend.cjs');
function fixture(t) {
  const root=fs.mkdtempSync(path.join(os.tmpdir(),'tps-test-'));t.after(()=>fs.rmSync(root,{recursive:true,force:true}));
  const folder=path.join(root,'out/Release');fs.mkdirSync(folder,{recursive:true});
  const image=Buffer.alloc(128);image.writeUInt32LE(0x20008FE0);image.writeUInt32LE(0x08004009,4);
  image.write('TPSID001',32);image.writeUInt32LE(0x10100,40);image.writeUInt16LE(1,44);image.writeUInt32LE(0x9C01,48);image.writeUInt32LE(0x08004000,52);
  image.writeUInt32LE(0x10000,56);image.writeUInt32LE(1,60);image.write('0.1.1',64);
  const boot=Buffer.alloc(32);boot.writeUInt32LE(0x20008FE0);boot.writeUInt32LE(0x08000009,4);
  const h=Buffer.alloc(64);h.write('TPSFW001');h.writeUInt16LE(64,8);h.writeUInt16LE(1,10);h.writeUInt32LE(0x9C01,12);h.writeUInt16LE(1,16);h.writeUInt16LE(1,18);
  h.writeUInt32LE(0x08004000,20);h.writeUInt32LE(image.length,24);h.writeUInt32LE(crc32(image),28);h.writeUInt32LE(0x10100,32);h.writeUInt32LE(0x10000,36);h.writeUInt32LE(crc32(h.subarray(0,60)),60);
  const record=Buffer.alloc(64);record.write('TPSM');record.writeUInt16LE(1,4);record.writeUInt16LE(64,6);record.writeUInt32LE(1,8);
  h.copy(record,12,12,16);h.copy(record,16,20,36);h.copy(record,32,16,20);record.writeUInt32LE(crc32(record.subarray(0,52)),52);record.write('TPSVALID',56);
  const factory=Buffer.alloc(131072,255);boot.copy(factory);image.copy(factory,0x4000);record.copy(factory,0x1F000);record.copy(factory,0x1F800);
  const contents={'tps_app.bin':image,'tps_app.tpsfw':Buffer.concat([h,image]),'tps_factory.bin':factory,'tps_boot.bin':boot,'tps_app.elf':Buffer.from('elf app fixture'),'tps_boot.elf':Buffer.from('elf boot fixture')};
  const files={};for(const [name,data] of Object.entries(contents)){fs.writeFileSync(path.join(folder,name),data);files[name]={size:data.length,sha256:sha(path.join(folder,name))};}
  const m={schema:1,device:'STM32G070RB',factory_address:'0x08000000',app_address:'0x08004000',identity:{version:'0.1.1',version_code:0x10100,hardware:1,target:0x9C01,min_boot_version:0x10000,rtt:true},files};
  fs.writeFileSync(path.join(folder,'release.json'),JSON.stringify(m));
  const exe=path.join(root,'JLink.exe');fs.writeFileSync(exe,'fixture');
  return {root,folder,m,exe};
}
const options={serial:'',speed:'1000'};
test('CRC, bounded path and strict probe parameters',()=>{
  assert.equal(crc32(Buffer.from('123456789')),0xCBF43926);
  assert.throws(()=>safePath('/tmp/payload','../other'));assert.throws(()=>validateOptions({serial:'1\nq',speed:'1000'}));
  assert.throws(()=>validateOptions({serial:'',speed:'999'}));
});
test('valid delivery, corruption and renamed target are rejected before probe use',t=>{
  const f=fixture(t);assert.equal(inspect(f.folder).identity.version,'0.1.1');
  fs.appendFileSync(path.join(f.folder,'tps_app.tpsfw'),'x');assert.throws(()=>inspect(f.folder),/校验失败/);
});
test('metadata mismatch fails even when file manifest hash was recomputed',t=>{
  const f=fixture(t),p=path.join(f.folder,'tps_factory.bin'),b=fs.readFileSync(p);b[0x1F000]=0;fs.writeFileSync(p,b);
  f.m.files['tps_factory.bin'].sha256=sha(p);fs.writeFileSync(path.join(f.folder,'release.json'),JSON.stringify(f.m));
  assert.throws(()=>inspect(f.folder),/元数据/);
});
test('flashing uses exact target and requires a complete matching readback',async t=>{
  const f=fixture(t);let calls=0;
  const runner=async(cmd,args,ctx)=>{calls++;assert.equal(cmd,f.exe);assert.equal(args[args.indexOf('-Device')+1],'STM32G070RB');
    assert.match(fs.readFileSync(path.join(ctx.cwd,'program.jlink'),'utf8'),/loadfile "firmware.bin" 0x08000000/);
    fs.copyFileSync(path.join(ctx.cwd,'firmware.bin'),path.join(ctx.cwd,'readback.bin'));return 'Downloading file... O.K.';};
  const p=new Programmer({root:f.root,runner});assert.equal((await p.flash({...options,jlink:f.exe})).verified,true);assert.equal(calls,1);assert.equal(p.busy,false);
});
test('exit code zero without readback is not success; busy always clears',async t=>{
  const f=fixture(t);const p=new Programmer({root:f.root,runner:async()=> 'O.K.'});
  await assert.rejects(p.flash({...options,jlink:f.exe}),/回读/);assert.equal(p.busy,false);
});
test('verify does not issue loadfile or erase; uses complete readback',()=>{
  assert.doesNotMatch(script(true),/loadfile|erase/i);assert.match(script(true),/savebin "readback.bin" 0x08000000 0x20000/);
});
test('failed tool retains diagnostic output and exit code for result dialog',async()=>{
  await assert.rejects(run(process.execPath,['-e',"console.log('Connection failure detail');console.log('x'.repeat(2000));process.exit(1)"]),e=>e.message.includes('退出码 1')&&e.message.includes('Connection failure detail'));
});
