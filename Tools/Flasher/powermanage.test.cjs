'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {PowerProgrammer,inspectPower,powerScript}=require('./powermanage.cjs'),{sha}=require('./backend.cjs');
function fixture(t){
  const root=fs.mkdtempSync(path.join(os.tmpdir(),'power-test-'));t.after(()=>fs.rmSync(root,{recursive:true,force:true}));const folder=path.join(root,'out/PowerManage');fs.mkdirSync(folder,{recursive:true});
  const app=Buffer.alloc(64);app.writeUInt32LE(0x20001000);app.writeUInt32LE(0x08000009,4);const full=Buffer.alloc(16384,255);app.copy(full);
  for(const [name,data] of Object.entries({'power_app.bin':app,'power_flash.bin':full,'power_app.elf':Buffer.from('fixture elf')}))fs.writeFileSync(path.join(folder,name),data);
  const files={};for(const name of ['power_app.bin','power_flash.bin','power_app.elf'])files[name]={size:fs.statSync(path.join(folder,name)).size,sha256:sha(path.join(folder,name))};
  const m={schema:1,board:'powermanage',device:'STM32F030F4',flash_address:'0x08000000',flash_size:16384,ram_size:4096,build_id:'test-build',files};fs.writeFileSync(path.join(folder,'release.json'),JSON.stringify(m));const exe=path.join(root,'JLink.exe');fs.writeFileSync(exe,'test');return {root,folder,m,exe};
}
test('power board rejects Slave target, corrupt image and out-of-RAM vector before invoking probe',t=>{
  const f=fixture(t);assert.equal(inspectPower(f.folder).device,'STM32F030F4');
  f.m.device='STM32G070RB';fs.writeFileSync(path.join(f.folder,'release.json'),JSON.stringify(f.m));assert.throws(()=>inspectPower(f.folder),/目标/);
  f.m.device='STM32F030F4';fs.writeFileSync(path.join(f.folder,'release.json'),JSON.stringify(f.m));fs.appendFileSync(path.join(f.folder,'power_app.bin'),'x');assert.throws(()=>inspectPower(f.folder),/哈希/);
  const g=fixture(t),app=fs.readFileSync(path.join(g.folder,'power_app.bin'));app.writeUInt32LE(0x20002000);fs.writeFileSync(path.join(g.folder,'power_app.bin'),app);const full=Buffer.alloc(16384,255);app.copy(full);fs.writeFileSync(path.join(g.folder,'power_flash.bin'),full);
  for(const name of ['power_app.bin','power_flash.bin'])g.m.files[name].sha256=sha(path.join(g.folder,name));fs.writeFileSync(path.join(g.folder,'release.json'),JSON.stringify(g.m));assert.throws(()=>inspectPower(g.folder),/向量/);
});
test('power board SWD uses F030 target and exact full16KiB readback',async t=>{
  const f=fixture(t);let calls=0;const p=new PowerProgrammer({root:f.root,runner:async(exe,args,ctx)=>{calls++;assert.equal(exe,f.exe);assert.equal(args[args.indexOf('-Device')+1],'STM32F030F4');assert.match(fs.readFileSync(path.join(ctx.cwd,'program.jlink'),'utf8'),/savebin "readback.bin" 0x08000000 0x4000/);fs.copyFileSync(path.join(ctx.cwd,'firmware.bin'),path.join(ctx.cwd,'readback.bin'));return 'O.K.';}});
  assert.equal((await p.flash({serial:'',speed:'1000',jlink:f.exe})).verified,true);assert.equal(calls,1);assert.equal(p.busy,false);
});
test('power board never reports success for missing readback and verify never writes flash',async t=>{
  const f=fixture(t),p=new PowerProgrammer({root:f.root,runner:async()=> 'O.K.'});await assert.rejects(p.flash({serial:'',speed:'1000',jlink:f.exe}),/回读/);assert.equal(p.busy,false);assert.doesNotMatch(powerScript(true),/loadfile|erase/i);
});
