'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path'),crypto=require('node:crypto');
const {nextName,payloadInfo,artifact,Publisher,DEVICE}=require('./publisher.cjs');
const {crc32}=require('./backend.cjs');
function fixture(){const image=Buffer.alloc(32);image.writeUInt32LE(0x20008FE0);image.writeUInt32LE(0x08004009,4);
  const h=Buffer.alloc(64);h.write('TPSFW001');h.writeUInt16LE(64,8);h.writeUInt16LE(1,10);h.writeUInt32LE(0x9C01,12);h.writeUInt16LE(1,16);h.writeUInt16LE(1,18);h.writeUInt32LE(0x08004000,20);h.writeUInt32LE(32,24);h.writeUInt32LE(crc32(image),28);h.writeUInt32LE(0x10100,32);h.writeUInt32LE(0x10000,36);h.writeUInt32LE(crc32(h.subarray(0,60)),60);return Buffer.concat([h,image]);}
function signing(dir){const keys=crypto.generateKeyPairSync('ec',{namedCurve:'prime256v1'});
  const privateKey=path.join(dir,'key'),publicKey=path.join(dir,'pub');
  fs.writeFileSync(privateKey,keys.privateKey.export({type:'pkcs8',format:'pem'}));fs.writeFileSync(publicKey,keys.publicKey.export({type:'spki',format:'pem'}));return {privateKey,publicKey};}
test('names increment patch from requested 1.0.2 floor, not list order or another product',()=>{
  assert.equal(nextName([]),'tps-mcu-1.0.2-20261111');
  assert.equal(nextName([{name:'tps-mcu-1.0.9-20261112'},{name:'other-1.0.99-20261111'},{name:'tps-mcu-1.0.1-20261111'}]),'tps-mcu-1.0.10-20261111');
});
test('payload version remains actual MCU version and corruption fails',()=>{
  const d=fixture();assert.equal(payloadInfo(d).version,'0.1.1.0');d[70]^=1;assert.throws(()=>payloadInfo(d),/CRC/);
});
test('unsigned output is forbidden; signed artifact uses independently supplied key',t=>{
  const d=fixture();assert.throws(()=>artifact(d,'tps-mcu-1.0.2-20261111'),/签名/);
  assert.throws(()=>artifact(d,'tps-mcu-1.0.2-20261111',{allowUnsigned:true}),/签名/);
  const dir=fs.mkdtempSync(path.join(os.tmpdir(),'tps-sign-'));t.after(()=>fs.rmSync(dir,{recursive:true,force:true}));
  const result=artifact(d,'tps-mcu-1.0.2-20261111',signing(dir));assert.equal(result.signed,true);assert.ok(result.bytes.includes(Buffer.from('manifest.sig')));
  const wrong=crypto.generateKeyPairSync('ec',{namedCurve:'prime256v1'});fs.writeFileSync(path.join(dir,'pub'),wrong.publicKey.export({type:'spki',format:'pem'}));
  assert.throws(()=>artifact(d,'tps-mcu-1.0.2-20261111',{privateKey:path.join(dir,'key'),publicKey:path.join(dir,'pub')}),/公钥/);
});
test('upload reconciles lost POST response by hash, never deploys or retries POST',async t=>{
  const dir=fs.mkdtempSync(path.join(os.tmpdir(),'tps-pub-'));t.after(()=>fs.rmSync(dir,{recursive:true,force:true}));
  const file=path.join(dir,'app.tpsfw');fs.writeFileSync(file,fixture());const info=payloadInfo(fixture());
  let uploaded=false,posts=0;
  const transport=async(base,route,opts)=>{
    assert.ok(!route.includes('/deployments/deployments'));
    if(opts.method==='POST'){posts++;uploaded=true;throw Error('response lost');}
    return {data:uploaded?[{id:'new-id',name:'tps-mcu-1.0.2-20261111',signed:true,artifact_provides:{'eyemeter.tps-mcu.version':info.version},device_types_compatible:['eyemeter-thermal-aarch64'],updates:[{type_info:{type:'eyemeter-tps-mcu'},files:[{name:'firmware.tpsfw',checksum:info.sha256,size:info.size}]}]}]:[]};
  };
  const p=new Publisher({settingsFile:path.join(dir,'settings'),records:path.join(dir,'records'),transport});fs.writeFileSync(p.settingsFile,JSON.stringify(signing(dir)));
  const result=await p.upload({data:fixture(),info,name:'tps-mcu-1.0.2-20261111',connection:{server:'test',token:'test'}});
  assert.equal(result.state,'VERIFIED');assert.equal(result.artifact_id,'new-id');assert.equal(result.deployment_created,false);assert.equal(posts,1);assert.equal(p.busy,false);
});
test('server version race rejects before upload',async()=>{
  const p=new Publisher({transport:async()=>({data:[{id:'existing',name:'tps-mcu-1.0.2-20261111'}]})});
  await assert.rejects(p.upload({name:'tps-mcu-1.0.2-20261111',connection:{server:'test',token:'test'},data:fixture()}),/新版本/);assert.equal(p.busy,false);
});
