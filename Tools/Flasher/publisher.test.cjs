'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path'),crypto=require('node:crypto');
const {nextName,payloadInfo,artifact,Publisher,DEVICE}=require('./publisher.cjs');
const {crc32}=require('./backend.cjs');
function fixture(){const image=Buffer.alloc(32);image.writeUInt32LE(0x20008FE0);image.writeUInt32LE(0x08004009,4);
  const h=Buffer.alloc(64);h.write('TPSFW001');h.writeUInt16LE(64,8);h.writeUInt16LE(1,10);h.writeUInt32LE(0x9C01,12);h.writeUInt16LE(1,16);h.writeUInt16LE(1,18);h.writeUInt32LE(0x08004000,20);h.writeUInt32LE(32,24);h.writeUInt32LE(crc32(image),28);h.writeUInt32LE(0x10100,32);h.writeUInt32LE(0x10000,36);h.writeUInt32LE(crc32(h.subarray(0,60)),60);return Buffer.concat([h,image]);}
function signing(dir){const keys=crypto.generateKeyPairSync('ec',{namedCurve:'prime256v1'});
  const privateKey=path.join(dir,'key'),publicKey=path.join(dir,'pub');
  fs.writeFileSync(privateKey,keys.privateKey.export({type:'pkcs8',format:'pem'}));fs.writeFileSync(publicKey,keys.publicKey.export({type:'spki',format:'pem'}));return {mode:'signed',privateKey,publicKey};}
test('names increment patch from requested 1.0.2 floor, not list order or another product',()=>{
  assert.equal(nextName([]),'tps-mcu-1.0.2-20261111');
  assert.equal(nextName([{name:'tps-mcu-1.0.9-20261112'},{name:'other-1.0.99-20261111'},{name:'tps-mcu-1.0.1-20261111'}]),'tps-mcu-1.0.10-20261111');
});
test('payload version remains actual MCU version and corruption fails',()=>{
  const d=fixture();assert.equal(payloadInfo(d).version,'0.1.1.0');d[70]^=1;assert.throws(()=>payloadInfo(d),/CRC/);
});
test('unsigned is the default; optional signed mode requires matching trusted key',t=>{
  const d=fixture();const unsigned=artifact(d,'tps-mcu-1.0.2-20261111');
  assert.equal(unsigned.signed,false);assert.equal(unsigned.bytes.includes(Buffer.from('manifest.sig')),false);
  assert.throws(()=>artifact(d,'tps-mcu-1.0.2-20261111',{mode:'signed'}),/签名/);
  const dir=fs.mkdtempSync(path.join(os.tmpdir(),'tps-sign-'));t.after(()=>fs.rmSync(dir,{recursive:true,force:true}));
  const result=artifact(d,'tps-mcu-1.0.2-20261111',signing(dir));assert.equal(result.signed,true);assert.ok(result.bytes.includes(Buffer.from('manifest.sig')));
  const wrong=crypto.generateKeyPairSync('ec',{namedCurve:'prime256v1'});fs.writeFileSync(path.join(dir,'pub'),wrong.publicKey.export({type:'spki',format:'pem'}));
  assert.throws(()=>artifact(d,'tps-mcu-1.0.2-20261111',{mode:'signed',privateKey:path.join(dir,'key'),publicKey:path.join(dir,'pub')}),/公钥/);
});
test('upload reconciles lost artifact POST, then creates only the exact device task',async t=>{
  const dir=fs.mkdtempSync(path.join(os.tmpdir(),'tps-pub-'));t.after(()=>fs.rmSync(dir,{recursive:true,force:true}));
  const file=path.join(dir,'app.tpsfw');fs.writeFileSync(file,fixture());const info=payloadInfo(fixture());
  let uploaded=false,posts=0,deployPosts=0;
  const transport=async(base,route,opts)=>{
    if(route.includes('/inventory/devices/'))return {data:{id:DEVICE,attributes:[{name:'device_type',value:'eyemeter-thermal-aarch64'}]}};
    if(route.includes('/deployments/deployments')){
      if(opts.method==='POST'){deployPosts++;assert.deepEqual(JSON.parse(opts.body).devices,[DEVICE]);return {headers:{location:'/deployments/task-1'},data:''};}
      if(route.endsWith('/task-1'))return {data:{id:'task-1',artifact_name:'tps-mcu-1.0.2-20261111',status:'pending',filter:{terms:[{scope:'identity',attribute:'id',type:'$in',value:[DEVICE]}]}}};
      return {data:[]};
    }
    if(opts.method==='POST'){posts++;uploaded=true;throw Error('response lost');}
    return {data:uploaded?[{id:'new-id',name:'tps-mcu-1.0.2-20261111',signed:true,artifact_provides:{'eyemeter.tps-mcu.version':info.version},device_types_compatible:['eyemeter-thermal-aarch64'],updates:[{type_info:{type:'eyemeter-tps-mcu'},files:[{name:'firmware.tpsfw',checksum:info.sha256,size:info.size}]}]}]:[]};
  };
  const p=new Publisher({settingsFile:path.join(dir,'settings'),records:path.join(dir,'records'),transport});fs.writeFileSync(p.settingsFile,JSON.stringify(signing(dir)));
  const result=await p.upload({data:fixture(),info,name:'tps-mcu-1.0.2-20261111',connection:{server:'test',token:'test'}});
  assert.equal(result.state,'VERIFIED');assert.equal(result.artifact_id,'new-id');assert.equal(result.deployment_created,true);assert.equal(result.deployment_id,'task-1');assert.equal(deployPosts,1);assert.equal(posts,1);assert.equal(p.busy,false);
});

test('deployment reuses an existing exact-target task and reconciles a lost creation response',async t=>{
  for(const existing of [true,false]){
    const dir=fs.mkdtempSync(path.join(os.tmpdir(),'tps-deploy-'));t.after(()=>fs.rmSync(dir,{recursive:true,force:true}));
    const info=payloadInfo(fixture()),name='tps-mcu-1.0.2-20261111',record={name,state:'VERIFIED',artifact_id:'artifact',device_reference:DEVICE,device_type:'eyemeter-thermal-aarch64',module:'eyemeter-tps-mcu',source_payload:info};
    let visible=existing,posts=0;
    const details={id:'task',artifact_name:name,status:'pending',filter:{terms:[{scope:'identity',attribute:'id',type:'$in',value:[DEVICE]}]}};
    const p=new Publisher({transport:async(base,route,opts)=>{
      if(route.includes('/inventory/'))return {data:{id:DEVICE,attributes:[{name:'device_type',value:record.device_type}]}};
      if(route.includes('/artifacts'))return {data:[{id:'artifact',name,device_types_compatible:[record.device_type],updates:[{type_info:{type:record.module},files:[{name:'firmware.tpsfw',checksum:info.sha256,size:info.size}]}]}]};
      if(opts.method==='POST'){posts++;visible=true;throw Error('lost response');}
      return {data:route.endsWith('/task')?details:visible?[details]:[]};
    }});
    const result=await p.deploy(record,{server:'test',token:'test'},dir);assert.equal(result.deployment_id,'task');assert.equal(posts,existing?0:1);
  }
});

test('uncertain deployment never repeats POST and wrong target fails verification',async t=>{
  const dir=fs.mkdtempSync(path.join(os.tmpdir(),'tps-uncertain-'));t.after(()=>fs.rmSync(dir,{recursive:true,force:true}));
  const info=payloadInfo(fixture()),name='tps-mcu-1.0.2-20261111',record={name,state:'VERIFIED',artifact_id:'artifact',device_reference:DEVICE,device_type:'eyemeter-thermal-aarch64',module:'eyemeter-tps-mcu',source_payload:info};let posts=0;
  const p=new Publisher({transport:async(base,route,opts)=>{
    if(route.includes('/inventory/'))return {data:{id:DEVICE,attributes:[{name:'device_type',value:record.device_type}]}};
    if(route.includes('/artifacts'))return {data:[{id:'artifact',name,device_types_compatible:[record.device_type],updates:[{type_info:{type:record.module},files:[{name:'firmware.tpsfw',checksum:info.sha256,size:info.size}]}]}]};
    if(opts.method==='POST'){posts++;throw Error('timeout');}return {data:[]};
  }});
  await assert.rejects(p.deploy(record,{server:'test'},dir),/未确认/);assert.equal(record.deployment_state,'UNCERTAIN');
  await assert.rejects(p.deploy(record,{server:'test'},dir),/重复POST/);assert.equal(posts,1);
  record.deployment_id='wrong';p.transport=async(base,route)=>route.endsWith('/wrong')?{data:{artifact_name:name,filter:{terms:[{scope:'identity',attribute:'id',type:'$in',value:['another-device']}]}}}:route.includes('/inventory/')?{data:{id:DEVICE,attributes:[{name:'device_type',value:record.device_type}]}}:{data:[{id:'artifact',name,device_types_compatible:[record.device_type],updates:[{type_info:{type:record.module},files:[{name:'firmware.tpsfw',checksum:info.sha256,size:info.size}]}]}]};
  await assert.rejects(p.deploy(record,{server:'test'},dir),/目标范围/);
});
test('server version race rejects before upload',async()=>{
  const p=new Publisher({transport:async()=>({data:[{id:'existing',name:'tps-mcu-1.0.2-20261111'}]})});
  await assert.rejects(p.upload({name:'tps-mcu-1.0.2-20261111',connection:{server:'test',token:'test'},data:fixture()}),/新版本/);assert.equal(p.busy,false);
});
