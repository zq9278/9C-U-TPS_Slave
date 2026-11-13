'use strict';
const fs=require('node:fs'),path=require('node:path'),https=require('node:https');
const crypto=require('node:crypto'),zlib=require('node:zlib');
const {crc32}=require('./backend.cjs');
const SERVERS=['aliyun','hk','oregon','oracle1','oracle2'].map(h=>`https://${h}.sanitlook.dpdns.org:19443`);
const DEVICE='d0b82a98-300b-41ae-b5d6-530f80fe87cd';
const DEVICE_TYPE='eyemeter-thermal-aarch64',MODULE='eyemeter-tps-mcu';
const API='/api/management/v1/deployments/artifacts';
const DEPLOY_API='/api/management/v1/deployments/deployments';
const DEFAULT_CREDENTIALS='D:/Technology_stack/Web/mender/admin-credentials.json';
const hash=data=>crypto.createHash('sha256').update(data).digest('hex');
function nextName(artifacts) {
  let patch=2;
  for(const a of artifacts){const m=/^tps-mcu-1\.0\.(\d+)-\d{8}$/.exec(a.name||'');if(m)patch=Math.max(patch,Number(m[1])+1);}
  if(!Number.isSafeInteger(patch))throw Error('服务器版本号超出范围');
  return `tps-mcu-1.0.${patch}-20261111`;
}
function payloadInfo(data) {
  if(data.length<72||data.length>110656)throw Error('请选择有效 .tpsfw App 包，不能上传 factory/bin/elf');
  const h=data.subarray(0,64),image=data.subarray(64);
  if(h.subarray(0,8).toString()!=='TPSFW001'||h.readUInt16LE(8)!==64||h.readUInt16LE(10)!==1||
     h.readUInt32LE(12)!==0x9C01||h.readUInt32LE(20)!==0x08004000||h.readUInt32LE(24)!==image.length||
     h.readUInt32LE(28)!==crc32(image)||h.readUInt32LE(60)!==crc32(h.subarray(0,60))||h.subarray(40,60).some(v=>v))throw Error('App 包头/目标/CRC不匹配');
  const sp=image.readUInt32LE(0),pc=image.readUInt32LE(4);
  if(sp<=0x20000000||sp>0x20008FE0||sp%8||!(pc&1)||(pc&0xFFFFFFFE)<0x08004000||(pc&0xFFFFFFFE)>=0x08004000+image.length)throw Error('App 向量不匹配');
  const min=h.readUInt16LE(16),max=h.readUInt16LE(18);if(min>max)throw Error('硬件范围错误');
  const code=h.readUInt32LE(32),version=[code>>>24,(code>>>16)&255,(code>>>8)&255,code&255].join('.');
  const id=image.indexOf(Buffer.from('TPSID001'));
  if(id>=0&&(id+64>image.length||image.readUInt32LE(id+8)!==code||image.readUInt32LE(id+16)!==0x9C01))throw Error('App 编译身份与包头不同');
  return {version,hw_min:min,hw_max:max,sha256:hash(data),size:data.length};
}
// USTAR containers matching the existing server Artifact v3 layout. Names are
// fixed, payload is App-only; no signed download URL logging.
function tar(entries) {
  const parts=[];
  for(const [name,content] of entries){const data=Buffer.isBuffer(content)?content:Buffer.from(content),h=Buffer.alloc(512);
    if(Buffer.byteLength(name)>100)throw Error('tar name too long');h.write(name);
    const oct=(offset,size,value)=>h.write(value.toString(8).padStart(size-1,'0')+'\0',offset,size,'ascii');
    oct(100,8,0o644);oct(108,8,0);oct(116,8,0);oct(124,12,data.length);oct(136,12,0);
    h.fill(32,148,156);h[156]=48;h.write('ustar\0',257,6);h.write('00',263,2);
    let sum=0;for(const b of h)sum+=b;h.write(sum.toString(8).padStart(6,'0')+'\0 ',148,8);
    parts.push(h,data,Buffer.alloc((512-data.length%512)%512));
  }parts.push(Buffer.alloc(1024));return Buffer.concat(parts);
}
function artifact(data,name,signing={}) {
  const info=payloadInfo(data);
  if(!/^tps-mcu-1\.0\.\d+-20261111$/.test(name))throw Error('无效发布名称');
  const version=Buffer.from('{"format":"mender","version":3}');
  const header=zlib.gzipSync(tar([
    ['header-info',JSON.stringify({payloads:[{type:MODULE}],artifact_provides:{artifact_name:name},artifact_depends:{device_type:[DEVICE_TYPE]}})],
    ['headers/0000/type-info',JSON.stringify({type:MODULE,artifact_provides:{'eyemeter.tps-mcu.target':'0x9C01','eyemeter.tps-mcu.version':info.version},clears_artifact_provides:['eyemeter.tps-mcu.*']})]
  ]));
  const payload=zlib.gzipSync(tar([['firmware.tpsfw',data]]));
  const manifest=Buffer.from(`${hash(data)}  data/0000/firmware.tpsfw\n${hash(header)}  header.tar.gz\n${hash(version)}  version\n`);
  const entries=[['version',version],['manifest',manifest]];
  let signed=false;
  if(signing.mode==='signed'){
    if(!signing.privateKey||!signing.publicKey)throw Error('未配置TPS签名私钥与设备信任公钥');
    const key=crypto.createPrivateKey(fs.readFileSync(signing.privateKey)),pub=crypto.createPublicKey(fs.readFileSync(signing.publicKey));
    if(!['rsa','ec'].includes(key.asymmetricKeyType)||key.asymmetricKeyType==='ec'&&key.asymmetricKeyDetails.namedCurve!=='prime256v1')throw Error('Mender仅支持RSA或ECDSA P-256密钥');
    // Mender serializes P-256 as fixed r||s (64 bytes), not ASN.1 DER.
    const signature=crypto.sign('sha256',manifest,{key,dsaEncoding:'ieee-p1363'});
    if(!crypto.verify('sha256',manifest,{key:pub,dsaEncoding:'ieee-p1363'},signature))throw Error('签名不被所选设备信任公钥接受');
    entries.push(['manifest.sig',Buffer.from(signature.toString('base64'))]);signed=true;
  }else if(signing.mode && signing.mode!=='unsigned')throw Error('无效发布模式');
  entries.push(['header.tar.gz',header],['data/0000.tar.gz',payload]);
  return {bytes:tar(entries),info,signed};
}
function request(base,route,{method='GET',body,token,headers={}}={}) {
  return new Promise((resolve,reject)=>{
    const req=https.request(base+route,{method,headers:{...headers,...(token?{Authorization:'Bearer '+token}:{}),...(body?{'Content-Length':body.length}:{})},rejectUnauthorized:true},res=>{
      const parts=[];let size=0;
      res.on('data',b=>{size+=b.length;if(size>16*1024*1024){res.destroy(Error('Response too large'));return;}parts.push(b);});
      res.on('error',reject);res.on('end',()=>{
        const bytes=Buffer.concat(parts);let data=bytes.toString();
        if((res.headers['content-type']||'').includes('application/json')){try{data=JSON.parse(data);}catch{reject(Error('服务器返回无效JSON'));return;}}
        if(res.statusCode<200||res.statusCode>=300){const e=Error('Mender HTTP '+res.statusCode);e.status=res.statusCode;reject(e);return;}
        resolve({data,headers:res.headers,status:res.statusCode});
      });
    });req.on('error',reject);req.setTimeout(30000,()=>req.destroy(Error('Mender连接超时')));if(body)req.write(body);req.end();
  });
}
class Publisher {
  constructor({settingsFile,records,transport=request}={}) {this.settingsFile=settingsFile;this.records=records;this.transport=transport;this.busy=false;}
  settings() {
    let s={};try{s=JSON.parse(fs.readFileSync(this.settingsFile,'utf8'));}catch{}
    return {credentials:DEFAULT_CREDENTIALS,...s,mode:s.mode||'unsigned'};
  }
  async connect() {
    const s=this.settings(),c=JSON.parse(fs.readFileSync(s.credentials,'utf8').replace(/^\uFEFF/,''));
    if(typeof c.username!=='string'||typeof c.password!=='string')throw Error('管理员凭据文件无效');
    for(const server of SERVERS){try{
      const r=await this.transport(server,'/api/management/v1/useradm/auth/login',{method:'POST',body:Buffer.alloc(0),headers:{Authorization:'Basic '+Buffer.from(c.username+':'+c.password).toString('base64')}});
      const token=typeof r.data==='string'?r.data.trim().replace(/^"|"$/g,''):r.data;
      if(typeof token!=='string'||!token)throw Error('认证未返回令牌');return {server,token};
    }catch(e){if(e.status===401||e.status===403)throw Error('Mender管理员认证失败');}}
    throw Error('五个Mender入口均无法验证连接；检查网络和HTTPS证书');
  }
  async list(connection) {
    const all=[];const ids=new Set();
    for(let page=1;page<=100;page++){
      const r=await this.transport(connection.server,API+`?page=${page}&per_page=100`,{token:connection.token});
      if(!Array.isArray(r.data))throw Error('服务器固件列表格式无效');
      for(const item of r.data){if(ids.has(item.id))throw Error('服务器分页重复，停止分配版本号');ids.add(item.id);all.push(item);}
      if(r.data.length<100)return all;
    }throw Error('服务器列表分页过多');
  }
  async prepare(file) {
    if(this.busy)throw Error('上传正在进行');
    const data=fs.readFileSync(file),info=payloadInfo(data),connection=await this.connect();
    await this.compatibleDevices(connection);
    const artifacts=await this.list(connection),name=nextName(artifacts);
    return {file,data,info,connection,name};
  }
  async compatibleDevices(connection) {
    const ids=new Set(),seen=new Set();
    for(let page=1;page<=100;page++){
      const r=await this.transport(connection.server,`/api/management/v1/inventory/devices?page=${page}&per_page=100`,{token:connection.token});
      if(!Array.isArray(r.data))throw Error('设备列表格式无效');
      for(const d of r.data){
        if(typeof d.id!=='string'||!/^[\w-]+$/.test(d.id)||seen.has(d.id))throw Error('设备列表ID无效或分页重复');seen.add(d.id);
        if(d.attributes?.some(a=>a.name==='device_type'&&a.value===DEVICE_TYPE))ids.add(d.id);
      }
      if(r.data.length<100){if(!ids.size)throw Error('服务器没有兼容的 TPS130 设备');return [...ids].sort();}
    }throw Error('设备列表分页过多');
  }
  async deploy(record,connection,folder) {
    if(record.state!=='VERIFIED'||record.device_reference!==DEVICE||record.device_type!==DEVICE_TYPE||record.module!==MODULE)throw Error('发布记录未核对或设备不匹配');
    const save=()=>fs.writeFileSync(path.join(folder,'upload.json'),JSON.stringify(record,null,2)+'\n');
    let targets;
    if(record.deployment_scope==='all-compatible'){
      const current=await this.compatibleDevices(connection);
      targets=record.deployment_devices||current;
      if(!Array.isArray(targets)||!targets.length||new Set(targets).size!==targets.length||targets.some(id=>!current.includes(id)))throw Error('已记录目标设备不再兼容，请核查服务器');
      targets=[...targets].sort();record.deployment_devices=targets;save();
    }else{
      const device=await this.transport(connection.server,'/api/management/v1/inventory/devices/'+DEVICE,{token:connection.token});
      if(device.data.id!==DEVICE||!device.data.attributes?.some(a=>a.name==='device_type'&&a.value===DEVICE_TYPE))throw Error('部署目标设备身份或兼容类型不匹配');
      targets=[DEVICE];
    }
    const artifacts=await this.list(connection),a=artifacts.find(a=>a.id===record.artifact_id&&a.name===record.name);
    if(!a||!a.device_types_compatible?.includes(DEVICE_TYPE)||!a.updates?.some(u=>u.type_info?.type===MODULE&&u.files?.some(f=>f.name==='firmware.tpsfw'&&f.checksum===record.source_payload.sha256&&f.size===record.source_payload.size)))throw Error('部署前服务器负载核对失败');
    const exactTarget=d=>{
      const terms=d.filter?.terms;
      return Array.isArray(terms)&&terms.length===1&&terms[0].scope==='identity'&&terms[0].attribute==='id'&&terms[0].type==='$in'&&
        Array.isArray(terms[0].value)&&terms[0].value.length===targets.length&&
        [...terms[0].value].sort().every((id,i)=>id===targets[i]);
    };
    const details=async id=>{
      if(typeof id!=='string'||!/^[\w-]+$/.test(id))throw Error('部署ID无效');
      const r=await this.transport(connection.server,DEPLOY_API+'/'+id,{token:connection.token});
      if(r.data.artifact_name!==record.name||!exactTarget(r.data))throw Error('部署目标范围或固件不匹配');
      return r.data;
    };
    const find=async()=>{
      if(record.deployment_id)return details(record.deployment_id);
      for(const status of ['pending','inprogress','finished']){
        for(let page=1;page<=100;page++){
          const r=await this.transport(connection.server,DEPLOY_API+`?status=${status}&page=${page}&per_page=100`,{token:connection.token});
          if(!Array.isArray(r.data))throw Error('部署列表格式无效');
          for(const d of r.data)if(d.artifact_name===record.name){const full=await this.transport(connection.server,DEPLOY_API+'/'+d.id,{token:connection.token});if(exactTarget(full.data)&&full.data.artifact_name===record.name)return full.data;}
          if(r.data.length<100)break;
          if(page===100)throw Error('部署列表分页过多');
        }
      }return null;
    };
    let found=await find();
    if(!found){
      if(['POSTING','UNCERTAIN','CREATED'].includes(record.deployment_state))throw Error('之前创建任务结果未确认，停止重复POST；请核对服务器后使用“补建/核对更新任务”');
      record.deployment_state='POSTING';save();
      try{
        const r=await this.transport(connection.server,DEPLOY_API,{method:'POST',token:connection.token,headers:{'Content-Type':'application/json'},body:Buffer.from(JSON.stringify({name:record.name,artifact_name:record.name,devices:targets}))});
        record.deployment_id=r.headers?.location?.split('/').filter(Boolean).at(-1)||r.data?.id;record.deployment_state='CREATED';save();
      }catch(e){record.deployment_state=e.status>=400&&e.status<500?'REJECTED':'UNCERTAIN';save();
        found=await find();if(!found)throw Error('包已上传，但更新任务未确认；请补建/核对更新任务，不要重复上传。');
      }
      if(!found)found=await find();
      if(!found)throw Error('包已上传，但服务器任务尚未核对到；请稍后补建/核对更新任务');
    }
    record.deployment_id=found.id;record.deployment_status=found.status;record.deployment_devices=targets;record.deployment_state='VERIFIED';record.deployment_created=true;save();
    return {...record,folder};
  }
  async resumeDeployment(file) {
    if(this.busy)throw Error('发布正在进行');this.busy=true;
    try{const record=JSON.parse(fs.readFileSync(file,'utf8').replace(/^\uFEFF/,''));return await this.deploy(record,await this.connect(),path.dirname(file));}
    finally{this.busy=false;}
  }
  async upload(prepared) {
    if(this.busy)throw Error('上传正在进行');this.busy=true;
    try{
      const {connection,name,data}=prepared;
      if(nextName(await this.list(connection))!==name)throw Error('服务器已有新版本，请重新准备并确认名称');
      const packed=artifact(data,name,this.settings());
      fs.mkdirSync(this.records,{recursive:true});const folder=path.join(this.records,name);fs.mkdirSync(folder,{recursive:false});
      const output=path.join(folder,name+'.mender');fs.writeFileSync(output,packed.bytes,{flag:'wx'});
      const record={name,device_reference:DEVICE,device_type:DEVICE_TYPE,module:MODULE,deployment_scope:'all-compatible',source_payload:packed.info,
        signed:packed.signed,artifact_sha256:hash(packed.bytes),state:'PREPARED',deployment_created:false};
      const save=()=>fs.writeFileSync(path.join(folder,'upload.json'),JSON.stringify(record,null,2)+'\n');save();
      const boundary='tps-'+crypto.randomBytes(16).toString('hex');
      const body=Buffer.concat([Buffer.from(`--${boundary}\r\nContent-Disposition: form-data; name="artifact"; filename="${name}.mender"\r\nContent-Type: application/octet-stream\r\n\r\n`),packed.bytes,Buffer.from(`\r\n--${boundary}--\r\n`)]);
      // Never blindly retry POST on another endpoint: a lost response may still
      // mean success. Reconcile by name and payload hash on this same backend.
      let postError;
      try{await this.transport(connection.server,API,{method:'POST',body,token:connection.token,headers:{'Content-Type':'multipart/form-data; boundary='+boundary}});record.state='UPLOADED_UNCONFIRMED';save();}
      catch(e){postError=e;record.state='UPLOAD_UNCERTAIN';save();}
      const list=await this.list(connection);const found=list.filter(a=>a.name===name);
      const match=found.find(a=>a.signed===packed.signed&&a.artifact_provides?.['eyemeter.tps-mcu.version']===packed.info.version&&a.device_types_compatible?.includes(DEVICE_TYPE)&&a.updates?.some(u=>u.type_info?.type===MODULE&&u.files?.some(f=>f.name==='firmware.tpsfw'&&f.checksum===packed.info.sha256&&f.size===data.length)));
      if(!match){record.state='UNCONFIRMED';save();throw Error(postError?'上传结果未确认；请检查服务器和本地upload.json，勿直接重复上传':'服务器未核对到相同负载，不能显示成功');}
      record.state='VERIFIED';record.artifact_id=match.id;save();
      try{return await this.deploy(record,connection,folder);}catch(e){throw Error(`包 ${name} 已上传；${e.message}\n发布记录 ${path.join(folder,'upload.json')}`);}
    }finally{this.busy=false;}
  }
}
module.exports={Publisher,nextName,payloadInfo,artifact,tar,SERVERS,DEVICE,DEVICE_TYPE,MODULE};
