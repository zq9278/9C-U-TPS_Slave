'use strict';
const $=id=>document.getElementById(id);
let busy=false,connected=false,bundled=false,available=false,logText='';
function state(text,error=false,success=false){$('status').textContent=text;$('status').className=error?'error':success?'success':'';}
function render(){
  for(const id of ['flash','verify'])$(id).disabled=busy||connected||!available;
  $('build').hidden=bundled;$('build').disabled=busy||connected;
  $('choose').hidden=bundled;$('choose').disabled=busy||connected;
  $('upload').disabled=busy||connected;$('publish-settings').disabled=busy||connected;
  for(const id of ['serial','speed'])$(id).disabled=busy||connected;
  $('connect').disabled=busy||connected;$('disconnect').disabled=busy||!connected;
  for(const id of ['rtt-info','rtt-status','rtt-help'])$(id).disabled=busy||!connected;
  $('rtt-state').textContent=connected?'已连接 · background RTT':'未连接';
}
function append(id,text){$(id).textContent=($(id).textContent+text).slice(-150000);$(id).scrollTop=$(id).scrollHeight;}
async function refresh(){
  const data=await window.tps.invoke('info');bundled=data.bundled;available=!!data.firmware;
  $('jlink').value=data.jlink;$('source').textContent=data.root;
  if(data.firmware){const m=data.firmware;
    $('firmware').textContent=`App ${m.identity.version}  ·  ${m.device}  ·  Target 0x${m.identity.target.toString(16).toUpperCase()}  ·  HW ${m.identity.hardware}  ·  RTT ON\n`+
      `Boot 16 KiB / App 108 KiB / Metadata 4 KiB  ·  ${m.preset}\n`+
      `Factory SHA256  ${m.files['tps_factory.bin'].sha256}\nOTA SHA256      ${m.files['tps_app.tpsfw'].sha256}\n`+
      `源码 ${m.source.commit.slice(0,12)}${m.source.dirty?' + 工作区快照':''}  ·  实机验收 ${m.hardware_validation.status}`;
  }else $('firmware').textContent='未找到有效固件：'+data.error+'\n请先重新编译并打包。';
  render();
}
function opts(){return {serial:$('serial').value.trim(),speed:$('speed').value};}
function showSuccess(title,detail){
  $('success-title').textContent=title;$('success-detail').textContent=detail;
  if(!$('success-dialog').open)$('success-dialog').showModal();
}
$('success-close').onclick=()=>$('success-dialog').close();
async function task(fn){
  if(busy)return;busy=true;render();
  try{await fn();}catch(e){state(e.message,true);append('flash-log',e.message+'\n');}
  finally{busy=false;render();}
}
$('build').onclick=()=>task(async()=>{state('编译中');await window.tps.invoke('build');await refresh();state('构建和固件校验完成',false,true);showSuccess('构建成功','固件编译、打包及文件校验已完成。');});
for(const id of ['flash','verify'])$(id).onclick=()=>task(async()=>{
  state(id==='flash'?'正在烧录':'正在校验');$('progress-wrap').hidden=false;$('progress').value=0;
  const result=await window.tps.invoke(id,opts());
  if(result.canceled){state('已取消');return;}
  state('Flash 回读 SHA256 校验通过',false,true);
  append('flash-log',`\n${result.version} 回读一致；已请求复位运行。App 业务启动仍需 UART 版本/心跳确认。\n`);
  showSuccess(id==='flash'?'烧录成功':'校验成功',`版本 ${result.version}\n完整 Flash 回读 SHA256 一致。\n已发送复位运行命令。`);
});
$('choose').onclick=()=>task(async()=>{await window.tps.invoke('choose-jlink');await refresh();});
$('upload').onclick=()=>task(async()=>{state('准备服务器发布');const result=await window.tps.invoke('upload');if(result.canceled){state('已取消上传');return;}state('上传已核对：'+result.name,false,true);showSuccess('上传成功',`${result.name}\n服务器负载哈希和大小已核对。\n仅上传 Releases，未创建部署。`);});
$('publish-settings').onclick=()=>task(async()=>{if(await window.tps.invoke('publish-settings'))state('本机发布设置已保存');});
$('connect').onclick=()=>task(async()=>{state('连接 RTT');const info=await window.tps.invoke('rtt-connect',opts());connected=true;append('rtt-log',`\nRTT ${info.control_block} · ${info.speed_khz} kHz\n`);state('RTT 已连接',false,true);});
$('disconnect').onclick=()=>task(async()=>{await window.tps.invoke('rtt-disconnect');connected=false;state('RTT 已断开');});
for(const name of ['info','status','help'])$('rtt-'+name).onclick=()=>task(async()=>{await window.tps.invoke('rtt-command',name);state('命令已发送，回复见 RTT 日志');});
$('output').onclick=()=>task(()=>window.tps.invoke('output'));
$('save').onclick=()=>task(()=>window.tps.invoke('save-log',logText));
$('clear').onclick=()=>{logText='';$('rtt-log').textContent='';};
$('pause').onchange=()=>{if(!$('pause').checked){$('rtt-log').textContent=logText;$('rtt-log').scrollTop=$('rtt-log').scrollHeight;}};
window.tps.onEvent((event,value)=>{
  if(event==='log'){append('flash-log',value);$('flash-details').open=true;}
  if(event==='progress'){const labels=['','1 / 3 · 连接探针、下载及校验','2 / 3 · 核对完整 Flash 回读','3 / 3 · 回读 SHA256 一致'];$('progress').value=value;$('progress-label').textContent=labels[value];}
  if(event==='rtt-log'){logText=(logText+value).slice(-150000);if(!$('pause').checked){$('rtt-log').textContent=logText;$('rtt-log').scrollTop=$('rtt-log').scrollHeight;}}
  if(event==='rtt-state'){connected=!!value;render();if(!connected&&!busy)state('RTT 已断开');}
});
refresh().then(()=>state('就绪')).catch(e=>state(e.message,true));
