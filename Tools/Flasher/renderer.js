'use strict';
const $=id=>document.getElementById(id),P=DebugPlatform;
let busy=false,connected=false,bundled=false,available=false,logText='',device=null;
const live=new P.TelemetryStore(),decoder=new P.LineDecoder();
let replay=null,replayIndex=0,replayPlaying=false,replayStart=0,replayHostStart=0;
let monitoring=false,polling=false,frozen=false,frozenEnd=0,recording=false,recorded=[],recordIdentity={},lastUiTime=0;
const visible=new Set(),pageNames={overview:'设备概览',workbench:'调试工作台',firmware:'固件与发布',tests:'检查与诊断',logs:'日志与记录',settings:'连接与设置'};
function state(text,error=false,success=false){$('status').textContent=text;$('status').className=error?'error':success?'success':'';}
function navigate(name){for(const [key,title] of Object.entries(pageNames)){$('page-'+key).hidden=key!==name;document.querySelector(`[data-page="${key}"]`).classList.toggle('selected',key===name);if(key===name)$('page-label').textContent=title;}draw();}
for(const b of document.querySelectorAll('[data-page]'))b.onclick=()=>navigate(b.dataset.page);
function showSuccess(title,detail){$('success-title').textContent=title;$('success-detail').textContent=detail;if(!$('success-dialog').open)$('success-dialog').showModal();}
$('success-close').onclick=()=>$('success-dialog').close();
function append(id,text){$(id).textContent=($(id).textContent+text).slice(-150000);$(id).scrollTop=$(id).scrollHeight;}
function render(){
  for(const id of ['flash','verify'])$(id).disabled=busy||connected||!available;
  $('build').hidden=bundled;$('build').disabled=busy||connected;
  $('choose').hidden=bundled;$('choose').disabled=busy||connected;
  for(const id of ['upload','publish-settings'])$(id).disabled=busy||connected;
  for(const id of ['serial','speed'])$(id).disabled=busy||connected;
  $('connect').disabled=busy||connected||!!replay;$('disconnect').disabled=busy||!connected;
  for(const id of ['rtt-info','rtt-status','rtt-help','check-device'])$(id).disabled=busy||!connected||!!replay;
  $('monitor').disabled=busy||!connected||!!replay;$('monitor').textContent=monitoring?'暂停采集':'开始采集';
  $('record').disabled=(!connected||!!replay)&&!recording;$('record').textContent=recording?'停止录制':'开始录制';
  $('return-live').hidden=!replay;$('replay-controls').hidden=!replay;
  $('rtt-state').textContent=connected?'RTT 已连接':'未连接';
  $('connection-pill').textContent=connected?'● RTT 在线':'● 未连接';$('connection-pill').classList.toggle('online',connected);
  $('side-connection').textContent=connected?'TPS130 · RTT 在线':'设备未连接';
  $('data-mode').textContent=replay?'REPLAY · 离线回放':connected?monitoring?'LIVE · 实时采集':'LIVE · 采集暂停':'LIVE · 等待连接';
  $('record-dot').classList.toggle('recording',recording);
  $('record-state').textContent=recording?`正在录制 · ${recorded.length} 个快照`:recorded.length?`已录制 ${recorded.length} 个快照，可保存记录`:'未开始录制';
}
function configure(d){
  device=P.validateDevice(d);
  $('parameter-notice').textContent=d.parameterNotice;$('telemetry-notice').textContent='RTT 状态快照 · 5 Hz 请求 · 非传感器原始采样';
  for(const c of d.channels){
    visible.add(c.id);const card=document.createElement('div');card.className='metric';card.style.setProperty('--metric-color',c.color);
    const label=document.createElement('div');label.className='metric-label';label.textContent=c.name;
    const symbol=document.createElement('span');symbol.className='metric-symbol';symbol.textContent=c.group==='temperature'?'◉':'⌁';label.append(symbol);
    const value=document.createElement('div');value.className='metric-value';const num=document.createElement('span');num.id='value-'+c.id;num.textContent='—';const unit=document.createElement('small');unit.textContent=c.unit;value.append(num,unit);
    const note=document.createElement('div');note.className='metric-note';note.id='note-'+c.id;note.textContent='等待设备回复';card.append(label,value,note);$('metric-grid').append(card);
    const toggle=document.createElement('label'),input=document.createElement('input'),dot=document.createElement('span');input.type='checkbox';input.checked=true;dot.className='legend-dot';dot.style.setProperty('--line-color',c.color);
    toggle.append(input,dot,document.createTextNode(c.name));$(c.group+'-legend').append(toggle);
    input.onchange=()=>{input.checked?visible.add(c.id):visible.delete(c.id);draw();};
  }
  for(const field of d.parameters){const row=document.createElement('div');row.className='parameter-field';const name=document.createElement('span'),value=document.createElement('strong');name.textContent=field.name;value.id='param-'+field.id;value.textContent='—';row.append(name,value);$('parameter-fields').append(row);}
}
function identity(){return replay?replay.identity||{}:live.identity;}
function updateIdentity(){if(!device)return;const data=identity();for(const f of device.parameters)$('param-'+f.id).textContent=data[f.id]||'—';
  const box=$('overview-identity');box.replaceChildren();for(const [name,value] of [['产品','TPS130'],['目标板',device.board],['设备实测 App',live.identity.app||'尚未读取'],['硬件修订',live.identity.hw||'尚未读取'],['连接通道',connected?'J-Link RTT':'未连接']]){const a=document.createElement('dt'),b=document.createElement('dd');a.textContent=name;b.textContent=value;box.append(a,b);}
}
async function refresh(){
  const data=await window.tps.invoke('info');bundled=data.bundled;available=!!data.firmware;
  if(!device)configure(data.device);
  $('jlink').value=data.jlink;$('source').textContent=data.root;$('software-version').textContent='TPS130 Studio '+data.version;
  if(data.firmware){const m=data.firmware;$('firmware').textContent=`内置 App ${m.identity.version} · ${m.device} · Target 0x${m.identity.target.toString(16).toUpperCase()} · HW ${m.identity.hardware} · RTT ON\nBoot 16 KiB / App 108 KiB / Metadata 4 KiB\nFactory SHA256  ${m.files['tps_factory.bin'].sha256}\nOTA SHA256      ${m.files['tps_app.tpsfw'].sha256}\n源码 ${m.source.commit.slice(0,12)}${m.source.dirty?' + 工作区快照':''} · 实机验收 ${m.hardware_validation.status}`;
  }else $('firmware').textContent='未找到有效交付固件：'+data.error;
  render();updateIdentity();draw();return data;
}
function opts(){return {serial:$('serial').value.trim(),speed:$('speed').value};}
async function task(fn){if(busy)return;busy=true;render();try{await fn();}catch(e){state(e.message,true);append('flash-log',e.message+'\n');$('flash-details').open=true;}finally{busy=false;render();}}
async function query(command){if(!connected||replay||polling)return;polling=true;try{await window.tps.invoke('rtt-command',command);}finally{polling=false;}}
function ingest(text){
  logText=(logText+text).slice(-150000);renderLogs();
  for(const line of decoder.feed(text)){const event=DeviceAdapter.decode(line);if(!event)continue;
    if(event.type==='identity'){live.identity=event.values;updateIdentity();}
    if(event.type==='sample'){
      const s=live.push(event.sample);
      if(recording){if(recorded.length<50000)recorded.push(s);else{recording=false;state('录制达到 50000 点上限，请保存记录');}}
      if($('check-result').dataset.pending==='1'){$('check-result').textContent=`已收到设备状态回复\n设备Tick ${s.deviceTick} · 治疗标志 ${s.running}\n设备App ${live.identity.app||'尚未收到身份回复'}`;delete $('check-result').dataset.pending;}
    }
  }render();draw();
}
function renderLogs(){if($('pause').checked)return;const filter=$('log-filter').value.toLowerCase();$('rtt-log').textContent=filter?logText.split('\n').filter(l=>l.toLowerCase().includes(filter)).join('\n'):logText;$('rtt-log').scrollTop=$('rtt-log').scrollHeight;}
function displayedSamples(){return replay?replay.samples.slice(0,replayIndex+1):live.samples;}
function draw(){
  if(!device)return;const all=displayedSamples(),end=replay?all.at(-1)?.time||Date.now():frozen?frozenEnd:Date.now();
  const samples=all.filter(s=>s.time<=end),last=samples.at(-1),isStale=!replay&&(!connected||!monitoring||!last||Date.now()-last.time>1500);
  for(const c of device.channels){const value=last?.values[c.id];$('value-'+c.id).textContent=Number.isFinite(value)?value.toFixed(c.precision):'—';$('note-'+c.id).textContent=replay?'离线记录':isStale?last?'历史快照 · 已过期':'等待设备回复':'设备实测 · RTT';}
  $('freshness').textContent=replay?'记录数据':isStale?'数据未更新':'设备回复正常';$('freshness').className='tag '+(isStale?'muted':'');
  $('sample-counter').textContent=`${samples.length} 个有效快照${frozen?' · 查看已暂停':''}`;
  $('runtime-running').textContent=last?String(last.running):'—';$('runtime-tick').textContent=last?last.deviceTick+' ms':'—';$('runtime-heap').textContent=last?.free_heap!==undefined?last.free_heap+' B':'—';$('runtime-drops').textContent=last?.log_drops!==undefined?String(last.log_drops):'—';
  for(const group of ['temperature','pressure'])drawTelemetryChart($(group+'-chart'),samples,device.channels.filter(c=>c.group===group),end,Number($('time-window').value),visible);
  if(replay){$('replay-time').textContent=last?new Date(last.time).toLocaleTimeString('zh-CN',{hour12:false}):'空记录';$('replay-position').value=replayIndex;}
}
$('build').onclick=()=>task(async()=>{state('编译中');await window.tps.invoke('build');await refresh();state('构建完成',false,true);showSuccess('构建成功','固件编译、打包及文件校验已完成。');});
for(const id of ['flash','verify'])$(id).onclick=()=>task(async()=>{state(id==='flash'?'正在烧录':'正在校验');$('progress-wrap').hidden=false;$('progress').value=0;const r=await window.tps.invoke(id,opts());if(r.canceled)return;state('Flash 回读校验通过',false,true);showSuccess(id==='flash'?'烧录成功':'校验成功',`版本 ${r.version}\n完整 Flash 回读 SHA256 一致。\n已发送复位运行命令。`);});
$('upload').onclick=()=>task(async()=>{state('准备服务器发布');const r=await window.tps.invoke('upload');if(r.canceled){state('已取消上传');return;}state('上传已核对：'+r.name,false,true);showSuccess('上传成功',`${r.name}\n服务器负载已核对。\n未创建部署任务。`);});
$('choose').onclick=()=>task(async()=>{await window.tps.invoke('choose-jlink');await refresh();});
$('publish-settings').onclick=()=>task(async()=>{if(await window.tps.invoke('publish-settings'))state('发布设置已保存');});
$('connect').onclick=()=>task(async()=>{state('连接 RTT');const r=await window.tps.invoke('rtt-connect',opts());connected=true;monitoring=true;decoder.reset();live.disconnect();live.identity={};append('rtt-log',`\n连接 RTT ${r.control_block}\n`);await query('info');state('RTT 已连接',false,true);updateIdentity();});
$('disconnect').onclick=()=>task(async()=>{monitoring=false;await window.tps.invoke('rtt-disconnect');connected=false;live.disconnect();recording=false;decoder.reset();updateIdentity();state('RTT 已断开');draw();});
for(const name of ['info','status','help'])$('rtt-'+name).onclick=()=>task(async()=>{await query(name);state('请求已发送，等待设备回复');});
$('monitor').onclick=()=>{monitoring=!monitoring;render();};
$('freeze').onclick=()=>{frozen=!frozen;frozenEnd=Date.now();$('freeze').textContent=frozen?'恢复跟随':'暂停查看';draw();};
$('time-window').onchange=draw;
$('record').onclick=()=>{if(recording){recording=false;}else{recorded=[];recordIdentity={...live.identity};recording=true;}render();};
async function saveData(csv){
  const samples=replay?replay.samples:recorded.length?recorded:live.samples;
  if(!samples.length){state('暂无设备数据可导出');return;}
  const content=csv?P.csv(samples,device.channels):JSON.stringify({schema:1,device:device.id,identity:replay?replay.identity:recorded.length?recordIdentity:live.identity,created_at:new Date().toISOString(),source:'RTT status snapshots',samples},null,2);
  if(await window.tps.invoke(csv?'save-csv':'save-recording',{content}))showSuccess('保存成功',csv?'已导出设备快照 CSV。':'已保存记录，可离线回放。');
}
$('export-session').onclick=()=>task(()=>saveData(false));$('export-csv').onclick=()=>task(()=>saveData(true));
$('load-session').onclick=()=>task(async()=>{if(recording){state('请先停止录制');return;}const r=await window.tps.invoke('load-recording');if(!r)return;replay=P.validateRecording(r,device);monitoring=false;replayIndex=0;replayPlaying=false;frozen=false;$('replay-position').max=Math.max(0,replay.samples.length-1);render();updateIdentity();draw();state('离线回放 · 不发送设备命令');});
$('return-live').onclick=()=>{replay=null;replayPlaying=false;monitoring=connected;render();updateIdentity();draw();};
$('replay-position').oninput=()=>{replayIndex=Number($('replay-position').value);replayPlaying=false;$('play-replay').textContent='播放';draw();};
$('play-replay').onclick=()=>{if(!replay?.samples.length)return;replayPlaying=!replayPlaying;if(replayPlaying){if(replayIndex===replay.samples.length-1)replayIndex=0;replayStart=replay.samples[replayIndex].time;replayHostStart=Date.now();}$('play-replay').textContent=replayPlaying?'暂停':'播放';};
$('identity-tab').onclick=()=>{$('identity-content').hidden=false;$('pid-content').hidden=true;$('identity-tab').classList.add('selected');$('pid-tab').classList.remove('selected');};
$('pid-tab').onclick=()=>{$('identity-content').hidden=true;$('pid-content').hidden=false;$('identity-tab').classList.remove('selected');$('pid-tab').classList.add('selected');};
$('check-device').onclick=()=>task(async()=>{$('check-result').textContent='等待真实设备回复…';$('check-result').dataset.pending='1';await query('info');await query('status');setTimeout(()=>{if($('check-result').dataset.pending==='1'){$('check-result').textContent='未收到状态回复，不能确认设备检查通过。';delete $('check-result').dataset.pending;}},2000);});
$('check-package').onclick=()=>task(async()=>{const data=await refresh();$('check-result').textContent=data.firmware?'交付文件哈希、镜像布局与身份检查通过。\n不代表实机烧录或设备运行通过。':'交付文件检查失败：'+data.error;});
$('output').onclick=()=>task(()=>window.tps.invoke('output'));$('save').onclick=()=>task(()=>window.tps.invoke('save-log',logText));
$('clear').onclick=()=>{logText='';$('rtt-log').textContent='';};$('pause').onchange=renderLogs;$('log-filter').oninput=renderLogs;
window.tps.onEvent((event,value)=>{
  if(event==='log')append('flash-log',value);
  if(event==='progress'){const labels=['','连接、下载及校验','核对完整 Flash 回读','Flash 回读 SHA256 一致'];$('progress').value=value;$('progress-label').textContent=labels[value];}
  if(event==='rtt-log')ingest(value);
  if(event==='rtt-state'){connected=!!value;if(!connected){monitoring=false;recording=false;live.disconnect();decoder.reset();}render();updateIdentity();draw();}
});
setInterval(async()=>{if(connected&&monitoring&&!replay&&!busy&&!polling){try{await query('status');}catch(e){state('采集请求失败：'+e.message,true);monitoring=false;render();}}},200);
setInterval(()=>{
  if(replayPlaying&&replay?.samples.length){const t=replayStart+Date.now()-replayHostStart;while(replayIndex<replay.samples.length-1&&replay.samples[replayIndex+1].time<=t)replayIndex++;if(replayIndex===replay.samples.length-1){replayPlaying=false;$('play-replay').textContent='播放';}}
  if(Date.now()-lastUiTime>200){lastUiTime=Date.now();draw();}
},100);
window.addEventListener('resize',draw);
refresh().then(()=>state('就绪 · 等待连接设备')).catch(e=>state(e.message,true));
