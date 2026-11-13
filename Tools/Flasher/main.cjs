'use strict';
const {app,BrowserWindow,ipcMain,dialog,shell,screen}=require('electron');
const fs=require('node:fs'),path=require('node:path');
const {ROOT,Programmer,findJLink,validateOptions}=require('./backend.cjs');
const {Rtt}=require('./rtt.cjs');
const {PowerProgrammer}=require('./powermanage.cjs');
const {Publisher}=require('./publisher.cjs');
const {stopChildren}=require('./processes.cjs');
// Smoke checks use a separate profile and lock, leaving an operator's running
// programmer and its saved publishing configuration untouched.
if(process.argv.includes('--smoke-test'))app.setPath('userData',path.join(app.getPath('temp'),'tps-flasher-smoke-profile'));
if(!app.requestSingleInstanceLock()) {app.quit();} else {
  app.setName('TPS130 Studio');
  if(process.platform==='win32')app.setAppUserModelId('Sanitlook.TPS130.Studio');
  const bundled=fs.existsSync(path.join(process.resourcesPath,'payload/bundle.json'));
  const payload=bundled?path.join(process.resourcesPath,'payload'):null;
  const programmer=new Programmer({payload});
  const powerProgrammer=new PowerProgrammer({payload});
  const publisher=new Publisher({settingsFile:path.join(app.getPath('userData'),'publisher.json'),records:path.join(app.getPath('documents'),'TPS130-Published')});
  let win, active=null, chosenJLink=findJLink();
  const emit=(event,data)=>{if(win&&!win.isDestroyed())win.webContents.send('event',event,data);};
  const rtt=new Rtt((event,data)=>{if(event==='rtt-state'&&!data&&active==='rtt'&&!rtt.child)active=null;emit(event,data);});
  function noTask() {if(active||programmer.busy||powerProgrammer.busy||publisher.busy||rtt.child)throw Error('请先结束当前任务或断开 RTT');}
  function options(value) {validateOptions(value);return {...value,jlink:chosenJLink};}
  ipcMain.handle('action',async(event,action,arg)=>{
    if(event.sender!==win.webContents)throw Error('Invalid sender');
    if(action==='info') {
      let firmware=null,error='';try{firmware=programmer.inspect();}catch(e){error=e.message;}
      const device=JSON.parse(fs.readFileSync(path.join(__dirname,'devices/tps130/manifest.json'),'utf8'));
      let powerFirmware=null,powerError='';try{powerFirmware=powerProgrammer.inspect();}catch(e){powerError=e.message;}
      return {bundled,root:bundled?process.resourcesPath:ROOT,jlink:programmer.runtime({jlink:chosenJLink}),firmware,error,powerFirmware,powerError,version:app.getVersion(),device};
    }
    if(action==='choose-jlink') {
      noTask();if(bundled)throw Error('分发版已内置 J-Link');
      const d=await dialog.showOpenDialog(win,{title:'选择 JLink.exe',properties:['openFile'],filters:[{name:'J-Link Commander',extensions:['exe']}]});
      if(!d.canceled) {if(path.basename(d.filePaths[0]).toLowerCase()!=='jlink.exe')throw Error('请选择 JLink.exe');chosenJLink=d.filePaths[0];}
      return chosenJLink;
    }
    if(action==='build') {
      noTask();active='build';try{return await programmer.build(t=>emit('log',t));}finally{active=null;}
    }
    if(action==='upload') {
      noTask();active='upload';
      try {
        const selected=await dialog.showOpenDialog(win,{title:'选择要发布的 TPS App 升级包',defaultPath:path.join(programmer.folder(),'tps_app.tpsfw'),properties:['openFile'],filters:[{name:'TPS MCU App',extensions:['tpsfw']}]});
        if(selected.canceled)return {canceled:true};
        emit('log','正在验证App包并查询Mender服务器版本…\n');
        const prepared=await publisher.prepare(selected.filePaths[0]);
        emit('log',`开始上传 ${prepared.name}，App ${prepared.info.version}\n`);
        const result=await publisher.upload(prepared);
        emit('log',`已上传并核对：${result.name}\nArtifact ID ${result.artifact_id}\n部署任务 ${result.deployment_id} · ${result.deployment_status}\n全部兼容 TPS130：${result.deployment_devices.length} 台\n${result.deployment_devices.join('\n')}\n发布记录 ${result.folder}\n请在各机器上手动确认更新。\n`);return result;
      }finally{active=null;}
    }
    if(action==='deploy-existing') {
      noTask();active='deploy';
      try{
        const selected=await dialog.showOpenDialog(win,{title:'选择已上传包的 upload.json 发布记录',defaultPath:path.join(app.getPath('documents'),'TPS130-Published'),properties:['openFile'],filters:[{name:'Upload record',extensions:['json']}]});
        if(selected.canceled)return {canceled:true};
        return await publisher.resumeDeployment(selected.filePaths[0]);
      }finally{active=null;}
    }
    if(action==='publish-settings') {
      noTask();active='settings';
      try {
        const answer=await dialog.showMessageBox(win,{type:'question',title:'本机发布设置',buttons:['取消','使用未签名格式','配置TPS签名密钥'],defaultId:0,cancelId:0,
          message:'默认沿用现有TPS未签名格式。',detail:'发布设置仅保存在本机。管理员凭据按接入说明读取；如以后启用签名，选择TPS私钥和设备已信任公钥。'});
        if(answer.response===0)return false;
        const settings=publisher.settings();
        if(answer.response===2){
          const key=await dialog.showOpenDialog(win,{title:'选择TPS Artifact签名私钥',properties:['openFile']});if(key.canceled)return false;
          const pub=await dialog.showOpenDialog(win,{title:'选择设备已信任的TPS验签公钥',properties:['openFile']});if(pub.canceled)return false;
          settings.privateKey=key.filePaths[0];settings.publicKey=pub.filePaths[0];settings.mode='signed';
        }else{settings.mode='unsigned';delete settings.privateKey;delete settings.publicKey;}
        fs.mkdirSync(path.dirname(publisher.settingsFile),{recursive:true});fs.writeFileSync(publisher.settingsFile,JSON.stringify(settings,null,2));return true;
      }finally{active=null;}
    }
    if(action==='power-flash'||action==='power-verify'){
      noTask();const o=options(arg);active=action;
      try{return await powerProgrammer.flash(o,t=>emit('log',t),n=>emit('progress',n),action==='power-verify');}finally{active=null;}
    }
    if(action==='power-output'){const error=await shell.openPath(powerProgrammer.folder());if(error)throw Error(error);return true;}
    if(action==='flash'||action==='verify') {
      noTask();const o=options(arg);active=action;
      try{return await programmer.flash(o,t=>emit('log',t),n=>emit('progress',n),action==='verify');}finally{active=null;}
    }
    if(action==='rtt-connect') {
      noTask();const o=options(arg);programmer.verifyRuntime();active='rtt';
      const executable=programmer.runtime(o);
      if(!executable){active=null;throw Error('未找到 J-Link');}
      const dll=path.join(path.dirname(executable),'JLink_x64.dll');
      if(!fs.existsSync(dll)){active=null;throw Error('未找到 JLink_x64.dll');}
      rtt.start(bundled?path.join(payload,'tps-rtt-bridge.exe'):(process.env.PYTHON||'python'),
        bundled?[]:['-u',path.join(__dirname,'rtt_bridge.py')],{...process.env,JLINK_DLL:dll});
      try{const result=await rtt.call('connect',{serial:o.serial});rtt.connected=true;emit('rtt-state',true);return result;}
      catch(e){await rtt.close();active=null;throw e;}
    }
    if(action==='rtt-disconnect') {if(active&&active!=='rtt')throw Error('当前任务未完成');try{await rtt.close();}finally{active=null;}return true;}
    if(action==='rtt-command') {
      if(active!=='rtt'||!rtt.connected||!require('./devices/tps130/commands.js').valid(arg))throw Error('RTT 未连接或命令无效');
      return rtt.call('command',arg);
    }
    if(action==='output') {const error=await shell.openPath(programmer.folder());if(error)throw Error(error);return true;}
    if(action==='save-log') {
      if(typeof arg!=='string'||arg.length>2000000)throw Error('日志长度无效');
      const d=await dialog.showSaveDialog(win,{defaultPath:'TPS130-debug.log',filters:[{name:'Log',extensions:['log']}]});
      if(!d.canceled)fs.writeFileSync(d.filePath,arg,'utf8');return !d.canceled;
    }
    if(action==='save-recording'||action==='save-csv'){
      if(!arg||typeof arg.content!=='string'||Buffer.byteLength(arg.content)>20*1024*1024)throw Error('导出数据超限');
      const ext=action==='save-csv'?'csv':'json';
      const d=await dialog.showSaveDialog(win,{defaultPath:(arg.kind==='stability'?'TPS130-stability-report.':'TPS130-session.')+ext,filters:[{name:'TPS130 data',extensions:[ext]}]});
      if(!d.canceled)fs.writeFileSync(d.filePath,arg.content,'utf8');return !d.canceled;
    }
    if(action==='load-recording'){
      const d=await dialog.showOpenDialog(win,{title:'打开TPS130调试记录',properties:['openFile'],filters:[{name:'Session',extensions:['json']}]});
      if(d.canceled)return null;
      if(fs.statSync(d.filePaths[0]).size>20*1024*1024)throw Error('记录文件超过20MB');
      const value=JSON.parse(fs.readFileSync(d.filePaths[0],'utf8').replace(/^\uFEFF/,''));
      const core=require('./platform/core.js'),device=JSON.parse(fs.readFileSync(path.join(__dirname,'devices/tps130/manifest.json'),'utf8'));
      return core.validateRecording(value,device);
    }
    throw Error('未知操作');
  });
  app.whenReady().then(async()=>{
    const area=screen.getPrimaryDisplay().workAreaSize;
    win=new BrowserWindow({title:'TPS130 Studio',icon:path.join(__dirname,'assets/tps130.ico'),width:Math.min(1440,area.width-24),height:Math.min(940,area.height-40),minWidth:1000,minHeight:700,show:!process.argv.includes('--smoke-test'),
      backgroundColor:'#f4f7f8',webPreferences:{preload:path.join(__dirname,'preload.cjs'),contextIsolation:true,nodeIntegration:false,sandbox:true}});
    win.removeMenu();win.webContents.setWindowOpenHandler(()=>({action:'deny'}));
    win.webContents.on('will-navigate',e=>e.preventDefault());
    win.on('close',e=>{
      // User-requested unconditional close: no dialogs, network waits or busy
      // guards. Terminate only our own subprocesses, then exit all windows.
      e.preventDefault();stopChildren();app.exit(0);
    });
    await win.loadFile(path.join(__dirname,'index.html'));
    if(process.argv.includes('--smoke-test')) {
      try {
        await new Promise(r=>setTimeout(r,1000));
        const okay=await win.webContents.executeJavaScript(`document.querySelector('#status').textContent!=='正在加载' && document.querySelector('#rtt-info').disabled && !!document.querySelector('#upload') && !!document.querySelector('#publish-settings') && document.documentElement.scrollWidth<=document.documentElement.clientWidth`);
        if(!okay)throw Error('Renderer smoke test failed');
        const deploymentUi=await win.webContents.executeJavaScript(`(() => {if(!document.getElementById('deploy-existing')||!document.getElementById('upload').textContent.includes('创建更新任务'))return false;deploymentSuccess({name:'UI test only',deployment_status:'pending'});const ok=document.getElementById('success-dialog').open&&document.getElementById('success-detail').textContent.includes('手动开始');document.getElementById('success-close').click();return ok;})()`);
        if(!deploymentUi)throw Error('Deployment UI smoke test failed');
        const workbenchOkay=await win.webContents.executeJavaScript(`(() => {
          const pages=['overview','workbench','power','firmware','tests','logs','settings'];
          for(const page of pages){navigate(page);if(document.getElementById('page-'+page).hidden || document.documentElement.scrollWidth>document.documentElement.clientWidth+1)return false;}
          if(!powerAvailable||!document.getElementById('power-firmware').textContent.includes('STM32F030F4')||document.getElementById('power-flash').disabled)return false;
          connected=true;render();if(!document.getElementById('power-flash').disabled)return false;connected=false;render();
          navigate('workbench');
          ingest('RTT> app=0.1.1 target=0x9C01 hw=1 base=0x08004000 boot_min=0x00010000\\n');
          ingest('RTT> tick=123 running=0 tempL=36.25 tempR=36.5 pressL=12.3 pressR=12.4 free_heap=4000 log_drops=0\\n');
          if(document.getElementById('value-tempL').textContent!=='36.25'||document.getElementById('param-app').textContent!=='0.1.1')return false;
          document.getElementById('pid-tab').click();if(document.getElementById('pid-content').hidden)return false;
          if(document.getElementById('pid-group').options.length!==2)return false;
          document.getElementById('pid-pressure').click();if(document.getElementById('pid-group').options.length!==6)return false;
          document.getElementById('pid-heat').click();
          ingest('RTT> PID sample=1 target=3 profile=2 tick=123 kp=0.7 ki=0.3 kd=0 p=1 i=2 d=0 error=2 output=3 mapped=4 setpoint=40 measurement=38\\n');drawPid();
          if(!document.getElementById('pid-values').textContent.includes('40'))return false;
          connected=true;monitoring=true;render();document.getElementById('stability-start').click();
          if(!stability||stability.result!=='RUNNING'||!document.getElementById('pid-write').disabled)return false;
          ingest('RTT> tick=323 running=0 tempL=36.25 tempR=36.5 pressL=10 pressR=20 free_heap=4000 log_drops=0\\n');drawStability();
          if(document.getElementById('stability-value-pressL').textContent!=='75.01'||document.getElementById('stability-stat-pressR-mean').textContent!=='150.01'||stability.history.length!==1||stability.report().stats.pressL.unit!=='mmHg')return false;
          document.getElementById('stability-stop').click();if(stability.result!=='ABORTED'||document.getElementById('stability-export').disabled)return false;
          stability=null;stabilityNotified=false;connected=false;monitoring=false;pidHistory.clear();
          document.getElementById('identity-tab').click();
          replay={schema:1,device:'tps130',identity:{app:'recorded'},samples:[{time:1000,deviceTick:1,running:0,values:{tempL:22}}]};replayIndex=0;render();updateIdentity();draw();
          if(document.getElementById('value-tempL').textContent!=='22.00'||document.getElementById('data-mode').textContent.indexOf('REPLAY')<0)return false;
          replay=null;live.samples=[];live.identity={};decoder.reset();logText='';render();updateIdentity();draw();return true;
        })()`);
        if(!workbenchOkay)throw Error('Device workbench / replay / navigation failed');
        console.log('PASS: seven pages, power-board identity and RTT exclusion, protocol parsing and offline replay');
        const successOkay=await win.webContents.executeJavaScript(`(() => {showSuccess('烧录成功','界面测试预览；未连接探针或实际烧录。');const d=document.getElementById('success-dialog');const okay=d.open && document.getElementById('success-title').textContent==='烧录成功';document.getElementById('success-close').click();return okay && !d.open;})()`);
        if(!successOkay)throw Error('Success modal failed');
        console.log('PASS: success modal opens and closes');
        if(bundled){programmer.inspect();powerProgrammer.inspect();programmer.verifyRuntime();}
        await win.webContents.executeJavaScript(`document.getElementById('rtt-log').textContent='等待连接设备。';`);
        if(process.argv.includes('--preview-success'))await win.webContents.executeJavaScript(`showSuccess('烧录成功','版本 0.1.1\\n完整 Flash 回读 SHA256 一致。\\n界面预览；未实际烧录。')`);
        if(process.argv.includes('--preview-tests')){
          await win.webContents.executeJavaScript(`navigate('tests');window.scrollTo(0,0);`);
          await new Promise(resolve=>setTimeout(resolve,200));
        }
        const capture=await win.webContents.capturePage();
        fs.writeFileSync(process.env.TPS_SMOKE_OUTPUT||path.join(app.getPath('temp'),'tps-flasher-preview.png'),capture.toPNG());
        console.log('PASS: GUI, firmware hashes and runtime; no probe connection');app.quit();
      } catch(e){console.error(e);app.exit(1);}
    }
  }).catch(e=>{console.error(e);app.exit(1);});
  app.on('window-all-closed',()=>app.quit());
}
