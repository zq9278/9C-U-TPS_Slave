'use strict';
const {app,BrowserWindow,ipcMain,dialog,shell,screen}=require('electron');
const fs=require('node:fs'),path=require('node:path');
const {ROOT,Programmer,findJLink,validateOptions}=require('./backend.cjs');
const {Rtt}=require('./rtt.cjs');
if(!app.requestSingleInstanceLock()) {app.quit();} else {
  app.setName('TPS100-Flasher');
  const bundled=fs.existsSync(path.join(process.resourcesPath,'payload/bundle.json'));
  const payload=bundled?path.join(process.resourcesPath,'payload'):null;
  const programmer=new Programmer({payload});
  let win, active=null, chosenJLink=findJLink();
  const emit=(event,data)=>{if(win&&!win.isDestroyed())win.webContents.send('event',event,data);};
  const rtt=new Rtt((event,data)=>{if(event==='rtt-state'&&!data&&active==='rtt'&&!rtt.child)active=null;emit(event,data);});
  function noTask() {if(active||programmer.busy||rtt.child)throw Error('请先结束当前任务或断开 RTT');}
  function options(value) {validateOptions(value);return {...value,jlink:chosenJLink};}
  ipcMain.handle('action',async(event,action,arg)=>{
    if(event.sender!==win.webContents)throw Error('Invalid sender');
    if(action==='info') {
      let firmware=null,error='';try{firmware=programmer.inspect();}catch(e){error=e.message;}
      return {bundled,root:bundled?process.resourcesPath:ROOT,jlink:programmer.runtime({jlink:chosenJLink}),firmware,error,version:app.getVersion()};
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
    if(action==='flash'||action==='verify') {
      noTask();const o=options(arg);const info=programmer.inspect();
      const answer=await dialog.showMessageBox(win,{type:'warning',buttons:['取消',action==='flash'?'确认烧录':'确认校验'],defaultId:0,cancelId:0,
        title:action==='flash'?'烧录完整固件':'读取并校验 Flash',
        message:action==='flash'?`确认将 ${info.identity.version} 的 Boot + App + 元数据写入 STM32G070RB？`:'校验会暂停 CPU，并在结束后请求复位运行。',
        detail:'请先停止治疗并确认连接的是 TPS Slave。烧录会覆盖现有固件；请关闭其他 J-Link 调试会话。'});
      if(answer.response!==1)return {canceled:true};
      noTask();active=action;
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
    if(action==='rtt-disconnect') {if(active&&active!=='rtt')throw Error('当前正在烧录或构建');try{await rtt.close();}finally{active=null;}return true;}
    if(action==='rtt-command') {
      if(active!=='rtt'||!rtt.connected||!['help','info','status'].includes(arg))throw Error('RTT 未连接或命令无效');
      return rtt.call('command',arg);
    }
    if(action==='output') {const error=await shell.openPath(programmer.folder());if(error)throw Error(error);return true;}
    if(action==='save-log') {
      if(typeof arg!=='string'||arg.length>2000000)throw Error('日志长度无效');
      const d=await dialog.showSaveDialog(win,{defaultPath:'TPS100-debug.log',filters:[{name:'Log',extensions:['log']}]});
      if(!d.canceled)fs.writeFileSync(d.filePath,arg,'utf8');return !d.canceled;
    }
    throw Error('未知操作');
  });
  app.whenReady().then(async()=>{
    const area=screen.getPrimaryDisplay().workAreaSize;
    win=new BrowserWindow({width:Math.min(1100,area.width-24),height:Math.min(940,area.height-40),minWidth:Math.min(880,area.width-24),minHeight:Math.min(720,area.height-40),show:!process.argv.includes('--smoke-test'),
      backgroundColor:'#f4f6f9',webPreferences:{preload:path.join(__dirname,'preload.cjs'),contextIsolation:true,nodeIntegration:false,sandbox:true}});
    win.removeMenu();win.webContents.setWindowOpenHandler(()=>({action:'deny'}));
    win.webContents.on('will-navigate',e=>e.preventDefault());
    win.on('close',e=>{
      if(active==='flash'||active==='verify'||active==='build'){e.preventDefault();emit('log','请等待当前操作完成再关闭。\n');}
      else if(rtt.child){e.preventDefault();rtt.close().finally(()=>{active=null;win.close();});}
    });
    await win.loadFile(path.join(__dirname,'index.html'));
    if(process.argv.includes('--smoke-test')) {
      try {
        await new Promise(r=>setTimeout(r,1000));
        const okay=await win.webContents.executeJavaScript(`document.querySelector('#status').textContent!=='正在加载' && document.querySelector('#rtt-info').disabled && document.documentElement.scrollWidth<=document.documentElement.clientWidth`);
        if(!okay)throw Error('Renderer smoke test failed');
        if(bundled){programmer.inspect();programmer.verifyRuntime();}
        await win.webContents.executeJavaScript(`document.getElementById('rtt-log').textContent='[APP] RTT 日志预览\\nRTT> app=0.1.1 target=0x9C01 hw=1\\n';`);
        const capture=await win.webContents.capturePage();
        fs.writeFileSync(process.env.TPS_SMOKE_OUTPUT||path.join(app.getPath('temp'),'tps-flasher-preview.png'),capture.toPNG());
        console.log('PASS: GUI, firmware hashes and runtime; no probe connection');app.quit();
      } catch(e){console.error(e);app.exit(1);}
    }
  }).catch(e=>{console.error(e);app.exit(1);});
  app.on('window-all-closed',()=>app.quit());
}
