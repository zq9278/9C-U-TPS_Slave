'use strict';
const cp=require('node:child_process');
const children=new Set();
function spawn(command,args,options){
  const child=cp.spawn(command,args,options);children.add(child);
  child.once('close',()=>children.delete(child));child.once('error',()=>children.delete(child));
  return child;
}
function stopChildren(){
  for(const child of children){
    if(!child.pid)continue;
    if(process.platform==='win32'){
      // Only process trees created by this app; includes PyInstaller workers.
      cp.spawnSync('taskkill.exe',['/PID',String(child.pid),'/T','/F'],{windowsHide:true,stdio:'ignore',timeout:1500});
    }else child.kill('SIGKILL');
  }
  children.clear();
}
module.exports={spawn,stopChildren};
