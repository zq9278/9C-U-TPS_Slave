const {contextBridge,ipcRenderer}=require('electron');
contextBridge.exposeInMainWorld('tps',{
  invoke:(action,arg)=>ipcRenderer.invoke('action',action,arg),
  onEvent:callback=>ipcRenderer.on('event',(_,name,value)=>callback(name,value))
});
