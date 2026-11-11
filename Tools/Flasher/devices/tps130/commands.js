(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PidCommands=api;})(globalThis,function(){
  'use strict';
  function valid(command){
    if(['help','info','status'].includes(command))return true;
    if(typeof command!=='string'||command.length>120||!/^pid (get|set|stream) [0-9]+(?: [0-9.eE+-]+)+$/.test(command))return false;
    const t=command.split(' '),v=t.slice(2).map(Number),[req,target,profile]=v;
    if(!v.every(Number.isFinite)||!Number.isInteger(req)||req<1||req>1000000)return false;
    if(t[1]==='stream')return t.length===4&&[0,1].includes(target);
    if(!Number.isInteger(target)||target<0||target>4||!(target<3?[0,1].includes(profile):profile===2))return false;
    return t[1]==='get'?t.length===5:t.length===8&&v.slice(3).every(x=>x>=0&&x<=1000);
  }
  return {valid};
});
