(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.DeviceAdapter=api;})(globalThis,function(){
  'use strict';
  function decode(line,now=Date.now()){
    if(!line.startsWith('RTT> '))return null;
    const fields={};for(const m of line.matchAll(/([A-Za-z_][\w]*)=([^\s]+)/g))fields[m[1]]=m[2];
    if(line.startsWith('RTT> PID ')){
      const numeric={};for(const [k,v] of Object.entries(fields))if(Number.isFinite(Number(v)))numeric[k]=Number(v);
      if('ack' in numeric&&[0,1].includes(numeric.ok))return {type:'pid-ack',values:numeric,reason:fields.reason};
      const required=['target','profile','tick','kp','ki','kd','p','i','d','error','output','mapped','setpoint','measurement'];
      if(numeric.sample===1&&required.every(k=>k in numeric)&&Number.isInteger(numeric.target)&&numeric.target>=0&&numeric.target<=4&&
          (numeric.target<3?[0,1].includes(numeric.profile):numeric.profile===2))return {type:'pid-sample',values:numeric,time:now};
      return null;
    }
    if('tick' in fields){const s={time:now,deviceTick:Number(fields.tick),running:Number(fields.running),values:{}};
      if(!Number.isFinite(s.deviceTick)||![0,1].includes(s.running))return null;
      for(const id of ['tempL','tempR','pressL','pressR'])if(fields[id]!==undefined&&Number.isFinite(Number(fields[id])))s.values[id]=Number(fields[id]);
      if(!Object.keys(s.values).length)return null;
      for(const id of ['free_heap','log_drops'])if(fields[id]!==undefined&&Number.isFinite(Number(fields[id])))s[id]=Number(fields[id]);
      return {type:'sample',sample:s};
    }
    if(fields.app&&fields.target&&fields.hw)return {type:'identity',values:fields};return null;
  }
  return {decode};
});
