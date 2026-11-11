(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.DebugPlatform=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
  'use strict';
  function validateDevice(d){
    if(d.schema!==1||d.platformApi!==1||!d.id||!Array.isArray(d.channels)||!Array.isArray(d.parameters))throw Error('设备适配包不兼容');
    const ids=new Set();for(const c of d.channels){if(!/^[a-zA-Z][\w]*$/.test(c.id)||ids.has(c.id)||!c.name||!c.unit)throw Error('无效测点定义');ids.add(c.id);}
    return d;
  }
  class LineDecoder {
    constructor(){this.buffer='';}
    feed(text){
      this.buffer+=text;const out=[];let i;
      while((i=this.buffer.indexOf('\n'))>=0){const line=this.buffer.slice(0,i).replace(/\r$/,'');this.buffer=this.buffer.slice(i+1);if(line.length<=2048)out.push(line);}
      if(this.buffer.length>2048)this.buffer='';return out;
    }
    reset(){this.buffer='';}
  }
  class TelemetryStore {
    constructor(capacity=18000){this.capacity=capacity;this.samples=[];this.identity={};this.resetPending=false;this.lastTick=null;}
    push(sample){
      if(this.lastTick!==null&&sample.deviceTick<this.lastTick)this.resetPending=true;
      const copy={...sample,values:{...sample.values},breakBefore:this.resetPending};this.resetPending=false;
      this.lastTick=sample.deviceTick;this.samples.push(copy);if(this.samples.length>this.capacity)this.samples.splice(0,this.samples.length-this.capacity);return copy;
    }
    disconnect(){this.resetPending=true;}
    latest(){return this.samples[this.samples.length-1]||null;}
    stale(now=Date.now()){return !this.latest()||now-this.latest().time>1500;}
  }
  function validateRecording(value,device){
    if(!value||value.schema!==1||value.device!==device.id||!Array.isArray(value.samples)||value.samples.length>50000)throw Error('记录格式、设备型号或数据量不匹配');
    let previous=-Infinity;
    for(const s of value.samples){
      if(!Number.isFinite(s.time)||s.time<0||s.time>8640000000000000||s.time<previous||!Number.isFinite(s.deviceTick)||s.deviceTick<0||![0,1].includes(s.running)||!s.values)throw Error('记录中的时间或状态无效');
      previous=s.time;
      for(const c of device.channels)if(c.id in s.values&&!Number.isFinite(s.values[c.id]))throw Error('记录包含无效测点');
    }return value;
  }
  function csv(samples,channels){
    const lines=[['host_time_iso','device_tick_ms','running',...channels.map(c=>c.id+'_'+c.unit)].join(',')];
    for(const s of samples)lines.push([new Date(s.time).toISOString(),s.deviceTick,s.running,...channels.map(c=>s.values[c.id]??'')].join(','));
    return '\uFEFF'+lines.join('\r\n');
  }
  return {validateDevice,LineDecoder,TelemetryStore,validateRecording,csv};
});
