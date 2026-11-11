(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.Stability=api;})(globalThis,function(){
  'use strict';
  class Run {
    constructor(config,channels,identity,now=Date.now()){
      if(!Number.isFinite(config.durationMs)||config.durationMs<1000||config.durationMs>86400000||!Number.isFinite(config.gapMs)||config.gapMs<1000||config.gapMs>60000)throw Error('测试时长应为 1 秒至 24 小时，超时阈值为 1 至 60 秒');
      for(const c of channels){const r=config.limits[c.id];if(!r||r.length!==2||!r.every(Number.isFinite)||r[0]>=r[1])throw Error('请检查温压上下限');}
      this.config=JSON.parse(JSON.stringify(config));this.channels=channels;this.identity={...identity};this.start=now;this.end=null;this.result='RUNNING';
      this.samples=0;this.last=null;this.lastRecord=-Infinity;this.trace=[];this.events=[];this.eventCount=0;this.active=new Set();this.stats={};this.maxGapMs=0;
      for(const c of channels)this.stats[c.id]={count:0,min:Infinity,max:-Infinity,mean:0,m2:0,violations:0};
    }
    flag(key,message,time){if(this.active.has(key))return;this.active.add(key);this.eventCount++;if(this.events.length<1000)this.events.push({time,key,message});}
    tick(now=Date.now()){
      if(this.result!=='RUNNING')return;
      const until=Math.min(now,this.start+this.config.durationMs),gap=until-(this.last?.time??this.start);
      this.maxGapMs=Math.max(this.maxGapMs,gap);
      if(gap>this.config.gapMs)this.flag('timeout','未收到完整实时快照，通信或采集超时',until);
      if(now>=this.start+this.config.durationMs)this.finish(false,this.start+this.config.durationMs);
    }
    push(sample){
      if(this.result!=='RUNNING')return;
      this.tick(sample.time);if(this.result!=='RUNNING')return;
      if(this.last&&sample.deviceTick<this.last.deviceTick&&!(this.last.deviceTick>0xf0000000&&sample.deviceTick<0x0fffffff))this.flag('restart','设备 Tick 回退，疑似重启',sample.time);
      if(this.last&&sample.deviceTick===this.last.deviceTick){this.flag('frozen','设备 Tick 未前进',sample.time);return;}
      if(this.channels.some(c=>!Number.isFinite(sample.values[c.id]))){this.flag('incomplete','快照缺少测点或含非有限值',sample.time);return;}
      this.active.delete('timeout');this.active.delete('frozen');this.active.delete('incomplete');
      if(this.last&&sample.log_drops>this.last.log_drops)this.flag('drops','固件 RTT 日志出现丢弃',sample.time);
      this.last=sample;this.samples++;
      for(const c of this.channels){const v=sample.values[c.id],s=this.stats[c.id],[lo,hi]=this.config.limits[c.id];s.count++;s.min=Math.min(s.min,v);s.max=Math.max(s.max,v);const d=v-s.mean;s.mean+=d/s.count;s.m2+=d*(v-s.mean);
        if(v<lo||v>hi){s.violations++;this.flag('limit-'+c.id,c.name+'越限：'+v+' '+c.unit,sample.time);}else this.active.delete('limit-'+c.id);
      }
      if(sample.time-this.lastRecord>=1000){this.trace.push({...sample,values:{...sample.values}});this.lastRecord=sample.time;}
    }
    disconnect(now=Date.now()){if(this.result==='RUNNING')this.flag('disconnect','RTT 连接中断',now);}
    finish(aborted=false,now=Date.now()){
      if(this.result!=='RUNNING')return;
      this.end=now;this.result=aborted?'ABORTED':this.eventCount||!this.samples?'FAIL':'PASS';
    }
    report(now=Date.now()){
      const stats={};for(const c of this.channels){const s=this.stats[c.id];stats[c.id]={unit:c.unit,count:s.count,min:s.count?s.min:null,max:s.count?s.max:null,mean:s.count?s.mean:null,stddev:s.count?Math.sqrt(s.m2/s.count):null,violations:s.violations};}
      return {schema:1,type:'stability',device:'tps130',identity:this.identity,source:'RTT status snapshots; passive observation; no automatic treatment',config:this.config,start:this.start,end:this.end,result:this.result,elapsedMs:(this.end??now)-this.start,samples:this.samples,maxGapMs:this.maxGapMs,stats,eventCount:this.eventCount,events:this.events,eventsTruncated:this.eventCount>this.events.length,trace:this.trace,traceIntervalMs:1000};
    }
  }
  return {Run};
});
