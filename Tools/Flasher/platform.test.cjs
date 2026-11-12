const {test}=require('node:test'),assert=require('node:assert/strict');
const P=require('./platform/core.js'),A=require('./devices/tps130/adapter.js');
const device=P.validateDevice(require('./devices/tps130/manifest.json'));
const {Run}=require('./platform/stability.js'),{valid}=require('./devices/tps130/commands.js');
test('PID command validation and sample parser reject malformed values',()=>{
  for(const cmd of ['pid get 1 0 0','pid set 2 4 2 0.7 0.3 0','pid stream 3 1'])assert.ok(valid(cmd));
  for(const cmd of ['pid set 1 0 2 1 1 1','pid set 1 3 2 NaN 0 0','pid stream 1 2','pid get 0 0 0','pid get 1 0.5 0','pid get 1 0 0\nreset'])assert.equal(valid(cmd),false);
  assert.equal(A.decode('RTT> PID ack=2 ok=1 target=4 profile=2 kp=0.7 ki=0.3 kd=0').type,'pid-ack');
  assert.equal(A.decode('RTT> PID sample=1 target=3 profile=2 tick=1 kp=1 ki=1 kd=0 p=1 i=1 d=0 error=2 output=3 mapped=4 setpoint=40 measurement=38').type,'pid-sample');
  assert.equal(A.decode('RTT> PID sample=1 target=3 tick=1'),null);
});
const config={durationMs:6000,gapMs:1500,limits:Object.fromEntries(device.channels.map(c=>[c.id,[-5,60]]))};
test('stability pressure conversion consistently drives limits, statistics and report trace in mmHg',()=>{
  const S=require('./platform/stability.js');
  const channels=S.channelsFor(device.channels);
  const original={time:200,deviceTick:200,running:1,log_drops:0,values:{tempL:36,tempR:37,pressL:10,pressR:20}};
  const converted=S.convertSample(original,channels);
  assert.equal(original.values.pressL,10);assert.equal(converted.values.tempL,36);assert.ok(Math.abs(converted.values.pressL-75.006168270417)<1e-10);
  const limits=Object.fromEntries(channels.map(c=>[c.id,c.group==='pressure'?[0,100]:[0,60]]));
  const run=new Run({...config,limits},channels,{},0);run.push(converted);run.finish(true,400);
  const report=run.report();assert.equal(report.schema,2);assert.equal(report.stats.pressL.unit,'mmHg');assert.equal(report.stats.pressL.violations,0);assert.equal(report.stats.pressR.violations,1);
  assert.equal(report.stats.pressL.mean,converted.values.pressL);assert.equal(report.trace[0].values.pressR,converted.values.pressR);assert.equal(run.history[0].values.pressL,converted.values.pressL);
  assert.deepEqual(report.channels.find(c=>c.id==='pressL'),{id:'pressL',unit:'mmHg',sourceUnit:'kPa',scale:S.KPA_TO_MMHG});
  assert.deepEqual(S.channelsFor(channels).map(c=>c.scale),[1,1,1,1]);
});
const sample=(time,tick=time,value=30)=>({time,deviceTick:tick,running:1,log_drops:0,values:Object.fromEntries(device.channels.map(c=>[c.id,value]))});
test('stability passes continuous observations with correct statistics and bounded trace',()=>{
  const r=new Run(config,device.channels,{app:'0.1.2'},0);
  for(let t=200;t<6000;t+=200)r.push(sample(t,t,t/200));r.tick(6000);
  const report=r.report();assert.equal(report.result,'PASS');assert.equal(report.samples,29);assert.equal(report.stats.tempL.mean,15);assert.equal(report.stats.tempL.min,1);assert.equal(report.stats.tempL.max,29);assert.ok(report.trace.length<=6);
});
test('stability records limits, timeout, restart, disconnect and manual abort',()=>{
  const r=new Run(config,device.channels,{},0);r.push(sample(200,1000,70));r.push(sample(400,1));r.disconnect(500);r.tick(3000);r.tick(6000);
  assert.equal(r.result,'FAIL');for(const key of ['limit-tempL','restart','disconnect','timeout'])assert.ok(r.events.some(e=>e.key===key));
  const aborted=new Run(config,device.channels,{},0);aborted.finish(true,1000);assert.equal(aborted.result,'ABORTED');
  const empty=new Run(config,device.channels,{},0);empty.tick(6000);assert.equal(empty.result,'FAIL');
  assert.throws(()=>new Run({...config,durationMs:Infinity},device.channels,{},0));
});
test('device package has explicit PID capabilities and unique channels',()=>{
  assert.equal(device.id,'tps130');assert.equal(device.capabilities.parameterWrite,true);assert.equal(device.channels.length,4);
  assert.throws(()=>P.validateDevice({...device,channels:[device.channels[0],device.channels[0]]}));
});
test('RTT parser supports split chunks, actual identity and invalid data rejection',()=>{
  const p=new P.LineDecoder();assert.deepEqual(p.feed('RTT> ti'),[]);
  const lines=p.feed('ck=123 running=1 tempL=36.20 tempR=37 pressL=12 pressR=13 free_heap=4000 log_drops=2\r\nRTT> app=0.1.1 target=0x9C01 hw=1 base=0x08004000 boot_min=0x00010000\n');
  const e=A.decode(lines[0],1000);assert.equal(e.sample.values.tempL,36.2);assert.equal(e.sample.time,1000);assert.equal(e.sample.free_heap,4000);
  assert.equal(A.decode(lines[1]).values.app,'0.1.1');assert.equal(A.decode('ordinary text'),null);
  assert.equal(A.decode('RTT> tick=bad running=1 tempL=2'),null);
  assert.equal(A.decode('RTT> tick=12 running=1 tempL=nan'),null);
  assert.deepEqual(p.feed('x'.repeat(3000)),[]);assert.equal(p.buffer,'');
});
test('bounded history, stale data and restart/disconnect gaps',()=>{
  const s=new P.TelemetryStore(2);s.push({time:1000,deviceTick:100,running:0,values:{tempL:3}});
  assert.equal(s.stale(1100),false);assert.equal(s.stale(3000),true);
  s.push({time:1100,deviceTick:200,running:0,values:{}});s.disconnect();s.push({time:1200,deviceTick:300,running:0,values:{}});
  assert.equal(s.samples.length,2);assert.equal(s.latest().breakBefore,true);
  s.push({time:1300,deviceTick:1,running:0,values:{}});assert.equal(s.latest().breakBefore,true);
});
test('recording validation forbids wrong product, unsafe times and nonfinite values; CSV preserves units',()=>{
  const rec={schema:1,device:'tps130',samples:[{time:1000,deviceTick:10,running:1,values:{tempL:36.5}}]};
  assert.equal(P.validateRecording(rec,device),rec);assert.throws(()=>P.validateRecording({...rec,device:'other'},device));
  assert.throws(()=>P.validateRecording({...rec,samples:[{...rec.samples[0],time:Infinity}]},device));
  assert.throws(()=>P.validateRecording({...rec,samples:[{...rec.samples[0],values:{tempL:NaN}}]},device));
  const csv=P.csv(rec.samples,device.channels);assert.ok(csv.includes('tempL_°C'));assert.ok(csv.includes('36.5'));
});
