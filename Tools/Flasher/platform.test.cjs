const {test}=require('node:test'),assert=require('node:assert/strict');
const P=require('./platform/core.js'),A=require('./devices/tps130/adapter.js');
const device=P.validateDevice(require('./devices/tps130/manifest.json'));
test('device package has explicit read-only capabilities and unique channels',()=>{
  assert.equal(device.id,'tps130');assert.equal(device.capabilities.parameterWrite,false);assert.equal(device.channels.length,4);
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
