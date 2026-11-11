'use strict';
const {spawn}=require('./processes.cjs');
class Rtt {
  constructor(notify) {this.notify=notify;this.child=null;this.pending=new Map();this.counter=0;this.connected=false;}
  start(exe,args,env) {
    if(this.child) throw Error('RTT 进程仍在运行');
    const child=spawn(exe,args,{env,windowsHide:true,stdio:['pipe','pipe','pipe']});this.child=child;
    let buffer='';child.stdout.setEncoding('utf8');
    child.stdout.on('data',text=>{
      buffer+=text;if(buffer.length>1000000){child.kill();return;}
      let i;while((i=buffer.indexOf('\n'))>=0){
        const line=buffer.slice(0,i);buffer=buffer.slice(i+1);let msg;
        try{msg=JSON.parse(line);}catch{this.notify('rtt-log',line+'\n');continue;}
        if(msg.event==='log') {this.notify('rtt-log',msg.text);continue;}
        if(msg.event==='disconnected') {this.fail(Error(msg.error));child.kill();continue;}
        const job=this.pending.get(msg.id);if(!job) continue;
        clearTimeout(job.timer);this.pending.delete(msg.id);msg.error?job.reject(Error(msg.error)):job.resolve(msg.result);
      }
    });
    child.stderr.on('data',t=>this.notify('rtt-log',t.toString()));
    child.once('error',e=>this.fail(e));
    child.once('close',()=>{if(this.child===child){this.child=null;this.fail(Error('RTT 已断开'));}});
  }
  fail(error) {
    this.connected=false;
    for(const job of this.pending.values()){clearTimeout(job.timer);job.reject(error);}this.pending.clear();
    this.notify('rtt-state',false);this.notify('rtt-log',error.message+'\n');
  }
  call(method,...args) {
    return new Promise((resolve,reject)=>{
      if(!this.child?.stdin.writable) {reject(Error('RTT 进程未启动'));return;}
      const id=++this.counter;
      const timer=setTimeout(()=>{this.pending.delete(id);reject(Error('RTT 响应超时'));this.child?.kill();},10000);
      this.pending.set(id,{resolve,reject,timer});
      this.child.stdin.write(JSON.stringify({id,method,args})+'\n',e=>{if(e){clearTimeout(timer);this.pending.delete(id);reject(e);}});
    });
  }
  async close() {
    const child=this.child;
    try{if(this.connected) await this.call('disconnect');}
    finally {
      this.connected=false;
      if(child) await new Promise(resolve=>{const timer=setTimeout(()=>{child.kill();resolve();},1000);child.once('close',()=>{clearTimeout(timer);resolve();});child.stdin.end();});
      if(this.child===child)this.child=null;this.notify('rtt-state',false);
    }
  }
}
module.exports={Rtt};
