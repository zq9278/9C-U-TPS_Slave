(function(root){
  'use strict';
  root.drawTelemetryChart=function(canvas,samples,channels,end,windowMs,visible){
    const rect=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1;
    if(!rect.width||!rect.height)return;
    canvas.width=Math.round(rect.width*ratio);canvas.height=Math.round(rect.height*ratio);
    const ctx=canvas.getContext('2d');ctx.scale(ratio,ratio);
    const w=rect.width,h=rect.height,left=46,right=16,top=14,bottom=29;
    ctx.clearRect(0,0,w,h);ctx.font='11px Segoe UI';
    const start=end-windowMs,data=samples.filter(s=>s.time>=start&&s.time<=end);
    const enabled=channels.filter(c=>visible.has(c.id));
    const values=data.flatMap(s=>enabled.map(c=>s.values[c.id]).filter(Number.isFinite));
    let min=values.length?values.reduce((a,b)=>Math.min(a,b),Infinity):0,max=values.length?values.reduce((a,b)=>Math.max(a,b),-Infinity):1;
    const pad=Math.max((max-min)*.15,.5);min-=pad;max+=pad;
    const x=t=>left+(t-start)/windowMs*(w-left-right),y=v=>h-bottom-(v-min)/(max-min)*(h-top-bottom);
    ctx.lineWidth=1;
    for(let i=0;i<5;i++){const v=min+(max-min)*i/4,py=y(v);ctx.strokeStyle='#e9eff1';ctx.beginPath();ctx.moveTo(left,py);ctx.lineTo(w-right,py);ctx.stroke();ctx.fillStyle='#89979b';ctx.textAlign='right';ctx.fillText(v.toFixed(1),left-8,py+4);}
    for(let i=0;i<5;i++){const t=start+windowMs*i/4;ctx.textAlign=i===0?'left':i===4?'right':'center';ctx.fillStyle='#89979b';ctx.fillText(new Date(t).toLocaleTimeString('zh-CN',{hour12:false}),x(t),h-8);}
    for(const c of enabled){ctx.strokeStyle=c.color;ctx.lineWidth=2;ctx.beginPath();let last=null;
      for(const s of data){const v=s.values[c.id];if(!Number.isFinite(v)){last=null;continue;}
        if(!last||s.breakBefore||s.time-last>1500)ctx.moveTo(x(s.time),y(v));else ctx.lineTo(x(s.time),y(v));last=s.time;
      }ctx.stroke();}
    if(!values.length){ctx.fillStyle='#8b9ca3';ctx.textAlign='center';ctx.fillText('等待设备数据 · 不显示模拟曲线',w/2,h/2);}
  };
})(globalThis);
