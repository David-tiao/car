(() => {
 'use strict';

 if(location.pathname.replace(/\/+$/,'')!=='/test')return;

 let busy=false;
 const label=document.createElement('div');
 label.style.cssText=
   'padding:6px 10px;color:#9cf4df;font:12px/1.5 system-ui';
 label.textContent='TEST 路线：等待规划结果';

 const show=()=>{
   (document.getElementById('carModeSwitch')||document.body)
     .append(label);
 };

 if(document.readyState==='loading')
   document.addEventListener('DOMContentLoaded',show,{once:true});
 else show();

 async function send(){
   if(busy||!window.Sim2RealCore?.getRoute)return;

   const raw=window.Sim2RealCore.getRoute();

   if(!Array.isArray(raw)||raw.length<2){
     label.textContent='TEST 路线：规划不可达';
     return;
   }

   const pts=[];

   for(const q of raw){
     const p=[Number(q.x),Number(q.y)];

     if(!p.every(v=>
       Number.isFinite(v)&&v>=0&&v<=2.4
     ))return;

     if(pts.length&&Math.hypot(
       p[0]-pts.at(-1)[0],
       p[1]-pts.at(-1)[1]
     )<.002)continue;

     // 仅合并共线航点，保留转弯位置。
     while(pts.length>=2){
       const a=pts.at(-2),b=pts.at(-1);
       const vx=b[0]-a[0],vy=b[1]-a[1];
       const wx=p[0]-b[0],wy=p[1]-b[1];

       if(Math.abs(vx*wy-vy*wx)<.00015
          &&vx*wx+vy*wy>0){
         pts.pop();
       }else break;
     }
     pts.push(p);
   }

   if(pts.length<2||pts.length>250){
     label.textContent='TEST 路线：航点数量不合法';
     return;
   }

   busy=true;

   try{
     const res=await fetch('/api/test/reference_path',{
       method:'POST',
       headers:{'Content-Type':'application/json'},
       body:JSON.stringify({points:pts}),
       cache:'no-store'
     });

     if(!res.ok)throw Error('HTTP '+res.status);

     const r=await res.json();

     label.textContent=
       `TEST 路线：已连接 ${pts.length} 个转折航点 / seq ${r.seq}`;

   }catch(e){
     label.textContent='TEST 路线同步失败：'+e.message;
   }finally{
     busy=false;
   }
 }

 setInterval(send,1000);
})();
