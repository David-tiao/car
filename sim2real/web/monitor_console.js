/* V13 Sim2Real operator console. Training commands are intentionally browser-only.
   No network endpoint in this module sends control to a robot. */
(()=>{
'use strict';
const $=id=>document.getElementById(id);
const core=window.Sim2RealCore;
if(!core){console.error('Sim2RealCore not found');return;}
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const format=(n,d=2)=>Number.isFinite(n)?Number(n).toFixed(d):'—';
const wrap=a=>Math.atan2(Math.sin(a),Math.cos(a));
const deg=Math.PI/180;
const manual={enabled:false,pose:null,route:[],routeIndex:0,heading:0,goal:null,grip:false,extension:0,keys:new Set(),lastFrame:0,active:false,halted:true};
const history=[];
const download=(name,content,type)=>{const blob=new Blob([content],{type});const url=URL.createObjectURL(blob);const a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
const serverLooksReplay=src=>/TEST|REPLAY|OFFLINE|LOG|CSV|GROWING/i.test(src||'');
const markStatus=(msg,warn=false)=>{$('manualState').textContent=msg;$('manualState').style.color=warn?'#ffb6a6':'#7df2d5';};
const feedback=msg=>{const b=$('manualGoal');b.textContent=msg;};
function fieldDraw(){
 const svg=$('fieldSvg'), old=$('manualTrainingOverlay');if(old)old.remove();
 if(!manual.enabled||!manual.pose)return;
 const g=document.createElementNS('http://www.w3.org/2000/svg','g');g.id='manualTrainingOverlay';g.setAttribute('pointer-events','none');
 const parent=$('robotLayer');parent.parentNode.insertBefore(g,parent);
 const pt=p=>[60+p.x*680/2.4,740-p.y*680/2.4];
 if(manual.route.length>1){const p=document.createElementNS('http://www.w3.org/2000/svg','path');p.setAttribute('d',manual.route.map((q,i)=>`${i?'L':'M'}${pt(q)[0].toFixed(1)} ${pt(q)[1].toFixed(1)}`).join(' '));p.setAttribute('fill','none');p.setAttribute('stroke','#b18cff');p.setAttribute('stroke-dasharray','6 6');p.setAttribute('stroke-width','4');p.setAttribute('opacity','.95');g.append(p)}
 if(manual.goal){const c=document.createElementNS('http://www.w3.org/2000/svg','circle');const [x,y]=pt(manual.goal);c.setAttribute('cx',x);c.setAttribute('cy',y);c.setAttribute('r','11');c.setAttribute('fill','none');c.setAttribute('stroke','#d4a8ff');c.setAttribute('stroke-width','3');g.append(c)}
}
function setPose(p){manual.pose={x:p.x,y:p.y,yaw:wrap(p.yaw)};core.manualPose(manual.pose,manual.extension);$('manualPose').textContent=`${format(p.x,2)}, ${format(p.y,2)} m / ${format(p.yaw/deg,0)}°`;}
function safeMove(to){
 if(!core.isSafe(to.x,to.y))return false;
 // Sample entire segment at <=10 mm to avoid cutting across a blocked cell.
 const dx=to.x-manual.pose.x,dy=to.y-manual.pose.y,steps=Math.max(1,Math.ceil(Math.hypot(dx,dy)/.01));
 for(let k=1;k<=steps;k++)if(!core.isSafe(manual.pose.x+dx*k/steps,manual.pose.y+dy*k/steps))return false;
 setPose(to);return true;
}
function clearRoute(){manual.route=[];manual.routeIndex=0;manual.goal=null;fieldDraw()}
function enter(){
 if(core.getMode()!=='simulation'){markStatus('只有模拟模式可以启用人工训练',true);return}
 core.cancelMission();manual.enabled=true;manual.halted=true;manual.keys.clear();manual.grip=false;manual.extension=0;
 window.Sim2RealManualGripOpen=false;const pos=core.getSimPose();setPose({x:pos.x,y:pos.y,yaw:pos.yaw||0});clearRoute();
 $('manualEnable').textContent='人工训练已启用';$('manualEnable').classList.add('ops-enabled');markStatus('就绪 · 点击地图选目标');
 $('manualGripState').textContent='闭合 · 已收回';document.body.classList.add('manual-training-active');
 feedback('单击场地设定目标');
}
function stop(message='已停车'){manual.halted=true;manual.keys.clear();clearRoute();if(manual.enabled)markStatus(message);}
function exit(){stop();core.manualReset();window.Sim2RealManualGripOpen=false;manual.enabled=false;document.body.classList.remove('manual-training-active');$('manualEnable').textContent='启用人工模拟';$('manualEnable').classList.remove('ops-enabled');markStatus('未启用');}
function setGoal(target){
 if(!manual.enabled||core.getMode()!=='simulation')return;
 if(!core.isSafe(target.x,target.y)){feedback('目标处车体会碰撞/越界，已拒绝');return}
 const result=core.manualPlan(manual.pose,target);
 if(!result||!result.path?.length){feedback('目标不可达：道路或障碍格阻断');return}
 manual.route=result.path;manual.routeIndex=1;manual.goal={...target};manual.halted=false;manual.keys.clear();
 markStatus('自动前往点击点 · 仿真');feedback(`目标 ${format(target.x,2)}, ${format(target.y,2)} m / 路径 ${format(result.length,2)} m`);fieldDraw();
}
function handleMapCapture(evt){
 if(!manual.enabled)return;
 evt.preventDefault();evt.stopImmediatePropagation();
 if(core.getMode()!=='simulation'){exit();return}
 const p=core.mapCoordinate(evt);if(p&&p.x>=0&&p.x<=2.4&&p.y>=0&&p.y<=2.4)setGoal(p);
}
function gripToggle(){if(!manual.enabled)return;manual.grip=!manual.grip;window.Sim2RealManualGripOpen=manual.grip;core.manualPose(manual.pose,manual.extension);gripStatus()}
function armToggle(){if(!manual.enabled)return;manual.extension=manual.extension>.01?0:.09;core.manualPose(manual.pose,manual.extension);gripStatus()}
function gripStatus(){$('manualGripState').textContent=(manual.grip?'打开':'闭合')+' · '+(manual.extension>.01?'伸出 '+Math.round(manual.extension*1000)+'mm':'已收回')}
function turn(degrees){if(!manual.enabled)return;clearRoute();manual.halted=true;setPose({...manual.pose,yaw:wrap(manual.pose.yaw+degrees*deg)});markStatus('已转向 '+degrees+'° · 仿真')}
function jog(key,dt){
 const speed=Number($('manualSpeed').value)||.15;
 const yaw=manual.pose.yaw;
 // Body-forward is x+ in world at yaw=0, body-left is +y.
 let fx=0,fy=0;
 if(key==='w')fx+=1;if(key==='s')fx-=1;if(key==='a')fy+=1;if(key==='d')fy-=1;
 let wz=0;if(key==='q')wz+=.75;if(key==='e')wz-=.75;
 return {vx:(Math.cos(yaw)*fx-Math.sin(yaw)*fy)*speed*dt,vy:(Math.sin(yaw)*fx+Math.cos(yaw)*fy)*speed*dt,wz:wz*dt};
}
function animate(ts){
 const dt=manual.lastFrame?clamp((ts-manual.lastFrame)/1000,0,.07):0;manual.lastFrame=ts;
 if(manual.enabled&&core.getMode()==='simulation'&&manual.pose){
   const keys=manual.keys;
   if(keys.size){
     if(manual.route.length)clearRoute();manual.halted=false;
     const move={vx:0,vy:0,wz:0};for(const key of keys){const p=jog(key,dt);move.vx+=p.vx;move.vy+=p.vy;move.wz+=p.wz}
     if(Math.abs(move.vx)+Math.abs(move.vy)>1e-9){const len=Math.hypot(move.vx,move.vy),max=Number($('manualSpeed').value)*dt;
       const s=len>max?max/len:1;const to={x:manual.pose.x+move.vx*s,y:manual.pose.y+move.vy*s,yaw:manual.pose.yaw};
       if(!safeMove(to)){manual.keys.clear();markStatus('安全阻断 · 无法穿过禁行区域',true)}}
     if(Math.abs(move.wz)>0)setPose({...manual.pose,yaw:manual.pose.yaw+move.wz});
     if(keys.size)markStatus('键盘点动中 · 仿真');
   }else if(!manual.halted&&manual.route.length){
     let remaining=Number($('manualSpeed').value)*dt,changed=false;
     while(remaining>0&&manual.routeIndex<manual.route.length){
       const dest=manual.route[manual.routeIndex],dx=dest.x-manual.pose.x,dy=dest.y-manual.pose.y,d=Math.hypot(dx,dy);
       if(d<1e-6){manual.routeIndex++;continue}
       const n=Math.min(d,remaining),to={x:manual.pose.x+dx/d*n,y:manual.pose.y+dy/d*n,yaw:Math.atan2(dy,dx)};
       if(!safeMove(to)){stop('安全阻断 · 停车');break}
       remaining-=n;changed=true;if(n>=d-1e-8)manual.routeIndex++;
     }
     if(changed)fieldDraw();
     if(manual.routeIndex>=manual.route.length&&manual.route.length){manual.route=[];manual.halted=true;markStatus('已到达目标点 · 仿真');fieldDraw()}
   }
 }
 requestAnimationFrame(animate);
}

// ---------- Real-time comparison against the planned *polyline*, not ground truth. ----------
function nearestSegment(p,path){
 let best=null;
 for(let i=1;i<path.length;i++){
   const a=path[i-1],b=path[i],dx=b.x-a.x,dy=b.y-a.y,l2=dx*dx+dy*dy;
   if(l2<1e-10)continue;
   const t=clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/l2,0,1),x=a.x+t*dx,y=a.y+t*dy;
   const err=Math.hypot(p.x-x,p.y-y);if(!best||err<best.distance){const cross=(dx*(p.y-a.y)-dy*(p.x-a.x))/Math.sqrt(l2);best={distance:err,signed:Math.sign(cross)*err,yaw:Math.atan2(dy,dx),segment:i}}
 }
 return best;
}
const statRms=a=>a.length?Math.sqrt(a.reduce((s,x)=>s+x*x,0)/a.length):NaN;
const percentile=(a,p)=>{if(!a.length)return NaN;const b=[...a].sort((x,y)=>x-y);return b[Math.floor((b.length-1)*p)]};
function telemetry(evt){
 const {raw,aligned,planned,referenceSpeed,planner}=evt.detail;
 if(!Number.isFinite(raw.x)||!Number.isFinite(raw.y)||!Number.isFinite(raw.yaw))return;
 const nearest=Array.isArray(planned)&&planned.length>=2?nearestSegment(aligned,planned):null;
 const speed=Math.hypot(raw.vx||0,raw.vy||0);
 const heading=nearest?wrap(aligned.yaw-nearest.yaw)/deg:NaN;
 const tCandidate=Number.isFinite(raw.received_at)?Math.round(raw.received_at*1000):Date.now();
 const stamp=history.length?Math.max(tCandidate,history.at(-1).t+1):tCandidate;
 const r={t:stamp,raw:{...raw},aligned:{...aligned},offset:nearest?.signed??NaN,absolute:nearest?.distance??NaN,yawErr:heading,speed,referenceSpeed,planner};
 history.push(r);if(history.length>3500)history.shift();
 if(history.length===1||history.length%3===0)refreshErrorUI();
}
function refreshErrorUI(){
 const offsets=history.map(r=>r.absolute).filter(Number.isFinite),heading=history.map(r=>r.yawErr).filter(Number.isFinite);
 $('errRms').textContent=format(statRms(offsets),3)+' m';$('errP95').textContent=format(percentile(offsets,.95),3)+' m';
 $('errMax').textContent=offsets.length?format(Math.max(...offsets),3)+' m':'—';
 $('errYawRms').textContent=format(statRms(heading),1)+' °';
 const last=history.at(-1);$('errSpeed').textContent=last?format(last.speed-last.referenceSpeed,3)+' m/s':'—';
 $('errCount').textContent=String(history.length);
 $('errConnection').textContent=last?'收到实车里程计记录 · '+history.length+' 帧':'等待实车 /odom';
 $('errSource').textContent=last?(serverLooksReplay(last.raw.source)?'测试/日志来源：'+last.raw.source:'来源：'+(last.raw.source||'ROS2')):'尚无数据';
 drawErrorChart();
}
function drawErrorChart(){
 const canvas=$('errorChart');if(!canvas||$('ops-error-pane').hidden||!$('diagnosticHub').open)return;
 const rect=canvas.getBoundingClientRect();if(rect.width<50)return;
 const dpr=Math.min(devicePixelRatio||1,2),w=rect.width,h=Math.max(220,rect.height||280);
 canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);
 const c=canvas.getContext('2d');c.scale(dpr,dpr);c.fillStyle='#081726';c.fillRect(0,0,w,h);
 const left=54,right=w-16,top=28,bottom=h-30;
 c.strokeStyle='#254256';c.lineWidth=1;c.font='11px system-ui';c.fillStyle='#8ca9c0';
 for(let k=0;k<=4;k++){const yy=top+(bottom-top)*k/4;c.beginPath();c.moveTo(left,yy);c.lineTo(right,yy);c.stroke();c.fillText(String(4-k),15,yy+3)}
 const rows=history.slice(-400);if(!rows.length){c.font='14px system-ui';c.fillStyle='#96b2c4';c.fillText('等待 /odom 真实样本或日志回放',left+16,(top+bottom)/2);return}
 const traces=[{name:'横向偏差',key:'absolute',color:'#61e8c9'},{name:'航向偏差',key:'yawErr',color:'#ffc67e'},{name:'实际速度',key:'speed',color:'#75baff'}];
 const maxima={absolute:Math.max(.05,...rows.map(r=>r.absolute).filter(Number.isFinite)),yawErr:Math.max(5,...rows.map(r=>Math.abs(r.yawErr)).filter(Number.isFinite)),speed:Math.max(.2,...rows.map(r=>r.speed).filter(Number.isFinite))};
 traces.forEach((def,j)=>{
   const vals=rows.map(r=>r[def.key]);c.strokeStyle=def.color;c.lineWidth=2.1;c.beginPath();let started=false;
   for(let i=0;i<vals.length;i++){const v=vals[i],x=left+(right-left)*i/Math.max(1,vals.length-1);if(!Number.isFinite(v)){started=false;continue}
     const value=def.key==='yawErr'?Math.abs(v):v, yy=bottom-clamp(value/maxima[def.key],0,1)*(bottom-top);
     if(!started){c.moveTo(x,yy);started=true}else c.lineTo(x,yy)
   }c.stroke();c.fillStyle=def.color;c.fillRect(left+j*135,9,11,4);c.fillStyle='#b9cede';c.fillText(def.name,left+15+j*135,15);
 });
 c.fillStyle='#7899ac';c.fillText('显示最近 '+rows.length+' 帧 · 各曲线独立缩放，仅比较形状',left,bottom+19);
}
function csvRows(){
 // Named-column CSV is explicitly NOT synthesized ULog43 hardware telemetry.
 const head='timestamp,position_x,position_y,yaw,vx,vy,omega_actual';
 return [head,...history.map(r=>[r.t,r.raw.x,r.raw.y,r.raw.yaw/deg,r.raw.vx,r.raw.vy,r.raw.wz].map(v=>Number.isFinite(v)?Number(v).toFixed(6):'').join(','))].join('\n')+'\n';
}
function enableLegacy(){
 $('diagnosticHub').open=true;selectTab('legacy');
 const fr=$('legacyFrame');if(!fr.getAttribute('src'))fr.src=fr.dataset.src;
 return fr;
}
function toLegacy(){if(history.length<2){$('errSource').textContent='至少需要 2 帧 /odom 才能转到调试台';return}
 const frame=enableLegacy();const load=()=>frame.contentWindow?.postMessage({type:'SIM2REAL_IMPORT_NAMED_CSV',csv:csvRows(),name:'sim2real_live_odom_named.csv'},location.origin);
 if(frame.contentDocument?.readyState==='complete')load();else frame.addEventListener('load',load,{once:true});
}
function selectTab(tab){
 const isError=tab==='error';$('ops-error-pane').hidden=!isError;$('ops-legacy-pane').hidden=isError;
 document.querySelectorAll('[data-monitor-tab]').forEach(b=>b.classList.toggle('active',b.dataset.monitorTab===tab));
 if(!isError){const frame=$('legacyFrame');if(!frame.getAttribute('src'))frame.src=frame.dataset.src}else requestAnimationFrame(drawErrorChart);
}
function initialBindings(){
 $('manualEnable').addEventListener('click',()=>manual.enabled?exit():enter());
 $('manualHalt').addEventListener('click',()=>stop('已停车 · 仿真'));
 $('manualExit').addEventListener('click',exit);
 $('manualGrip').addEventListener('click',gripToggle);$('manualArm').addEventListener('click',armToggle);
 $('manualTurnLeft').addEventListener('click',()=>turn(15));$('manualTurnRight').addEventListener('click',()=>turn(-15));
 $('manualSpeed').addEventListener('input',e=>$('manualSpeedLabel').textContent=Number(e.target.value).toFixed(2)+' m/s');
 $('mapHitArea').addEventListener('pointerdown',handleMapCapture,true);
 const norm=k=>String(k).toLowerCase();
 function editable(t){return t?.closest?.('input,textarea,select,[contenteditable],iframe')}
 window.addEventListener('keydown',e=>{
   if(!manual.enabled||core.getMode()!=='simulation'||editable(e.target)||e.altKey||e.ctrlKey||e.metaKey)return;
   const k=norm(e.key);
   if(['w','a','s','d','q','e'].includes(k)){e.preventDefault();if(!e.repeat)manual.keys.add(k)}
   if(e.repeat)return;
   if(k==='g'){gripToggle();e.preventDefault()}
   if(k==='r'){armToggle();e.preventDefault()}
   if(e.code==='Space'){stop('已停车 · 空格');e.preventDefault()}
 },true);
 window.addEventListener('keyup',e=>{manual.keys.delete(norm(e.key))},true);
 window.addEventListener('blur',()=>manual.keys.clear());
 document.addEventListener('visibilitychange',()=>{if(document.hidden)manual.keys.clear()});
 document.querySelectorAll('[data-jog]').forEach(b=>{
   const k=b.dataset.jog;
   b.addEventListener('pointerdown',e=>{if(!manual.enabled)return;e.preventDefault();b.setPointerCapture(e.pointerId);manual.keys.add(k)});
   const release=()=>manual.keys.delete(k);b.addEventListener('pointerup',release);b.addEventListener('pointercancel',release);b.addEventListener('lostpointercapture',release);
 });
 window.addEventListener('sim2real:mode',e=>{if(e.detail.mode==='live'&&manual.enabled)exit();$('manualEnable').disabled=e.detail.mode==='live';$('manualState').textContent=e.detail.mode==='live'?'实车模式：禁止人工指令':'未启用'});
 window.addEventListener('sim2real:telemetry',telemetry);
 $('errExport').addEventListener('click',()=>{if(history.length)download('sim2real_odom_record.csv',csvRows(),'text/csv;charset=utf-8')});
 $('errLoadLegacy').addEventListener('click',toLegacy);
 $('errClear').addEventListener('click',()=>{history.length=0;refreshErrorUI()});
 document.querySelectorAll('[data-monitor-tab]').forEach(b=>b.addEventListener('click',()=>selectTab(b.dataset.monitorTab)));
 $('openManualHub').addEventListener('click',()=>{$('manualHub').open=true;$('manualHub').scrollIntoView({behavior:'smooth',block:'start'})});
 $('openDiagnosticHub').addEventListener('click',()=>{$('diagnosticHub').open=true;$('diagnosticHub').scrollIntoView({behavior:'smooth',block:'start'})});
 $('diagnosticHub').addEventListener('toggle',()=>{if($('diagnosticHub').open)drawErrorChart()});
 $('cameraDashboard').addEventListener('toggle',()=>window.dispatchEvent(new Event('sim2real:cameratoggle')));
 window.addEventListener('resize',drawErrorChart);
 requestAnimationFrame(animate);refreshErrorUI();
}
if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',initialBindings);else initialBindings();
})();
