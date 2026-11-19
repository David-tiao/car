/* V12 ROS2 navigation visualization. Strictly read-only, with frame-aware overlays. */
(() => {
'use strict';
const $ = id => document.getElementById(id);
const NAMES = {map:'静态 /map',scan:'激光 /scan',odom:'底盘 /odom',global_costmap:'全局代价地图',local_costmap:'局部代价地图',global_plan:'全局 /plan',local_plan:'局部 /local_plan',road_path:'道路拓扑路径',road_graph:'道路图 Marker',start_markers:'启停区 Marker',cmd_vel:'控制指令 /cmd_vel',diagnostics:'诊断 /diagnostics',nav_status:'Nav2 目标状态'};
const state = {snapshot:null, layers:{}, lastSeq:0, loading:false, lastGrid:{}, mode:'map',
  center:{x:1.2,y:1.2}, span:{x:3.1,y:3.1}, mapReady:false, drag:null, keys:{},
  switches:{map:true,global_costmap:false,local_costmap:false,scan:true,global_plan:true,local_plan:true,road:true,markers:true,robot:true,grid:true}};
const cv=$('rosCanvas');if(!cv)return;
const ctx=cv.getContext('2d');
const palette={background:'#071422',grid:'#173147',scan:'#42f1ce',plan:'#67b8ff',local:'#f7c96a',robot:'#fff4b2'};
const f=(x,n=2)=>Number.isFinite(x)?Number(x).toFixed(n):'—';
const status=id=>(document.getElementById(id));
function screen(x,y){return [cv.width/2+(x-state.center.x)*state.scale, cv.height/2-(y-state.center.y)*state.scale]}
function world(x,y){return [state.center.x+(x-cv.width/2)/state.scale,state.center.y-(y-cv.height/2)/state.scale]}
function drawLine(arr,color,width=2,dash=[]){
 if(!arr||!arr.length)return;ctx.save();ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);
 let started=false;
 for(const p of arr){if(!Array.isArray(p)||p.length<2||!p.every(Number.isFinite)){started=false;continue}
 const [x,y]=screen(p[0],p[1]);if(!started){ctx.moveTo(x,y);started=true}else ctx.lineTo(x,y)}
 ctx.stroke();ctx.restore();
}
function arrow(p,color){if(!p)return;const [px,py]=screen(p.x,p.y),a=-p.yaw;
 ctx.save();ctx.translate(px,py);ctx.rotate(a);ctx.strokeStyle=color;ctx.lineWidth=2;ctx.fillStyle='rgba(77,208,241,.18)';
 const w=.26*state.scale,h=.29*state.scale;
 ctx.fillRect(-h/2,-w/2,h,w);ctx.strokeRect(-h/2,-w/2,h,w);
 ctx.beginPath();ctx.moveTo(h/2+8,0);ctx.lineTo(h/2-9,-7);ctx.lineTo(h/2-9,7);ctx.closePath();ctx.fillStyle=color;ctx.fill();ctx.restore();
}
function gridImage(layer, obj){
 const {width,height,data}=obj;const values=atob(data);if(values.length!==width*height)throw Error('occupancy data dimensions mismatch');
 const canvas=document.createElement('canvas');canvas.width=width;canvas.height=height;
 const c=canvas.getContext('2d',{willReadFrequently:true});const img=c.createImageData(width,height);
 for(let i=0;i<width*height;i++){
   const v=values.charCodeAt(i),k=i*4;
   if(v===255){img.data[k]=101;img.data[k+1]=117;img.data[k+2]=133;img.data[k+3]=layer==='map'?68:0;continue}
   if(layer==='map'){
      const shade=Math.round(233-v*1.8);img.data[k]=shade;img.data[k+1]=shade;img.data[k+2]=shade+Math.round(v*.04);img.data[k+3]=235;
   }else{
      img.data[k]=Math.min(255,55+v*2);img.data[k+1]=Math.max(20,225-v*2);img.data[k+2]=100;img.data[k+3]=v===0?12:Math.min(225,15+v*2.2);
   }
 }
 c.putImageData(img,0,0);return {canvas,...obj};
}
function paintLayer(layer){
 const data=state.layers[layer];if(!data)return false;
 const frame=state.snapshot?.display_frame||'map';if(data.frame!==frame)return false;
 const a=-data.origin_yaw,[x,y]=screen(data.origin_x,data.origin_y);
 ctx.save();ctx.translate(x,y);ctx.rotate(a);ctx.scale(state.scale*data.resolution,-state.scale*data.resolution);
 ctx.drawImage(data.canvas,0,0);ctx.restore();return true;
}
function drawMarkers(items,frame){
 if(!Array.isArray(items))return;
 for(const mark of items){
  if(mark.frame!==frame)continue;
  const [r,g,b,alpha]=mark.color||[0.6,0.8,1,1];
  const rgb=`rgba(${Math.round(r*255)},${Math.round(g*255)},${Math.round(b*255)},${Math.max(.2,Math.min(1,alpha))})`;
  const yaw=Number(mark.yaw)||0,ca=Math.cos(yaw),sa=Math.sin(yaw),px=Number(mark.x)||0,py=Number(mark.y)||0;
  const points=(mark.points||[]).map(p=>[px+ca*p[0]-sa*p[1],py+sa*p[0]+ca*p[1]]);
  if(mark.type===4||mark.type===5){drawLine(points,rgb,2);continue}
  const [sx,sy]=screen(px,py);
  if(mark.type===9){ctx.save();ctx.font='12px sans-serif';ctx.textAlign='center';ctx.fillStyle=rgb;ctx.fillText(mark.text||'',sx,sy-5);ctx.restore();continue}
  if(mark.type===1){ctx.save();ctx.translate(sx,sy);ctx.rotate(-yaw);ctx.fillStyle=rgb;ctx.globalAlpha=.25;ctx.fillRect(-mark.sx*state.scale/2,-mark.sy*state.scale/2,mark.sx*state.scale,mark.sy*state.scale);ctx.globalAlpha=1;ctx.strokeStyle=rgb;ctx.strokeRect(-mark.sx*state.scale/2,-mark.sy*state.scale/2,mark.sx*state.scale,mark.sy*state.scale);ctx.restore();continue}
  if(mark.type===2){ctx.save();ctx.beginPath();ctx.arc(sx,sy,Math.max(2,(mark.sx||.04)*state.scale/2),0,Math.PI*2);ctx.fillStyle=rgb;ctx.fill();ctx.restore()}
 }
}
function render(){
 const rect=cv.getBoundingClientRect();if(rect.width<1||rect.height<1)return;
 const dpr=Math.min(window.devicePixelRatio||1,2);cv.width=Math.max(1,Math.floor(rect.width*dpr));cv.height=Math.max(1,Math.floor(rect.height*dpr));
 const s=state.span;state.scale=Math.max(.01,Math.min(cv.width/s.x,cv.height/s.y));
 ctx.fillStyle=palette.background;ctx.fillRect(0,0,cv.width,cv.height);
 const isLocal=state.mode==='scan';const snap=state.snapshot,frame=snap?.display_frame||'map';
 if(state.switches.grid){const step=isLocal?.5:.48;const minX=state.center.x-cv.width/(2*state.scale),maxX=state.center.x+cv.width/(2*state.scale);
   const minY=state.center.y-cv.height/(2*state.scale),maxY=state.center.y+cv.height/(2*state.scale);
   ctx.strokeStyle=palette.grid;ctx.lineWidth=1;
   for(let x=Math.ceil(minX/step)*step;x<=maxX;x+=step){const sx=screen(x,0)[0];ctx.beginPath();ctx.moveTo(sx,0);ctx.lineTo(sx,cv.height);ctx.stroke()}
   for(let y=Math.ceil(minY/step)*step;y<=maxY;y+=step){const sy=screen(0,y)[1];ctx.beginPath();ctx.moveTo(0,sy);ctx.lineTo(cv.width,sy);ctx.stroke()}}
 if(!isLocal){
  if(state.switches.map)paintLayer('map');
  if(state.switches.global_costmap)paintLayer('global_costmap');
  if(state.switches.local_costmap)paintLayer('local_costmap');
  if(state.switches.global_plan&&snap?.paths?.global?.frame===frame)drawLine(snap.paths.global.points,palette.plan,3.5);
  if(state.switches.local_plan&&snap?.paths?.local?.frame===frame)drawLine(snap.paths.local.points,palette.local,2.5);
  if(state.switches.road&&snap?.paths?.road?.frame===frame)drawLine(snap.paths.road.points,'#bb9cff',2,[6,4]);
  if(state.switches.markers){drawMarkers(snap?.markers?.road_graph,frame);drawMarkers(snap?.markers?.start_markers,frame)}
  if(state.switches.scan&&snap?.scan?.transformed&&snap?.scan?.target_frame===frame){
   ctx.fillStyle=palette.scan;
   for(const p of snap.scan.points||[]){const [x,y]=screen(p[0],p[1]);ctx.fillRect(x-1.5,y-1.5,3,3)}
   if(snap.scan.origin){const [x,y]=screen(snap.scan.origin.x,snap.scan.origin.y);ctx.beginPath();ctx.arc(x,y,4,0,2*Math.PI);ctx.fill()}}
  if(state.switches.robot&&snap?.robot?.frame===frame)arrow(snap.robot,palette.robot);
 }else{
  if(state.switches.scan){ctx.fillStyle=palette.scan;for(const p of snap?.scan?.local_points||[]){const [x,y]=screen(p[0],p[1]);ctx.fillRect(x-1.5,y-1.5,3,3)}}
  const [cx,cy]=screen(0,0);ctx.strokeStyle='#f8c77b';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(cx-10,cy);ctx.lineTo(cx+10,cy);ctx.moveTo(cx,cy-10);ctx.lineTo(cx,cy+10);ctx.stroke();
 }
 ctx.save();ctx.fillStyle='#aac0d4';ctx.font=`${12*dpr}px sans-serif`;ctx.fillText(isLocal?'激光局部坐标 · 米（不叠加比赛地图）':`显示坐标系: ${frame} · 米`,14*dpr,22*dpr);
 if(!snap?.online) {ctx.fillStyle='#f3cb8d';ctx.font=`${14*dpr}px sans-serif`;ctx.fillText('等待 ROS2 可视化桥接数据（尚未连接）',14*dpr,45*dpr)}
 else if(!isLocal && state.switches.scan && snap.scan && !snap.scan.transformed){ctx.fillStyle='#f3cb8d';ctx.fillText(`激光未叠加：缺少 ${frame} → ${snap.scan.frame} TF；请切换到“激光局部视图”`,14*dpr,44*dpr)}
 ctx.restore();
}
function fit(){
 if(state.mode==='scan'){
   const rm=Math.max(1.2,Math.min(12,state.snapshot?.scan?.range_max||2.5));state.center={x:0,y:0};state.span={x:rm*2.3,y:rm*2.3};
 }else{
   const map=state.layers.map;
   if(map){const a=map.origin_yaw,ca=Math.cos(a),sa=Math.sin(a),w=map.width*map.resolution,h=map.height*map.resolution;
    const corners=[[0,0],[w,0],[0,h],[w,h]].map(([x,y])=>[map.origin_x+ca*x-sa*y,map.origin_y+sa*x+ca*y]);
    const xs=corners.map(p=>p[0]),ys=corners.map(p=>p[1]),loX=Math.min(...xs),hiX=Math.max(...xs),loY=Math.min(...ys),hiY=Math.max(...ys);
    state.center={x:(loX+hiX)/2,y:(loY+hiY)/2};state.span={x:Math.max(hiX-loX,1)*1.13,y:Math.max(hiY-loY,1)*1.13};
   }else{state.center={x:1.2,y:1.2};state.span={x:3,y:3}}
 }
 render();
}
function markStatus(data){
 const online=!!data?.online;
 $('rosOnline').textContent=online?`ROS2 已接入 · ${f(data.age_s,1)}s`:'ROS2 未连接 / 数据超时';
 $('rosOnline').dataset.online=online?'true':'false';
 $('rosFrame').textContent='参考坐标: '+(data?.display_frame||'—');
 $('rosBridge').textContent=online?'/api/ros/state 正在更新':'请在 Ubuntu 启动 start_ros_visualization.sh';
 const robot=data?.robot;
 $('rosRobotPos').textContent=robot?`${f(robot.x,2)}, ${f(robot.y,2)} m · ${f(robot.yaw*180/Math.PI,1)}°`:'无 map→base_link TF；不显示地图车体';
 $('rosTF').textContent=['map_odom','odom_base','base_laser'].map(k=>{const n={map_odom:'map→odom',odom_base:'odom→base_link',base_laser:'base_link→laser'}[k];return n+': '+(data?.tf?.[k]?'✓':'×')}).join('   ');
 $('rosFrameWarn').textContent=data?.scan&&!data.scan.transformed?'⚠ /scan 存在但缺少到地图的 TF，已禁用地图叠加。可使用激光局部视图。':'';
 $('rosCmd').textContent=data?.cmd_vel?`${f(data.cmd_vel.vx,3)} / ${f(data.cmd_vel.vy,3)} m/s · ${f(data.cmd_vel.wz,3)} rad/s`:'未收到 /cmd_vel（指令，不是实测）';
 $('rosNav').textContent=data?.nav?.status===undefined||data.nav.status===null?'尚未收到 Nav2 目标状态':`NavigateToPose 状态码: ${data.nav.status}`;
 const rows=Object.entries(data?.topics||{}).map(([role,v])=>{
  const healthy=v.received&&v.age_s!==null&&v.age_s<(role==='map'?60:role.includes('costmap')?10:5);
  const name=NAMES[role]||role;
  return `<div class="ros-topic"><span class="ros-light ${healthy?'good':''}"></span><span class="ros-topic-name" title="${escapeHtml(v.topic)}">${escapeHtml(name)}</span><code>${escapeHtml(v.topic)}</code><b>${v.received?f(v.hz,1)+' Hz':'未收到'}</b></div>`;
 }).join('');
 $('rosTopics').innerHTML=rows||'<p class="ros-muted">运行 ROS2 桥接后，话题状态会显示在这里。</p>';
 $('rosGraph').textContent=data?.bridge?.graph_topics?.length?`网络发现 ${data.bridge.graph_topics.length} 个话题 · ${data.nodes?.length||0} 个节点`:'等待 ROS2 话题发现';
 const listed=(data?.bridge?.graph_topics||[]).map(t=>`${t.name}  [${(t.types||[]).join(', ')}]`).join('\n');
 $('rosGraphDetail').textContent=listed||'ROS2 图谱尚未连接';
 const diags=(data?.diagnostics||[]).slice(0,10);
 $('rosDiagnostics').textContent=diags.length?diags.map(d=>`[${d.level}] ${d.name}: ${d.message}`).join('\n'):'尚无 /diagnostics 数据';
}
function escapeHtml(s){return String(s||'').replace(/[&<>"']/g,x=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[x]));}
async function getGrid(layer,seq){
 try{
   const r=await fetch((window.CarWebMode?.rosUrl||'/api/ros')+'/grid/'+layer,{cache:'no-store'});
   if(!r.ok)throw Error(`HTTP ${r.status}`);
   const v=await r.json();if(!v.grid||v.seq!==seq)return;
   state.layers[layer]=gridImage(layer,v.grid);state.lastGrid[layer]=seq;
   if(layer==='map'&&!state.mapReady){state.mapReady=true;fit()}
 }catch(e){console.warn('ROS grid read failed',layer,e)}
}
async function poll(){
 if(window.CarWebMode?.mode==='sim')return;
 if(state.loading)return;state.loading=true;
 try{
   const r=await fetch((window.CarWebMode?.rosUrl||'/api/ros')+'/state',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);
   const data=await r.json();state.snapshot=data;markStatus(data);
   for(const [name,v] of Object.entries(data.grids||{}))if(v.seq>0&&v.seq!==state.lastGrid[name])await getGrid(name,v.seq);
   render();
 }catch(e){markStatus(null);state.snapshot=null;render()}finally{state.loading=false}
}
const toggles=document.querySelectorAll('[data-ros-layer]');
toggles.forEach(el=>el.addEventListener('change',()=>{state.switches[el.dataset.rosLayer]=el.checked;render()}));
$('rosViewMode').addEventListener('change',e=>{state.mode=e.target.value;fit()});
$('rosFit').addEventListener('click',fit);
$('rosPanel').addEventListener('toggle',()=>{if($('rosPanel').open){fit();poll()}});
$('rosOpenSettings').addEventListener('click',()=>{
  const btn=document.querySelector('[data-settings-tab="ros"]');btn?.click();
});
cv.addEventListener('pointerdown',e=>{const b=cv.getBoundingClientRect();state.drag={x:e.clientX,y:e.clientY,center:{...state.center}};cv.setPointerCapture?.(e.pointerId)});
cv.addEventListener('pointermove',e=>{if(!state.drag)return;const d=state.drag;state.center.x=d.center.x-(e.clientX-d.x)*(cv.width/cv.clientWidth)/state.scale;state.center.y=d.center.y+(e.clientY-d.y)*(cv.height/cv.clientHeight)/state.scale;render()});
cv.addEventListener('pointerup',e=>{state.drag=null;cv.releasePointerCapture?.(e.pointerId)});
cv.addEventListener('pointercancel',()=>{state.drag=null});
cv.addEventListener('wheel',e=>{e.preventDefault();const factor=e.deltaY>0?1.16:1/1.16;state.span.x=Math.min(50,Math.max(.25,state.span.x*factor));state.span.y=Math.min(50,Math.max(.25,state.span.y*factor));render()},{passive:false});
window.addEventListener('resize',render);
if(typeof ResizeObserver!=='undefined')new ResizeObserver(render).observe(cv.parentElement);
markStatus(null);fit();poll();setInterval(()=>{if($('rosPanel').open)poll()},400);
})();
