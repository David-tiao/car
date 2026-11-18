/* Competition simulator V11.0: numbered 5x5 serpentine field occupancy map + adjustable rotor + ROS odom telemetry. */
(() => {
  'use strict';
  const NS='http://www.w3.org/2000/svg', FIELD=2.4, SCALE=680/FIELD, NAV_STEP=.05, BLOCK_DIAMETER=.05, CLEARANCE=0, DEFAULT_GRID_STEP=.48;
  const $=id=>document.getElementById(id);
  const POS={yellow:[[.55,.55,1,1],[1.4,.55,1.85,1],[.55,1.4,1,1.85],[1.4,1.4,1.85,1.85]],blue:[[2.1,2.1,2.4,2.4],[2.1,0,2.4,.3]],work:[[0,.91,.15,1.49],[.91,0,1.49,.15]],raw:{x:1.2,y:2.4,radius:.15},storage:[[.08,1.34],[.08,1.2],[.08,1.06]],process:[[1.05,.08],[1.2,.08],[1.35,.08]]};
  // 4 large 400x400 mm road sectors as marked 1–4 in the user's field diagram.
  const SLOTS=[
    {id:1,x:1.2,y:1.75,rect:[1,1.55,1.4,1.95],name:'上方主车道'},
    {id:2,x:1.75,y:1.2,rect:[1.55,1,1.95,1.4],name:'右侧主车道'},
    {id:3,x:1.2,y:.65,rect:[1,.45,1.4,.85],name:'下方主车道'},
    {id:4,x:.65,y:1.2,rect:[.45,1,.85,1.4],name:'左侧主车道'}
  ];
  const STATIONS={qr:{x:2.15,y:1.20},raw:{x:1.2,y:2.08},process:{x:1.2,y:.33},storage:{x:.33,y:1.2}};
  const TONES={1:'#f34b58',2:'#f2cf46',3:'#4376f1',4:'#36bd79',5:'#343b47',6:'#62d7ef'};
  const TONE_NAMES={1:'红',2:'黄',3:'蓝',4:'绿',5:'黑',6:'浅蓝'};
  const STAGE_DEFS=[
    ['qr','01 扫描二维码'],['raw','02 第一批依次抓取'],['process','03 第一批粗加工及收回'],['storage','04 第一批送暂存区'],
    ['raw','05 第二批依次抓取'],['process','06 第二批粗加工及收回'],['storage','07 按同色码垛'],['return','08 驶往终点启停区']
  ];
  const state={obstacles:[],start:{x:2.25,y:2.25},goal:{x:.33,y:1.2},startZone:1,tool:'obstacle',mode:'mission',planner:'astar',goalPolicy:'other',
    robotWidth:.26,robotLength:.29,safetyMargin:0,navStep:NAV_STEP,gridStep:DEFAULT_GRID_STEP,velocity:.25,handling:.30,showGrid:true,pathView:'global',turnDirection:'cw',
    runSource:'simulation',rotorDwell:4,rotorRate:45,rotorAngle:120,rawX:1.2,rawY:2.4,qrY:1.2,alignOffset:.015,alignSpeed:.04,alignYaw:2,armReach:.09,armSpeed:.1,ringFollow:true,
    task:'156+123+516+231',plan:null, sim:null, turnOrigin:0};
  const shared={ready:false,inbound:false,dirty:false,pushing:false,revision:0,seq:0,timer:null};
  // Live mode is display-only. The ROS bridge is the ONLY publisher of /api/telemetry.
  // Real robot /odom can begin at an arbitrary origin; first received pose is aligned to chosen field start.
  const live={lastSeq:0,anchor:null,pose:null,trail:[],lastSeen:0,online:false,polling:false,frame:'odom',source:'—'};
  // Official Fig.1: raw-table centre along north wall x=1100..1300mm; QR centre along east wall y=1100..1300mm.
  // Only movement along those respective walls is adjustable; other coordinates are fixed by the field diagram.
  // Raw-Y is an experimental allowance, NOT an officially specified installation tolerance.
  // User requested a small vertical shift; limit it to one physical disc radius from nominal Y=2400 mm.
  const FIXTURE_BOUNDS={raw:[1.1,1.3],rawY:[2.25,2.55],qr:[1.1,1.3]};
  function inFixtureRange(type,value){const [min,max]=FIXTURE_BOUNDS[type];return Number.isFinite(value)&&value>=min-1e-9&&value<=max+1e-9;}
  function applyFixtureValues(){
    POS.raw.x=state.rawX;POS.raw.y=state.rawY;
    STATIONS.raw.x=state.rawX;STATIONS.raw.y=state.rawY-.32;
    STATIONS.qr.y=state.qrY;
  }
  function updateFixtureUI(){
    $('rawPosition').value=Math.round(state.rawX*1000);$('rawVertical').value=Math.round(state.rawY*1000);$('qrPosition').value=Math.round(state.qrY*1000);
    $('rawPositionValue').textContent=Math.round(state.rawX*1000)+' mm';
    $('rawVerticalValue').textContent=Math.round(state.rawY*1000)+' mm';
    $('qrPositionValue').textContent=Math.round(state.qrY*1000)+' mm';
  }
  function changeFixture(type,value){
    if(!inFixtureRange(type,value)){tell('装置位置超出滑块范围，请按图示范围调整。',true);return;}
    if(type==='raw')state.rawX=+value.toFixed(3);else if(type==='rawY')state.rawY=+value.toFixed(3);else state.qrY=+value.toFixed(3);
    applyFixtureValues();
    const before=state.obstacles.length;state.obstacles=state.obstacles.filter(physicalObstacleFits);
    updateFixtureUI();recalc();
    tell((type==='raw'?'原料转盘 X':type==='rawY'?'原料转盘 Y':'二维码板 Y')+' 已设置为 '+Math.round(value*1000)+' mm；全程路线已重规划。'+(before>state.obstacles.length?'与转盘冲突的障碍物已自动移除。':''));
  }
  function updateRotorControls(){
    $('rotorDwellValue').textContent=state.rotorDwell.toFixed(1)+' s';
    $('rotorRateValue').textContent=state.rotorRate.toFixed(0)+' °/s · '+(360/state.rotorRate).toFixed(1)+' s/圈';
    $('rotorAngleValue').textContent=state.rotorAngle.toFixed(0)+' °';
  }
  // V10 directory: keep all legacy element IDs; only change which group is visible.
  const SETTINGS_IDS=['planner','grid','fixture','drive','rotor','task','live','export','camera','ros'];
  const cameraFrames={openmv:{id:'camOpenmvImage',status:'camOpenmvStatus',placeholder:'camOpenmvPlaceholder',fps:'camOpenmvFps',info:'cameraOpenmvInfo'},pi5:{id:'camPi5Image',status:'camPi5Status',placeholder:'camPi5Placeholder',fps:'camPi5Fps',info:'cameraPi5Info'}};
  function updateCameraStreamMode(){
    const isLive=state.runSource==='live'&&Boolean($('cameraDashboard')?.open);
    for(const [name,parts] of Object.entries(cameraFrames)){
      const el=$(parts.id);if(!el)continue;
      if(isLive){if(!el.getAttribute('src'))el.src='/api/camera/'+name+'/stream.mjpg?build=11';}
      else{el.removeAttribute('src');el.style.display='none';$(parts.placeholder).style.display='flex';}
    }
  }
  async function updateCameraStatus(){
    if(state.runSource!=='live')return;
    try{
      const r=await fetch('/api/cameras',{cache:'no-store'});if(!r.ok)throw new Error('status '+r.status);
      const json=await r.json();let count=0;
      for(const [name,parts] of Object.entries(cameraFrames)){
        const data=json.cameras?.[name];const online=Boolean(data?.online);if(online)count++;
        $(parts.status).textContent=online?'实时在线':'离线';$(parts.status).classList.toggle('online',online);
        $(parts.placeholder).style.display=online?'none':'flex';$(parts.id).style.display=online?'block':'none';
        $(parts.fps).textContent=online?(data.fps.toFixed(1)+' fps'):'— fps';
        $(parts.info).textContent=online?'已连接 · '+data.fps.toFixed(1)+' fps':('未连接'+(data?.age_s!=null?' · 上次帧 '+data.age_s+' s 前':''));
      }
      $('cameraSummary').textContent=count===2?'两路视频在线':count===1?'1 / 2 路在线':'两路未连接';
    }catch(e){$('cameraSummary').textContent='视频服务器未响应';}
  }

  function activateSettingsTab(id){
    if(!SETTINGS_IDS.includes(id))return;
    if((id==='live'||id==='camera')&&state.runSource!=='live'){switchRunSource('live');activateSettingsTab(id);return;}
    if((id==='drive'||id==='rotor')&&state.runSource==='live'){switchRunSource('simulation');activateSettingsTab(id);return;}
    document.querySelectorAll('[data-settings-view]').forEach(el=>{el.hidden=el.dataset.settingsView!==id;});
    document.querySelectorAll('[data-settings-tab]').forEach(btn=>{const selected=btn.dataset.settingsTab===id;btn.classList.toggle('active',selected);btn.setAttribute('aria-pressed',String(selected));});
    try{localStorage.setItem('competition_v10_settings_tab',id);}catch(e){}
  }
  function syncModeShell(){
    document.body.dataset.runMode=state.runSource;
    document.querySelectorAll('button[data-run-mode]').forEach(btn=>{const active=btn.dataset.runMode===state.runSource;btn.classList.toggle('active',active);btn.setAttribute('aria-pressed',String(active));});
    const isLive=state.runSource==='live';
    const fresh=live.online&&(Date.now()-live.lastSeen<2000);
    const playback=/REPLAY|TEST|OFFLINE/i.test(live.source);
    $('modeStatus').textContent=isLive?(fresh?(playback?'回放 / 测试数据在线':'实时数据已接入'):'等待实车数据'):'模拟模式就绪';
    $('modeDescription').textContent=isLive?'当前：实车监视。网页只按实测 /odom 更新位置与轨迹，不发送电机指令；无数据时不会自行运动。':'当前：模拟运行。按规划路线、模拟速度和抓放参数播放，不代表真实底盘反馈。';
  }
  function bindDirectory(){
    document.querySelectorAll('button[data-run-mode]').forEach(btn=>btn.addEventListener('click',()=>{
      if(btn.dataset.runMode===state.runSource){activateSettingsTab(state.runSource==='live'?'live':'planner');return;}
      $('runSource').value=btn.dataset.runMode;$('runSource').dispatchEvent(new Event('change',{bubbles:true}));
    }));
    document.querySelectorAll('[data-settings-tab]').forEach(btn=>btn.addEventListener('click',()=>{activateSettingsTab(btn.dataset.settingsTab);const current=document.querySelector('[data-settings-view]:not([hidden])');current?.scrollIntoView({behavior:'smooth',block:'nearest'});}));
    let saved='planner';try{saved=localStorage.getItem('competition_v10_settings_tab')||'planner';}catch(e){}
    activateSettingsTab(saved==='live'?'planner':saved);
    syncModeShell();
  }
  function switchRunSource(mode){if(!['simulation','live'].includes(mode))return;
    if(state.sim){state.sim.running=false;state.sim.phase='idle';state.sim.poseOverride=null;state.sim.armExtension=0;state.sim.motion=null;state.sim.armMove=null;}
    state.runSource=mode;live.anchor=null;live.pose=null;live.trail=[];live.lastSeq=0;
    syncRunUI();activateSettingsTab(mode==='live'?'live':'planner');paintLiveTrail();paintRobot();updateCameraStreamMode();
    window.dispatchEvent(new CustomEvent('sim2real:mode',{detail:{mode}}));
    tell(mode==='live'?'已切换实车实时模式：等待 /odom 反馈；该模式不使用预设速度播放。':'已切换模拟运行模式：可使用速度滑块播放规划任务。');
  }
  function syncRunUI(){
    $('runSource').value=state.runSource;
    const isLive=state.runSource==='live';
    $('livePanel').hidden=!isLive;
    $('playBtn').disabled=isLive||!state.plan?.ok;
    $('playBtn').textContent=isLive?'实车由 /odom 驱动':'▶ 开始完整任务仿真';
    $('velocity').disabled=isLive;
    $('simStatusTag').textContent=isLive?'实车监控':'待机';if(isLive)$('alignDisplay').textContent='实车模式：没有机械臂反馈，仅显示实车里程计';
    if(isLive)$('actionDisplay').textContent='等待实车里程计数据（不自动模拟抓取）';
    updateLiveReadout();syncModeShell();
  }
  function resetLiveAnchor(){live.anchor=null;live.pose=null;live.trail=[];paintLiveTrail();paintRobot();
    $('liveState').textContent='等待下一帧对齐';}
  function alignedPose(s){
    if(!live.anchor)live.anchor={x:s.x,y:s.y,yaw:s.yaw};
    const ax=Number($('liveAnchorX').value),ay=Number($('liveAnchorY').value),ang=Number($('liveAnchorYaw').value)*Math.PI/180;
    const a=ang-live.anchor.yaw,dx=s.x-live.anchor.x,dy=s.y-live.anchor.y;
    return {x:ax+Math.cos(a)*dx-Math.sin(a)*dy,y:ay+Math.sin(a)*dx+Math.cos(a)*dy,yaw:s.yaw+a};
  }
  function paintLiveTrail(){const layer=$('liveTrailLayer');empty(layer);if(state.runSource!=='live'||live.trail.length<2)return;
    const path=live.trail.map((p,i)=>`${i?'L':'M'}${fx(p.x).toFixed(1)} ${fy(p.y).toFixed(1)}`).join(' ');
    node('path',{d:path,fill:'none',stroke:'#f59e0b','stroke-width':4.5,'stroke-linecap':'round','stroke-linejoin':'round'},layer);
  }
  function updateLiveReadout(){
    const isLive=state.runSource==='live',fresh=live.online&&(Date.now()-live.lastSeen<2000);
    $('liveState').textContent=isLive?(fresh?(/REPLAY|TEST|OFFLINE/i.test(live.source)?'测试/回放数据在线（非实车）':/growing_file/i.test(live.source)?'实时日志接入（待核对来源）':'ROS2 实车数据在线'):live.pose?'数据中断 / 超时':'等待 ROS2 /odom'):'未启用';
    $('liveState').dataset.online=fresh?'true':'false';
    if(!isLive)return;
    for(const [id,v,digits,unit] of [['liveVx',live.latest?.vx,3,'m/s'],['liveVy',live.latest?.vy,3,'m/s'],['liveWz',live.latest?.wz,2,'rad/s'],
     ['liveX',live.pose?.x,3,'m'],['liveY',live.pose?.y,3,'m'],['liveYaw',live.pose?.yaw===undefined?undefined:live.pose.yaw*180/Math.PI,1,'°']]){
      $(id).textContent=typeof v==='number'&&Number.isFinite(v)?v.toFixed(digits)+' '+unit:'—';
    }
    $('liveSource').textContent=live.source+' · '+live.frame;syncModeShell();
  }
  async function pollTelemetry(){if(live.polling)return;live.polling=true;
    try{const res=await fetch('/api/telemetry?after='+live.lastSeq,{cache:'no-store'});if(!res.ok)throw Error('HTTP '+res.status);
      const packet=await res.json();live.online=!!packet.online;
      for(const s of packet.samples||[]){
        if(!Number.isFinite(s.x)||!Number.isFinite(s.y)||!Number.isFinite(s.yaw))continue;
        live.lastSeq=Math.max(live.lastSeq,s.seq||0);live.source=s.source||'ROS2';live.frame=s.frame||'odom';live.latest=s;
        // Maintain recent pose only in live mode; another visitor can still choose simulation.
        if(state.runSource==='live'){
          const p=alignedPose(s);live.pose=p;live.lastSeen=Date.now();
          const prev=live.trail[live.trail.length-1];if(!prev||Math.hypot(prev.x-p.x,prev.y-p.y)>.003)live.trail.push(p);
          if(live.trail.length>1800)live.trail.splice(0,live.trail.length-1800);
          window.dispatchEvent(new CustomEvent('sim2real:telemetry',{detail:{raw:{...s},aligned:{...p},planned:state.plan?.fullPath||[],referenceSpeed:state.velocity,planner:state.planner}}));
        }
      }
      if(state.runSource==='live'){paintLiveTrail();paintRobot();updateLiveReadout();}
    }catch(e){live.online=false;updateLiveReadout();}finally{live.polling=false;}
  }
  const fx=x=>60+x*SCALE, fy=y=>740-y*SCALE;
  const inField=(x,y)=>x>=0&&x<=FIELD&&y>=0&&y<=FIELD;
  const inside=(x,y,r)=>x>=r[0]&&x<=r[2]&&y>=r[1]&&y<=r[3];
  const f=(v,n=2)=>Number(v).toFixed(n);
  function node(tag,attrs={},parent=null,text=null){const e=document.createElementNS(NS,tag);for(const [k,v] of Object.entries(attrs))e.setAttribute(k,String(v));if(text!==null)e.textContent=String(text);if(parent)parent.append(e);return e}
  function empty(e){e.replaceChildren()}
  function label(layer,text,x,y,size=16,color='#354352',rotate=0){return node('text',{x:fx(x),y:fy(y),'text-anchor':'middle','dominant-baseline':'middle','font-size':size,'font-weight':'650',fill:color,...(rotate?{transform:`rotate(${rotate} ${fx(x)} ${fy(y)})`}:{})},layer,text)}
  function worldRect(parent,r,attrs={}){return node('rect',{x:fx(r[0]),y:fy(r[3]),width:(r[2]-r[0])*SCALE,height:(r[3]-r[1])*SCALE,...attrs},parent)}
  function sqHitsRect(x,y,rect,half){return x+half>rect[0]&&x-half<rect[2]&&y+half>rect[1]&&y-half<rect[3]}
  function sqHitsDisc(x,y,circle,half){const dx=Math.max(Math.abs(x-circle.x)-half,0),dy=Math.max(Math.abs(y-circle.y)-half,0);return dx*dx+dy*dy<circle.radius*circle.radius-1e-10}
  // Axis-aligned 260x290 mm rectangular footprint, with optional user safety margin.
  function staticCollision(x,y){const hx=state.robotWidth/2+CLEARANCE+state.safetyMargin,hy=state.robotLength/2+CLEARANCE+state.safetyMargin;
    if(x-hx<0||y-hy<0||x+hx>FIELD||y+hy>FIELD)return true;
    return POS.yellow.some(r=>x+hx>r[0]&&x-hx<r[2]&&y+hy>r[1]&&y-hy<r[3]) ||
      POS.work.some(r=>x+hx>r[0]&&x-hx<r[2]&&y+hy>r[1]&&y-hy<r[3]) ||
      (Math.max(Math.abs(x-POS.raw.x)-hx,0)**2+Math.max(Math.abs(y-POS.raw.y)-hy,0)**2<POS.raw.radius**2);}
  // The user-visible occupancy map covers the ENTIRE field. Exactly ONE grid cell is blocked per obstacle.
  // Robot footprint still intersects occupied cells; no additional red inflation is drawn.
  function occupiedSector(o){const g=guideId(o),d=state.gridStep;return [g.c*d,g.r*d,(g.c+1)*d,(g.r+1)*d];}
  function dynamicCollision(x,y){
    const hx=state.robotWidth/2,hy=state.robotLength/2;
    return state.obstacles.some(o=>{
      const r=occupiedSector(o);
      return x+hx>r[0]+1e-9&&x-hx<r[2]-1e-9&&y+hy>r[1]+1e-9&&y-hy<r[3]-1e-9;
    });
  }
  function sectorTouchesBlue(p){const r=occupiedSector(p);
    return POS.blue.some(b=>r[0]<b[2]&&r[2]>b[0]&&r[1]<b[3]&&r[3]>b[1]);
  }
  function physicalObstacleFits(p){const r=BLOCK_DIAMETER/2;if(p.x<r||p.y<r||p.x>FIELD-r||p.y>FIELD-r||sectorTouchesBlue(p))return false;
    const forbidden=[...POS.yellow,...POS.blue,...POS.work];
    return !forbidden.some(box=>
      (Math.max(box[0]-p.x,0,p.x-box[2])**2+Math.max(box[1]-p.y,0,p.y-box[3])**2<r*r)) &&
      Math.hypot(p.x-POS.raw.x,p.y-POS.raw.y)>=POS.raw.radius+r;
  }
  function endZone(){if(state.goalPolicy==='home')return state.startZone===2?2:1;
    return state.startZone===2?1:(state.startZone===1?2:(state.start.y>=1.2?2:1));}
  function endStation(){return {x:2.25,y:endZone()===1?2.25:.15};}
  // Cell IDs match the user's screenshot: bottom row 0..4, next 9..5,
  // then 10..14, 19..15, and top row 20..24.
  function numberedCell(c,r){const n=5;return r*n+(r%2===0?c:n-1-c);}
  function cellFromNumber(id){const r=Math.floor(id/5),k=id%5;return {c:r%2===0?k:4-k,r};}
  function guideId(p){const n=Math.round(FIELD/state.gridStep);return {c:Math.min(n-1,Math.max(0,Math.floor(p.x/state.gridStep))),r:Math.min(n-1,Math.max(0,Math.floor(p.y/state.gridStep)))};}
  const safe=(x,y)=>!staticCollision(x,y)&&!dynamicCollision(x,y);
  function tell(message,bad=false){$('notice').textContent=message;$('notice').classList.toggle('error',bad)}
  function parseTask(code){const g=String(code||'').trim().split('+');
    if(!/^[1-6]{3}\+[1-3]{3}\+[1-6]{3}\+[1-3]{3}$/.test(String(code||'').trim()))return {valid:false,error:'格式：颜色三位(1–6)+环号三位(1–3)+颜色三位+环号三位。'};
    for(const i of [0,2])if(new Set(g[i]).size!==3)return {valid:false,error:'每批需要三个不同颜色。'};
    for(const i of [1,3])if(g[i].split('').sort().join('')!=='123')return {valid:false,error:'本场地每批依次使用 1、2、3 三个环号（允许调换顺序）。'};
    if(g[0].split('').sort().join('')!==g[2].split('').sort().join(''))return {valid:false,error:'第二批必须包含与第一批相同的三种颜色，才能完成同色码垛。'};
    const batch1=g[0].split('').map((c,i)=>({color:+c,ring:+g[1][i]}));
    const batch2=g[2].split('').map((c,i)=>({color:+c,ring:+g[3][i]}));
    const stack=batch2.map(it=>({...it,stackRing:batch1.find(j=>j.color===it.color).ring}));
    return {valid:true,g,batch1,batch2,stack};
  }
  function paintField(){const layer=$('staticZones');empty(layer);
    POS.yellow.forEach(r=>worldRect(layer,r,{fill:'#f3edc6',stroke:'#d7c99f','stroke-width':'1.4'}));
    POS.blue.forEach(r=>worldRect(layer,r,{fill:'#184bfb',stroke:'#123fc0','stroke-width':'1.8'}));
    POS.work.forEach(r=>worldRect(layer,r,{fill:'#fdfdfd',stroke:'#7a8590','stroke-width':'1.7'}));
    node('circle',{cx:fx(state.rawX),cy:fy(state.rawY),r:.15*SCALE,fill:'#fffaf0',stroke:'#5e6870','stroke-width':2.1},layer);
    node('circle',{cx:fx(state.rawX),cy:fy(state.rawY),r:.102*SCALE,fill:'none',stroke:'#adada1','stroke-width':1.3,'stroke-dasharray':'4 4'},layer);
    for(const [key,rings] of [['process',POS.process],['storage',POS.storage]])rings.forEach(([x,y],i)=>{
      // color-coded physical rings are illustrative; score concentric rings are not modeled at millimetre accuracy.
      node('circle',{cx:fx(x),cy:fy(y),r:.033*SCALE,fill:'#fff',stroke:'#7091aa','stroke-width':2.4},layer);
      node('text',{x:fx(x),y:fy(y)+3.6,'text-anchor':'middle','font-size':11,'font-weight':800,fill:'#34495d'},layer,String(i+1));
    });
    node('rect',{x:fx(2.38)-3,y:fy(state.qrY+.15),width:6,height:.30*SCALE,fill:'#313a44'},layer);
    label(layer,'二维码板',2.01,state.qrY,14,'#47505b',-90);
    label(layer,'启停区 1',2.25,2.25,16,'#fff');label(layer,'启停区 2',2.25,.15,16,'#fff');
    label(layer,'暂存区',.085,1.2,15,'#35465a',-90);label(layer,'粗加工区',1.2,.195,15,'#344d5c');
    label(layer,'原料转盘',state.rawX-.46,Math.min(2.33,state.rawY-.10),16,'#344354');
    // Limited placement ranges from the competition field diagram (not free-field movement).
    node('line',{x1:fx(1.1),y1:fy(2.4)+3,x2:fx(1.3),y2:fy(2.4)+3,stroke:'#ad6b13','stroke-width':5,'stroke-opacity':.65},layer);
    node('line',{x1:fx(2.4)-4,y1:fy(1.1),x2:fx(2.4)-4,y2:fy(1.3),stroke:'#ad6b13','stroke-width':5,'stroke-opacity':.65},layer);
    node('text',{x:400,y:783,'text-anchor':'middle','font-size':18,'font-weight':700,fill:'#405468'},layer,'2 400 × 2 400 mm  ·  COMPETITION FIELD');
    for(let i=0;i<=4;i++){const v=i*.6;
      node('line',{x1:fx(v),y1:741,x2:fx(v),y2:751,stroke:'#687b8f'},layer);
      node('text',{x:fx(v),y:766,'text-anchor':'middle','font-size':11,fill:'#4a5d73'},layer,String(Math.round(v*1000)));
      node('line',{x1:49,y1:fy(v),x2:60,y2:fy(v),stroke:'#687b8f'},layer);
      node('text',{x:46,y:fy(v)+4,'text-anchor':'end','font-size':11,fill:'#4a5d73'},layer,String(Math.round(v*1000)));
    }
  }
  function mapGrid(){const n=Math.round(FIELD/NAV_STEP),blocked=[],stat=[],dynamic=[];
    for(let r=0;r<n;r++){const br=[],sr=[],dr=[];for(let c=0;c<n;c++){
      const x=(c+.5)*NAV_STEP,y=(r+.5)*NAV_STEP,st=staticCollision(x,y),dy=dynamicCollision(x,y);
      br.push(st||dy);sr.push(st);dr.push(!st&&dy);
    }blocked.push(br);stat.push(sr);dynamic.push(dr)}
    return {n,blocked,stat,dynamic};
  }
  function paintGrid(){const layer=$('gridLayer');empty(layer);if(!state.showGrid)return;
    // Fixed 5 x 5 field lattice, 480 mm a side. Draw solid visible cell borders.
    for(let i=1;i<5;i++){
      const v=i*state.gridStep;
      node('line',{x1:fx(v),y1:fy(0),x2:fx(v),y2:fy(FIELD),stroke:'#425c75','stroke-opacity':.7,'stroke-width':1.45},layer);
      node('line',{x1:fx(0),y1:fy(v),x2:fx(FIELD),y2:fy(v),stroke:'#425c75','stroke-opacity':.7,'stroke-width':1.45},layer);
    }
    // One number at the centre of every cell, with the exact snake numbering in the reference.
    for(let r=0;r<5;r++)for(let c=0;c<5;c++){
      const id=numberedCell(c,r),x=(c+.5)*state.gridStep,y=(r+.5)*state.gridStep;
      node('text',{x:fx(x),y:fy(y)+5,'text-anchor':'middle','font-family':'Consolas, monospace',
        'font-size':17,'font-weight':700,fill:'#35495c','fill-opacity':.86},layer,String(id));
    }
  }
  // One visible block per obstacle. Fine-grid counts are diagnostic only.
  function paintInflation(grid){
    empty($('inflationLayer'));
    $('affectedCells').textContent=state.obstacles.length+' / 2 个封锁格';
    $('blockedMetric').textContent=grid.blocked.flat().filter(Boolean).length+' / '+grid.n*grid.n;
  }
  function paintObstacles(){const layer=$('obstacleLayer');empty(layer);
    state.obstacles.forEach((o,i)=>{
      const box=occupiedSector(o),cx=fx((box[0]+box[2])/2),cy=fy((box[1]+box[3])/2);
      // Exactly one red grid cell, independent of the yellow keepout map.
      worldRect(layer,box,{fill:'#de3b51','fill-opacity':.43,stroke:'#bd1e37','stroke-width':3,rx:3});
      node('path',{d:`M ${cx-20} ${cy-20} L ${cx+20} ${cy+20} M ${cx+20} ${cy-20} L ${cx-20} ${cy+20}`,
        stroke:'#fff','stroke-width':6,'stroke-linecap':'round'},layer);
      // The black disc locates the actual 50mm obstacle within the blocked sector.
      node('circle',{cx:fx(o.x),cy:fy(o.y),r:BLOCK_DIAMETER/2*SCALE,fill:'#15181d',stroke:'#fff','stroke-width':1.8},layer);
      node('text',{x:cx,y:cy-state.gridStep*SCALE*.34,'text-anchor':'middle','font-size':12,'font-weight':900,
        fill:'#fff',stroke:'#af1930','stroke-width':3,'paint-order':'stroke'},layer,`CELL ${numberedCell(guideId(o).c,guideId(o).r)}`);
    });
    $('obstacleMetric').innerHTML=state.obstacles.length+' <em>/ 2 格</em>';
    $('physicalCells').textContent=state.obstacles.map((o,i)=>{
      const g=guideId(o);return `${i+1}: #${numberedCell(g.c,g.r)}（第${g.c+1}列/第${g.r+1}行）`;
    }).join(' / ')||'未放置';
  }
  function resetMaterials(){const p=parseTask(state.task),b1=p.valid?p.batch1:[];
    state.sim={running:false,phase:'idle',lastFrame:0,elapsed:0,legIndex:0,legDistance:0,actionIndex:0,actionElapsed:0,actions:[],logs:[],read:false,
      batch:1,raw:b1.map((it,i)=>({color:it.color,slot:i})),cargo:[],process:[null,null,null],storage:[[],[],[]],complete:false,poseOverride:null,stationArrival:null,motion:null,armExtension:0,armMove:null,stationKey:null,adjustCount:0,alignmentError:null};
    $('playBtn').textContent='▶ 开始完整任务仿真';$('simStatusTag').textContent='待机';$('actionDisplay').textContent='等待启动';$('elapsedDisplay').textContent='0.0 s';
    $('operationLog').textContent='按二维码编码执行两批搬运；每次到站后先对位，夹爪伸缩再逐件作业。';$('alignDisplay').textContent='待命 · 夹爪已收回';
    renderMaterials();
  }
  function materialBadge(item,idx){const chip=document.createElement('div');chip.className='mat-chip'+(item?'':' empty');
    if(item){const dot=document.createElement('span');dot.className='mat-dot';dot.style.background=TONES[item.color];chip.append(dot);
      chip.append(document.createTextNode(`${idx}: ${TONE_NAMES[item.color]}`+(item.count>1?` ×${item.count}`:'')));
    }else chip.textContent=`${idx}: —`;
    return chip;
  }
  function renderMaterials(){const sim=state.sim;if(!sim)return;
    for(const [id,items] of [
      ['rawMaterials',[0,1,2].map(i=>sim.raw.find(x=>x.slot===i))],
      ['cargoMaterials',sim.cargo],
      ['processMaterials',sim.process],
      ['storageMaterials',sim.storage.map(arr=>arr.length?{color:arr[arr.length-1],count:arr.length}:null)]
    ]){const dest=$(id);empty(dest);if(id==='cargoMaterials'&&!items.length){dest.append(materialBadge(null,'空载'));continue}
      items.forEach((item,i)=>dest.append(materialBadge(item,id==='cargoMaterials'?i+1:i+1)));
    }
    paintMaterials();
  }
  function paintMaterials(){const layer=$('labelsLayer');empty(layer);const sim=state.sim;if(!sim)return;
    const turnGroup=node('g',{id:'turntableGroup'},layer);
    for(const item of sim.raw){const angle=(-90+item.slot*120)*Math.PI/180;
      const x=fx(state.rawX+.105*Math.cos(angle)),y=fy(state.rawY+.105*Math.sin(angle));
      node('circle',{cx:x,cy:y,r:10.5,fill:TONES[item.color],stroke:'#263744','stroke-width':2},turnGroup);
      node('circle',{cx:x-2.7,cy:y-3.0,r:2.5,fill:'#fff','fill-opacity':.58},turnGroup);
    }
    for(const [key,rings] of [['process',POS.process],['storage',POS.storage]])rings.forEach(([x,y],i)=>{
      const items=key==='process'?(sim.process[i]?[sim.process[i].color]:[]):sim.storage[i];if(!items.length)return;
      const top=items[items.length-1],cx=fx(x),cy=fy(y);
      if(items.length>1)node('circle',{cx:cx+4,cy:cy+4,r:9.5,fill:TONES[items[0]],stroke:'#394f5d','stroke-width':1.3},layer);
      node('circle',{cx,cy,r:9.5,fill:TONES[top],stroke:'#fff','stroke-width':1.9},layer);
      if(items.length>1)node('text',{x:cx+11,y:cy-8,'font-size':11,'font-weight':800,fill:'#183852',stroke:'#fff','stroke-width':2.6,'paint-order':'stroke'},layer,'×2');
    });
    if(state.mode==='mission')for(const [k,p] of Object.entries(STATIONS)){
      node('circle',{cx:fx(p.x),cy:fy(p.y),r:4.6,fill:'#58e6ca',stroke:'#fff','stroke-width':1.2},layer);
    }
    updateRotor(performance.now());
  }
  function updateRotor(ts){
    if(state.runSource==='live'){
      const rotor=$('turntableGroup');if(rotor)rotor.setAttribute('transform',`rotate(0 ${fx(state.rawX)} ${fy(state.rawY)})`);
      $('turnState').textContent='实车模式：未接入转盘传感器，角度不代表实测';
      $('turnSummary').textContent='旋转参数仅对模拟运行生效';return;
    }
    if(!state.turnOrigin)state.turnOrigin=ts;
    const seconds=Math.max(0,(ts-state.turnOrigin)/1000),moving=state.rotorAngle/state.rotorRate;
    const cycleDuration=state.rotorDwell+moving,cycle=Math.floor(seconds/cycleDuration),part=seconds%cycleDuration;
    const done=part<state.rotorDwell?0:Math.min(state.rotorAngle,(part-state.rotorDwell)*state.rotorRate);
    const angle=cycle*state.rotorAngle+done,sign=state.turnDirection==='cw'?1:-1;
    const g=$('turntableGroup');if(g)g.setAttribute('transform',`rotate(${(sign*angle).toFixed(3)} ${fx(state.rawX)} ${fy(state.rawY)})`);
    $('turnState').textContent=part<state.rotorDwell?`停止 ${(state.rotorDwell-part).toFixed(1)} s，下一次转 ${state.rotorAngle}°`:
      `旋转 ${done.toFixed(0)}/${state.rotorAngle}° · ${state.rotorRate}°/s`;
    $('turnSummary').textContent=`物料间隔 120° · 每次转 ${state.rotorAngle}° · ${state.rotorRate}°/s（${(360/state.rotorRate).toFixed(1)}s/圈）· 停顿 ${state.rotorDwell.toFixed(1)}s`;
  }
  function paintRobot(point=state.start,heading=0){const layer=$('robotLayer');empty(layer);
    if(state.runSource==='live'){if(!live.pose)return;point={x:live.pose.x,y:live.pose.y};heading=90-live.pose.yaw*180/Math.PI;}
    const sx=fx(state.start.x),sy=fy(state.start.y);
    node('circle',{cx:sx,cy:sy,r:9.5,fill:'none',stroke:'#00a78f','stroke-width':3},layer);
    node('circle',{cx:sx,cy:sy,r:3.3,fill:'#00c6ad'},layer);
    if(state.mode==='direct'){node('circle',{cx:fx(state.goal.x),cy:fy(state.goal.y),r:10,fill:'none',stroke:'#dd5b8d','stroke-width':3},layer)}
    if(state.runSource==='simulation'&&state.sim?.poseOverride){point=state.sim.poseOverride.point;heading=state.sim.poseOverride.heading;}
    const x=fx(point.x),y=fy(point.y),w=.26*SCALE,h=.29*SCALE;
    const g=node('g',{transform:`rotate(${heading.toFixed(2)} ${x} ${y})`},layer);
    node('rect',{x:x-w/2,y:y-h/2,width:w,height:h,rx:9,fill:'#146f88','fill-opacity':.44,stroke:'#115d80','stroke-width':2.1},g);
    // Telescopic arm is visibly coupled to the chassis front, and NEVER animates from /odom data.
    const extension=state.runSource==='simulation'?(state.sim?.armExtension||0):0;
    const armPx=extension*SCALE,front=y-h/2,tip=front-armPx;
    const armGroup=node('g',{'data-extension-mm':Math.round(extension*1000)},g);
    node('rect',{x:x-9,y:tip-3,width:18,height:Math.max(15,armPx+18),rx:4,fill:'#bccbd7',stroke:'#344f65','stroke-width':1.3},armGroup);
    node('line',{x1:x-5,y1:front+7,x2:x-5,y2:tip,stroke:'#8ee2f1','stroke-width':2.2},armGroup);
    node('line',{x1:x+5,y1:front+7,x2:x+5,y2:tip,stroke:'#8ee2f1','stroke-width':2.2},armGroup);
    node('rect',{x:x-11,y:tip-5,width:22,height:7,rx:2,fill:'#ffbd71',stroke:'#684d39','stroke-width':1},armGroup);
    const jaw=window.Sim2RealManualGripOpen&&state.runSource==='simulation'?8:0;
    node('path',{d:`M ${x-10-jaw} ${tip-5} L ${x-14-jaw} ${tip-12} M ${x+10+jaw} ${tip-5} L ${x+14+jaw} ${tip-12}`,stroke:'#ffe0aa','stroke-width':3,'stroke-linecap':'round',fill:'none'},armGroup);
    node('rect',{x:x-12,y:y-17,width:24,height:34,rx:7,fill:'#075370',stroke:'#fff','stroke-width':1.6},g);
    node('path',{d:`M ${x-7} ${y-8} L ${x} ${y-14} L ${x+7} ${y-8}`,fill:'none',stroke:'#fff','stroke-width':2.1},g);
    const cargo=state.sim?.cargo||[];cargo.forEach((it,i)=>node('circle',{cx:x-7+(i%3)*7,cy:y+4,r:3.2,fill:TONES[it.color],stroke:'#fff','stroke-width':.5},g));
  }
  function segmentSafe(a,b){const d=Math.hypot(a.x-b.x,a.y-b.y),steps=Math.max(1,Math.ceil(d/.015));
    for(let k=0;k<=steps;k++){const t=k/steps;if(!safe(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t))return false}return true}
  function nearestCell(p,grid,maxDistance=.34){let best=null,min=Infinity;
    for(let r=0;r<grid.n;r++)for(let c=0;c<grid.n;c++){
      if(grid.blocked[r][c])continue;const x=(c+.5)*NAV_STEP,y=(r+.5)*NAV_STEP,d=Math.hypot(x-p.x,y-p.y);
      if(d<min&&segmentSafe(p,{x,y})){min=d;best={x,y,c,r}}
    }return min<=maxDistance?best:null;
  }
  function findPath(graph,grid,start,goal){const a=nearestCell(start,grid),b=nearestCell(goal,grid);if(!a||!b)return null;
    const result=GridPlanning.solve(graph,a.r*grid.n+a.c,b.r*grid.n+b.c,state.planner);
    if(!result.path)return null;
    const path=result.path.map(k=>GridPlanning.cell(k,grid.n));
    // To compare the search algorithms fairly, all keep identical graph costs; only safe station links are added.
    if(safe(start.x,start.y)&&segmentSafe(start,path[0]))path.unshift({...start});
    if(safe(goal.x,goal.y)&&segmentSafe(path[path.length-1],goal))path.push({...goal});
    let length=0;for(let i=1;i<path.length;i++)length+=Math.hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);
    return {path,length,start:a,end:b,expanded:result.expanded};
  }
  function planRoutes(grid){const task=parseTask(state.task),sections=state.mode==='direct'?[{key:'goal',label:'自定义起终点',target:state.goal}]:
    STAGE_DEFS.map(([key,label])=>({key,label:key==='return'?`08 到达启停区 ${endZone()}`:label,target:key==='return'?endStation():STATIONS[key]}));
    const graph=GridPlanning.buildGraph(grid,segmentSafe);
    if(!safe(state.start.x,state.start.y))return {ok:false,reason:'起点不安全：当前占用格/禁行区与 290mm 车体包络相交。',sections,legs:[],task};
    let from=state.start,total=0;const legs=[];
    for(const stage of sections){
      if(!safe(stage.target.x,stage.target.y))return {ok:false,reason:`「${stage.label}」目标点被封锁或车体无法进入。`,sections,legs,task};
      const result=findPath(graph,grid,from,stage.target);
      if(!result)return {ok:false,reason:`「${stage.label}」不可达。请擦除相应封锁格，或调整任务目标。`,sections,legs,task};
      legs.push({...stage,...result});total+=result.length;from=result.path[result.path.length-1];
    }
    const fullPath=legs.flatMap((leg,i)=>i?leg.path.slice(1):leg.path);
    return {ok:true,legs,total,sections,task,fullPath,expanded:legs.reduce((v,leg)=>v+leg.expanded,0)};
  }
  function paintPath(){const layer=$('pathLayer');empty(layer);const p=state.plan;if(!p?.legs?.length)return;
    const current=Math.min(state.sim?.legIndex||0,p.legs.length-1);
    const draw=(path,alpha,width,color)=>{if(!path?.length)return;const d=path.map((v,i)=>`${i?'L':'M'}${fx(v.x).toFixed(1)} ${fy(v.y).toFixed(1)}`).join(' ');
      node('path',{d,fill:'none',stroke:'#ffffff','stroke-opacity':alpha*.8,'stroke-width':width+3,'stroke-linejoin':'round','stroke-linecap':'round'},layer);
      node('path',{d,fill:'none',stroke:color,'stroke-opacity':alpha,'stroke-width':width,'stroke-linejoin':'round','stroke-linecap':'round'},layer);
    };
    if(state.pathView==='global'){
      draw(p.fullPath||p.legs.flatMap((leg,i)=>i?leg.path.slice(1):leg.path),.96,4.4,'#0cae9c');
      p.legs.forEach((leg,i)=>{const last=leg.path[leg.path.length-1];if(!last)return;
        node('circle',{cx:fx(last.x),cy:fy(last.y),r:i===p.legs.length-1?8:5,fill:i===p.legs.length-1?'#fa9c34':'#119e94',stroke:'#fff','stroke-width':1.8},layer);
      });
    } else draw(p.legs[current].path,1,4.5,'#08b7a5');
  }
  function decodedTask(){const area=$('decodedTask');empty(area);const p=parseTask(state.task);if(!p.valid)return;
    for(const [name,batch,stack] of [['首批抓取→环位',p.batch1,false],['第二批粗加工',p.batch2,false],['第二批暂存码垛',p.stack,true]]){
      const row=document.createElement('div');row.className='decoded-row';const lbl=document.createElement('span');lbl.textContent=name;const div=document.createElement('div');div.className='decoded-colors';
      batch.forEach((part,i)=>{const chip=document.createElement('div');chip.className='decoded-color';const dot=document.createElement('span');dot.className='mat-dot';dot.style.background=TONES[part.color];chip.append(dot,document.createTextNode(`${i+1}.${TONE_NAMES[part.color]} → ${stack?part.stackRing:part.ring}号环`));div.append(chip)});
      row.append(lbl,div);area.append(row);
    }
  }
  function paintReport(){const p=state.plan,ok=p?.ok,valid=p?.task?.valid;
    $('routeTag').textContent=ok?'✓ 全程路径可通行':'✕ 存在不可达路段';$('routeTag').className='result-tag '+(ok?'ok':'fail');
    $('distanceMetric').innerHTML=ok?`${f(p.total)} <em>m</em>`:'— <em>m</em>';
    $('distanceFoot').textContent=ok?`全程 8 段联合显示 · ${state.planner==='dstar'?'D* Lite':state.planner==='dijkstra'?'Dijkstra':'A*'} · 50 mm 内部栅格`:'有阻断路段';
    $('reportLength').textContent=ok?f(p.total)+' m':'不可达';
    // One QR scan (1 s) plus 24 material actions (6 raw picks, 6 process drops, 6 process pickups, 6 storage drops).
    const stationCount=state.mode==='mission'?7:0;const travel=p.total/state.velocity;
    const actionsTime=state.mode==='mission'?1+24*state.handling:0;
    const moveOverhead=stationCount*(.5+2*state.alignOffset/state.alignSpeed);
    const armOverhead=state.mode==='mission'?24*2*state.armReach/state.armSpeed:0;
    $('reportTime').textContent=ok?'≈ '+f(travel+actionsTime+moveOverhead+armOverhead,1)+' s':'—';
    $('legsMetric').textContent=`${p.legs.length} / ${p.sections.length}`;
    const timeline=$('stageTimeline');empty(timeline);p.sections.forEach((section,i)=>{
      const div=document.createElement('div');div.className='stage-pill';if(p.legs[i])div.classList.add('success');else if(!ok&&i===p.legs.length)div.classList.add('failed');
      if(state.sim?.running&&i===state.sim.legIndex)div.classList.add('active');
      const strong=document.createElement('b'),small=document.createElement('small');strong.textContent=section.label;small.textContent=p.legs[i]?f(p.legs[i].length)+' m':(!ok&&i===p.legs.length?'通道被封锁':'等待规划');div.append(strong,small);timeline.append(div);
    });
    const validator=$('taskValidation');validator.classList.toggle('error',!valid);
    validator.textContent=valid?'✓ 任务码可执行，第二批粗加工按第 4 组环号，暂存按颜色匹配第一批码垛。':p.task.error;
    decodedTask();$('playBtn').disabled=state.runSource==='live'||!ok||(state.mode==='mission'&&!valid);
    if(!ok)tell(p.reason,true);else if(!valid&&state.mode==='mission')tell('路线可达，但二维码任务码无效，需修改后才能播放抓取与码垛。',true);
    else tell(`✓ ${state.planner==='dstar'?'D* Lite':state.planner==='dijkstra'?'Dijkstra':'A*'} 全程规划：${p.legs.length} 站，总长 ${f(p.total)} m，终点启停区 ${endZone()}。最多封锁 2 个 '+Math.round(state.gridStep*1000)+'mm 栅格，严禁从封锁格穿行。`);
  }
  function comparePlanners(){const grid=mapGrid(),rows=[];const original=state.planner;
    try{for(const [id,name] of [['astar','A*'],['dijkstra','Dijkstra'],['dstar','D* Lite']]){
      state.planner=id;const t0=performance.now(),res=planRoutes(grid);rows.push({name,dist:res.ok?f(res.total)+' m':'不可达',expanded:res.expanded||0,ms:Math.round(performance.now()-t0)});
    }}finally{state.planner=original}
    const el=$('compareResult');el.replaceChildren();const head=document.createElement('strong');head.textContent='同一地图、两个障碍物、相同任务顺序的算法对照';el.append(head);
    const table=document.createElement('table');table.innerHTML='<thead><tr><th>算法</th><th>全程长度</th><th>扩展节点</th><th>运行耗时</th></tr></thead>';
    const tbody=document.createElement('tbody');for(const r of rows){const tr=document.createElement('tr');for(const v of [r.name,r.dist,String(r.expanded),r.ms+' ms']){const td=document.createElement('td');td.textContent=v;tr.append(td)}tbody.append(tr)}
    table.append(tbody);el.append(table);
    const note=document.createElement('small');note.textContent='D* Lite 为标准反向 D* Lite 搜索；当前静态障碍变更后重新初始化搜索，不宣称增量修复。耗时受浏览器性能影响。';el.append(note);
  }
  function recalc(){const wasRunning=state.sim?.running;const result=$('compareResult');if(result)result.replaceChildren();resetMaterials();const grid=mapGrid();state.plan=planRoutes(grid);
    paintField();paintGrid();paintInflation(grid);paintObstacles();paintReport();paintPath();renderMaterials();paintLiveTrail();paintRobot();
    const visibleN=Math.round(FIELD/state.gridStep);$('gridMetric').textContent=visibleN+' × '+visibleN;
    $('gridFoot').textContent='480 mm · 0–24 蛇形编号；50 mm 内部寻路';
    $('gridDetail').textContent='480 × 480 mm / 格 · 5 × 5 / 25 格';
    syncRunUI();
    if(wasRunning)tell('地图设置已更新，仿真已重置并重新规划。');
    persist();if(shared.ready&&!shared.inbound)markDirty();
  }
  function taskActions(i){const p=parseTask(state.task);if(!p.valid)return [];
    const batch=i<4?p.batch1:p.batch2;
    if(i===0)return [{kind:'scan',name:'读取二维码 '+state.task}];
    if(i===1||i===4)return batch.map(t=>({kind:'pickRaw',color:t.color,name:`抓取${TONE_NAMES[t.color]}色原料 → 车载` }));
    if(i===2||i===5)return [...batch.map(t=>({kind:'placeProcess',color:t.color,ring:t.ring,name:`${TONE_NAMES[t.color]}色 → 粗加工 ${t.ring} 号环`})),
      ...batch.map(t=>({kind:'pickProcess',color:t.color,ring:t.ring,name:`从粗加工 ${t.ring} 号环收回${TONE_NAMES[t.color]}色物料`}))];
    if(i===3)return p.batch1.map(t=>({kind:'placeStorage',color:t.color,ring:t.ring,name:`${TONE_NAMES[t.color]}色 → 暂存 ${t.ring} 号环`}));
    if(i===6)return p.stack.map(t=>({kind:'stack',color:t.color,ring:t.stackRing,name:`${TONE_NAMES[t.color]}色 → 暂存 ${t.stackRing} 号环同色码垛`}));
    return [];
  }
  function logWork(text){const sim=state.sim;sim.logs.push(text);$('operationLog').textContent=sim.logs.slice(-3).join(' / ')}
  function executeAction(act){const sim=state.sim;
    if(act.kind==='scan'){sim.read=true;logWork('已读取 '+state.task);return}
    if(act.kind==='pickRaw'){const i=sim.raw.findIndex(v=>v.color===act.color);if(i===-1){logWork('抓取失败：转盘未找到 '+TONE_NAMES[act.color]);return}
      const [picked]=sim.raw.splice(i,1);sim.cargo.push({color:picked.color});logWork('已夹取'+TONE_NAMES[act.color]+'，放入车载仓');}
    if(act.kind==='placeProcess'){const i=sim.cargo.findIndex(v=>v.color===act.color);if(i<0){logWork('操作失败：车上没有对应物料');return}
      sim.process[act.ring-1]=sim.cargo.splice(i,1)[0];logWork('已放置粗加工区 '+act.ring+' 号环');}
    if(act.kind==='pickProcess'){const item=sim.process[act.ring-1];if(!item||item.color!==act.color){logWork('加工区抓取颜色/环号不符');return}
      sim.cargo.push(item);sim.process[act.ring-1]=null;logWork('已收回粗加工区 '+act.ring+' 号环物料');}
    if(act.kind==='placeStorage'||act.kind==='stack'){const i=sim.cargo.findIndex(v=>v.color===act.color);if(i<0){logWork('暂存区放置失败：车上缺少物料');return}
      if(act.kind==='stack'&&sim.storage[act.ring-1][0]!==act.color){logWork('码垛失败：底层颜色不匹配');return}
      sim.cargo.splice(i,1);sim.storage[act.ring-1].push(act.color);logWork((act.kind==='stack'?'同色码垛':'暂存放置')+' '+TONE_NAMES[act.color]+' → '+act.ring+' 号环');}
    renderMaterials();paintRobot(robotPosition().point,robotPosition().heading);
  }
  // The robot can only start manipulation after a physically visible, bounded station adjustment.
  // This is an open-loop *illustration*. The real robot mode is driven solely by odometry.
  const STATION_FACE={qr:90,raw:0,process:180,storage:-90};
  function angleDelta(a,b){return ((b-a+540)%360)-180;}
  function interpolatedPose(a,b,u){const t=Math.max(0,Math.min(1,u));const smooth=t*t*(3-2*t);
    return {point:{x:a.point.x+(b.point.x-a.point.x)*smooth,y:a.point.y+(b.point.y-a.point.y)*smooth},heading:a.heading+angleDelta(a.heading,b.heading)*smooth};}
  function adjustedStationPose(stage,action){
    const base=STATIONS[stage],p={x:base.x,y:base.y};let facing=STATION_FACE[stage]??0;
    if(stage==='process'&&state.ringFollow&&action?.ring)p.x=POS.process[action.ring-1][0];
    if(stage==='storage'&&state.ringFollow&&action?.ring)p.y=POS.storage[action.ring-1][1];
    const offset=state.alignOffset;
    if(stage==='qr')p.x+=offset;else if(stage==='raw')p.y+=offset;
    else if(stage==='process')p.y-=offset;else if(stage==='storage')p.x-=offset;
    const smallYaw=state.alignYaw*(stage==='qr'||stage==='process'?1:-1);
    return {point:p,heading:facing+smallYaw};
  }
  function setAlignmentText(msg){$('alignDisplay').textContent=msg;}
  function failStation(reason){const sim=state.sim;sim.running=false;sim.phase='blocked';sim.motion=null;sim.armMove=null;
    $('simStatusTag').textContent='作业对位不可达';$('simStatusTag').className='result-tag fail';$('playBtn').textContent='↻ 检查障碍后重试';
    $('actionDisplay').textContent=reason;setAlignmentText('停止：对位或车体运动发生碰撞约束冲突');tell(reason,true);logWork('✕ '+reason);
  }
  function beginPoseMove(to,phase,title){const sim=state.sim;
    const from=sim.poseOverride?{point:{...sim.poseOverride.point},heading:sim.poseOverride.heading}:robotPosition();
    if(!safe(to.point.x,to.point.y)||!segmentSafe(from.point,to.point)){
      failStation('无法完成 '+title+'：微调轨迹会进入封锁格或禁行区。请调整障碍物 / 工位参数。');return;
    }
    const d=Math.hypot(to.point.x-from.point.x,to.point.y-from.point.y),dyaw=Math.abs(angleDelta(from.heading,to.heading));
    const duration=Math.max(.18,d/state.alignSpeed,dyaw/120);
    sim.phase=phase;sim.motion={from,to,elapsed:0,duration,title};sim.poseOverride=from;
    $('actionDisplay').textContent=title;setAlignmentText(`${title} · ${(d*1000).toFixed(0)} mm / ${dyaw.toFixed(0)}°`);
  }
  function beginArmMove(extend){const sim=state.sim;const stage=sim.stationKey;
    if(stage==='qr'||state.armReach<=0){if(extend)beginWork();else afterRetraction();return;}
    sim.phase=extend?'extending':'retracting';sim.armMove={from:sim.armExtension,to:extend?state.armReach:0,elapsed:0};
    const action=sim.actions[sim.actionIndex];$('actionDisplay').textContent=(extend?'伸出夹爪：':'收回夹爪：')+(action?.name||'准备下一站');
    setAlignmentText((extend?'伸出':'收回')+'夹爪 · '+Math.round(sim.armMove.to*1000)+' mm 目标行程');
  }
  function beginWork(){const sim=state.sim;sim.phase='working';sim.actionElapsed=0;
    const action=sim.actions[sim.actionIndex];$('actionDisplay').textContent=action?.name||'执行作业';
    setAlignmentText((sim.stationKey==='qr'?'相机对位完成 · 扫描二维码':'夹爪已伸出 '+Math.round(sim.armExtension*1000)+' mm · 抓放作业')+' · 操作中');
  }
  function afterRetraction(){const sim=state.sim;if(sim.actionIndex<sim.actions.length){
      const target=adjustedStationPose(sim.stationKey,sim.actions[sim.actionIndex]);
      beginPoseMove(target,'repositioning','逐环跟位 · '+sim.actions[sim.actionIndex].name);
      return;
    }
    if(sim.stationArrival)beginPoseMove(sim.stationArrival,'recovering','作业完成 · 回到规划停车位');else finishLeg();
  }
  function arrive(){const sim=state.sim,i=sim.legIndex;
    sim.actions=state.mode==='mission'?taskActions(i):[];sim.actionIndex=0;sim.actionElapsed=0;
    if(sim.actions.length){sim.stationKey=state.plan.sections[i].key;
      sim.stationArrival=robotPosition();sim.poseOverride=sim.stationArrival;sim.adjustCount=0;
      const target=adjustedStationPose(sim.stationKey,sim.actions[0]);
      beginPoseMove(target,'aligning','到达'+state.plan.sections[i].label+' · 视觉/机械对位');return;
    }
    finishLeg();
  }
  function finishLeg(){const sim=state.sim;
    sim.poseOverride=null;sim.motion=null;sim.stationArrival=null;sim.stationKey=null;sim.armMove=null;sim.armExtension=0;
    setAlignmentText('工位操作结束 · 夹爪已收回');
    if(state.mode==='mission'&&sim.legIndex===3){sim.batch=2;const t=parseTask(state.task);sim.raw=t.batch2.map((v,i)=>({color:v.color,slot:i}));renderMaterials();logWork('第一批完成，转盘补充第二批（示意补料）')}
    if(sim.legIndex>=state.plan.legs.length-1){sim.phase='done';sim.running=false;sim.complete=true;
      $('playBtn').textContent='↻ 重新开始仿真';$('simStatusTag').textContent='任务完成';$('actionDisplay').textContent=state.mode==='mission'?`完成两批同色码垛，并到达启停区 ${endZone()}`:'到达自定义终点';
      logWork('✓ 已完成完整网页模拟流程');const pose=robotPosition();paintRobot(pose.point,pose.heading);return}
    sim.legIndex++;sim.legDistance=0;sim.phase='driving';sim.actions=[];sim.actionIndex=0;
    $('actionDisplay').textContent='行驶至 '+state.plan.sections[sim.legIndex].label;
    paintPath();paintReport();
  }
  function robotPosition(){const sim=state.sim,leg=state.plan?.legs?.[Math.min(sim?.legIndex||0,state.plan.legs.length-1)];
    if(!leg?.path?.length)return {point:state.start,heading:0};
    let remain=sim.legDistance;
    for(let i=1;i<leg.path.length;i++){const a=leg.path[i-1],b=leg.path[i],len=Math.hypot(a.x-b.x,a.y-b.y);
      if(remain<=len){const t=len>0?remain/len:0;return {point:{x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t},heading:Math.atan2(b.y-a.y,b.x-a.x)*-180/Math.PI+90}}
      remain-=len;
    }
    if(leg.path.length>1){const a=leg.path.at(-2),b=leg.path.at(-1);return {point:b,heading:Math.atan2(b.y-a.y,b.x-a.x)*-180/Math.PI+90};}
    return {point:leg.path[leg.path.length-1],heading:0};
  }
  function tickSimulation(dt){const sim=state.sim;if(state.runSource==='live'||!sim?.running||!state.plan?.ok)return;
    sim.elapsed+=dt;$('elapsedDisplay').textContent=f(sim.elapsed,1)+' s';
    if(sim.phase==='driving'){const leg=state.plan.legs[sim.legIndex];sim.legDistance=Math.min(leg.length,sim.legDistance+dt*state.velocity);
      const pos=robotPosition();paintRobot(pos.point,pos.heading);
      if(sim.legDistance>=leg.length-1e-7){arrive();paintPath();paintReport()}
    }else if(['aligning','repositioning','recovering'].includes(sim.phase)){
      const motion=sim.motion;if(!motion){failStation('缺少工位对位运动数据');return}
      motion.elapsed=Math.min(motion.duration,motion.elapsed+dt);
      sim.poseOverride=interpolatedPose(motion.from,motion.to,motion.elapsed/motion.duration);
      paintRobot();
      if(motion.elapsed>=motion.duration-1e-6){const phase=sim.phase;sim.motion=null;
        if(phase==='recovering')finishLeg();else {sim.adjustCount++;beginArmMove(true)}
      }
    }else if(sim.phase==='extending'||sim.phase==='retracting'){
      const m=sim.armMove;if(!m){failStation('夹爪伸缩状态异常');return}
      m.elapsed+=dt;const duration=Math.max(.08,Math.abs(m.to-m.from)/state.armSpeed);
      sim.armExtension=m.from+(m.to-m.from)*Math.min(1,m.elapsed/duration);paintRobot();
      if(m.elapsed>=duration-1e-6){sim.armExtension=m.to;sim.armMove=null;if(sim.phase==='extending')beginWork();else afterRetraction()}
    }else if(sim.phase==='working'){
      sim.actionElapsed+=dt;const duration=sim.actions[sim.actionIndex]?.kind==='scan'?1.0:state.handling;
      if(sim.actionElapsed>=duration){sim.actionElapsed=0;executeAction(sim.actions[sim.actionIndex]);sim.actionIndex++;
        if(sim.stationKey==='qr')afterRetraction();else beginArmMove(false);
      }
    }
  }
  function animationLoop(ts){if(!state.sim)requestAnimationFrame(animationLoop);else{
      const last=state.sim.lastFrame,dt=last?Math.max(0,Math.min((ts-last)/1000,.07)):0;
      state.sim.lastFrame=ts;updateRotor(ts);tickSimulation(dt);requestAnimationFrame(animationLoop);
    }}
  function startPause(){if(state.runSource==='live'){tell('实车模式由 ROS2 /odom 驱动，不能播放预设仿真动画。');return;}if(!state.plan?.ok)return;
    if(state.sim.running){state.sim.running=false;$('playBtn').textContent='▶ 继续仿真';$('simStatusTag').textContent='已暂停';return}
    if(state.sim.complete)resetMaterials();
    state.sim.running=true;$('simStatusTag').textContent='运行中';$('simStatusTag').className='result-tag active';
    if(state.sim.phase==='idle'){state.sim.phase='driving';$('actionDisplay').textContent='行驶至 '+state.plan.sections[0].label}
    $('playBtn').textContent='Ⅱ 暂停仿真';paintPath();
  }
  function posAt(evt){const svg=$('fieldSvg'),p=svg.createSVGPoint();p.x=evt.clientX;p.y=evt.clientY;const t=svg.getScreenCTM();if(!t)return null;
    const q=p.matrixTransform(t.inverse());return {x:(q.x-60)/SCALE,y:(740-q.y)/SCALE};
  }
  function chooseTool(tool){state.tool=tool;document.querySelectorAll('[data-tool]').forEach(e=>e.classList.toggle('active',e.dataset.tool===tool))}
  function onMapClick(evt){const p=posAt(evt);if(!p||!inField(p.x,p.y))return;
    if(state.tool==='raw'){
      if(!inFixtureRange('raw',p.x)||p.y<2.10){tell('原料转盘只能在上边缘 X=1100–1300 mm 范围内调整；请点击场地上方靠近转盘的位置。',true);return;}
      changeFixture('raw',Math.round(p.x*100)/100);
      if(p.y>=2.25)changeFixture('rawY',Math.round(Math.min(2.55,Math.max(2.25,p.y))*100)/100);return;
    }
    if(state.tool==='qr'){
      if(!inFixtureRange('qr',p.y)||p.x<2.05){tell('二维码板只能在右边缘 Y=1100–1300 mm 范围内调整；请点击场地右侧靠近二维码的位置。',true);return;}
      changeFixture('qr',Math.round(p.y*100)/100);return;
    }
    if(state.tool==='obstacle'||state.tool==='erase'){
      const clicked=guideId(p),near=state.obstacles.findIndex(o=>{const g=guideId(o);return clicked.c===g.c&&clicked.r===g.r});
      if(state.tool==='erase'){
        if(near<0){tell('此格没有障碍物；可点击带 × 的红色格子删除。',true);return}
        state.obstacles.splice(near,1);recalc();return;
      }
      if(near>=0){state.obstacles.splice(near,1);recalc();tell('已再次点击并取消当前障碍格；重新规划完成。');return;}
      if(state.obstacles.length>=2){tell('比赛场景最多放置 2 个障碍物，请先删除一个。',true);return;}
      const targetCell=guideId(p);const center={x:+((targetCell.c+.5)*state.gridStep).toFixed(4),y:+((targetCell.r+.5)*state.gridStep).toFixed(4)};
      if(!physicalObstacleFits(center)){tell('该格中心不可用于放障碍：不能占用黄色区、作业区、转盘或启停区。',true);return;}
      if(state.obstacles.some(o=>{const a=guideId(o),b=guideId(p);return a.c===b.c&&a.r===b.r;})){tell('一个大格只能放置一个障碍物。',true);return;}
      state.obstacles.push(center);recalc();tell('已封锁 #'+numberedCell(targetCell.c,targetCell.r)+' 号格；再次点击同一格即可取消。');return;
    }
    if(!safe(p.x,p.y)){tell('此位置无法放置车体中心，可能与禁行区或碰撞包络冲突。',true);return}
    const target={x:+p.x.toFixed(3),y:+p.y.toFixed(3)};
    if(state.tool==='start'){state.start=target;state.startZone=0;zoneButtons()}
    if(state.tool==='goal'){state.goal=target;state.mode='direct';$('modeSelect').value='direct'}
    recalc();
  }
  function zoneButtons(){$('start1Btn').classList.toggle('active-choice',state.startZone===1);$('start2Btn').classList.toggle('active-choice',state.startZone===2)}
  function useZone(id){state.startZone=id;state.start={x:2.25,y:id===1?2.25:.15};
    if(state.runSource==='live'){$('liveAnchorX').value='2.25';$('liveAnchorY').value=id===1?'2.25':'0.15';resetLiveAnchor();}
    zoneButtons();recalc()}
  function randomObstacles(){const points=[];
    for(let r=0;r<5;r++)for(let c=0;c<5;c++){
      const candidate={x:+((c+.5)*state.gridStep).toFixed(4),y:+((r+.5)*state.gridStep).toFixed(4)};
      if(physicalObstacleFits(candidate))points.push(candidate);
    }
    for(let i=points.length-1;i>0;i--){const j=Math.floor(Math.random()*(i+1));[points[j],points[i]]=[points[i],points[j]]}
    state.obstacles=points.slice(0,2);recalc()}
  function getScenario(){return {scenarioVersion:10,rawX:state.rawX,rawY:state.rawY,qrY:state.qrY,alignOffset:state.alignOffset,alignSpeed:state.alignSpeed,alignYaw:state.alignYaw,armReach:state.armReach,armSpeed:state.armSpeed,ringFollow:state.ringFollow,obstacles:state.obstacles,start:state.start,goal:state.goal,startZone:state.startZone,gridStep:state.gridStep,velocity:state.velocity,handling:state.handling,mode:state.mode,planner:state.planner,goalPolicy:state.goalPolicy,task:state.task,showGrid:state.showGrid,pathView:state.pathView,turnDirection:state.turnDirection,rotorDwell:state.rotorDwell,rotorRate:state.rotorRate,rotorAngle:state.rotorAngle}}
  function applyScenario(data){const o=data?.scenario?{...data.scenario,task:data.scenario.task_code,blockedSlots:data.scenario.blocked_slots,velocity:data.scenario.velocity_m_s}:data;
    if(!o||typeof o!=='object')throw Error('无法识别的场景格式');
    if(inFixtureRange('raw',+o.rawX))state.rawX=+o.rawX;
    if(inFixtureRange('rawY',+o.rawY))state.rawY=+o.rawY;
    if(inFixtureRange('qr',+o.qrY))state.qrY=+o.qrY;
    applyFixtureValues();
    // Migrate old 200/300/400 mm scenarios by snapping obstacles to V9 cell centres.
    // The visible / occupied grid is fixed at 5x5; do not restore obsolete step values.
    state.gridStep=DEFAULT_GRID_STEP;
    if(Array.isArray(o.obstacles)){state.obstacles=o.obstacles.slice(0,2).filter(p=>p&&Number.isFinite(+p.x)&&Number.isFinite(+p.y)).map(p=>{
      const g=guideId({x:+p.x,y:+p.y});return {x:+((g.c+.5)*state.gridStep).toFixed(4),y:+((g.r+.5)*state.gridStep).toFixed(4)};
    }).filter(physicalObstacleFits).filter((p,i,list)=>list.findIndex(q=>{const a=guideId(q),b=guideId(p);return a.c===b.c&&a.r===b.r})===i)}
    else if(Array.isArray(o.blockedSlots)){state.obstacles=o.blockedSlots.slice(0,2).map(id=>SLOTS.find(s=>s.id===+id)).filter(Boolean).map(s=>{const g=guideId(s);return {x:+((g.c+.5)*state.gridStep).toFixed(4),y:+((g.r+.5)*state.gridStep).toFixed(4)}}).filter(physicalObstacleFits)}
    const validPos=pos=>pos&&inField(Number(pos.x),Number(pos.y));
    if(validPos(o.start))state.start={x:+o.start.x,y:+o.start.y};if(validPos(o.goal))state.goal={x:+o.goal.x,y:+o.goal.y};
    if([0,1,2].includes(Number(o.startZone)))state.startZone=Number(o.startZone);
    if(Number(o.velocity)>=.05&&Number(o.velocity)<=1.2)state.velocity=+o.velocity;
    
    if(Number(o.scenarioVersion)>=5&&Number(o.handling)>=.05&&Number(o.handling)<=3)state.handling=+o.handling;
    if(Number(o.rotorDwell)>=0&&Number(o.rotorDwell)<=15)state.rotorDwell=+o.rotorDwell;
    if(Number(o.scenarioVersion)>=8&&Number(o.rotorRate)>=36&&Number(o.rotorRate)<=60)state.rotorRate=+o.rotorRate;
    if(Number(o.scenarioVersion)>=8&&Number(o.rotorAngle)>=5&&Number(o.rotorAngle)<=180)state.rotorAngle=+o.rotorAngle;
    for(const [key,min,max] of [['alignOffset',0,.04],['alignSpeed',.01,.12],['alignYaw',0,10],['armReach',0,.15],['armSpeed',.02,.2]]){
      const v=Number(o[key]);if(Number.isFinite(v)&&v>=min-1e-8&&v<=max+1e-8)state[key]=v;
    }
    if(typeof o.ringFollow==='boolean')state.ringFollow=o.ringFollow;
    if(['mission','direct'].includes(o.mode))state.mode=o.mode;
    if(typeof o.task==='string'&&o.task.length<40)state.task=o.task;
    if(typeof o.showGrid==='boolean')state.showGrid=o.showGrid;
    if(Number(o.scenarioVersion)<10)state.showGrid=true; // Make the new numbered map visible after V8 migration.
    if(['current','all','global'].includes(o.pathView))state.pathView=o.pathView==='all'?'global':o.pathView;
    if(['astar','dijkstra','dstar'].includes(o.planner))state.planner=o.planner;
    if(['other','home'].includes(o.goalPolicy))state.goalPolicy=o.goalPolicy;
    if(['cw','ccw'].includes(o.turnDirection))state.turnDirection=o.turnDirection;
    syncUI();recalc();
  }
  function exportJSON(){const grid=mapGrid(),p=state.plan;
    const result={meta:{version:'10.0.0',units:'m',field_size:[2.4,2.4],navigation_grid_step:.05,obstacle_cell_step:state.gridStep,cell_count:[5,5],cell_numbering:'0-24, snake bottom-to-top, left-to-right on even rows',blocked_cell_ids:state.obstacles.map(o=>{const g=guideId(o);return numberedCell(g.c,g.r)}),fixture_positions_m:{raw_center:[state.rawX,state.rawY],qr_y:state.qrY},robot_width:.26,robot_length:.29,collision_approximation:'one grid cell per phi50mm obstacle; robot rectangular overlap check; no visual inflation',turntable:{dwell_s:state.rotorDwell,angular_speed_deg_s:state.rotorRate,revolution_s:360/state.rotorRate,step_deg:state.rotorAngle,material_spacing_deg:120},station_alignment:{offset_m:state.alignOffset,velocity_m_s:state.alignSpeed,yaw_nudge_deg:state.alignYaw,extension_m:state.armReach,extension_velocity_m_s:state.armSpeed,ring_follow:state.ringFollow,mechanism_collision_checked:false}},
      scenario:{...getScenario(),task_code:state.task,velocity_m_s:state.velocity},
      obstacles_m:state.obstacles.map(o=>[o.x,o.y]),
      numbered_grid_bottom_to_top:Array.from({length:5},(_,r)=>Array.from({length:5},(_,c)=>numberedCell(c,r))),
      occupied_grid_bottom_to_top:Array.from({length:5},(_,r)=>Array.from({length:5},(_,c)=>state.obstacles.some(o=>{const g=guideId(o);return g.c===c&&g.r===r})?1:0)),
      blocked_grid: grid.blocked.map(row=>row.map(v=>v?1:0)),
      task:parseTask(state.task),
      plan:p?.ok?{reachable:true,algorithm:state.planner,goal_zone:state.mode==='mission'?endZone():null,length_m:p.total,full_path_m:p.fullPath.map(q=>[+q.x.toFixed(3),+q.y.toFixed(3)]),legs:p.legs.map(o=>({stage:o.label,length_m:o.length,expanded:o.expanded,path_m:o.path.map(q=>[+q.x.toFixed(3),+q.y.toFixed(3)])}))}:{reachable:false,reason:p?.reason}};
    const blob=new Blob([JSON.stringify(result,null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=`competition_simulation_${Date.now()}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  }
  function persist(){try{localStorage.setItem('competition_simulator_v6',JSON.stringify(getScenario()))}catch(_){} }
  function markDirty(){shared.dirty=true;shared.seq++;clearTimeout(shared.timer);shared.timer=setTimeout(pushShared,250)}
  async function pushShared(){if(!shared.ready||!shared.dirty||shared.pushing)return;shared.pushing=true;const seq=shared.seq;
    try{const res=await fetch('/api/scene',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({scene:getScenario()}),cache:'no-store'});if(!res.ok)throw Error('HTTP '+res.status);
      const v=await res.json();shared.revision=Math.max(shared.revision,v.revision||0);if(shared.seq===seq)shared.dirty=false;
    }catch(e){console.warn('Sync failed',e)}finally{shared.pushing=false;if(shared.dirty){clearTimeout(shared.timer);shared.timer=setTimeout(pushShared,2000)}}
  }
  async function pollShared(){if(!shared.ready||shared.dirty||shared.pushing)return;try{const res=await fetch('/api/scene',{cache:'no-store'}),data=await res.json();if(data.scene&&data.revision>shared.revision){shared.revision=data.revision;shared.inbound=true;try{applyScenario(data.scene)}finally{shared.inbound=false}}}catch(e){console.warn('Shared scene poll failed',e)}}
  async function startShared(){if(!/^https?:$/.test(location.protocol))return;
    try{const res=await fetch('/api/scene',{cache:'no-store'});if(!res.ok)return;const data=await res.json();if(typeof data.revision!=='number')return;
      shared.revision=data.revision;shared.ready=true;if(data.scene){shared.inbound=true;try{applyScenario(data.scene)}finally{shared.inbound=false}}else markDirty();
      const status=document.querySelector('.topbar-right > span:nth-child(2)');if(status)status.textContent='多人实时共享障碍布局';
      window.setInterval(pollShared,1500);
    }catch(e){console.warn('Offline mode',e)}
  }
  function shareLink(){let url=location.href.split('#')[0];
    if(!shared.ready){const data=btoa(unescape(encodeURIComponent(JSON.stringify(getScenario())))).replace(/\+/g,'-').replace(/\//g,'_').replace(/=+$/,'');url+='#scene='+data}
    navigator.clipboard?.writeText(url).then(()=>tell('✓ 已复制访问链接。')).catch(()=>prompt('复制链接：',url))||prompt('复制链接：',url);
  }
  function syncUI(){$('modeSelect').value=state.mode;$('pathView').value=state.pathView;$('plannerSelect').value=state.planner;$('goalPolicy').value=state.goalPolicy;$('velocity').value=state.velocity;$('handling').value=state.handling;
    $('velocityValue').textContent=f(state.velocity)+' m/s';$('handlingValue').textContent=f(state.handling,2)+' s';
    $('turnDirection').value=state.turnDirection;
    $('gridStep').value=state.gridStep;
    $('rotorDwell').value=state.rotorDwell;$('rotorRate').value=state.rotorRate;$('rotorAngle').value=state.rotorAngle;updateRotor(performance.now());updateRotorControls();
    for(const [k,v,suffix] of [['alignOffset',Math.round(state.alignOffset*1000),' mm'],['alignSpeed',state.alignSpeed.toFixed(2),' m/s'],['alignYaw',state.alignYaw,'°'],['armReach',Math.round(state.armReach*1000),' mm'],['armSpeed',state.armSpeed.toFixed(2),' m/s']]){
      $(k).value=k==='alignOffset'||k==='armReach'?v:state[k];$(k+'Value').textContent=v+suffix;
    }
    $('ringFollow').checked=state.ringFollow;
    $('taskCode').value=state.task;$('showGrid').checked=state.showGrid;updateFixtureUI();zoneButtons();syncRunUI();
  }
  function attach(){
    $('mapHitArea').addEventListener('pointerdown',onMapClick);
    $('rawPosition').addEventListener('input',e=>changeFixture('raw',Number(e.target.value)/1000));
    $('rawVertical').addEventListener('input',e=>changeFixture('rawY',Number(e.target.value)/1000));
    $('qrPosition').addEventListener('input',e=>changeFixture('qr',Number(e.target.value)/1000));
    $('mapHitArea').addEventListener('pointermove',evt=>{const p=posAt(evt);if(!p||!inField(p.x,p.y))return;const s=guideId(p);$('mouseCoord').textContent=`x=${Math.round(p.x*1000)} mm，y=${Math.round(p.y*1000)} mm ｜ 格号 #${numberedCell(s.c,s.r)}（列${s.c+1}/行${s.r+1}）`});
    $('mapHitArea').addEventListener('pointerleave',()=>{$('mouseCoord').textContent='指针坐标：—'});
    document.querySelectorAll('[data-tool]').forEach(btn=>btn.addEventListener('click',()=>chooseTool(btn.dataset.tool)));
    $('planBtn').addEventListener('click',recalc);$('compareBtn').addEventListener('click',comparePlanners);$('playBtn').addEventListener('click',startPause);$('resetBtn').addEventListener('click',()=>{recalc();tell('任务已重置。')});
    $('randomBtn').addEventListener('click',randomObstacles);$('clearBtn').addEventListener('click',()=>{state.obstacles=[];recalc()});
    $('runSource').addEventListener('change',e=>switchRunSource(e.target.value));
    $('modeSelect').addEventListener('change',e=>{state.mode=e.target.value;recalc()});
    $('gridStep').addEventListener('change',()=>{$('gridStep').value='0.48';tell('按你的参考图固定为 5×5、480 mm、0–24 蛇形编号。');});
    $('plannerSelect').addEventListener('change',e=>{state.planner=e.target.value;recalc()});
    $('goalPolicy').addEventListener('change',e=>{state.goalPolicy=e.target.value;recalc()});
    $('pathView').addEventListener('change',e=>{state.pathView=e.target.value;paintPath();persist();if(shared.ready)markDirty()});
    $('showGrid').addEventListener('change',e=>{state.showGrid=e.target.checked;paintGrid();paintObstacles();persist();if(shared.ready)markDirty()});
    $('velocity').addEventListener('input',e=>{state.velocity=+e.target.value;$('velocityValue').textContent=f(state.velocity)+' m/s';paintReport();persist();if(shared.ready)markDirty()});
    $('handling').addEventListener('input',e=>{state.handling=+e.target.value;$('handlingValue').textContent=f(state.handling,2)+' s';paintReport();persist();if(shared.ready)markDirty()});
    $('turnDirection').addEventListener('change',e=>{state.turnDirection=e.target.value;persist();if(shared.ready)markDirty()});
    for(const id of ['alignOffset','alignSpeed','alignYaw','armReach','armSpeed']){
      $(id).addEventListener('input',e=>{
        let value=+e.target.value;if(['alignOffset','armReach'].includes(id))value/=1000;
        state[id]=value;syncUI();paintReport();persist();if(shared.ready)markDirty();
      });
    }
    $('ringFollow').addEventListener('change',e=>{state.ringFollow=e.target.checked;persist();if(shared.ready)markDirty()});
    for(const [key,id] of [['rotorDwell','rotorDwell'],['rotorRate','rotorRate'],['rotorAngle','rotorAngle']])$(''+id).addEventListener('input',e=>{state[key]=+e.target.value;state.turnOrigin=performance.now();updateRotorControls();persist();if(shared.ready)markDirty();});
    $('liveAlignBtn').addEventListener('click',resetLiveAnchor);
    for(const id of ['liveAnchorX','liveAnchorY','liveAnchorYaw'])$(id).addEventListener('change',resetLiveAnchor);
    $('taskCode').addEventListener('input',e=>{state.task=e.target.value.trim();recalc()});
    $('start1Btn').addEventListener('click',()=>useZone(1));$('start2Btn').addEventListener('click',()=>useZone(2));
    $('exportBtn').addEventListener('click',exportJSON);$('importBtn').addEventListener('click',()=>{$('importFile').click()});
    $('importFile').addEventListener('change',async e=>{const file=e.target.files?.[0];if(!file)return;try{applyScenario(JSON.parse(await file.text()));tell('✓ 场景导入成功。')}catch(err){tell('导入失败：'+err.message,true)}e.target.value=''});
    $('shareBtn').addEventListener('click',shareLink);
    window.addEventListener('keydown',e=>{if(e.target.matches('input,select,textarea'))return;if(e.key==='Escape')chooseTool('obstacle');if(e.key==='Delete'||e.key==='Backspace')chooseTool('erase')});
  }
  function start(){applyFixtureValues();try{const hash=location.hash.startsWith('#scene=')?location.hash.slice(7):null,saved=localStorage.getItem('competition_simulator_v6');
      if(hash)applyScenario(JSON.parse(decodeURIComponent(escape(atob(hash.replace(/-/g,'+').replace(/_/g,'/'))))));else if(saved)applyScenario(JSON.parse(saved));
    }catch(e){console.warn('Saved scene unavailable',e)}
    syncUI();attach();bindDirectory();recalc();state.turnOrigin=performance.now();requestAnimationFrame(animationLoop);startShared();
    window.setInterval(pollTelemetry,140);window.setInterval(updateLiveReadout,500);pollTelemetry();
    window.setInterval(updateCameraStatus,1100);updateCameraStreamMode();window.addEventListener('sim2real:cameratoggle',updateCameraStreamMode);
  }

  // Read-only telemetry and offline training API for the V13 operator console.
  // There is deliberately NO hardware command endpoint exposed here.
  window.Sim2RealCore={
    getMode:()=>state.runSource,
    getReferenceSpeed:()=>state.velocity,
    getPlanner:()=>state.planner,
    getRoute:()=>state.plan?.ok?state.plan.fullPath.map(p=>({...p})):[],
    getSimPose:()=>({...(state.sim?.poseOverride?.point||state.start),yaw:state.sim?.poseOverride?.heading!==undefined?(90-state.sim.poseOverride.heading)*Math.PI/180:0}),
    isSafe:(x,y)=>safe(x,y),
    manualPlan:(a,b)=>{if(state.runSource!=='simulation')return null;const grid=mapGrid();return findPath(GridPlanning.buildGraph(grid,segmentSafe),grid,a,b)},
    manualReset:()=>{if(state.sim){state.sim.poseOverride=null;state.sim.armExtension=0;}if(state.runSource==='simulation')paintRobot();},
    manualPose:(p,extension=0)=>{if(state.runSource!=='simulation')return false;
      if(!state.sim)return false;state.sim.running=false;state.sim.phase='idle';
      state.sim.poseOverride={point:{x:p.x,y:p.y},heading:90-p.yaw*180/Math.PI};
      state.sim.armExtension=Math.max(0,Math.min(.15,extension));paintRobot();return true;},
    cancelMission:()=>{if(state.sim){state.sim.running=false;state.sim.phase='idle';state.sim.poseOverride=null;}},
    mapCoordinate:evt=>posAt(evt),
  };

  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',start);else start();
})();
