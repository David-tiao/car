/* TEST-only STM32 workbench, separate from the REAL monitor.
 * PTY mode: original CAR-LINK mock serial; C mode: GCC-compiled STM32 C gate.
 * Every completed road step animates an orange MOCK vehicle on the field SVG.
 * Never drives motors or writes /api/real, /cmd_vel, or /odom.
 */
(() => {
  'use strict';
  if (window.CarWebMode?.mode !== 'test') return;
  const API='/api/test/stm32_fsm_lab';
  const SVG='http://www.w3.org/2000/svg';
  const $=(tag,parent,text) => {const n=document.createElement(tag);if(text!==undefined)n.textContent=text;parent.append(n);return n;};
  const addSVG=(parent,tag,attributes) => {
    const n=document.createElementNS(SVG,tag);
    Object.entries(attributes).forEach(([k,v])=>n.setAttribute(k,String(v)));
    parent.appendChild(n);return n;
  };
  const point=(p)=>[60+p[0]/2.4*680,740-p[1]/2.4*680];
  const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));

  function mount() {
    const map=document.querySelector('.map-panel');
    const slot=document.getElementById('fieldWorkbenchSlot');
    const panel=$('section',slot||document.querySelector('.app-shell')||document.body);
    panel.id='carTestStm32Workbench';
    panel.className='car-fsm-workbench';
    if(slot)slot.parentElement.classList.add('has-workbench');
    else if(map)map.after(panel);
    else document.getElementById('carModeSwitch')?.after(panel);

    const head=$('div',panel);head.className='cfsm-heading';
    $('span',head,'02 / HARDWARE-FREE EXPERIMENT · MOCK ONLY').className='cfsm-eyebrow';
    $('h2',head,'STM32 任务状态机 · 双引擎实验');
    $('p',head,'地图中的橙色小车会跟随每次已完成的道路级步骤连续移动；这只是离散路网可视化，不是 /odom 或实车运动。抓放等非行驶动作原地反馈。');

    const toggle=$('div',panel);toggle.className='cfsm-toggle';toggle.setAttribute('role','group');toggle.setAttribute('aria-label','选择 STM32 实验后端');
    const pty=$('button',toggle,'串口模拟 · CAR-LINK PTY');
    const code=$('button',toggle,'C 代码状态机 · GCC');
    for(const b of [pty,code])b.type='button';
    pty.dataset.engine='pty';code.dataset.engine='code_fsm';

    const editor=$('div',panel);editor.className='cfsm-fields';
    function field(title,value){
      const label=$('label',editor,title);
      const input=$('input',label);input.value=value;
      return input;
    }
    const qr=field('比赛二维码','156+123+516+231');qr.maxLength=15;
    const roads=field('附加手动封路（可留空）','');roads.placeholder='地图红格自动同步；可补充 B3-C3';
    const zoneLabel=$('label',editor,'出发区域');
    const zone=$('select',zoneLabel);
    for(const [id,name] of [['upper_right','右上启停区'],['lower_right','右下启停区']]){
      const option=$('option',zone,name);option.value=id;
    }
    const faultLabel=$('label',editor,'故障注入');
    const fault=$('select',faultLabel);
    for(const [id,name] of [['none','正常执行'],['failed_at_3','第 3 步失败'],['missing_ack_at_2','第 2 步无 ACK'],['qr_timeout','C 扫码超时']]){
      const option=$('option',fault,name);option.value=id;
    }

    const controls=$('div',panel);controls.className='cfsm-controls';
    const buttons=[];
    for(const [label,act] of [['初始化任务','start'],['执行一步','step'],['连续运行','run'],['重置','reset']]){
      const b=$('button',controls,label);b.type='button';b.dataset.action=act;buttons.push(b);
    }
    const [start,step,run,reset]=buttons;
    const mapInfo=$('div',panel,'等待读取场地障碍格…');mapInfo.className='cfsm-map-status';
    const output=$('div',panel);output.className='cfsm-output';
    const state=$('div',output,'未初始化');state.className='cfsm-status';
    const progress=$('div',output,'等待模拟 STM32 状态机');
    const details=$('div',output,'数据来源：MOCK；硬件控制：禁止');
    const log=$('pre',output,'操作后显示 ACK / COMPLETED / FAILED 事件');
    log.className='cfsm-log';
    const note=$('p',panel,'串口模式：匿名 PTY + CAR-LINK MockPeer；C 模式：编译运行原生 STM32 RaceTaskPlan / RaceTaskGate（不是 Keil 实机）。切换模式会重新初始化独立实验，不会更改真实通道。');
    note.className='cfsm-notice';

    let selected='pty',busy=false,needsInit=true,epoch=0,vehicle=null,trail=null,group=null;
    let fieldSnapshot=null,previewSerial=0,blockedOverlay=null;
    function choose(mode){
      if(selected!==mode)needsInit=true;
      selected=mode;
      const qrTimeout=[...fault.options].find(option=>option.value==='qr_timeout');
      if(qrTimeout)qrTimeout.disabled=mode!=='code_fsm';
      if(mode!=='code_fsm'&&fault.value==='qr_timeout')fault.value='none';
      for(const b of [pty,code]){
        const active=b.dataset.engine===mode;
        b.setAttribute('aria-pressed',String(active));
        b.classList.toggle('selected',active);
      }
      details.textContent=(mode==='pty'?'原有虚拟 STM32 串口':'C RaceFsm 扫码路线（8 步）→ 物料任务计划（34 步）')+
        ' · 切换后请点击初始化任务';
      step.disabled=busy||needsInit;run.disabled=busy||needsInit;
    }
    choose('pty');
    start.disabled=true;
    for(const b of [pty,code])b.addEventListener('click',()=>{if(!busy)choose(b.dataset.engine);});
    function layer(){
      const svg=document.getElementById('fieldSvg');
      if(!svg)return null;
      if(!group||!group.isConnected){
        group=addSVG(svg,'g',{'id':'testMockVehicleLayer','pointer-events':'none'});
        trail=addSVG(group,'polyline',{fill:'none',stroke:'#fb805a','stroke-width':4,
          'stroke-linecap':'round','stroke-linejoin':'round','stroke-dasharray':'10 5',
          opacity:.9,points:''});
        vehicle=addSVG(group,'g',{'aria-label':'模拟 STM32 路网车辆标记'});
        addSVG(vehicle,'rect',{x:-15,y:-20,width:30,height:40,rx:7,
          fill:'#f46b52',stroke:'#5e1b21','stroke-width':2.5});
        addSVG(vehicle,'rect',{x:-8,y:-13,width:16,height:10,rx:3,
          fill:'#fcf0df',opacity:.95});
        addSVG(vehicle,'path',{d:'M0,-25 L-7,-14 L7,-14 Z',fill:'#ffd67d'});
        addSVG(vehicle,'circle',{cx:0,cy:0,r:4,fill:'#fff'});
        addSVG(group,'text',{x:70,y:84,fill:'#ae2940',
          'font-size':17,'font-weight':800,'paint-order':'stroke',stroke:'#fff',
          'stroke-width':3}).textContent='MOCK';
      }
      return group;
    }
    let segments=[];
    function pos(p,angle=0){
      if(!layer())return;
      const [x,y]=point(p);
      vehicle.setAttribute('transform',`translate(${x.toFixed(2)} ${y.toFixed(2)}) rotate(${angle.toFixed(1)})`);
    }
    function resetMap(p){
      segments=[];if(layer()){trail.setAttribute('points','');pos(p);}
    }
    function pinPoint(p){
      const [x,y]=point(p);segments.push(`${x.toFixed(1)},${y.toFixed(1)}`);
      if(segments.length>500)segments=segments.slice(-500);
      trail.setAttribute('points',segments.join(' '));
    }
    async function animateTransition(transition,seq){
      if(!transition||!Array.isArray(transition.points)||!transition.points.length)return;
      const pts=transition.points;
      if(pts.length===1){
        pos(pts[0]);return;
      }
      pinPoint(pts[0]);
      for(let i=1;i<pts.length;i++){
        const from=pts[i-1],to=pts[i];
        const sx=point(from),ex=point(to);
        const heading=Math.atan2(ex[0]-sx[0],sx[1]-ex[1])*180/Math.PI;
        if(Math.hypot(ex[0]-sx[0],ex[1]-sx[1])<.1){pos(to,heading);continue;}
        const duration=560;
        await new Promise(resolve=>{
          let startAt=null;
          function frame(now){
            if(seq!==epoch){resolve();return;}
            if(startAt===null)startAt=now;
            const t=Math.min(1,(now-startAt)/duration);
            const eased=t*t*(3-2*t);
            const px=from[0]+(to[0]-from[0])*eased;
            const py=from[1]+(to[1]-from[1])*eased;
            pos([px,py],heading);
            if(t<1)requestAnimationFrame(frame);else resolve();
          }
          requestAnimationFrame(frame);
        });
        if(seq!==epoch)return;
        pinPoint(to);
      }
    }
    function validate(data){
      if(data.source_type!=='MOCK'||data.mode!=='SIM_ONLY'||
         data.hardware_authorized!==false||
         !['pty','code_fsm',null].includes(data.engine)||
         !['ANONYMOUS_PTY_CARLINK_TEST_ONLY','NATIVE_GCC_C_FSM_NO_UART'].includes(data.transport)||
         !['PYTHON_SIMEXECUTOR_NOT_COMPILED_STM32',
           'COMPILED_STM32_RACEFSM_QR_AND_TASK_GATE_HOST_ONLY'].includes(data.model_kind))
         throw Error('拒绝非法测试状态源');
      const s=data.snapshot;
      if(s && (s.source_type!=='MOCK'||s.hardware_authorized!==false||
         s.pose_source!=='DISCRETE_ROAD_NODE'))throw Error('不可将测试节点当成实车里程计');
    }
    function render(data){
      validate(data);
      const s=data.snapshot;
      const field=currentField();
      const compatible=!data.case || (field &&
        JSON.stringify((data.case.blocked_cells||[]).slice().sort((a,b)=>a-b))===
        JSON.stringify(field.blocked_cells.slice().sort((a,b)=>a-b)) &&
        data.case.start_zone===field.start_zone);
      needsInit=!(s && data.engine===selected && compatible);
      step.disabled=busy||needsInit;run.disabled=busy||needsInit;
      if(!s){
        state.textContent='MOCK · 尚未启动';progress.textContent='—';
        details.textContent='准备就绪：'+(selected==='pty'?'虚拟 CAR-LINK 串口':'原生 C 任务表');
        log.textContent='暂无状态机事件';return;
      }
      state.textContent=`MOCK · ${data.engine==='code_fsm'?'STM32 C':'CAR-LINK PTY'} · ${s.state}${s.error?' · '+s.error:''}`;
      state.dataset.state=s.state;
      progress.textContent=`进度 ${s.completed_steps}/${s.total_steps} · #${s.cmd_id} ${s.kind} · 模拟节点 ${s.node}`;
      details.textContent=`${s.phase||'PTY'} · ${s.event} · 二维码 ${s.qr_valid?'已确认':'等待确认'}${s.qr_session?' · session '+s.qr_session:''} · 模拟抓取 ${s.pick_count} / 放置 ${s.place_count} / 码垛 ${s.stack_count} · 返程 ${s.home_confirmed?'完成':'未完成'}`;
      log.textContent=(s.events||[]).map(e=>`#${e.cmd_id} ${e.kind||''} · ${e.event}${e.reason?' — '+e.reason:''}`).join('\n')||'等待 ACK / COMPLETED 回执';
    }
    async function request(body){
      const response=await fetch(API,{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify(body),cache:'no-store'});
      const result=await response.json();
      if(!response.ok)throw Error(result.error||('HTTP '+response.status));
      validate(result);return result;
    }
    async function singleStep(seq){
      const result=await request({action:'step'});
      render(result);
      await animateTransition(result.transition,seq);
      return result;
    }
    async function send(action){
      if(busy)return;
      if((action==='step'||action==='run')&&needsInit)return;
      busy=true;epoch++;const seq=epoch;
      for(const b of [...buttons,pty,code])b.disabled=true;
      try{
        if(action==='start'){
          const data=await request({action:'start',engine:selected,
             start_zone:zone.value,task_code:qr.value.trim(),
             blocked_roads:roads.value.split(',').map(x=>x.trim()).filter(Boolean),
             blocked_cells:fieldSnapshot?.blocked_cells||[],
             fault:fault.value});
          render(data);
          resetMap(data.transition?.points?.[0] || [2.05,2.05]);
        }else if(action==='reset'){
          const data=await request({action:'reset'});
          render(data);
          if(group)group.remove();group=null;vehicle=null;trail=null;
        }else if(action==='step'){
          await singleStep(seq);
        }else{
          for(let n=0;n<80;n++){
            if(seq!==epoch)break;
            const data=await singleStep(seq);
            if(data.snapshot?.state==='DONE'||data.snapshot?.state==='FAULT')break;
            // Non-drive actions are rendered at their node, without fake travel.
            if(data.transition?.points?.length===1)await pause(180);
          }
        }
      }catch(err){state.textContent='MOCK 实验出错：'+String(err);state.dataset.state='FAULT';}
      finally{
        busy=false;
        for(const b of [...buttons,pty,code])b.disabled=false;
        start.disabled=!currentField()?.start_zone;
        step.disabled=needsInit;
        run.disabled=needsInit;
      }
    }
    // Authoritative shared map source: the 5×5 field editor publishes the
    // occupied red CELL IDs. The backend projects them to blocked road edges.
    function currentField(){
      const data=window.Sim2RealCore?.getTestFieldSnapshot?.();
      if(!data || data.source_type!=='MAP_EDITOR_TEST_ONLY' ||
         !Array.isArray(data.blocked_cells))return null;
      return data;
    }
    function overlayBlockedRoads(segments){
      const svg=document.getElementById('fieldSvg');
      if(!svg)return;
      if(blockedOverlay)blockedOverlay.remove();
      blockedOverlay=addSVG(svg,'g',{
        id:'testBlockedRoadOverlay','pointer-events':'none',
        'aria-label':'由地图障碍格推导的封闭路段'
      });
      for(const seg of segments){
        const a=point(seg.from),b=point(seg.to);
        addSVG(blockedOverlay,'line',{
          x1:a[0],y1:a[1],x2:b[0],y2:b[1],
          stroke:'#ef4755','stroke-width':12,'stroke-opacity':.48,
          'stroke-dasharray':'18 9','stroke-linecap':'round'
        });
      }
    }
    async function refreshFieldPreview(){
      const snapshot=currentField();
      if(!snapshot){mapInfo.textContent='场地编辑器尚未就绪';return;}
      fieldSnapshot=snapshot;
      if(snapshot.start_zone){
        zone.value=snapshot.start_zone;
        start.disabled=false;
      }else{
        start.disabled=true;
        mapInfo.textContent='请先在地图上选右上或右下的合法启停区；不能用任意车道坐标启动比赛任务。';
        return;
      }
      const id=++previewSerial;
      try{
        const data=await request({
          action:'preview',blocked_cells:snapshot.blocked_cells,
          blocked_roads:roads.value.split(',').map(x=>x.trim()).filter(Boolean)
        });
        if(id!==previewSerial)return;
        const preview=data.preview;
        if(!preview||preview.source_type!=='MAP_EDITOR_TEST_ONLY')throw Error('地图封路预览来源错误');
        const summary=preview.cell_mapping.map(m=>
          `CELL ${m.cell} → ${m.roads.length?m.roads.join('、'):'不与道路中心线相交'}`).join('  |  ');
        mapInfo.textContent='地图自动封路：'+(summary||'无红色障碍格')+
            ' · 当前禁行路段：'+(preview.blocked_roads.join('、')||'无')+
            ' · 仅按道路中心线投影，不代表真实车体碰撞校验';
        overlayBlockedRoads(preview.road_segments||[]);
      }catch(err){
        if(id===previewSerial)mapInfo.textContent='地图封路同步失败：'+String(err);
      }
    }
    async function onFieldChanged(){
      const old=fieldSnapshot;
      const next=currentField();
      if(!next)return;
      fieldSnapshot=next;
      if(old && (old.blocked_cells.slice().sort((a,b)=>a-b).join(',')!==
                  next.blocked_cells.slice().sort((a,b)=>a-b).join(',') ||
                 old.start_zone!==next.start_zone) && !needsInit){
        epoch++;
        needsInit=true;step.disabled=true;run.disabled=true;
        state.textContent='地图障碍格或启停区已改变：旧模拟路线失效，请重新初始化任务';
        state.dataset.state='FAULT';
        try{await request({action:'reset'});}
        catch(err){mapInfo.textContent='清理旧模拟会话失败：'+String(err);}
        if(group){group.remove();group=null;vehicle=null;trail=null;}
      }
      refreshFieldPreview();
    }
    zone.addEventListener('change',()=>{
      window.Sim2RealCore?.setTestStartZone?.(zone.value);
    });
    window.addEventListener('sim2real:field-changed',onFieldChanged);
    roads.addEventListener('change',()=>{
      needsInit=true;step.disabled=true;run.disabled=true;
      state.textContent='附加封锁路段改变，需重新初始化';
      refreshFieldPreview();
    });
    refreshFieldPreview();
    for(const b of buttons)b.addEventListener('click',()=>send(b.dataset.action));
    fetch(API,{cache:'no-store'}).then(r=>r.json()).then(data=>{
      validate(data);render(data);
      if(data.snapshot)resetMap([data.snapshot.x_m,data.snapshot.y_m]);
    }).catch(e=>{state.textContent='模拟接口不可用：'+String(e);});
  }
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',mount);
  else mount();
})();
