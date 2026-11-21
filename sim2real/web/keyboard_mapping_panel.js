/* /keyboard: direct PC keyboard -> actual PC ROS2 subscriber -> SVG pose.
 * W/A/S/D/Q/E move immediately in local mode, no FSM, Shift, obstacle,
 * acceleration or destination constraints. Never a motor/REAL /odom command.
 */
(() => {
  'use strict';
  if(window.CarWebMode?.mode!=='keyboard')return;
  const ENDPOINT='/api/keyboard';
  const $=(tag,parent,text)=>{
    const node=document.createElement(tag);
    if(text!==undefined)node.textContent=text;
    parent.appendChild(node);
    return node;
  };
  const millis=()=>Date.now();
  const KEYS=new Set(['w','a','s','d','q','e']);
  function mount(){
    const slot=document.getElementById('fieldKeyboardSlot');
    const map=document.querySelector('.map-panel');
    const panel=$('section',slot||document.querySelector('.app-shell')||document.body);
    panel.className='keyboard-mapping-panel';
    panel.id='carKeyboardMapping';
    if(slot)slot.parentElement.classList.add('has-keyboard');
    else if(map)map.after(panel);
    const header=$('div',panel);
    header.className='keyboard-heading';
    $('div',header,'03 / KEYBOARD MAPPING · ROS2 DRY RUN').className='keyboard-eyebrow';
    $('h2',header,'键盘直控映射 · ROS2 实时回环');
    $('p',header,'本机模式：W/S 前后、A/D 横移、Q/E 转向；按下立即输出 ROS2 速度，网页只按实际订阅值积分绘图。无需 Shift、任务状态机、避障或树莓派。未来实车链路仍单独预留。');
    const signalChip=$('span',header,'ROS2 回环：等待话题');
    signalChip.className='keyboard-signal-chip';
    signalChip.dataset.online='false';
    signalChip.setAttribute('role','status');
    signalChip.setAttribute('aria-live','polite');

    const modes=$('div',panel);modes.className='keyboard-route-switch';
    modes.setAttribute('role','group');modes.setAttribute('aria-label','ROS2 键盘控制链路');
    const routeLocal=$('button',modes,'本机键盘直控 → ROS2 → 网页');
    const routePi=$('button',modes,'电脑 → 树莓派 → STM32 · 预留');
    routeLocal.type='button';routePi.type='button';
    routeLocal.dataset.route='local_ros2';routePi.dataset.route='pi_stm32';
    const routeNotice=$('p',panel,'本机模式：按键 → 电脑 ROS 2 发布/订阅 → 网页。显示话题回环，不代表车辆已运动。');
    routeNotice.className='keyboard-route-notice';

    const status=$('div',panel,'默认断开 · 不向实体电机发送控制');
    status.id='keyboardLinkState';status.className='keyboard-state';
    const healthStatus=$('div',panel,'诊断：等待服务器、ROS2 与 MOCK 状态');
    healthStatus.className='keyboard-link-health';
    healthStatus.setAttribute('aria-live','polite');
    const enabledLabel=$('label',panel,'完整实车链路安全使能（仅预留模式）');
    enabledLabel.className='keyboard-allow';
    const enabled=$('input',enabledLabel);enabled.type='checkbox';
    enabledLabel.prepend(enabled);
    const help=$('p',panel,'本机直控：直接按住 W/S 前后、A/D 左右平移、Q/E 转向；可同时按多个键。松开立即归零，空格/Esc 停车。不需要 Shift 或勾选使能。输入框聚焦时不捕获键盘。');
    help.className='keyboard-notice';
    const grid=$('div',panel);grid.className='keyboard-keygrid';
    const keyButtons=new Map();
    for(const [key,label] of [
        ['q','Q ↶ 左转'],['w','W ↑ 前进'],['e','E ↷ 右转'],
        ['a','A ← 左移'],['s','S ↓ 后退'],['d','D → 右移']]){
      const button=$('button',grid,label);
      button.type='button';button.className='keyboard-key';
      button.setAttribute('aria-label','模拟按住 '+label);
      button.setAttribute('aria-keyshortcuts',key.toUpperCase());
      button.dataset.key=key;keyButtons.set(key,button);
    }
    const calibrate=$('div',panel);calibrate.className='keyboard-tuning';
    function tunable(label,min,max,step,value,unit){
      const wrapper=$('label',calibrate,label);
      wrapper.className='keyboard-tune';
      const val=$('b',wrapper,value.toFixed(2)+' '+unit);
      const input=$('input',wrapper);
      input.type='range';input.min=min;input.max=max;input.step=step;input.value=value;
      input.addEventListener('input',()=>{
        val.textContent=Number(input.value).toFixed(2)+' '+unit;sendNow();
      });
      return input;
    }
    const speed=tunable('期望平移速度',0.03,0.25,0.01,0.12,'m/s');
    const turn=tunable('期望角速度',0.05,0.50,0.01,0.30,'rad/s');
    const controls=$('div',panel);controls.className='keyboard-actions';
    const stopButton=$('button',controls,'■ 停车 / 清除按键');
    stopButton.className='keyboard-stop';stopButton.type='button';
    const holdTestButton=$('button',controls,'按住 W 前进（鼠标/触屏）');
    holdTestButton.type='button';
    holdTestButton.className='keyboard-hold-test';
    holdTestButton.title='本机 ROS2 模式下按住相当于键盘 W；松开立即归零。绝不发送真实电机控制。';
    const exportButton=$('button',controls,'↓ 导出误差 CSV');
    exportButton.type='button';
    const noMechanisms=$('div',panel,'G/R 与实车机械动作仍是预留接口。下方的转盘、升降、旋转和夹爪操作只改变浏览器 MOCK 示意，不发送机构指令；实车仍需先验证 STM32 协议、限位与回执。');
    noMechanisms.className='keyboard-caution';
    const stats=$('div',panel);stats.className='keyboard-metrics';
    const metric={};
    for(const [key,label] of [
      ['desired','键盘期望 vx / vy / ω'],
      ['ros_observed','ROS2 话题收到 vx / vy / ω'],
      ['ros_delta','ROS2 命令回环差值 Δvx / Δvy / Δω'],
      ['ros_latency','ROS2 接收状态 / 延迟估计'],
      ['actual','真实底盘实际 vx / vy / ω'],
      ['pose','实测里程计 X / Y / 航向'],
      ['error','速度误差 Δvx / Δvy / Δω'],
      ['rms','速度误差 RMS'],
      ['position','指令积分参考与里程计位置偏差'],
      ['fresh','测量状态 / 时效']]){
      const card=$('div',stats);card.className='keyboard-metric';
      $('span',card,label);
      metric[key]=$('strong',card,'—');
      metric[key].parentElement.dataset.metric=key;
    }
    const graph=$('div',panel);graph.className='keyboard-graph';
    const graphHeading=$('h3',graph,'本机 ROS2 回环差值趋势（不是实体小车误差）');
    const chart=$('canvas',graph);chart.width=800;chart.height=175;
    chart.setAttribute('aria-label','参考速度与实际速度偏差曲线');

    const twinPanel=$('section',panel);
    twinPanel.className='keyboard-twin-panel';
    $('h3',twinPanel,'ROS2 速度直通 · 网页位置预览（MOCK）');
    const twinHint=$('p',twinPanel,
      'ROS2 订阅到 vx/vy/ω 就立刻积分位置和朝向。没有加减速限制、障碍物停车、任务阶段或自动复位；地图只作背景，虚拟车可以越过边界。紫色车不是真实 /odom。');
    twinHint.className='keyboard-note';
    const twinToolbar=$('div',twinPanel);twinToolbar.className='keyboard-twin-toolbar';
    const twinReset=$('button',twinToolbar,'手动回到起点（只影响网页预览）');
    twinReset.type='button';
    const twinStatus=$('div',twinPanel,'MOCK 底盘：等待本机 ROS 2 回环');
    twinStatus.className='keyboard-twin-status';
    const twinStats=$('div',twinPanel,'直通位姿：— · 实际订阅速度：—');
    twinStats.className='keyboard-twin-stats';
    // Preserve the subscribed ROS2 position preview as-is.
    controls.after(twinPanel);
    if(!window.CarMechanismPreview)
      throw Error('机构 MOCK 预览模块未加载，请刷新浏览器缓存');
    const mechanism=window.CarMechanismPreview.create(panel);
    twinPanel.after(mechanism.root);
    const pipeline=$('div',panel);pipeline.className='keyboard-pipeline';
    pipeline.textContent='本机模式：网页 → 本机 HTTP → PC ROS2 /car/teleop/local_test_twist → PC ROS2 订阅 → 网页';
    const detail=$('p',panel,'本机 ROS2 话题反馈仅用于验证通信和命令一致性；实体车速必须来自独立 REAL /odom。缺少树莓派或 STM32 时不会虚构实测车辆运动。');
    detail.className='keyboard-note';

    let active=new Set(),shift=false,holdTest=false,
        sequence=millis()*1000,sendBusy=false,
        pendingStop=false,disposed=false,lastHistory=[],routeMode='local_ros2',routeChanging=false;
    const nextSeq=()=>++sequence;
    const editing=t=>{
      if(!t)return false;
      if(t.isContentEditable||t.tagName==='TEXTAREA'||t.tagName==='SELECT')
        return true;
      // Non-text input controls (checkboxes, speed sliders, buttons) must
      // not swallow WASDQE just because they retain keyboard focus.
      return t.tagName==='INPUT' &&
        !['checkbox','radio','button','range','color'].includes(
          String(t.type).toLowerCase());
    };
    const localOnly=()=>
      ['127.0.0.1','localhost','::1'].includes(window.location.hostname);

    let mockLayer=null, mockResetInProgress=false;
    let mockResetVersion=0;
    let refreshRequestSeq=0, refreshAppliedSeq=0;
    // ROS2/HTTP alone defines the authoritative MOCK pose. A persistent
    // display tracker bridges packet arrivals instead of teleporting back to
    // each new 75ms HTTP snapshot.
    const motionTracker=window.CarKeyboardPreviewMotion.createTracker();
    let motionBusy=false, motionEpoch=0;
    let queuedDesired=false;
    const svgNS='http://www.w3.org/2000/svg';
    const fx=x=>60+x*680/2.4, fy=y=>740-y*680/2.4;
    function paintMock(data){
      if(routeMode!=='local_ros2'){
        if(mockLayer)mockLayer.remove();
        mockLayer=null;return;
      }
      if(data?.source_type!=='MOCK_KINEMATIC_ONLY'||
         data.hardware_authorized!==false||data.real_odom!==false||
         data.frame!=='field_preview_m_not_odom'){
        throw Error('虚拟底盘数据来源不符合 MOCK 隔离要求');
      }
      if(data.preview_mode!=='direct')
        throw Error('映射集禁止启用碰撞/任务状态机预览');
      if(!window.CarKeyboardPreviewMotion?.valid(data))
        throw Error('ROS2 反馈运动数据不完整');
      motionTracker.ingest(data,performance.now());
      twinStatus.textContent='电脑 ROS2 → 直通积分 · '+
        (data.state==='RUNNING'?'正在按订阅速度运动':'已停止（松键归零）');
      twinStats.textContent=
        `位置 (${fmt(data.x_m)}, ${fmt(data.y_m)}) m · `+
        `朝向 ${fmt(data.yaw_rad)} rad · `+
        `订阅速度 vx/vy/ω = ${fmt(data.velocity.vx)} / ${fmt(data.velocity.vy)} / ${fmt(data.velocity.wz)}`;
      const svg=document.getElementById('fieldSvg');
      if(!svg)return;
      if(!mockLayer||!mockLayer.isConnected){
        mockLayer=document.createElementNS(svgNS,'g');
        mockLayer.id='keyboardMockChassisLayer';
        mockLayer.setAttribute('pointer-events','none');
        svg.append(mockLayer);
        const trail=document.createElementNS(svgNS,'polyline');
        trail.id='keyboardMockChassisTrail';
        trail.setAttribute('stroke','#bd7cfa');
        trail.setAttribute('stroke-width','4');
        trail.setAttribute('stroke-dasharray','6 5');
        trail.setAttribute('fill','none');
        mockLayer.append(trail);
        const car=document.createElementNS(svgNS,'g');
        car.id='keyboardMockChassisBody';
        mockLayer.append(car);
        const body=document.createElementNS(svgNS,'rect');
        for(const [k,v] of Object.entries({x:-11,y:-15,width:22,height:30,rx:5,
          fill:'#b773e5',stroke:'#fff', 'stroke-width':2})){
          body.setAttribute(k,String(v));
        }
        car.append(body);
        const forward=document.createElementNS(svgNS,'path');
        forward.setAttribute('d','M -6,-15 L 0,-22 L 6,-15 Z');
        forward.setAttribute('fill','#fff');
        car.append(forward);
        // Decorative CAD-derived tray / lift / gripper follows the local
        // MOCK chassis; controls never enter a ROS2 or STM32 endpoint.
        mechanism.attachMap(car);
        const text=document.createElementNS(svgNS,'text');
        text.id='keyboardMockChassisCaption';
        text.textContent='MOCK';
        for(const [k,v] of Object.entries({'font-size':15,'font-weight':900,
          'paint-order':'stroke',stroke:'#fff','stroke-width':3,fill:'#7531a3'})){
          text.setAttribute(k,String(v));
        }
        mockLayer.append(text);
      }
      const trail=mockLayer.querySelector('#keyboardMockChassisTrail');
      trail?.setAttribute('points',(data.trace||[]).map(p=>
        fx(p[0]).toFixed(1)+','+fy(p[1]).toFixed(1)).join(' '));
      // The main animation loop renders the tracked pose (never raw data).
      // Only a new generation/reset may snap to its authoritative origin.
    }
    function renderMockPose(pose) {
      if(!mockLayer||!mockLayer.isConnected)return;
      const angle=90-pose.yaw_rad*180/Math.PI;
      mockLayer.querySelector('#keyboardMockChassisBody')?.setAttribute(
        'transform',`translate(${fx(pose.x_m).toFixed(3)} ${fy(pose.y_m).toFixed(3)}) rotate(${angle.toFixed(3)})`);
      const text=mockLayer.querySelector('#keyboardMockChassisCaption');
      text?.setAttribute('x',String(fx(pose.x_m)+24));
      text?.setAttribute('y',String(fy(pose.y_m)-23));
    }
    function animateMock(frameNow) {
      // Frame timestamp is shared by all mechanism projections; it is
      // independent of the 75ms ROS2 pose sampling and 300ms metrics loop.
      mechanism.tick(frameNow);
      // Visual continuity is independent of network arrival. Smoothing only
      // follows ROS2-observed /dryrun poses; never synthesizes physical odom.
      if(!disposed && routeMode==='local_ros2' &&
         !mockResetInProgress && !document.hidden) {
        const pose=motionTracker.frame(frameNow);
        if(pose)renderMockPose(pose);
      }
      if(!disposed)requestAnimationFrame(animateMock);
    }
    async function refreshMotion() {
      if(disposed||routeMode!=='local_ros2'||mockResetInProgress||
         motionBusy||document.hidden)return;
      motionBusy=true;
      const startedEpoch=motionEpoch;
      try {
        const response=await fetch(ENDPOINT+'/dryrun',{cache:'no-store'});
        if(!response.ok)throw Error('HTTP '+response.status);
        const mock=await response.json();
        if(startedEpoch!==motionEpoch||routeMode!=='local_ros2'||
           mockResetInProgress)return;
        paintMock(mock);
      }catch(err){
        // On HTTP outage the short extrapolation cap prevents endless motion.
        if(startedEpoch===motionEpoch)
          twinStatus.textContent='ROS2 位姿采样不可用：'+String(err);
      }finally{
        motionBusy=false;
      }
    }
    async function mockPost(payload){
      const res=await fetch(ENDPOINT+'/dryrun',{method:'POST',
        headers:{'Content-Type':'application/json'},
        body:JSON.stringify(payload),cache:'no-store'});
      const out=await res.json();
      if(!res.ok)throw Error(out.error||('HTTP '+res.status));
      return out;
    }
    function currentMap(){
      // In keyboard mode the field drawing is visual context ONLY.
      // No red obstacles, mission destination or map-change event affects
      // direct WASDQE movement or position.
      return {action:'reset',start_zone:'upper_right',blocked_cells:[]};
    }
    async function resetMock(stopKeys=true, reason='手动复位'){
      if(!localOnly())return;
      // Invalidate any in-flight GET /dryrun response: the previous state
      // must never redraw an old vehicle position after a scene reset.
      refreshAppliedSeq=++refreshRequestSeq;
      motionEpoch++; // Discard any pre-reset in-flight motion snapshot.
      motionTracker.clear();
      const payload=currentMap();
      payload.reset_seq=nextSeq();
      const version=++mockResetVersion;
      mockResetInProgress=true;
      try{
        if(stopKeys){
          await stopNow();
          if(version!==mockResetVersion)return;
        }
        const twin=await mockPost(payload);
        if(version!==mockResetVersion)return;
        motionTracker.reset(twin,performance.now());
        paintMock(twin);
        // A deliberate scene reset changes the origin immediately.
        const resetPose=motionTracker.frame(performance.now());
        if(resetPose)renderMockPose(resetPose);
        if(stopKeys){
          const note='网页位置已手动归零；直接按住 W/A/S/D/Q/E 继续。';
          twinStatus.textContent=note;
          healthStatus.textContent=note;
          healthStatus.dataset.reason='SCENE_REARM_REQUIRED';
          status.textContent=note;
        }
      }catch(err){
        if(version===mockResetVersion)
          twinStatus.textContent='MOCK 复位失败：'+err;
      }finally{
        if(version===mockResetVersion)mockResetInProgress=false;
      }
    }
    twinReset.addEventListener('click',()=>resetMock(true,'手动复位'));
    // In PC-only direct mode there is no Shift, enable checkbox or FSM.
    // In the FUTURE Pi/STM32 route the strict Shift deadman stays enabled.
    const deadmanHeld=()=>routeMode==='local_ros2'?
      active.size>0:enabled.checked&&(shift||holdTest);
    const wantKeys=()=>deadmanHeld()?[...active]:[];
    const desiredBody=()=>({
      action:'keys',client_seq:nextSeq(),keys:wantKeys().sort(),
      speed_mps:Number(speed.value),turn_rps:Number(turn.value),
      deadman:deadmanHeld()
    });
    async function post(body){
      const res=await fetch(ENDPOINT+'/intent',{
        method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify(body),cache:'no-store'});
      const data=await res.json();
      if(!res.ok)throw Error(data.error||('HTTP '+res.status));
      return data;
    }
    function paintRoute(){
      for(const button of [routeLocal,routePi]){
        const selected=button.dataset.route===routeMode;
        button.classList.toggle('selected',selected);
        button.setAttribute('aria-pressed',String(selected));
      }
      routeNotice.textContent=routeMode==='local_ros2' ?
        '本机直通：W/A/S/D/Q/E → ROS2 → 网页积分预览。无 Shift、任务状态机、障碍停车或自动回起点；没有真实电机控制。':
        '完整链路：电脑 ROS2 /car/teleop/desired_twist → 树莓派 ROS2 候选 → STM32（UART 尚未接入）→ REAL /odom 实测回传。未装设备时应显示离线。';
      pipeline.textContent=routeMode==='local_ros2' ?
        '网页键盘 → 本机 HTTP → PC ROS2 /car/teleop/local_test_twist → 本机 ROS2 订阅 → 网页':
        '网页键盘 → PC ROS2 /car/teleop/desired_twist → Pi ROS2 /car/teleop/stm32_candidate → [STM32 UART 未接入] → 实车 /odom';
      graphHeading.textContent=routeMode==='local_ros2'?
        '本机 ROS2 收发差值趋势（非小车运动误差）':
        'REAL /odom 速度跟踪误差趋势（必须连接实车）';
      twinPanel.hidden=routeMode!=='local_ros2';
      signalChip.dataset.online='false';
      signalChip.textContent=routeMode==='local_ros2'?
        '本机 ROS2：等待话题':'Pi 候选链路未连接';
      holdTestButton.disabled=routeMode!=='local_ros2';
      enabledLabel.hidden=routeMode==='local_ros2';
      noMechanisms.hidden=routeMode==='local_ros2';
      enabled.checked=routeMode==='local_ros2';
      help.textContent=routeMode==='local_ros2'?
        'W/S 前进后退 · A/D 左右横移 · Q/E 左右转向。直接按，不用 Shift；松开就停。方向基于车体 base_link，不考虑场地障碍。':
        '完整链路预留：须勾选使能并按住 Shift+W/A/S/D/Q/E；不发送真实电机指令。';
      for(const key of ['actual','pose','error','rms','position','fresh'])
        metric[key].parentElement.hidden=routeMode==='local_ros2';
      if(twinPanel.hidden&&mockLayer){mockLayer.remove();mockLayer=null;}
    }
    async function switchRoute(mode){
      if(routeChanging||mode===routeMode)return;
      routeChanging=true;
      motionEpoch++; // Discard any old route's in-flight motion snapshot.
      motionTracker.clear();
      enabled.checked=false;active.clear();shift=false;holdTest=false;
      refreshKeys();
      for(const b of [routeLocal,routePi])b.disabled=true;
      try{
        const result=await post({
          action:'route',client_seq:nextSeq(),route_mode:mode});
        routeMode=result.route_mode;
        paintRoute();
        // Switching back to PC ROS2 does not reset the recorded preview pose.
        status.textContent='链路已切换 · 键盘候选指令清零 · '+(
          routeMode==='local_ros2'?'无需树莓派':'等待树莓派 / STM32');
        status.dataset.state='stopped';
      }catch(err){
        status.textContent='链路切换失败：'+err;
      }finally{
        for(const b of [routeLocal,routePi])b.disabled=false;
        routeChanging=false;
      }
    }
    paintRoute();
    routeLocal.addEventListener('click',()=>switchRoute('local_ros2'));
    routePi.addEventListener('click',()=>switchRoute('pi_stm32'));
    async function stopNow(disable=false){
      active.clear();
      queuedDesired=false;
      // A released key must stop the visual prediction immediately, without
      // waiting for the next HTTP round trip or creating a fake ROS2 receipt.
      motionEpoch++;
      motionTracker.freeze(performance.now());
      shift=false;holdTest=false;
      if(disable)enabled.checked=false;
      for(const b of keyButtons.values())b.classList.remove('pressed');
      // Send STOP outside the nonzero-command queue so a user can always
      // release or halt immediately; client_seq rejects any older late POST.
      pendingStop=true;
      try{
        const result=await post({action:'stop',client_seq:nextSeq()});
        status.textContent='已停止候选指令 · STM32 实际发送仍禁止';
        status.dataset.state='stopped';
      }catch(err){status.textContent='停车请求失败，服务器 300ms 超时自动输出零速度：'+err;}
      finally{
        pendingStop=false;
        if(queuedDesired) {
          queuedDesired=false;
          sendNow();
        }
      }
    }
    function sendNow(){
      if(!localOnly()){
        status.textContent='映射集仅允许从本机 localhost 操作';
        return;
      }
      if(routeChanging)return;
      if(sendBusy||pendingStop){
        // Keep the latest key combination; do not lose A/D while W's POST
        // is still in flight. client_seq still rejects late older packets.
        queuedDesired=true;
        return;
      }
      sendBusy=true;
      const packet=desiredBody();
      post(packet).then(()=>{
        if(!pendingStop&&packet.client_seq===sequence)
          status.textContent=deadmanHeld()?
            '按键已发至本机 HTTP · 等待 ROS2 订阅反馈':
            '键盘已松开 · ROS2 零速度';
      }).catch(e=>{
        // A superseded keydown must not overwrite the STOP indicator.
        if(!pendingStop && packet.client_seq===sequence)
          status.textContent='键盘候选指令未送达：'+e;
      }).finally(()=>{
        sendBusy=false;
        if(queuedDesired && !pendingStop && !routeChanging) {
          queuedDesired=false;
          sendNow();
        }
      });
    }
    function refreshKeys(){
      for(const [key,b] of keyButtons)b.classList.toggle(
        'pressed',active.has(key)&&deadmanHeld());
    }
    // A physical hold button is a deliberate deadman, only for the
    // PC-only route. It requires no Raspberry Pi or physical keyboard.
    holdTestButton.addEventListener('pointerdown',ev=>{
      if(routeMode!=='local_ros2'||routeChanging)return;
      enabled.checked=true;
      holdTest=true;
      active.add('w');
      refreshKeys();
      sendNow();
      try{holdTestButton.setPointerCapture(ev.pointerId);}catch(_){}
      ev.preventDefault();
    });
    function releaseTestHold(){
      if(!holdTest)return;
      holdTest=false;
      stopNow();
    }
    holdTestButton.addEventListener('pointerup',releaseTestHold);
    holdTestButton.addEventListener('pointercancel',releaseTestHold);
    holdTestButton.addEventListener('lostpointercapture',releaseTestHold);
    enabled.addEventListener('change',()=>{
      if(!enabled.checked)stopNow(true);else sendNow();
    });
    stopButton.addEventListener('click',()=>stopNow());
    window.addEventListener('keydown',ev=>{
      if(editing(ev.target)||ev.ctrlKey||ev.altKey||ev.metaKey)return;
      const key=ev.key.toLowerCase();
      if(key==='shift'){
        if(routeMode!=='local_ros2'&&!shift){
          shift=true;sendNow();refreshKeys();
        }
        return;
      }
      if(ev.code==='Space'||key==='escape'){
        ev.preventDefault();stopNow();return;
      }
      if(!KEYS.has(key)||
        (routeMode!=='local_ros2'&&!enabled.checked))return;
      ev.preventDefault();
      if(active.has(key))return; // OS key-repeat must not flood HTTP.
      active.add(key);refreshKeys();sendNow();
    });
    window.addEventListener('keyup',ev=>{
      const key=ev.key.toLowerCase();
      if(key==='shift'){
        if(routeMode!=='local_ros2'){shift=false;stopNow();}
        return;
      }
      if(!KEYS.has(key)||!active.has(key))return;
      active.delete(key);refreshKeys();
      if(active.size===0)stopNow();else sendNow();
    });
    for(const [key,b] of keyButtons){
      b.addEventListener('pointerdown',ev=>{
        if(routeMode!=='local_ros2'&&!enabled.checked)return;
        if(!active.has(key)) {
          active.add(key);refreshKeys();sendNow();
        }
        try{b.setPointerCapture(ev.pointerId);}catch(_){}
        ev.preventDefault();
      });
      const released=()=>{
        if(active.delete(key)){
          refreshKeys();
          if(active.size===0)stopNow();else sendNow();
        }
      };
      b.addEventListener('pointerup',released);
      b.addEventListener('pointercancel',released);
      b.addEventListener('lostpointercapture',released);
    }
    window.addEventListener('blur',()=>stopNow());
    document.addEventListener('visibilitychange',()=>{
      if(document.hidden)stopNow();
    });
    window.addEventListener('pagehide',()=>{
      // Send zero command on page exit if available. 300 ms server TTL
      // remains the authoritative fail-safe for browser shutdown.
      const body=JSON.stringify({action:'stop',client_seq:nextSeq()});
      if(navigator.sendBeacon)
        navigator.sendBeacon(ENDPOINT+'/intent',
          new Blob([body],{type:'application/json'}));
    });
    function fmt(v,digits=3){
      return typeof v==='number'&&Number.isFinite(v)?v.toFixed(digits):'—';
    }
    function draw(samples){
      const ctx=chart.getContext('2d');if(!ctx)return;
      // The CSS size stays stable while the drawing buffer follows device
      // pixel density. This keeps axes/labels crisp on HiDPI laptop displays.
      const bounds=chart.getBoundingClientRect();
      const w=Math.max(240,Math.round(bounds.width||chart.clientWidth||800));
      const h=175;
      const ratio=Math.min(2,Math.max(1,window.devicePixelRatio||1));
      const pixelWidth=Math.round(w*ratio),pixelHeight=Math.round(h*ratio);
      if(chart.width!==pixelWidth||chart.height!==pixelHeight){
        chart.width=pixelWidth;chart.height=pixelHeight;
      }
      ctx.setTransform(ratio,0,0,ratio,0,0);
      ctx.clearRect(0,0,w,h);ctx.fillStyle='#071629';ctx.fillRect(0,0,w,h);
      for(let i=0;i<=4;i++){
        const y=16+i*(h-32)/4;
        ctx.beginPath();ctx.moveTo(33,y);ctx.lineTo(w-12,y);
        ctx.strokeStyle='#294054';ctx.lineWidth=1;ctx.stroke();
      }
      const data=samples.slice(-100);
      if(data.length<2){
        ctx.fillStyle='#a6bccd';ctx.font='14px sans-serif';
        ctx.fillText(routeMode==='local_ros2'?
          '等待本机 ROS2 订阅回环，非实际车速':'等待新鲜 REAL /odom（不能用回环替代）',48,h/2);
        return;
      }
      const series=[
        {key:'vx',color:'#6aaeff',label:'Δvx'},
        {key:'vy',color:'#f4948b',label:'Δvy'},
        {key:'wz',color:'#e6bc68',label:'Δω'}
      ];
      const max=Math.max(0.12,...data.flatMap(s=>series.map(
        p=>Math.abs(s.error?.[p.key]||0))));
      for(const row of series){
        ctx.beginPath();
        data.forEach((point,i)=>{
          const x=33+(w-46)*i/(data.length-1);
          const y=h/2-(point.error?.[row.key]||0)/max*(h/2-19);
          if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
        });
        ctx.strokeStyle=row.color;ctx.lineWidth=2;ctx.stroke();
      }
      ctx.font='12px sans-serif';
      ctx.fillStyle='#9cb8c9';ctx.fillText('± '+max.toFixed(2),8,18);
    }
    async function refresh(){
      if(disposed)return;
      const requestSeq=++refreshRequestSeq;
      try{
        const [rs,rh,ra]=await Promise.all([
          fetch(ENDPOINT+'/status',{cache:'no-store'}),
          fetch(ENDPOINT+'/history',{cache:'no-store'}),
          fetch(ENDPOINT+'/health',{cache:'no-store'})
        ]);
        if(!rs.ok||!rh.ok||!ra.ok)throw Error(
          '映射集 API 不完整；更新代码后需要重启 server.py');
        const data=await rs.json(),history=await rh.json(),
          health=await ra.json();
        // HTTP GETs can return out of order. Never repaint stale vehicle
        // positions or older control status on top of newer samples.
        if(requestSeq<=refreshAppliedSeq||mockResetInProgress)return;
        refreshAppliedSeq=requestSeq;
        if(health.source_type!=='LOCAL_ROS2_AND_MOCK_DIAGNOSTIC_ONLY'||
          health.hardware_authorized!==false)
          throw Error('链路诊断数据来源不合法');
        healthStatus.textContent='ROS2 直通诊断 '+health.reason+'：'+health.hint;
        healthStatus.dataset.reason=health.reason;
        if(data.hardware_authorized!==false||data.stm32_tx_enabled!==false)
          throw Error('拒绝未授权的映射数据');
        if(data.route_mode!=='local_ros2'&&data.route_mode!=='pi_stm32')
          throw Error('未知通信链路');
        if(!routeChanging&&routeMode!==data.route_mode){
          routeMode=data.route_mode;paintRoute();
        }
        const d=data.desired,c=data.comparison||{},r=data.ros2_feedback||{},
          observed=r.observed||{},delta=r.delta||{},
          m=c.measured||{},p=c.pose||{},e=c.error||{};
        const receiptOnline=r.online===true && r.source===routeMode;
        const receiptText=routeMode==='local_ros2' ?
          (receiptOnline?'本机 ROS2 回环在线':'本机 ROS2：等待话题') :
          (receiptOnline?'Pi 候选话题已收到（非电机反馈）':'Pi 候选链路未连接');
        signalChip.dataset.online=String(receiptOnline);
        if(signalChip.textContent!==receiptText)
          signalChip.textContent=receiptText;
        metric.ros_observed.textContent=r.online?
          `${fmt(observed.vx)} / ${fmt(observed.vy)} / ${fmt(observed.wz)}`:'—（话题未到达）';
        metric.ros_delta.textContent=r.online?
          `${fmt(delta.vx)} / ${fmt(delta.vy)} / ${fmt(delta.wz)}`:'—';
        metric.ros_latency.textContent=r.online?
          `${r.status} · ${fmt(r.approx_ros2_latency_ms,1)} ms（估计）`:
          r.status||'ROS2 订阅端离线';
        const realMode=routeMode==='pi_stm32';
        metric.pose.textContent=realMode?
          `${fmt(p.x)} / ${fmt(p.y)} / ${fmt(p.yaw)}`:'—（本机无实体里程计）';
        metric.desired.textContent=`${fmt(d.vx)} / ${fmt(d.vy)} / ${fmt(d.wz)}`;
        metric.actual.textContent=realMode?
          `${fmt(m.vx)} / ${fmt(m.vy)} / ${fmt(m.wz)}`:'—（ROS2 回环不等于实测）';
        metric.error.textContent=realMode&&c.error?
          `${fmt(e.vx)} / ${fmt(e.vy)} / ${fmt(e.wz)}`:'—（无实体跟踪数据）';
        metric.rms.textContent=realMode&&c.rms_linear_mps!=null?
          fmt(c.rms_linear_mps)+' m/s':'—';
        metric.position.textContent=realMode&&c.error?
          fmt(e.relative_position_error_m)+' m / '+fmt(e.relative_yaw_error_rad)+' rad':'—';
        metric.fresh.textContent=realMode?
          c.status+(c.odom_age_s!=null?' · '+fmt(c.odom_age_s)+' s':''):
          'LOCAL_ROS2_ONLY · 无实际车体反馈';
        const realHistory=Array.isArray(history.samples)?history.samples:[];
        const rosHistory=Array.isArray(history.ros2_samples)?history.ros2_samples:[];
        lastHistory=routeMode==='local_ros2'?
          rosHistory.filter(x=>x.source==='local_ros2')
            .map(x=>({...x,error:x.delta||{}})):
          realHistory;
        draw(lastHistory);
      }catch(err){
        const msg='映射集查询失败：'+String(err);
        metric.fresh.textContent=msg;
        healthStatus.textContent=msg;
        healthStatus.dataset.reason='API_ERROR';
      }
    }
    exportButton.addEventListener('click',()=>{
      const cols=['time_s','odom_seq','desired_vx','desired_vy','desired_wz',
        'actual_vx','actual_vy','actual_wz','actual_x','actual_y','actual_yaw',
        'integrated_target_x','integrated_target_y','integrated_target_yaw',
        'error_vx','error_vy','error_wz','error_linear_norm',
        'relative_position_error_m','relative_yaw_error_rad','source'];
      if(routeMode==='local_ros2'){
        for(const [a,b] of [['actual_vx','ros2_received_vx'],
                             ['actual_vy','ros2_received_vy'],
                             ['actual_wz','ros2_received_wz'],
                             ['error_vx','ros2_command_delta_vx'],
                             ['error_vy','ros2_command_delta_vy'],
                             ['error_wz','ros2_command_delta_wz']]){
          const index=cols.indexOf(a);
          if(index>=0)cols[index]=b;
        }
      }
      const rows=[cols.join(',')];
      for(const x of lastHistory){
        rows.push([x.time_s??x.received_wall_s,x.odom_seq??'',
          x.command?.vx??x.desired_at_receive?.vx,
          x.command?.vy??x.desired_at_receive?.vy,
          x.command?.wz??x.desired_at_receive?.wz,
          x.actual?.vx??x.twist?.vx,x.actual?.vy??x.twist?.vy,
          x.actual?.wz??x.twist?.wz,
          x.actual_pose?.x,x.actual_pose?.y,x.actual_pose?.yaw,
          x.integrated_target_pose?.x,x.integrated_target_pose?.y,
          x.integrated_target_pose?.yaw,
          x.error?.vx,x.error?.vy,x.error?.wz,x.error?.linear_norm_mps,
          x.error?.relative_position_error_m,x.error?.relative_yaw_error_rad,
          x.sample_source??x.source].map(y=>y==null?'':String(y)).join(','));
      }
      const blob=new Blob([rows.join('\n')+'\n'],{type:'text/csv;charset=utf-8'});
      const url=URL.createObjectURL(blob);
      const a=document.createElement('a');a.href=url;a.download='keyboard_mapping_error.csv';
      a.download=routeMode==='local_ros2'?'keyboard_local_ros2_echo.csv':'keyboard_real_odom_error.csv';
      a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
    });
    if(!localOnly()){
      enabled.disabled=true;
      status.textContent='需在运行 HTTP 服务的电脑上通过 localhost 打开映射集';
    }
    draw([]);
    refreshMotion();
    refresh();
    if(window.requestAnimationFrame)requestAnimationFrame(animateMock);
    setInterval(refreshMotion,75); // 13Hz authoritative motion snapshots.
    setInterval(()=>{
      // This is only a network-loss watchdog heartbeat, NOT a motion FSM:
      // refresh active direct keys while held; release transmits zero.
      if(deadmanHeld()&&active.size>0&&!document.hidden)sendNow();
    },100);
    setInterval(refresh,300); // Metrics / history need not block 60fps SVG.
  }
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',mount);
  else mount();
})();
