/* /keyboard Mapping Set: PC keyboard -> ROS2 candidates, REAL-only odom errors.
 * No actual motor command, no /cmd_vel, no /dev/tty; Shift deadman + TTL.
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
    $('h2',header,'键盘映射 · 双通信链路');
    $('p',header,'没有树莓派也能先在电脑 ROS 2 中完成真实话题发布与订阅回环；将来切换完整链路后，再连接树莓派和 STM32。两种模式都不向实体电机发送指令。');

    const modes=$('div',panel);modes.className='keyboard-route-switch';
    modes.setAttribute('role','group');modes.setAttribute('aria-label','ROS2 键盘控制链路');
    const routeLocal=$('button',modes,'本机 ROS2 回环 · 现在可用');
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
    const enabledLabel=$('label',panel,'本机启用键盘 ROS2 候选指令');
    enabledLabel.className='keyboard-allow';
    const enabled=$('input',enabledLabel);enabled.type='checkbox';
    enabledLabel.prepend(enabled);
    const help=$('p',panel,'必须持续按住 Shift（使能）才能发送非零速度，松开立即停车。W/S 前后，A/D 左右平移，Q/E 旋转，空格/Esc 停车。键盘输入框聚焦时不触发运动。');
    help.className='keyboard-notice';
    const grid=$('div',panel);grid.className='keyboard-keygrid';
    const keyButtons=new Map();
    for(const [key,label] of [
        ['q','Q ↶ 左转'],['w','W ↑ 前进'],['e','E ↷ 右转'],
        ['a','A ← 左移'],['s','S ↓ 后退'],['d','D → 右移']]){
      const button=$('button',grid,label);
      button.type='button';button.className='keyboard-key';
      button.setAttribute('aria-label','模拟按住 '+label);
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
    const holdTestButton=$('button',controls,'按住本机前进 · 松开停车');
    holdTestButton.type='button';
    holdTestButton.className='keyboard-hold-test';
    holdTestButton.title='仅本机 ROS2 回环有效；鼠标或触屏按住相当于按住 Shift+W。没有实体电机发送。';
    const exportButton=$('button',controls,'↓ 导出误差 CSV');
    exportButton.type='button';
    const noMechanisms=$('div',panel,'G 夹爪、R 伸缩：仅预留映射，不发送机构指令。必须先对齐 STM32 机械动作协议、限位与完成回执。');
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
    }
    const graph=$('div',panel);graph.className='keyboard-graph';
    const graphHeading=$('h3',graph,'本机 ROS2 回环差值趋势（不是实体小车误差）');
    const chart=$('canvas',graph);chart.width=800;chart.height=175;
    chart.setAttribute('aria-label','参考速度与实际速度偏差曲线');

    const twinPanel=$('section',panel);
    twinPanel.className='keyboard-twin-panel';
    $('h3',twinPanel,'虚拟麦轮底盘 · 场地运动预览（MOCK）');
    const twinHint=$('p',twinPanel,
      '运动输入来自电脑 ROS2 真正订阅到的本机话题；模型包含加速度和场地障碍检查，但没有实际电机、编码器、轮胎滑移或雷达。');
    twinHint.className='keyboard-note';
    const twinToolbar=$('div',twinPanel);twinToolbar.className='keyboard-twin-toolbar';
    const twinReset=$('button',twinToolbar,'复位虚拟底盘');
    twinReset.type='button';
    const twinQa=$('button',twinToolbar,'运行 7 项离线验收');
    twinQa.type='button';
    const faultLabel=$('label',twinToolbar,'软件故障注入');
    const twinFault=$('select',faultLabel);
    for(const [value,title] of [
      ['none','无故障'],['motor_stall','模拟电机卡滞'],
      ['slow_response','模拟加速迟缓'],['link_loss','模拟控制链丢失']]){
      const op=$('option',twinFault,title);op.value=value;
    }
    const twinStatus=$('div',twinPanel,'MOCK 底盘：等待本机 ROS 2 回环');
    twinStatus.className='keyboard-twin-status';
    const twinStats=$('div',twinPanel,'运动位置：— · 模拟速度：— · 模型误差：—');
    twinStats.className='keyboard-twin-stats';
    const twinQaResults=$('pre',twinPanel,'七项验收尚未运行');
    twinQaResults.className='keyboard-twin-qa';
    // Keep the simulation controls above long REAL-error graphs so the
    // field and movement/FAULT buttons can be viewed together.
    controls.after(twinPanel);
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
      // The "enable keyboard" checkbox remains focused after click; it is
      // not a text editor and must not swallow Shift+W/A/S/D/Q/E.
      return t.tagName==='INPUT' &&
        !['checkbox','radio','button'].includes(String(t.type).toLowerCase());
    };
    const localOnly=()=>
      ['127.0.0.1','localhost','::1'].includes(window.location.hostname);

    let mockLayer=null, lastMapSignature=null, mockResetInProgress=false;
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
      twinStatus.textContent='MOCK · '+data.state+
        (data.collision_reason?' · 已停止：'+data.collision_reason:'')+
        ' · '+data.fault_mode;
      twinStats.textContent=
        `位置 (${fmt(data.x_m)}, ${fmt(data.y_m)}) m · `+
        `航向 ${fmt(data.yaw_rad)} rad · `+
        `速度 ${fmt(data.velocity.vx)}, ${fmt(data.velocity.vy)}, ${fmt(data.velocity.wz)} · `+
        `模型速度误差 ${fmt(data.model_error.linear_norm_mps)} m/s`;
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
      const angle=90-data.yaw_rad*180/Math.PI;
      mockLayer.querySelector('#keyboardMockChassisBody')?.setAttribute(
        'transform',`translate(${fx(data.x_m).toFixed(2)} ${fy(data.y_m).toFixed(2)}) rotate(${angle.toFixed(1)})`);
      const text=mockLayer.querySelector('#keyboardMockChassisCaption');
      text?.setAttribute('x',String(fx(data.x_m)+15));
      text?.setAttribute('y',String(fy(data.y_m)-16));
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
      const field=window.Sim2RealCore?.getTestFieldSnapshot?.();
      return {
        action:'reset',
        start_zone:['upper_right','lower_right'].includes(field?.start_zone)?
          field.start_zone:'upper_right',
        blocked_cells:Array.isArray(field?.blocked_cells)?
          field.blocked_cells:[]
      };
    }
    async function resetMock(stopKeys=true){
      if(mockResetInProgress||!localOnly())return;
      mockResetInProgress=true;
      try{
        if(stopKeys)await stopNow();
        const payload=currentMap();
        lastMapSignature=JSON.stringify(payload);
        const twin=await mockPost(payload);
        twinFault.value='none';
        paintMock(twin);
      }catch(err){
        twinStatus.textContent='MOCK 复位失败：'+err;
      }finally{
        mockResetInProgress=false;
      }
    }
    twinReset.addEventListener('click',()=>resetMock(true));
    twinQa.addEventListener('click',async()=>{
      twinQa.disabled=true;
      twinQaResults.textContent='运行软件模型故障矩阵…';
      try{
        const res=await fetch(ENDPOINT+'/mock_acceptance',{cache:'no-store'});
        const result=await res.json();
        if(!res.ok||result.source_type!=='MOCK_KINEMATIC_ONLY'||
           result.hardware_authorized!==false)
          throw Error('离线验收返回非法结果');
        twinQaResults.textContent=
          (result.passed?'离线模型：PASS':'离线模型：FAIL')+
          ' · '+result.summary.PASS+'/'+result.cases.length+' 项\n'+
          result.cases.map(c=>(c.passed?'✓ ':'✗ ')+c.case+
            ' — '+c.description).join('\n');
      }catch(err){
        twinQaResults.textContent='测试执行失败：'+err;
      }finally{twinQa.disabled=false;}
    });
    twinFault.addEventListener('change',async()=>{
      try{
        const data=await mockPost({action:'fault',
          fault_mode:twinFault.value});
        paintMock(data);
      }catch(err){twinStatus.textContent='故障注入失败：'+err;}
    });
    window.addEventListener('sim2real:field-changed',()=>{
      const signature=JSON.stringify(currentMap());
      if(signature!==lastMapSignature)resetMock(true);
    });
    const deadmanHeld=()=>enabled.checked&&(shift||holdTest);
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
        '本机备用链路：键盘 → 电脑 ROS2 /car/teleop/local_test_twist → 独立 ROS2 订阅节点 → 网页。只验证话题传输，不是车辆实际运动。':
        '完整链路：电脑 ROS2 /car/teleop/desired_twist → 树莓派 ROS2 候选 → STM32（UART 尚未接入）→ REAL /odom 实测回传。未装设备时应显示离线。';
      pipeline.textContent=routeMode==='local_ros2' ?
        '网页键盘 → 本机 HTTP → PC ROS2 /car/teleop/local_test_twist → 本机 ROS2 订阅 → 网页':
        '网页键盘 → PC ROS2 /car/teleop/desired_twist → Pi ROS2 /car/teleop/stm32_candidate → [STM32 UART 未接入] → 实车 /odom';
      graphHeading.textContent=routeMode==='local_ros2'?
        '本机 ROS2 收发差值趋势（非小车运动误差）':
        'REAL /odom 速度跟踪误差趋势（必须连接实车）';
      twinPanel.hidden=routeMode!=='local_ros2';
      holdTestButton.disabled=routeMode!=='local_ros2';
      if(twinPanel.hidden&&mockLayer){mockLayer.remove();mockLayer=null;}
    }
    async function switchRoute(mode){
      if(routeChanging||mode===routeMode)return;
      routeChanging=true;
      enabled.checked=false;active.clear();shift=false;holdTest=false;
      refreshKeys();
      for(const b of [routeLocal,routePi])b.disabled=true;
      try{
        const result=await post({
          action:'route',client_seq:nextSeq(),route_mode:mode});
        routeMode=result.route_mode;
        paintRoute();
        if(routeMode==='local_ros2')await resetMock(false);
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
      finally{pendingStop=false;}
    }
    function sendNow(){
      if(!localOnly()){
        status.textContent='映射集仅允许从本机 localhost 操作';
        return;
      }
      if(sendBusy||pendingStop||routeChanging)return;
      sendBusy=true;
      const packet=desiredBody();
      post(packet).then(()=>{
        if(!pendingStop && packet.client_seq===sequence && enabled.checked)
          status.textContent=(shift||holdTest)?
            '按键意图已交给本机服务器 · 等待 ROS2 回环确认':
            '映射准备就绪 · 按住 Shift 使能';
      }).catch(e=>{
        // A superseded keydown must not overwrite the STOP indicator.
        if(!pendingStop && packet.client_seq===sequence)
          status.textContent='键盘候选指令未送达：'+e;
      }).finally(()=>{sendBusy=false;});
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
        if(!shift){shift=true;sendNow();refreshKeys();}
        return;
      }
      if(ev.code==='Space'||key==='escape'){
        ev.preventDefault();stopNow();return;
      }
      if(!KEYS.has(key)||!enabled.checked)return;
      ev.preventDefault();
      active.add(key);refreshKeys();sendNow();
    });
    window.addEventListener('keyup',ev=>{
      const key=ev.key.toLowerCase();
      if(key==='shift'){
        shift=false;stopNow();return;
      }
      if(!KEYS.has(key))return;
      active.delete(key);refreshKeys();sendNow();
    });
    for(const [key,b] of keyButtons){
      b.addEventListener('pointerdown',ev=>{
        if(!enabled.checked)return;
        active.add(key);refreshKeys();sendNow();
        try{b.setPointerCapture(ev.pointerId);}catch(_){}
        ev.preventDefault();
      });
      const released=()=>{
        active.delete(key);refreshKeys();sendNow();
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
      const w=chart.width,h=chart.height;
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
      try{
        const [rs,rh,rm,ra]=await Promise.all([
          fetch(ENDPOINT+'/status',{cache:'no-store'}),
          fetch(ENDPOINT+'/history',{cache:'no-store'}),
          fetch(ENDPOINT+'/dryrun',{cache:'no-store'}),
          fetch(ENDPOINT+'/health',{cache:'no-store'})
        ]);
        if(!rs.ok||!rh.ok||!rm.ok||!ra.ok)throw Error(
          '映射集 API 不完整；更新代码后需要重启 server.py');
        const data=await rs.json(),history=await rh.json(),
          mock=await rm.json(),health=await ra.json();
        if(health.source_type!=='LOCAL_ROS2_AND_MOCK_DIAGNOSTIC_ONLY'||
          health.hardware_authorized!==false)
          throw Error('链路诊断数据来源不合法');
        healthStatus.textContent='链路诊断 '+health.reason+'：'+health.hint;
        healthStatus.dataset.reason=health.reason;
        if(routeMode==='local_ros2')paintMock(mock);
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
    resetMock(false).then(refresh);
    setInterval(()=>{
      if(enabled.checked&&shift&&!document.hidden)sendNow();
    },100);
    setInterval(refresh,250);
  }
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',mount);
  else mount();
})();
