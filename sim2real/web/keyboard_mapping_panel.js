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
    $('h2',header,'键盘映射与实车跟踪误差');
    $('p',header,'电脑 ROS2 → 树莓派 5 ROS2 → STM32 候选接口。当前物理串口发送禁止；误差数据只来自 REAL /odom，不使用模拟车位置。');

    const status=$('div',panel,'默认断开 · 不向实体电机发送控制');
    status.id='keyboardLinkState';status.className='keyboard-state';
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
    const exportButton=$('button',controls,'↓ 导出误差 CSV');
    exportButton.type='button';
    const noMechanisms=$('div',panel,'G 夹爪、R 伸缩：仅预留映射，不发送机构指令。必须先对齐 STM32 机械动作协议、限位与完成回执。');
    noMechanisms.className='keyboard-caution';
    const stats=$('div',panel);stats.className='keyboard-metrics';
    const metric={};
    for(const [key,label] of [
      ['desired','期望 vx / vy / ω'],
      ['actual','实际 vx / vy / ω'],
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
    $('h3',graph,'速度跟踪误差趋势 · 仅真实 /odom 样本');
    const chart=$('canvas',graph);chart.width=800;chart.height=175;
    chart.setAttribute('aria-label','参考速度与实际速度偏差曲线');
    const pipeline=$('div',panel);pipeline.className='keyboard-pipeline';
    pipeline.textContent='PC HTTP intent → /car/teleop/desired_twist → Pi /car/teleop/stm32_candidate → [STM32 UART TX 尚未接入]';
    const detail=$('p',panel,'期望值采用 base_link 坐标；实际值来自 REAL /odom 频道。相对位姿偏差由指令积分估算，不能当作绝对地图定位误差。');
    detail.className='keyboard-note';

    let active=new Set(),shift=false,sequence=millis()*1000,sendBusy=false,
        pendingStop=false,disposed=false,lastHistory=[];
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
    const wantKeys=()=>enabled.checked&&shift?[...active]:[];
    const desiredBody=()=>({
      action:'keys',client_seq:nextSeq(),keys:wantKeys().sort(),
      speed_mps:Number(speed.value),turn_rps:Number(turn.value),
      deadman:enabled.checked&&shift
    });
    async function post(body){
      const res=await fetch(ENDPOINT+'/intent',{
        method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify(body),cache:'no-store'});
      const data=await res.json();
      if(!res.ok)throw Error(data.error||('HTTP '+res.status));
      return data;
    }
    async function stopNow(disable=false){
      active.clear();
      shift=false;
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
      if(sendBusy||pendingStop)return;
      sendBusy=true;
      const packet=desiredBody();
      post(packet).then(()=>{
        if(!pendingStop && packet.client_seq===sequence && enabled.checked)
          status.textContent=shift?
            '已发送 ROS2 候选速度（非实体电机）':
            '映射准备就绪 · 按住 Shift 使能';
      }).catch(e=>{
        // A superseded keydown must not overwrite the STOP indicator.
        if(!pendingStop && packet.client_seq===sequence)
          status.textContent='键盘候选指令未送达：'+e;
      }).finally(()=>{sendBusy=false;});
    }
    function refreshKeys(){
      for(const [key,b] of keyButtons)b.classList.toggle(
        'pressed',active.has(key)&&shift&&enabled.checked);
    }
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
        ctx.fillText('等待新鲜 REAL /odom，与键盘意图完成同步后绘图',48,h/2);
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
        const [rs,rh]=await Promise.all([
          fetch(ENDPOINT+'/status',{cache:'no-store'}),
          fetch(ENDPOINT+'/history',{cache:'no-store'})
        ]);
        if(!rs.ok||!rh.ok)throw Error('HTTP 状态接口离线');
        const data=await rs.json(),history=await rh.json();
        if(data.hardware_authorized!==false||data.stm32_tx_enabled!==false)
          throw Error('拒绝未授权的映射数据');
        const d=data.desired,c=data.comparison||{},m=c.measured||{},p=c.pose||{},e=c.error||{};
        metric.pose.textContent=`${fmt(p.x)} / ${fmt(p.y)} / ${fmt(p.yaw)}`;
        metric.desired.textContent=`${fmt(d.vx)} / ${fmt(d.vy)} / ${fmt(d.wz)}`;
        metric.actual.textContent=`${fmt(m.vx)} / ${fmt(m.vy)} / ${fmt(m.wz)}`;
        metric.error.textContent=c.error?
          `${fmt(e.vx)} / ${fmt(e.vy)} / ${fmt(e.wz)}`:'—（未满足对比条件）';
        metric.rms.textContent=c.rms_linear_mps!=null?
          fmt(c.rms_linear_mps)+' m/s':'—';
        metric.position.textContent=c.error?
          fmt(e.relative_position_error_m)+' m / '+fmt(e.relative_yaw_error_rad)+' rad':'—';
        metric.fresh.textContent=c.status+
          (c.odom_age_s!=null?' · '+fmt(c.odom_age_s)+' s':'');
        lastHistory=Array.isArray(history.samples)?history.samples:[];
        draw(lastHistory);
      }catch(err){
        metric.fresh.textContent='数据接口异常：'+String(err);
      }
    }
    exportButton.addEventListener('click',()=>{
      const cols=['time_s','odom_seq','desired_vx','desired_vy','desired_wz',
        'actual_vx','actual_vy','actual_wz','actual_x','actual_y','actual_yaw',
        'integrated_target_x','integrated_target_y','integrated_target_yaw',
        'error_vx','error_vy','error_wz','error_linear_norm',
        'relative_position_error_m','relative_yaw_error_rad','source'];
      const rows=[cols.join(',')];
      for(const x of lastHistory){
        rows.push([x.time_s,x.odom_seq,x.command?.vx,x.command?.vy,
          x.command?.wz,x.actual?.vx,x.actual?.vy,x.actual?.wz,
          x.actual_pose?.x,x.actual_pose?.y,x.actual_pose?.yaw,
          x.integrated_target_pose?.x,x.integrated_target_pose?.y,
          x.integrated_target_pose?.yaw,
          x.error?.vx,x.error?.vy,x.error?.wz,x.error?.linear_norm_mps,
          x.error?.relative_position_error_m,x.error?.relative_yaw_error_rad,
          x.sample_source].map(y=>y==null?'':String(y)).join(','));
      }
      const blob=new Blob([rows.join('\n')+'\n'],{type:'text/csv;charset=utf-8'});
      const url=URL.createObjectURL(blob);
      const a=document.createElement('a');a.href=url;a.download='keyboard_mapping_error.csv';
      a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
    });
    if(!localOnly()){
      enabled.disabled=true;
      status.textContent='需在运行 HTTP 服务的电脑上通过 localhost 打开映射集';
    }
    draw([]);
    refresh();
    setInterval(()=>{
      if(enabled.checked&&shift&&!document.hidden)sendNow();
    },100);
    setInterval(refresh,250);
  }
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',mount);
  else mount();
})();
