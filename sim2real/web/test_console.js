/* CAR-LINK TEST inspection panel: real ROS2 mock feedback, never browser animation.
   All dynamic text comes from textContent (no untrusted HTML rendering). */
(() => {
  'use strict';
  if (window.CarWebMode?.mode !== 'test') return;
  const $ = id => document.getElementById(id);
  const numeric = (x, digits=2, suffix='') =>
    typeof x === 'number' && Number.isFinite(x) ? x.toFixed(digits)+suffix : '—';
  const numberOrDash = x => typeof x === 'number' && Number.isFinite(x) ? String(x) : '—';
  const field = (key, value) => { const el=$(key); if (el) el.textContent=value; };
  const names = {chassis:'底盘位姿',wheels:'四轮脉冲',link:'串口链路',mission:'任务状态',
                 vision:'OpenMV 视觉事件',feedback:'命令执行回执',mechanism:'机械机构',power:'电源'};
  const statusName = x => x === true ? '有效' : x === false ? '无效' : '未接收';

  function mount() {
    const style=document.createElement('style');
    style.textContent=`
     .car-test-panel{max-width:1500px;margin:12px auto 22px;padding:18px;border:1px solid #46617a;
       border-radius:16px;background:#101f2f;color:#e8f5ff;font:14px/1.55 system-ui,sans-serif}
     .car-test-panel h2,.car-test-panel h3{margin:0 0 8px;font:700 18px/1.35 system-ui,sans-serif;color:#f1fbff}
     .car-test-panel h3{font-size:15px}
     .car-test-panel p{margin:4px 0 11px;color:#b1cada}
     .car-test-panel .ct-head{display:flex;flex-wrap:wrap;align-items:center;justify-content:space-between;gap:10px}
     .car-test-panel .ct-status{border:1px solid #756a52;background:#352b23;color:#ffd499;padding:5px 12px;border-radius:99px}
     .car-test-panel .ct-status[data-online=true]{background:#173e35;border-color:#377769;color:#92ebc6}
     .car-test-panel .ct-cards{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:11px;margin:13px 0}
     .car-test-panel .ct-card{border:1px solid #354f68;border-radius:11px;background:#172c40;padding:12px;min-width:0}
     .car-test-panel .ct-card small{display:block;color:#b2cedb;margin-bottom:8px}
     .car-test-panel .ct-card strong{display:block;font:700 21px/1.3 ui-monospace,monospace;color:#ecfbff;overflow-wrap:anywhere}
     .car-test-panel .ct-two{display:grid;grid-template-columns:1.15fr 1fr;gap:12px}
     .car-test-panel table{border-collapse:collapse;width:100%;font-size:12px}
     .car-test-panel th,.car-test-panel td{padding:7px 5px;border-bottom:1px solid #344e63;text-align:left}
     .car-test-panel th{font-weight:600;color:#afd0dc}
     .car-test-panel .ct-events{max-height:220px;overflow:auto;white-space:pre-wrap;
       font:12px/1.7 ui-monospace,monospace;color:#d3e8f3;min-height:75px}
     .car-test-panel .ct-topics{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:7px}
     .car-test-panel .ct-topic{border:1px solid #304c62;border-radius:8px;padding:7px;font-size:12px}
     .car-test-panel .ct-topic[data-fresh=true]{border-color:#367661}
     .car-test-panel .ct-topic strong{display:block;font-size:12px}
     .car-test-panel pre{max-height:250px;overflow:auto;font-size:11px;color:#c5dae6}
     @media(max-width:1000px){.car-test-panel .ct-cards{grid-template-columns:repeat(2,minmax(0,1fr))}
       .car-test-panel .ct-topics{grid-template-columns:repeat(2,minmax(0,1fr))}}
     @media(max-width:700px){.car-test-panel .ct-two{grid-template-columns:1fr}}
    `;
    document.head.append(style);
    const panel=document.createElement('section');
    panel.id='carTestPanel';panel.className='car-test-panel';
    panel.setAttribute('aria-label','CAR-LINK V1 测试集遥测台');
    panel.innerHTML=`
      <div class="ct-head"><h2>TEST · CAR-LINK V1 端到端测试台</h2>
        <span id="ctStatus" class="ct-status" data-online="false">等待模拟 MCU 与 ROS 2</span></div>
      <p>这是 Linux PTY 模拟 STM32 经 CAR-LINK CRC16 解帧后的 <b>ROS 2 回传数据</b>，不是网页播放的预设轨迹。所有 B2 命令回执均由模拟 MCU 定时生成，<b>不代表有真实命令下发</b>。</p>
      <div class="ct-cards">
        <div class="ct-card"><small>反馈位姿 X / Y · m</small><strong id="ctPose">—</strong></div>
        <div class="ct-card"><small>当前底盘速度 vx / vy · m/s</small><strong id="ctSpeed">—</strong></div>
        <div class="ct-card"><small>UART 有效帧 / CRC 错误</small><strong id="ctFrames">—</strong></div>
        <div class="ct-card"><small>ROS 2 底盘话题频率</small><strong id="ctRate">—</strong></div>
      </div>
      <div class="ct-two">
        <div class="ct-card"><h3>四轮目标 / 实际编码器速度（pulse/s）</h3>
          <table><thead><tr><th>车轮</th><th>目标</th><th>实际</th><th>偏差</th></tr></thead>
          <tbody><tr><td>FL 左前</td><td id="ctFLTarget">—</td><td id="ctFLActual">—</td><td id="ctFLErr">—</td></tr>
          <tr><td>FR 右前</td><td id="ctFRTarget">—</td><td id="ctFRActual">—</td><td id="ctFRErr">—</td></tr>
          <tr><td>RL 左后</td><td id="ctRLTarget">—</td><td id="ctRLActual">—</td><td id="ctRLErr">—</td></tr>
          <tr><td>RR 右后</td><td id="ctRRTarget">—</td><td id="ctRRActual">—</td><td id="ctRRErr">—</td></tr></tbody></table>
          <p id="ctWheelValidity">等待轮速报文</p></div>
        <div class="ct-card"><h3>任务 / 视觉 / 控制反馈</h3>
          <p><b>任务：</b><span id="ctMission">—</span></p>
          <p><b>视觉：</b><span id="ctVision">—</span></p>
          <p><b>执行反馈：</b><span id="ctCommand">—</span></p>
          <p><b>机构：</b><span id="ctMechanism">—</span></p>
          <p><b>电源：</b><span id="ctPower">—</span></p>
          <p><b>安全：</b>仅监控；本页没有向 STM32 发送指令的功能</p></div>
      </div>
      <div class="ct-card" style="margin-top:12px"><h3>ROS 2 话题新鲜度与频率</h3><div id="ctTopics" class="ct-topics"></div></div>
      <div class="ct-two" style="margin-top:12px">
        <div class="ct-card"><h3>模拟 MCU 事件日志</h3><div id="ctEvents" class="ct-events">尚无事件</div></div>
        <div class="ct-card"><h3>启动及联调检查</h3>
          <p>工作目录 <code>~/Desktop/car</code>，运行 <code>bash algorithm/car_ws/bash/web.sh test</code>。</p>
          <p>串口由 Mock MCU 写入 PTY；ROS 2 发布 /odom 和 /car/*；两个 HTTP 桥接分别推送轨迹和诊断。</p>
          <p id="ctPipeline">等待链路可用</p>
          <details><summary>查看最近一次 TEST 诊断原始 JSON</summary><pre id="ctRaw">—</pre></details>
        </div>
      </div>`;
    const nav=$('carModeSwitch');
    if(nav)nav.after(panel);else document.querySelector('main')?.before(panel);
    const topics=$('ctTopics');
    for(const name of Object.keys(names)) {
      const item=document.createElement('div');item.className='ct-topic';item.id='ctTopic'+name;
      const strong=document.createElement('strong');strong.textContent=names[name];
      const content=document.createElement('span');content.id='ctTopicData'+name;content.textContent='未接收';
      item.append(strong,content);topics.append(item);
    }
    const anchorX=$('liveAnchorX'),anchorY=$('liveAnchorY');
    // The mock's bounded 2D trajectory is translated to the middle of the 2.4m field.
    if(anchorX&&anchorY){anchorX.value='1.20';anchorY.value='1.20';}
    poll();setInterval(poll,450);
  }

  let fetching=false,lastSeq=0;
  async function poll(){
    if(fetching)return;fetching=true;
    try{
      const response=await fetch('/api/test/inspection',{cache:'no-store'});
      if(!response.ok)throw Error('HTTP '+response.status);
      const packet=await response.json();const s=packet.snapshot||{};
      const t=s.topics||{},samples=s.samples||{};
      const healthy=!!(packet.online&&t.link?.fresh&&samples.link?.connected&&samples.link?.heartbeat_fresh&&t.chassis?.fresh);
      $('ctStatus').dataset.online=String(healthy);
      field('ctStatus',healthy?'MOCK 在线 · 心跳有效':'离线 / 等待有效模拟链路');
      if(!packet.online){field('ctPipeline','模拟诊断通道超时；停止更新，不会伪造实测运动。');return;}
      field('ctPipeline',`模拟监控桥接在线；链路心跳 ${statusName(samples.link?.heartbeat_fresh)} · ROS 2 里程计数据请查看下方轨迹`);
      const c=samples.chassis||{}, w=samples.wheels||{}, l=samples.link||{}, m=samples.mission||{}, v=samples.vision||{}, f=samples.feedback||{}, a=samples.mechanism||{}, p=samples.power||{};
      field('ctPose',numeric(c.x_m,3)+' / '+numeric(c.y_m,3));
      field('ctSpeed',numeric(c.vx_mps,3)+' / '+numeric(c.vy_mps,3));
      field('ctFrames',numberOrDash(l.received_frames)+' / '+numberOrDash(l.crc_errors));
      field('ctRate',numeric(t.chassis?.hz,1,' Hz'));
      for(const [i,code] of ['FL','FR','RL','RR'].entries()) {
        const target=w.target_pulse_per_s?.[i],actual=w.actual_pulse_per_s?.[i];
        field('ct'+code+'Target',numberOrDash(target));
        field('ct'+code+'Actual',numberOrDash(actual));
        field('ct'+code+'Err',typeof target==='number'&&typeof actual==='number'?numeric(actual-target,1):'—');
      }
      field('ctWheelValidity',w.per_wheel_pid_output_valid?'PID 报文字段有效；编码器累计计数并未由 V1 提供':'未收到有效的四轮 PID 输出标志');
      field('ctMission',m.action_index===undefined?'尚无事件':`phase=${m.phase_code} / action=${m.action_index} / state=${m.status_code} · MOCK`);
      field('ctVision',v.event_type?`${v.event_type} / 颜色 ${numberOrDash(v.color_id)} / (${numeric(v.pixel_u,0)},${numeric(v.pixel_v,0)}) px`:'尚无视觉事件');
      const fb={144:'ACK≠完成',145:'REJECTED',146:'STARTED',147:'PROGRESS',148:'COMPLETED',149:'FAILED',150:'CONTROL_STATUS'};
      field('ctCommand',f.event_type===undefined?'尚无回执':`${fb[f.event_type]||f.event_type} / id=${f.cmd_id} / ${f.progress_percent}% · 合成`);
      field('ctMechanism',a.gripper_valid?`夹爪 ${numeric(a.gripper_angle_deg,1,'°')}`:'机构数据无有效标志');
      field('ctPower',p.driver_voltage_valid?`${numeric(p.driver_voltage_v,2,'V')} 驱动电压`:'未收到有效电压');
      for(const name of Object.keys(names)) {
        const record=t[name]||{};
        const el=$('ctTopic'+name);if(el)el.dataset.fresh=String(!!record.fresh);
        field('ctTopicData'+name,record.count>0?`${numeric(record.hz,1,' Hz')} · ${numeric(record.age_s,2,' s前')} · ${record.count} 帧`:'尚未接收');
      }
      if(packet.seq!==lastSeq){
        lastSeq=packet.seq;
        const events=(s.events||[]).slice(-12);
        field('ctEvents',events.length?events.map(x=>`#${x.id} [${x.kind}] ${x.text}`).join('\n'):'尚无事件');
        field('ctRaw',JSON.stringify(s,null,2));
      }
    } catch(e){$('ctStatus').dataset.online='false';field('ctStatus','TEST API 不可用');field('ctPipeline',String(e));}
    finally{fetching=false;}
  }
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',mount);
  else mount();
})();
