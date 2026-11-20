/* REAL page has a visibly isolated, opt-in MOCK STM32 PTY workbench.
 * It never writes to /api/real, /cmd_vel or an attached serial device. */
(() => {
  'use strict';
  if (window.CarWebMode?.mode !== 'real') return;
  const API = '/api/test/stm32_fsm_lab';
  function el(tag, parent, text) {
    const node = document.createElement(tag);
    if (text !== undefined) node.textContent = text;
    parent.appendChild(node);
    return node;
  }
  function mount() {
    const shell = el('section', document.body);
    const live = document.getElementById('livePanel') ||
                 document.getElementById('carModeSwitch');
    if (live) live.after(shell);
    shell.id = 'realMockStm32Workbench';
    shell.style.cssText = 'max-width:1500px;margin:12px auto;padding:18px;border:2px dashed #beae70;border-radius:12px;background:#162636;color:#e6eff4;font:14px/1.6 system-ui,sans-serif';
    el('h2', shell, '运行集 · 模拟 STM32 PTY 状态机实验');
    el('p', shell, '隔离实验区（MOCK / SIM_ONLY）。复用已有虚拟 STM32 CAR-LINK 匿名 PTY 端口，绝不是实际 STM32、底盘位置或机械反馈；上方 REAL 监控数据不会被修改。');
    const controls = el('div', shell);
    controls.style.cssText = 'display:flex;flex-wrap:wrap;align-items:end;gap:10px;margin:12px 0';
    function field(label, value) {
      const item = el('label', controls, label);
      item.style.cssText = 'display:flex;flex-direction:column;gap:5px;font-weight:600';
      const input = el('input', item);
      input.value = value;
      input.style.cssText = 'padding:8px;border:1px solid #7193a5;border-radius:6px;background:#0f293d;color:#fff';
      return input;
    }
    const code = field('二维码任务码', '156+123+516+231');
    code.maxLength = 15;
    const blocked = field('封锁道路（逗号分隔，可留空）', '');
    blocked.placeholder = '例如 B3-C3';
    const startLabel = el('label', controls, '出发区');
    const zone = el('select', startLabel);
    const faultLabel = el('label', controls, '故障注入');
    const fault = el('select', faultLabel);
    for (const [value,name] of [['upper_right','右上启停区'],['lower_right','右下启停区']]) {
      const option = el('option',zone,name);option.value=value;
    }
    for (const [value,name] of [['none','正常'],['failed_at_3','第 3 个命令 FAILED'],['missing_ack_at_2','第 2 个命令丢 ACK']]) {
      const option=el('option',fault,name);option.value=value;
    }
    for (const select of [zone,fault]) {
      select.style.cssText = 'display:block;padding:8px;background:#0f293d;color:#fff;border:1px solid #7193a5;border-radius:6px';
    }
    for (const label of [startLabel,faultLabel]) label.style.cssText = 'display:flex;flex-direction:column;gap:5px;font-weight:600';
    const buttons = el('div',shell);
    buttons.style.cssText = 'display:flex;flex-wrap:wrap;gap:8px;margin:10px 0';
    const btn = (title,action) => {
      const b=el('button',buttons,title);b.type='button';
      b.style.cssText='padding:8px 13px;border:1px solid #6fa4c4;border-radius:6px;background:#195273;color:#fff;cursor:pointer';
      b.dataset.action=action;
      return b;
    };
    const start = btn('初始化虚拟 STM32','start');
    const step = btn('执行一步','step');
    const run = btn('运行至 DONE/FAULT','run');
    const reset = btn('清除本次模拟','reset');
    el('p',shell,'安全约束：不会打开 /dev/tty*；不会发送真实运动命令；路网节点坐标仅是离散预览。');
    const status = el('p',shell,'模拟端尚未启动');
    status.style.fontWeight='bold';
    const progress=el('p',shell,'—');
    const events=el('pre',shell,'等待模拟反馈...');
    events.style.cssText='max-height:300px;overflow:auto;white-space:pre-wrap;word-break:break-word;background:#0c1d2d;padding:10px;border-radius:8px;font:12px/1.6 ui-monospace,monospace';
    let busy=false;
    const allButtons=[start,step,run,reset];
    function render(data) {
      if (data.source_type!=='MOCK' || data.mode!=='SIM_ONLY' ||
          data.hardware_authorized!==false ||
          data.transport!=='ANONYMOUS_PTY_CARLINK_TEST_ONLY' ||
          data.model_kind!=='PYTHON_SIMEXECUTOR_NOT_COMPILED_STM32') {
        throw Error('拒绝非 MOCK 的运行集实验数据');
      }
      const s=data.snapshot;
      if (!s) {
        status.textContent='MOCK · 尚未初始化匿名 PTY';
        progress.textContent='—';events.textContent='等待模拟反馈...';return;
      }
      if (s.source_type!=='MOCK' || s.hardware_authorized!==false ||
          s.pose_source!=='DISCRETE_ROAD_NODE') throw Error('非法模拟状态');
      status.textContent='MOCK / SIM_ONLY · '+s.state+
                         (s.error?' · '+s.error:'')+
                         ' · 虚拟 MCU 执行 '+s.mock_executions+' 次';
      progress.textContent='任务步骤 '+s.completed_steps+'/'+s.total_steps+
        ' · '+s.kind+' (#'+s.cmd_id+') · 位置预览 '+s.node+
        ' · 模拟抓取 '+s.pick_count+' / 放置 '+s.place_count+
        ' / 码垛 '+s.stack_count+' · 返程 '+(s.home_confirmed?'完成':'未完成');
      events.textContent=(s.events||[]).map(e =>
        '#'+e.cmd_id+' '+e.event+(e.reason?' / '+e.reason:'')
      ).join('\n') || '当前没有反馈事件';
    }
    async function fetchStatus() {
      const res=await fetch(API,{cache:'no-store'});
      if (!res.ok) throw Error('GET '+res.status);
      return res.json();
    }
    async function send(action) {
      if (busy) return;
      busy=true;for(const b of allButtons)b.disabled=true;
      try {
        const body=action==='start' ?
          {action,task_code:code.value.trim(),start_zone:zone.value,
           blocked_roads:blocked.value.split(',').map(x=>x.trim()).filter(Boolean),
           fault:fault.value} : {action};
        const res=await fetch(API,{method:'POST',
          headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
        const data=await res.json();
        if (!res.ok) throw Error(data.error||('HTTP '+res.status));
        render(data);
      } catch (err) {
        status.textContent='MOCK 实验失败：'+String(err);
      } finally {busy=false;for(const b of allButtons)b.disabled=false;}
    }
    for(const b of allButtons)b.addEventListener('click',()=>send(b.dataset.action));
    fetchStatus().then(render).catch(e=>{
      status.textContent='MOCK 状态接口不可用：'+String(e);
    });
  }
  if (document.readyState==='loading') document.addEventListener('DOMContentLoaded',mount);
  else mount();
})();
