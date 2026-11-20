/* Four fixed web entry points, sharing the original Sim2Real V13 dashboard.
   This script never sends motor commands.  Pages are intentionally isolated. */
(() => {
  'use strict';
  const known = ['sim', 'test', 'real', 'keyboard'];
  const requested = location.pathname.replace(/^\/+|\/+$/g, '');
  const mode = known.includes(requested) ? requested : 'sim';
  const channel = mode === 'keyboard' ? 'real' : mode;
  const descriptions = {
    sim: ['纯模拟集', 'SIM', '网页本地运动仿真；不使用 ROS 2 或实体底盘反馈'],
    test: ['测试集', 'TEST', '模拟 MCU → 伪串口 → CAR-LINK → ROS 2 → 网页'],
    real: ['运行集', 'REAL', '真实 STM32 → ROS 2 → 网页；仅监控，不发控制指令'],
    keyboard: ['映射反馈', 'READ ONLY', '读取电脑 ROS2 速度指令并实时估算运动；不向实车发指令'],
  };
  const config = Object.freeze({
    mode, channel,
    runSource: mode === 'sim' ? 'simulation' : 'live',
    telemetryUrl: mode === 'sim' ? null : '/api/' + channel + '/telemetry',
    rosUrl: mode === 'sim' ? null : '/api/' + channel + '/ros',
  });
  window.CarWebMode = config;

  function mount() {
    const sheet = document.createElement('style');
    sheet.textContent = `
      #carModeSwitch {margin:14px auto 18px;max-width:1500px;padding:12px 18px;border:1px solid #35516b;
        background:#10263a;color:#ecf7ff;border-radius:14px;font:14px/1.5 system-ui,sans-serif;}
      #carModeSwitch .car-mode-title {display:flex;gap:10px;align-items:center;justify-content:space-between;flex-wrap:wrap;margin-bottom:10px;}
      #carModeSwitch .car-mode-links {display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;}
      #carModeSwitch a {color:#d7e7fa;text-decoration:none;display:block;border:1px solid #466079;border-radius:9px;
        padding:10px;min-width:0;background:#193148;}
      #carModeSwitch a:hover, #carModeSwitch a:focus {border-color:#63d5e8;outline-offset:3px;}
      #carModeSwitch a[aria-current=page] {background:#144e5a;border-color:#65e5d1;color:white;}
      #carModeSwitch a strong,#carModeSwitch a small {display:block;}
      #carModeSwitch a small {font-size:11px;opacity:.82;margin-top:3px;}
      #carModeSwitch .car-mode-caption {margin:8px 0 0;font-size:12px;color:#b6d0df;}
      body .run-mode-bar {display:none!important;}
      @media(max-width:700px){#carModeSwitch .car-mode-links{grid-template-columns:repeat(2,minmax(0,1fr))}}
    `;
    document.head.append(sheet);
    const nav = document.createElement('nav');
    nav.id = 'carModeSwitch';
    nav.setAttribute('aria-label', '四套运行模式');
    const top = document.createElement('div');
    top.className = 'car-mode-title';
    const title = document.createElement('strong');
    title.textContent = '工创智能搬运 · 四模式控制台';
    const status = document.createElement('span');
    status.id = 'carModeDataStatus';
    status.textContent = descriptions[mode][1] + ' · 等待检查';
    top.append(title, status);
    const links = document.createElement('div');
    links.className = 'car-mode-links';
    for (const key of known) {
      const a = document.createElement('a');
      a.href = '/' + key;
      a.setAttribute('aria-label', descriptions[key][0]);
      if (mode === key) a.setAttribute('aria-current', 'page');
      const strong = document.createElement('strong');
      strong.textContent = descriptions[key][0];
      const small = document.createElement('small');
      small.textContent = descriptions[key][1];
      a.append(strong, small);
      links.append(a);
    }
    const note = document.createElement('p');
    note.className = 'car-mode-caption';
    note.textContent = descriptions[mode][2];
    nav.append(top, links, note);
    if (mode !== 'sim') {
      for (const element of document.querySelectorAll('#manualHub button, #manualHub input')) {
        element.disabled = true;
      }
    }
    const insertion = document.querySelector('.release-banner') || document.querySelector('main');
    if (insertion) insertion.before(nav);
    else document.body.prepend(nav);

    if (mode === 'keyboard') {
      const tip = document.createElement('p');
      tip.style.cssText = 'margin:6px 0;color:#ffc39c;font-weight:700';
      tip.textContent = '实车键盘控制当前禁用：尚无经过验证的 STM32 双向控制权、急停和命令反馈协议。';
      nav.append(tip);
    }

    async function refresh() {
      if (mode === 'sim') {
        status.textContent = 'SIM · 浏览器本地模拟（不依赖 ROS 2）';
        return;
      }
      try {
        const response = await fetch('/api/mode/status', {cache: 'no-store'});
        if (!response.ok) throw Error('HTTP ' + response.status);
        const data = await response.json();
        const feed = data.channels?.[channel];
        const connected = !!feed?.telemetry_online;
        status.textContent = (mode === 'test' ? 'TEST · ' : mode === 'real' ? 'REAL · ' : 'KEYBOARD · ') +
          (connected ? '收到对应通道里程计' : '对应通道离线 / 等待数据') +
          (mode === 'keyboard' ? ' · 控制禁用' : '');
      } catch (_) {
        status.textContent = descriptions[mode][1] + ' · 接口不可用';
      }
    }
    refresh();
    setInterval(refresh, 1200);
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount);
  else mount();
})();
