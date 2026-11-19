/* Read-only, TEST-only mission replay view. Never displays mock road nodes as odometry. */
(() => {
  'use strict';
  if (window.CarWebMode?.mode !== 'test') return;
  const el = (tag, parent, text) => {
    const node = document.createElement(tag);
    if (text !== undefined) node.textContent = text;
    parent.appendChild(node);
    return node;
  };
  function mount() {
    const panel = el('section', document.body);
    const testPanel = document.getElementById('carTestPanel');
    if (testPanel) testPanel.after(panel);
    panel.id = 'carTestMissionReplay';
    panel.style.cssText = 'max-width:1450px;margin:14px auto;padding:18px;border:1px solid #47617b;border-radius:12px;background:#112638;color:#e3f3fc;font:14px/1.6 system-ui,sans-serif';
    el('h2', panel, 'TEST · 比赛任务状态机与虚拟串口回执');
    el('p', panel, '以下全部来自 MOCK_ONLY 匿名 PTY；节点位置只是路网离散预览，不是 /odom，也不代表真实抓放成功。');
    const status = el('p', panel, '等待 /car/test/mission/state');
    status.style.fontWeight = 'bold';
    const progress = el('p', panel, '—');
    const location = el('p', panel, '—');
    const counters = el('p', panel, '—');
    const log = el('pre', panel, '等待事件');
    log.style.cssText = 'max-height:210px;overflow:auto;white-space:pre-wrap;font:12px/1.5 ui-monospace,monospace';
    let busy = false;
    async function refresh() {
      if (busy) return;
      busy = true;
      try {
        const response = await fetch('/api/test/mission_replay', {cache:'no-store'});
        if (!response.ok) throw new Error('HTTP '+response.status);
        const data = await response.json();
        if (data.source_type !== 'MOCK' || !data.online || !data.snapshot) {
          status.textContent = 'MOCK 回放离线（不会填充旧数据）';
          progress.textContent = '—'; location.textContent='—'; counters.textContent='—'; log.textContent='无实时事件';
          return;
        }
        const s = data.snapshot;
        if (s.hardware_authorized !== false || s.mode !== 'SIM_ONLY' || s.pose_source !== 'DISCRETE_ROAD_NODE') {
          throw new Error('拒绝非模拟任务数据');
        }
        status.textContent = 'MOCK · '+s.state+(s.error ? ' · '+s.error : '');
        progress.textContent = '步骤 '+s.completed_steps+'/'+s.total_steps+' · 当前 #'+s.cmd_id+' '+s.kind+' · '+s.event;
        location.textContent = '预览路网节点 '+s.node+' ('+s.x_m+', '+s.y_m+' m)，非真实定位';
        counters.textContent = '模拟抓取 '+s.pick_count+' / 平面及码垛放置 '+s.place_count+
                               ' / 码垛 '+s.stack_count+' · 返程 '+(s.home_confirmed?'完成':'未完成');
        log.textContent = (s.events||[]).slice(-12).map(x => '#'+x.cmd_id+' '+x.event+
                          (x.reason ? ' '+x.reason : '')).join('\n') || '尚无事件';
      } catch (err) {
        status.textContent = 'TEST 任务回放不可用：'+String(err);
      } finally {busy=false;}
    }
    refresh();
    setInterval(refresh, 600);
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount);
  else mount();
})();
