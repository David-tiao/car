/* TEST-only visual experiment for V4 OpenMV ↔ STM32 QR handshake.
 * The server runs a deterministic protocol contract model, NOT actual C
 * firmware. No /api/real, serial, cmd_vel or hardware changes. */
(() => {
  'use strict';
  if (window.CarWebMode?.mode !== 'test') return;
  const endpoint = '/api/test/v4_qr_lab';
  const scenarios = [
    ['success', '正常扫码 → DONE'],
    ['duplicate_seq', '重复二维码帧 → 只接收一次'],
    ['qr_ack_lost', 'ACK 丢失 → QR 重发、补 ACK'],
    ['stale_session', '旧 session → 拒绝'],
    ['bad_xor', '坏 XOR 帧 → 拒绝'],
    ['invalid_task', '错误任务码 → 拒绝'],
    ['missing_job_ack', 'JOB ACK 丢失 → FAULT'],
    ['missing_qr', '二维码超时 → FAULT'],
    ['cancel', '主动取消 → CANCELLED'],
  ];
  function el(tag, parent, text) {
    const node = document.createElement(tag);
    if (text !== undefined) node.textContent = text;
    parent.appendChild(node);
    return node;
  }
  function mount() {
    const after = document.getElementById('carTestMissionReplay') ||
                  document.getElementById('carTestPanel');
    const panel = el('section', document.body);
    if (after) after.after(panel);
    panel.id = 'carV4QrLab';
    panel.style.cssText = 'max-width:1450px;margin:14px auto;padding:18px;border:1px solid #47617b;border-radius:12px;background:#112638;color:#e3f3fc;font:14px/1.6 system-ui,sans-serif';
    el('h2', panel, 'TEST · OpenMV V4 ↔ STM32 二维码协议状态机');
    el('p', panel, '仅运行独立 Python V4 协议契约模型，展示帧、XOR、session、状态变化；不是烧录的 STM32 C 固件，不代表实车成功。');
    const select = el('select', panel);
    select.setAttribute('aria-label', '选择协议故障实验');
    select.style.cssText = 'padding:8px;margin-right:12px;background:#193a50;color:#fff;border:1px solid #6483a0;border-radius:6px';
    for (const [value, label] of scenarios) {
      const option = el('option', select, label);
      option.value = value;
    }
    const run = el('button', panel, '运行协议实验');
    run.type = 'button';
    run.style.cssText = 'padding:8px 16px;background:#2474a8;color:white;border:0;border-radius:6px;cursor:pointer;margin-right:9px';
    const step = el('button', panel, '下一帧');
    step.type = 'button'; step.disabled = true;
    step.style.cssText = run.style.cssText;
    const badge = el('p', panel, '尚未运行 V4 QR 实验');
    badge.style.fontWeight = 'bold';
    const detail = el('p', panel, '所有实验硬件授权：false');
    const log = el('pre', panel, '选择场景后点击“运行协议实验”');
    log.style.cssText = 'max-height:340px;overflow:auto;white-space:pre-wrap;word-break:break-all;font:12px/1.6 ui-monospace,monospace';
    let report = null, index = 0, busy = false;
    function draw() {
      if (!report) return;
      const total = report.events.length;
      const visible = report.events.slice(0, index);
      badge.textContent = '场景：'+report.label+' · '+(report.passed?'预期 PASS':'异常')+
                          ' · 当前帧 '+index+'/'+total;
      const current = visible.length ? visible[visible.length - 1].state : 'IDLE';
      detail.textContent = '当前状态 '+current+' · 最终状态 '+report.state+
          ' · session '+report.session+' · 接受 QR 数 '+report.accepted_qr+
          ' · 源 CONTRACT_MODEL / MOCK（无实体硬件）';
      log.textContent = visible.map((e, n) =>
        String(n+1).padStart(2,'0')+' '+e.direction+' '+e.body+
        '\n    '+(e.wire||'(状态变化)')+
        '\n    '+e.state+' / '+(e.accepted?'接受':'拒绝')+
        (e.note?' · '+e.note:'')).join('\n') || '等待下一帧';
      step.disabled = index >= total;
    }
    run.addEventListener('click', async () => {
      if (busy) return;
      busy = true; run.disabled = true; step.disabled = true;
      try {
        const res = await fetch(endpoint, {
          method:'POST', headers:{'Content-Type':'application/json'},
          body:JSON.stringify({scenario:select.value}), cache:'no-store'
        });
        if (!res.ok) throw new Error('HTTP '+res.status);
        const incoming = await res.json();
        if (incoming.source_type !== 'MOCK' ||
            incoming.mode !== 'SIM_ONLY' ||
            incoming.hardware_authorized !== false ||
            incoming.model_kind !== 'CONTRACT_MODEL_NOT_STM32_FIRMWARE' ||
            !Array.isArray(incoming.events)) throw new Error('拒绝不可信测试数据');
        report = incoming;
        index = 0;
        draw();
      } catch (err) {
        badge.textContent = 'TEST 协议实验失败：'+String(err);
        log.textContent = '检查网页进程是否为更新后的 server.py';
      } finally { busy = false; run.disabled = false; }
    });
    step.addEventListener('click', () => {
      if (!report || index >= report.events.length) return;
      index++;
      draw();
    });
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount);
  else mount();
})();
