/* Read-only ROS2 keyboard velocity feedback. No key events or control publisher. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const canvas = $('view');
  const context = canvas.getContext('2d');
  const trail = [];
  let newest = null;
  let lastSequence = -1;
  let connected = false;
  let awaiting = false;
  let lastWidth = 0, lastHeight = 0;
  const scale = 76; // visual pixels per meter; not a movement constraint

  function point(x, y, pose, width, height) {
    return [width / 2 + (x - pose.x) * scale, height / 2 - (y - pose.y) * scale];
  }

  function draw() {
    const dpr = window.devicePixelRatio || 1;
    const width = canvas.clientWidth, height = canvas.clientHeight;
    if (width < 1 || height < 1) return;
    if (width !== lastWidth || height !== lastHeight || canvas.width !== Math.round(width * dpr)) {
      canvas.width = Math.round(width * dpr);
      canvas.height = Math.round(height * dpr);
      lastWidth = width; lastHeight = height;
    }
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    context.fillStyle = '#081c2c';
    context.fillRect(0, 0, width, height);
    const pose = newest || {x:0, y:0, yaw:0};
    const left = pose.x - width/(2*scale), right = pose.x + width/(2*scale);
    const bottom = pose.y - height/(2*scale), top = pose.y + height/(2*scale);

    context.lineWidth = 1;
    for (let x = Math.floor(left); x <= Math.ceil(right); x++) {
      const [sx] = point(x, 0, pose, width, height);
      context.strokeStyle = x === 0 ? '#3b647d' : '#254257';
      context.beginPath(); context.moveTo(sx, 0); context.lineTo(sx, height); context.stroke();
    }
    for (let y = Math.floor(bottom); y <= Math.ceil(top); y++) {
      const [,sy] = point(0, y, pose, width, height);
      context.strokeStyle = y === 0 ? '#3b647d' : '#254257';
      context.beginPath(); context.moveTo(0, sy); context.lineTo(width, sy); context.stroke();
    }
    if (trail.length > 1) {
      context.strokeStyle = '#70d0fb';
      context.lineWidth = 2.5;
      context.beginPath();
      trail.forEach((pt,i) => {
        const [sx,sy] = point(pt.x, pt.y, pose, width, height);
        if (i === 0) context.moveTo(sx,sy); else context.lineTo(sx,sy);
      });
      context.stroke();
    }
    context.save();
    context.translate(width/2, height/2);
    context.rotate(-pose.yaw);
    context.fillStyle = '#51d4bb';
    context.strokeStyle = '#d2fff3';
    context.lineWidth = 2;
    context.fillRect(-22,-15,44,30);
    context.strokeRect(-22,-15,44,30);
    context.fillStyle = '#fff';
    context.beginPath();
    context.moveTo(28,0); context.lineTo(12,-9); context.lineTo(12,9); context.closePath();
    context.fill();
    context.restore();
    context.fillStyle = '#aec4d5';
    context.font = '12px system-ui,sans-serif';
    context.fillText('X →   Y ↑   (箭头为车头方向)', 14, height-16);
    if (!connected) {
      context.fillStyle = '#ffcaa3';
      context.font = '14px system-ui,sans-serif';
      context.fillText('等待 ROS2 键盘反馈', 14, 60);
    }
  }

  function number(id, value) { $(id).textContent = Number(value || 0).toFixed(3); }

  async function poll() {
    if (awaiting) return;
    awaiting = true;
    try {
      const response = await fetch('/api/keyboard/feedback', {cache:'no-store'});
      if (!response.ok) throw new Error('HTTP '+response.status);
      const data = await response.json();
      connected = Boolean(data.online);
      $('link').dataset.online = String(connected);
      $('link').textContent = !connected ? '等待 ROS2' : data.command_recent ? '指令在线' : '桥接在线 · 无新指令';
      if (connected && data.sample) {
        const s = data.sample;
        newest = s;
        $('topic').textContent = s.topic;
        number('vx', s.vx); number('vy', s.vy); number('wz', s.wz);
        $('pos').textContent = 'x '+s.x.toFixed(3)+' m / y '+s.y.toFixed(3)+' m / θ '+(s.yaw*180/Math.PI).toFixed(1)+'°';
        if (data.seq !== lastSequence) {
          lastSequence = data.seq;
          if (!trail.length || Math.hypot(s.x-trail[trail.length-1].x,s.y-trail[trail.length-1].y) > .0001) {
            trail.push({x:s.x, y:s.y});
            if (trail.length > 10000) trail.shift(); // history cap, not spatial bound
          }
        }
      } else {
        number('vx',0); number('vy',0); number('wz',0);
      }
    } catch(error) {
      connected = false;
      $('link').dataset.online = 'false';
      $('link').textContent = '网页接口离线';
      $('hint').textContent = '无法读取反馈 API：'+String(error.message || error);
      number('vx',0); number('vy',0); number('wz',0);
    } finally {
      awaiting = false;
      draw();
    }
  }
  setInterval(poll,100);
  window.addEventListener('resize',draw);
  poll();
})();
