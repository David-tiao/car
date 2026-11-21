/* Hardware-free numerical regression for 60fps ROS2 echo rendering. */
'use strict';
const assert = require('node:assert/strict');
const motion = require('../web/keyboard_preview_motion.js');

function snapshot(overrides = {}) {
  return {
    source_type:'MOCK_KINEMATIC_ONLY', hardware_authorized:false,
    real_odom:false, frame:'field_preview_m_not_odom', preview_mode:'direct',
    input_provenance:'OBSERVED_PC_ROS2_LOCAL_TEST_TWIST', state:'RUNNING',
    x_m:2.16,y_m:2.16,yaw_rad:0,
    velocity:{vx:0.2,vy:0,wz:0},
    ...overrides
  };
}
function near(actual, expected, eps=1e-7) {
  assert.ok(Math.abs(actual-expected) <= eps, actual + ' differs from ' + expected);
}

// Rendering is geometry only. ROS2/HTTP snapshots remain authoritative.
let s=snapshot();
let a=motion.predict(s, 50);
near(a.x_m,2.17);
near(a.y_m,2.16);
near(s.x_m,2.16); // Immutable snapshot.
let b=motion.predict(s, 120);
near(b.x_m,2.184);
near(motion.predict(s,2000).x_m,b.x_m); // Never keep moving forever on loss.
near(motion.predict(s,-100).x_m,2.16);
near(motion.predict(s,Number.NaN).x_m,2.16);

// Clockwise/CCW body movement, lateral movement and rotation use midpoint body kinematics.
s=snapshot({x_m:0,y_m:0,yaw_rad:Math.PI/2,velocity:{vx:0.1,vy:0,wz:0}});
a=motion.predict(s,100);
near(a.x_m,0);
near(a.y_m,0.01);
s=snapshot({x_m:0,y_m:0,yaw_rad:0,velocity:{vx:0,vy:0.1,wz:0.4}});
a=motion.predict(s,100);
near(a.x_m,-Math.sin(.02)*.01,1e-6);
near(a.y_m,Math.cos(.02)*.01,1e-6);
near(a.yaw_rad,.04);

// Stopped/unobserved messages cannot predict motion from an HTTP command.
s=snapshot({state:'STOPPED'});
near(motion.predict(s,120).x_m,s.x_m);
s=snapshot({input_provenance:'ZERO_NO_FRESH_LOCAL_ROS2_OR_DEADMAN'});
near(motion.predict(s,120).x_m,s.x_m);

// Defensive schema requires MOCK not REAL and remains unbounded.
assert.throws(()=>motion.predict(snapshot({source_type:'REAL'}),50));
assert.throws(()=>motion.predict(snapshot({velocity:{vx:NaN,vy:0,wz:0}}),50));
s=snapshot({x_m:5000,y_m:-5000});
near(motion.predict(s,120).x_m,5000.024);

// Display smoothing NEVER owns HTTP/ROS2 state. It only blends two finite
// poses and leaves incoming server data and keyboard control unchanged.
let display=null, incoming={x_m:0,y_m:0,yaw_rad:0};
display=motion.smoothPose(display,incoming,16);
near(display.x_m,0);
let previous=display.x_m;
for(let t=16;t<=960;t+=16){
  // Irregular 75-110ms server packets; no snap-back at packet boundaries.
  if([80,160,256,336,448,560,720,848].includes(t)) {
    incoming={x_m:.2*t/1000,y_m:0,yaw_rad:0};
  }
  const before={...display};
  display=motion.smoothPose(display,incoming,16);
  assert.ok(display.x_m>=before.x_m-1e-9,'smooth motion must not bounce backwards');
  assert.ok(display.x_m-before.x_m<.009,'max frame jump should remain sub-centimeter');
  previous=display.x_m;
}
assert.ok(previous>.1&&previous<.21,'smooth preview must keep moving while W held');
// Shortest-arc angle smoothing (179deg -> -179deg, not full spin).
const toRad=n=>n*Math.PI/180;
const start={x_m:0,y_m:0,yaw_rad:toRad(179)};
const end={x_m:0,y_m:0,yaw_rad:toRad(-179)};
const wrap=n=>Math.atan2(Math.sin(n),Math.cos(n));
const eased=motion.smoothPose(start,end,16);
assert.ok(Math.abs(wrap(eased.yaw_rad-start.yaw_rad))<toRad(2));
assert.ok(Math.abs(wrap(eased.yaw_rad-start.yaw_rad))>0);
// Paused/hidden tab does not take a giant single-frame leap.
const long=motion.smoothPose({x_m:0,y_m:0,yaw_rad:0},
  {x_m:10,y_m:0,yaw_rad:0},5000);
assert.ok(long.x_m>0&&long.x_m<10);
// Bad data cannot make the map display NaN.
assert.throws(()=>motion.smoothPose(null,{x_m:NaN,y_m:0,yaw_rad:0},16));

// Ensure key commands remain on the original known-working path; the
// rendering regression must not rewrite deadman/HTTP handling again.
const fs=require('node:fs'),path=require('node:path');
const ui=fs.readFileSync(path.join(__dirname,'../web/keyboard_mapping_panel.js'),'utf8');
assert.ok(ui.includes('renderMockPose(data);'),'HTTP must keep direct rendering fallback');
assert.ok(ui.includes('renderMockPose(window.CarKeyboardPreviewMotion.predict('));
assert.ok(ui.includes("active.add(key);refreshKeys();sendNow();"));
assert.ok(ui.includes("active.delete(key);refreshKeys();sendNow();"));
assert.ok(!ui.includes('createTracker()'),'known-working ROS2 input path must be retained');

console.log('PASS: 60fps ROS2 preview stays geometrically correct, bounded in time, unbounded in space');
