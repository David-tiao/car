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
console.log('PASS: 60fps ROS2 preview stays geometrically correct, bounded in time, unbounded in space');
