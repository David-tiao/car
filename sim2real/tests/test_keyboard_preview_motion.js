/* Hardware-free ROS2 MOCK preview regression: smooth 60Hz jitter protection.
 * /dryrun remains the only authoritative pose; no motors or actual /odom.
 */
'use strict';
const assert=require('node:assert/strict');
const motion=require('../web/keyboard_preview_motion.js');
const near=(a,b,eps=1e-7)=>assert.ok(Math.abs(a-b)<=eps,String(a)+' != '+String(b));
const wrap=x=>Math.atan2(Math.sin(x),Math.cos(x));
function snapshot(overrides={}) {
  return {
    source_type:'MOCK_KINEMATIC_ONLY',hardware_authorized:false,
    real_odom:false,frame:'field_preview_m_not_odom',preview_mode:'direct',
    input_provenance:'OBSERVED_PC_ROS2_LOCAL_TEST_TWIST',state:'RUNNING',
    generation:1,samples:1,sim_time_s:0,
    x_m:2.16,y_m:2.16,yaw_rad:0,
    velocity:{vx:.2,vy:0,wz:0},
    ...overrides
  };
}
// Unmodified geometric extrapolation and physical safety boundaries.
let s=snapshot();
let p=motion.predict(s,50);
near(p.x_m,2.17);near(p.y_m,2.16);near(s.x_m,2.16);
near(motion.predict(s,120).x_m,2.184);
near(motion.predict(s,2000).x_m,2.184);
near(motion.predict(s,NaN).x_m,2.16);
s=snapshot({x_m:0,y_m:0,yaw_rad:Math.PI/2,velocity:{vx:.1,vy:0,wz:0}});
near(motion.predict(s,100).y_m,.01);
s=snapshot({x_m:0,y_m:0,yaw_rad:0,velocity:{vx:0,vy:.1,wz:.4}});
p=motion.predict(s,100);
near(p.x_m,-Math.sin(.02)*.01,1e-6);
near(p.y_m,Math.cos(.02)*.01,1e-6);
near(p.yaw_rad,.04);
s=snapshot({state:'STOPPED'});near(motion.predict(s,120).x_m,s.x_m);
s=snapshot({input_provenance:'ZERO_NO_FRESH_LOCAL_ROS2_OR_DEADMAN'});
near(motion.predict(s,120).x_m,s.x_m);
assert.throws(()=>motion.predict(snapshot({source_type:'REAL'}),50));
assert.throws(()=>motion.predict(snapshot({velocity:{vx:NaN,vy:0,wz:0}}),50));
s=snapshot({x_m:5000,y_m:-5000});
near(motion.predict(s,120).x_m,5000.024);

// Key regression: 75ms HTTP arrivals update target, never reset draw pose.
const tracker=motion.createTracker();
assert.equal(tracker.frame(0),null);
tracker.ingest(snapshot({x_m:0,y_m:0,yaw_rad:0}),0);
let last=tracker.frame(0).x_m;
let measured=0;
for(let t=16;t<=960;t+=16){
  if([80,160,256,336,448,560,720,848].includes(t)){
    measured=.2*t/1000;
    const before=tracker.frame(t);
    tracker.ingest(snapshot({x_m:measured,y_m:0,yaw_rad:0,
      samples:t,sim_time_s:t/1000}),t);
    const after=tracker.frame(t);
    near(after.x_m,before.x_m,1e-10);
  }
  const frame=tracker.frame(t);
  assert.ok(frame.x_m>=last-1e-9,'visual must not bounce backwards on W hold');
  assert.ok(frame.x_m-last<.009,'large frame jump at '+t+'ms');
  last=frame.x_m;
}
assert.ok(last>.09 && last<.21,'motion actually progressed');

// Angle wrapping is 2 degrees rather than 358 degrees.
const yaw=motion.createTracker();
yaw.reset(snapshot({yaw_rad:179*Math.PI/180,x_m:0,y_m:0,velocity:{vx:0,vy:0,wz:0},
  state:'STOPPED'}),0);
yaw.ingest(snapshot({yaw_rad:-179*Math.PI/180,x_m:0,y_m:0,
  velocity:{vx:0,vy:0,wz:0},state:'STOPPED',samples:2,sim_time_s:.08}),80);
const angleAt=yaw.frame(96).yaw_rad;
assert.ok(Math.abs(wrap(angleAt-179*Math.PI/180))<.04);

// Late out-of-order packets cannot rewind motion.
const stable=tracker.frame(960);
assert.equal(tracker.ingest(snapshot({samples:2,sim_time_s:.01,x_m:-10}),980),false);
assert.ok(tracker.frame(980).x_m>=stable.x_m);
assert.equal(tracker.hasSource(),true);

// ROS2 outage: after 120ms max prediction, do not keep moving forever.
const limit=motion.predict(snapshot({x_m:measured,y_m:0}),120).x_m;
let final=0;
for(let t=1000;t<=4000;t+=16)final=tracker.frame(t).x_m;
assert.ok(final<=limit+.001,'offline motion must not continue indefinitely');
near(tracker.frame(5000).x_m,final,.000001);

// Key release freezes immediately; no fabricated ROS2 acknowledgements.
const frozen=tracker.freeze(5016);
assert.equal(tracker.hasSource(),false);
for(let t=5032;t<5600;t+=16)near(tracker.frame(t).x_m,frozen.x_m);

// Explicit reset generation repositions at the authoritative start.
const rearm=snapshot({generation:2,samples:0,sim_time_s:0,
  x_m:2.16,y_m:.24,state:'STOPPED'});
tracker.ingest(rearm,6000);
near(tracker.frame(6000).x_m,2.16);near(tracker.frame(6000).y_m,.24);
tracker.clear();assert.equal(tracker.frame(6016),null);

// Ensure the live panel never uses raw HTTP packets to set SVG position.
const fs=require('node:fs');
const path=require('node:path');
const panel=fs.readFileSync(path.join(__dirname,'..','web','keyboard_mapping_panel.js'),'utf8');
assert.match(panel,/CarKeyboardPreviewMotion\.createTracker\(\)/);
assert.match(panel,/motionTracker\.ingest\(data,performance\.now\(\)\)/);
assert.match(panel,/motionTracker\.frame\(frameNow\)/);
assert.match(panel,/motionTracker\.freeze\(performance\.now\(\)\)/);
assert.ok(!panel.includes('renderMockPose(data);'));
console.log('PASS: ROS2 60fps smooth tracking with jittered packets, yaw, STOP, stale and outage safety');
