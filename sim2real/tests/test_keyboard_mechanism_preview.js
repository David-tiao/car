/* Mechanical MOCK acceptance, independent of ROS2, UART or browser hardware.
 * Firmware limits are for DEMONSTRATION targets, not actual travel safety.
 */
'use strict';
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const web=path.join(__dirname,'..','web');
const api=require(path.join(web,'keyboard_mechanism_preview.js'));
const near=(a,b,e=.00001)=>assert.ok(Math.abs(a-b)<e,String(a)+' != '+String(b));

assert.deepEqual(Object.keys(api.AXES),
  ['tray_deg','lift_mm','holder_deg','grip_deg']);
assert.deepEqual(api.PRESETS.tray_deg,[0,90,180,270]);
assert.deepEqual(api.PRESETS.lift_mm,[0,24,80,141,170,175]);
assert.deepEqual(api.PRESETS.holder_deg,[0,180]);
assert.deepEqual(api.PRESETS.grip_deg,[0,110,140]);

const model=api.fresh();
for(const name of Object.keys(api.AXES)){
  assert.equal(model.target[name],0);
  assert.equal(model.shown[name],0);
}
assert.equal(api.setTarget(model,'lift_mm',999),175);
assert.equal(api.setTarget(model,'holder_deg',-50),0);
assert.throws(()=>api.setTarget(model,'unknown',3),RangeError);
assert.throws(()=>api.setTarget(model,'tray_deg',NaN),RangeError);
assert.throws(()=>api.setTarget(model,'tray_deg','90'),RangeError);
assert.throws(()=>api.setTarget(model,'grip_deg',Infinity),RangeError);

// Animation is deterministic, rate-limited and capped to a 50ms RAF step.
// Page hidden for seconds may NOT cause the arm to teleport after wake.
api.reset(model);
api.setTarget(model,'tray_deg',270);
api.setTarget(model,'lift_mm',175);
api.setTarget(model,'holder_deg',180);
api.setTarget(model,'grip_deg',140);
api.advance(model,.05);
near(model.shown.tray_deg,3);
near(model.shown.lift_mm,3.25);
near(model.shown.holder_deg,6.3882);
near(model.shown.grip_deg,36);
api.advance(model,200);
near(model.shown.tray_deg,6); // one 50 ms step only
for(let i=0;i<1000;i++)api.advance(model,.05);
near(model.shown.tray_deg,270);
near(model.shown.lift_mm,175);
near(model.shown.holder_deg,180);
near(model.shown.grip_deg,140);
api.reset(model);
for(const name of Object.keys(api.AXES))near(model.shown[name],0);

const code=fs.readFileSync(path.join(web,'keyboard_mechanism_preview.js'),'utf8');
const panel=fs.readFileSync(path.join(web,'keyboard_mapping_panel.js'),'utf8');
const html=fs.readFileSync(path.join(web,'index.html'),'utf8');
const css=fs.readFileSync(path.join(web,'keyboard_mechanism.css'),'utf8');
assert.match(html,/keyboard_mechanism\.css\?v=1\.0\.0/);
assert.ok(html.indexOf('keyboard_mechanism_preview.js')<
          html.indexOf('keyboard_mapping_panel.js'),'load mechanism before mapping');
assert.ok(panel.includes('CarMechanismPreview.create(panel)'),'UI must mount mechanism controls');
assert.ok(panel.includes('mechanism.attachMap(car)'),'robot map overlay must be attached');
assert.ok(panel.includes('mechanism.tick(performance.now())'),'MOCK animation must advance in RAF');
assert.match(css, /keyboard-mechanism-controls/);
assert.match(css, /max-width:650px/);
for(const label of ['MOCK ONLY · STM32 TX OFF','PC7','PC6 / PC8 / PC9','PB15','PB14']){
  assert.ok(code.includes(label),'missing hardware/Mock descriptor: '+label);
}
// No I/O path in the newly introduced mechanism module, intentionally unlike
// existing WASDQE chassis posting. Mechanism sliders MUST NOT call HTTP.
assert.ok(!/\bfetch\s*\(|XMLHttpRequest|sendBeacon\s*\(|WebSocket\s*\(|create_publisher\s*\(|serial\.Serial\s*\(/.test(code));
assert.ok(!/\/api\/keyboard\/intent|\/cmd_vel|\/dev\/tty/.test(code));
console.log('PASS: mechanical CAD mapping, target bounds, animation rate and NO motor/network output');
