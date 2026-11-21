/* Regression: smooth browser-only mechanism and identical CAD profile in all modes.
 * NO ROS2, physical mechanism data, STM32 or browser hardware required.
 */
'use strict';
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const web=path.join(__dirname,'..','web');
const api=require(path.join(web,'keyboard_mechanism_preview.js'));
const nearly=(a,b,e=.0001)=>assert.ok(Math.abs(a-b)<e,String(a)+' != '+String(b));
assert.deepEqual(Object.keys(api.AXES),
  ['base_yaw_deg','tray_deg','lift_mm','holder_deg','grip_deg']);
assert.deepEqual(api.PRESETS.base_yaw_deg,[-90,0,90]);
assert.equal(api.AXES.base_yaw_deg.firmwareRate,null);
assert.equal(api.AXES.base_yaw_deg.mockOnly,true);
assert.deepEqual(api.PRESETS.tray_deg,[0,90,180,270]);
assert.deepEqual(api.PRESETS.lift_mm,[0,24,80,141,170,175]);
assert.deepEqual(api.PRESETS.holder_deg,[0,180]);
assert.deepEqual(api.PRESETS.grip_deg,[0,110,140]);
assert.equal(api.AXES.tray_deg.firmwareRate,60);
assert.equal(api.AXES.holder_deg.firmwareRate,127.764);
assert.equal(api.AXES.grip_deg.firmwareRate,720);
assert.ok(api.AXES.grip_deg.previewRate<api.AXES.grip_deg.firmwareRate);
assert.equal(api.AXES.lift_mm.firmwareRate,null); // no invented firmware mm/s

const model=api.fresh();
for(const n of Object.keys(api.AXES)){
  nearly(model.target[n],0);nearly(model.shown[n],0);nearly(model.velocity[n],0);
}
assert.equal(api.setTarget(model,'lift_mm',999),175);
assert.equal(api.setTarget(model,'base_yaw_deg',-999),-90);
assert.equal(api.setTarget(model,'base_yaw_deg',999),90);
assert.throws(()=>api.setTarget(model,'base_yaw_deg',NaN),RangeError);
assert.equal(api.setTarget(model,'holder_deg',-30),0);
assert.throws(()=>api.setTarget(model,'unknown',3),RangeError);
assert.throws(()=>api.setTarget(model,'tray_deg',NaN),RangeError);
assert.throws(()=>api.setTarget(model,'tray_deg','90'),RangeError);
assert.throws(()=>api.setTarget(model,'grip_deg',Infinity),RangeError);

// Smooth starts: first step cannot jump instantly to a constant-speed move.
api.reset(model);
for(const name of Object.keys(api.AXES))
  api.setTarget(model,name,api.AXES[name].max);
api.advance(model,.05);
const first={...model.shown};
for(const name of Object.keys(api.AXES)){
  const a=api.AXES[name];
  assert.ok(first[name]>0);
  assert.ok(first[name]<a.previewRate*.05);
  assert.ok(Math.abs(model.velocity[name])<=a.previewAccel*.05+1e-7);
}
api.advance(model,.05);
for(const name of Object.keys(api.AXES))
  assert.ok(model.shown[name]>first[name]);

// 200s of missed frames does not teleport: at most one 150ms visual step.
const before=model.shown.lift_mm;
api.advance(model,200);
assert.ok(model.shown.lift_mm-before<=api.AXES.lift_mm.previewRate*.15+1e-7);

// The same sequence of small steps converges to the exact target with zero speed.
for(let i=0;i<1200;i++)api.advance(model,.05);
for(const name of Object.keys(api.AXES)){
  nearly(model.shown[name],api.AXES[name].max);
  nearly(model.velocity[name],0);
}
// Retarget mid-motion must brake/reverse without exceeding axis bounds.
api.setTarget(model,'tray_deg',0);
for(let i=0;i<300;i++){
  api.advance(model,.05);
  assert.ok(model.shown.tray_deg>=0 && model.shown.tray_deg<=270);
}
nearly(model.shown.tray_deg,0);
nearly(model.velocity.tray_deg,0);
api.reset(model);
for(const name of Object.keys(api.AXES))nearly(model.shown[name],0);

// The SAME computed pose feeds detail + map; top geometry builder is shared.
const p=api.computeMechanismPose({
  base_yaw_deg:-60,tray_deg:90,lift_mm:175,holder_deg:180,grip_deg:140
});
nearly(p.baseRotate,-60);
nearly(p.trayRotate,90);
nearly(p.absoluteTrayRotate,30);
nearly(p.liftOffset,-104);
nearly(p.holderRotate,-180);
nearly(p.jawOffset,13);
nearly(p.sideJawScale,.65);
assert.ok(Object.isFrozen(p));
// PC7 is the SAME yaw degree of freedom in top and side projections.
// The side view used to be hard-coded bowls and never changed on rotation.
const side0=api.projectTraySide(0);
const side90=api.projectTraySide(90);
const side180=api.projectTraySide(180);
assert.equal(side0.length,3);
assert.ok(side0.every(st=>Object.isFrozen(st)));
nearly(side0[0].x,0);
nearly(side90[0].x,31);
nearly(side180[0].x,0);
assert.ok(side90.some((st,i)=>Math.abs(st.x-side0[i].x)>10));
for(const angle of [0,45,90,180,270]){
  const positions=api.projectTraySide(angle);
  assert.equal(positions.length,3);
  for(const st of positions){
    assert.ok(Number.isFinite(st.x)&&Number.isFinite(st.y));
    assert.ok(st.x>=-31.001&&st.x<=31.001);
    assert.ok(st.opacity>.6&&st.opacity<=1);
  }
}
assert.throws(()=>api.projectTraySide(NaN),RangeError);
// The fifth DOF and PC7 are independent: the same absolute cup orientation
// may be reached with different relative gear states.
const compoundA=api.projectTraySide(0,90);
const compoundB=api.projectTraySide(90,0);
for(let i=0;i<3;i++)nearly(compoundA[i].x,compoundB[i].x);
assert.ok(api.projectTraySide(0,-90)[0].x < -30.9);
for(let i=0;i<3;i++)
  nearly(api.projectTraySide(0,999)[i].x,api.projectTraySide(0,90)[i].x);
// Values outside provisional limits CLAMP; malformed numbers are rejected.
assert.throws(()=>api.projectTraySide(0,NaN),RangeError);
// Side-on a yawed platform foreshortens; does not roll or tilt the tray.
nearly(api.projectRearBaseSide(0).widthRatio,1);
nearly(api.projectRearBaseSide(90).widthRatio,.33);
nearly(api.projectRearBaseSide(-90).pointerX,-25);
nearly(api.projectRearBaseSide(90).pointerX,25);
nearly(api.projectRearBaseSide(1000).widthRatio,api.projectRearBaseSide(90).widthRatio);
assert.throws(()=>api.projectRearBaseSide(NaN),RangeError);
// Detailed TOP rig has a distinct rotating platform with PC7 relative
// beneath it; changing platform angle cannot mutate the PC7 target itself.
const previousDoc=global.document;
global.document={createElementNS(_ns,tag){
  return {tag,children:[],attrs:{},
    setAttribute(k,v){this.attrs[k]=v;},
    appendChild(n){this.children.push(n);return n;}};
}};
try{
  const node={children:[],appendChild(x){this.children.push(x);return x;}};
  const rig=api.buildTopRig(node);
  api.applyTopPose(rig,api.computeMechanismPose({
    base_yaw_deg:90,tray_deg:0,lift_mm:0,holder_deg:0,grip_deg:0}));
  assert.equal(rig.rearPlatform.attrs.transform,'rotate(90.000)');
  assert.equal(rig.tray.attrs.transform,'rotate(0.000)');
  api.applyTopPose(rig,api.computeMechanismPose({
    base_yaw_deg:0,tray_deg:180,lift_mm:0,holder_deg:0,grip_deg:0}));
  assert.equal(rig.rearPlatform.attrs.transform,'rotate(0.000)');
  assert.equal(rig.tray.attrs.transform,'rotate(180.000)');
  assert.equal(rig.tray.attrs['data-km-pc7-tray'],'RELATIVE_TO_PLATFORM');
}finally {global.document=previousDoc;}
const code=fs.readFileSync(path.join(web,'keyboard_mechanism_preview.js'),'utf8');
const panel=fs.readFileSync(path.join(web,'keyboard_mapping_panel.js'),'utf8');
const sim=fs.readFileSync(path.join(web,'app.js'),'utf8');
const test=fs.readFileSync(path.join(web,'test_stm32_fsm_workbench.js'),'utf8');
const html=fs.readFileSync(path.join(web,'index.html'),'utf8');
const css=fs.readFileSync(path.join(web,'keyboard_mechanism.css'),'utf8');
assert.match(html,/keyboard_mechanism\.css\?v=1\.3\.0/);
assert.ok(html.indexOf('keyboard_mechanism_preview.js')<html.indexOf('app.js'),
  'CAD generator must load before SIM/REAL initial rendering');
assert.ok(html.indexOf('keyboard_mechanism_preview.js')<
          html.indexOf('keyboard_mapping_panel.js'));
assert.match(panel,/CarMechanismPreview\.create\(panel\)/);
assert.match(panel,/mechanism\.attachMap\(car\)/);
assert.match(panel,/mechanism\.tick\(frameNow\)/);
assert.match(sim,/CarMechanismPreview\.decorateChassis\(g/);
assert.match(test,/CarMechanismPreview\.decorateChassis\(vehicle/);
assert.match(code,/const top=buildTopRig\(/);
assert.match(code,/const rig=buildTopRig\(mount,\{compact:true\}\)/);
assert.match(code,/applyTopPose\(top,pose\)/);
assert.match(code,/mapThumb\.update\(s\)/);
assert.match(code,/UNMEASURED_SCHEMATIC_ONLY/);
assert.match(css,/keyboard-mechanism-controls/);
assert.match(css,/max-width:650px/);
for(const label of ['MOCK ONLY · STM32 TX OFF','PC7','PC6 / PC8 / PC9',
  'PB15','PB14','车尾平台独立旋转 · 未接 STM32','-90']){
  assert.ok(code.includes(label),label);
}
assert.ok(!/\bfetch\s*\(|XMLHttpRequest|sendBeacon\s*\(|WebSocket\s*\(|create_publisher\s*\(|serial\.Serial\s*\(/.test(code));
assert.ok(!/\/api\/keyboard\/intent|\/cmd_vel|\/dev\/tty/.test(code));
console.log('PASS: eased mechanism motion, common CAD silhouette, cross-mode read-only depiction, no motor I/O');
