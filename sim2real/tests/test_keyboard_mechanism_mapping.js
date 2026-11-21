/* Offline tests: browser MOCK mechanism keys never touch ROS2/STM32. */
'use strict';
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const web=path.join(__dirname,'..','web');
const api=require(path.join(web,'keyboard_mechanism_mapping.js'));
const preview=require(path.join(web,'keyboard_mechanism_preview.js'));
class FakeTarget {
  constructor(){this.listeners=new Map();this.hidden=false;}
  addEventListener(type,cb){
    if(!this.listeners.has(type))this.listeners.set(type,new Set());
    this.listeners.get(type).add(cb);
  }
  removeEventListener(type,cb){this.listeners.get(type)?.delete(cb);}
  fire(type,props={}){
    const ev={key:'',target:{tagName:'BODY'},repeat:false,
      ctrlKey:false,metaKey:false,altKey:false,defaultPrevented:false,
      preventDefault(){this.defaultPrevented=true;},...props};
    for(const fn of this.listeners.get(type)||[])fn(ev);
    return ev;
  }
}
const near=(x,y,e=1e-8)=>assert.ok(Math.abs(x-y)<=e,String(x)+' != '+String(y));
const win=new FakeTarget(),doc=new FakeTarget(),model=preview.fresh();
let updates=0;
const keys=api.create({model,axes:preview.AXES,setTarget:preview.setTarget,
  windowTarget:win,documentTarget:doc,onChange:()=>updates++});
assert.deepEqual(Object.keys(api.HELD).sort(),
  ['[',']','i','j','k','l','m','n','o','u'].sort());
assert.equal(api.eventKey({key:'【',code:'BracketLeft'}),'[');
assert.equal(api.eventKey({key:'】',code:'BracketRight'}),']');
assert.equal(api.eventKey({key:'L'}),'l');
assert.equal(api.isEditable({tagName:'INPUT',type:'range'}),false);
assert.equal(api.isEditable({tagName:'INPUT',type:'checkbox'}),false);
assert.equal(api.isEditable({tagName:'INPUT',type:'text'}),true);
assert.equal(api.isEditable({tagName:'INPUT',type:'number'}),true);
assert.equal(api.isEditable({tagName:'TEXTAREA'}),true);
assert.equal(api.isEditable({tagName:'DIV'}),false);
assert.equal(win.fire('keydown',{key:'w'}).defaultPrevented,false);
assert.deepEqual(keys.heldKeys(),[]);
keys.tick(1000);
// Fifth axis turns the upright lift column, not the PC7 tray.
assert.equal(win.fire('keydown',{key:'【',code:'BracketLeft'}).defaultPrevented,true);
keys.tick(1050);
near(model.target.column_yaw_deg,-preview.AXES.column_yaw_deg.previewRate*.05);
near(model.target.tray_deg,0);
win.fire('keyup',{key:'【',code:'BracketLeft'});
const columnHold=model.target.column_yaw_deg;
keys.tick(1075);near(model.target.column_yaw_deg,columnHold);
// Opposite bracket keys cancel even on non-US character layouts.
win.fire('keydown',{key:']',code:'BracketRight'});
win.fire('keydown',{key:'【',code:'BracketLeft'});
keys.tick(1100);
near(model.target.column_yaw_deg,columnHold);
win.fire('keyup',{key:']',code:'BracketRight'});
win.fire('keyup',{key:'【',code:'BracketLeft'});
win.fire('keydown',{key:'6'});near(model.target.column_yaw_deg,0);
assert.equal(win.fire('keydown',{key:'L'}).defaultPrevented,true);
keys.tick(1150);
near(model.target.tray_deg,preview.AXES.tray_deg.previewRate*.05);
near(model.shown.tray_deg,0);
keys.tick(1200);
near(model.target.tray_deg,preview.AXES.tray_deg.previewRate*.10);
win.fire('keydown',{key:'l',repeat:true});
assert.deepEqual(keys.heldKeys(),['l']);
const unchanged=model.target.tray_deg;
win.fire('keydown',{key:'j'});
keys.tick(1250);
near(model.target.tray_deg,unchanged);
win.fire('keyup',{key:'j'});
win.fire('keyup',{key:'l'});
const released=model.target.tray_deg;
keys.tick(1300);near(model.target.tray_deg,released);
win.fire('keydown',{key:'i'});
win.fire('keydown',{key:'o'});
win.fire('keydown',{key:'m'});
keys.tick(1350);
assert.ok(model.target.lift_mm>0);
assert.ok(model.target.holder_deg>0);
assert.ok(model.target.grip_deg>0);
win.fire('keyup',{key:'i'});win.fire('keyup',{key:'o'});win.fire('keyup',{key:'m'});
for(const [k,v] of [['1',0],['2',90],['3',180],['4',270]]){
  win.fire('keydown',{key:k});near(model.target.tray_deg,v);
}
for(const [k,v] of [['5',-90],['6',0],['7',90],['8',180],['9',-180]]){
  win.fire('keydown',{key:k});near(model.target.column_yaw_deg,v);
}
// Both 180-degree directions should be reachable through presets and
// forward/reverse continuous bracket keys, without changing PC7.
const trayBefore=model.target.tray_deg;
win.fire('keydown',{key:'8'});
near(model.target.column_yaw_deg,180);
win.fire('keydown',{key:']',code:'BracketRight'});
keys.tick(1360); // saturation remains 180
near(model.target.column_yaw_deg,180);
win.fire('keyup',{key:']',code:'BracketRight'});
win.fire('keydown',{key:'9'});
near(model.target.column_yaw_deg,-180);
win.fire('keydown',{key:'[',code:'BracketLeft'});
keys.tick(1370);
near(model.target.column_yaw_deg,-180);
win.fire('keyup',{key:'[',code:'BracketLeft'});
near(model.target.tray_deg,trayBefore);
for(const [k,v] of [['z',0],['x',110],['c',140]]){
  win.fire('keydown',{key:k});near(model.target.grip_deg,v);
}
win.fire('keydown',{key:'t'});near(model.target.lift_mm,24);
win.fire('keydown',{key:'r'});
for(const n of Object.keys(preview.AXES))near(model.target[n],0);
// A focused slider should not swallow column or gripper keys.
const sliderKey=win.fire('keydown',{
  key:'l',target:{tagName:'INPUT',type:'range'}});
assert.equal(sliderKey.defaultPrevented,true);
keys.tick(1400);
assert.ok(model.target.tray_deg>0);
win.fire('keyup',{key:'l'});
const focusedColumn=win.fire('keydown',{
  key:'[',code:'BracketLeft',target:{tagName:'INPUT',type:'range'}});
assert.equal(focusedColumn.defaultPrevented,true);
keys.tick(1420);
assert.ok(model.target.column_yaw_deg<0);
win.fire('keyup',{key:'[',code:'BracketLeft'});
win.fire('keydown',{key:'r'});
near(model.target.tray_deg,0);
near(model.target.column_yaw_deg,0);
// Text and number input must still be protected.
assert.equal(win.fire('keydown',{
  key:'l',target:{tagName:'INPUT',type:'text'}}).defaultPrevented,false);
win.fire('keydown',{key:'i'});
assert.deepEqual(keys.heldKeys(),['i']);
win.fire('blur');assert.deepEqual(keys.heldKeys(),[]);
win.fire('keydown',{key:'m'});
doc.hidden=true;doc.fire('visibilitychange');
assert.deepEqual(keys.heldKeys(),[]);
const before=updates;keys.destroy();win.fire('keydown',{key:'l'});
keys.tick(1400);assert.equal(updates,before);near(model.target.tray_deg,0);

// Verify the detailed top-view geometry uses one reusable CAD generator.
const previousDocument=global.document;
global.document={createElementNS(_ns,tag){
  return {tag,children:[],attrs:{},setAttribute(k,v){this.attrs[k]=v;},
    appendChild(n){this.children.push(n);return n;}};
}};
try {
  const root={children:[],appendChild(n){this.children.push(n);}};
  const rig=preview.buildTopRig(root);
  assert.equal(root.children[0].attrs['data-km-top-rig'],'CAD_SHARED');
  assert.equal(root.children[0].attrs['data-km-visual-version'],'2');
  preview.applyTopPose(rig,preview.computeMechanismPose({
    column_yaw_deg:-45,tray_deg:90,lift_mm:141,holder_deg:180,grip_deg:110}));
  assert.ok(rig.tray.attrs.transform.includes('90'));
  assert.equal(rig.columnTurn.attrs.transform,'translate(-40 0) rotate(-45.000)');
  assert.equal(rig.tray.attrs.transform,'rotate(90.000)');
  function count(n){return 1+(n.children||[]).reduce((a,x)=>a+count(x),0);}
  assert.ok(count(root)>80,'detailed rollers, turntable bearings and gripper');
}finally {global.document=previousDocument;}

const content=fs.readFileSync(path.join(web,'keyboard_mechanism_mapping.js'),'utf8');
const panel=fs.readFileSync(path.join(web,'keyboard_mapping_panel.js'),'utf8');
const visual=fs.readFileSync(path.join(web,'keyboard_mechanism_preview.js'),'utf8');
const html=fs.readFileSync(path.join(web,'index.html'),'utf8');
assert.ok(html.indexOf('keyboard_mechanism_mapping.js')<
          html.indexOf('keyboard_mechanism_preview.js'));
assert.ok(html.indexOf('keyboard_mechanism_preview.js')<
          html.indexOf('keyboard_mapping_panel.js'));
assert.match(visual,/CarMechanismKeyboard\.create/);
assert.match(visual,/keyboardController\.tick\(now\)/);
assert.match(visual,/MOCK ONLY · STM32 TX OFF/);
assert.match(panel,/active\.add\(key\);refreshKeys\(\);sendNow\(\)/);
assert.match(panel,/active\.delete\(key\);refreshKeys\(\);sendNow\(\)/);
assert.ok(!/\bfetch\s*\(|XMLHttpRequest|WebSocket\s*\(|sendBeacon\s*\(|create_publisher\s*\(/.test(content));
assert.ok(!/\/dev\/tty|serial\.Serial/.test(content));
console.log('PASS: mechanism key mappings, focus, blur, single CAD rig and no motor I/O');
