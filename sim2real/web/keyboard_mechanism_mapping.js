/* Browser-only mechanism keymap. NEVER posts /api/keyboard/intent,
 * publishes ROS2, opens UART or controls STM32.
 * A separate control layer so WASDQE keyboard transport remains untouched.
 */
(function(root,make){
  'use strict';
  const api=make();
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
  if(root)root.CarMechanismKeyboard=api;
})(typeof window==='undefined'?null:window,function(){
  'use strict';
  const HELD=Object.freeze({
    j:{axis:'tray_deg',direction:-1},
    l:{axis:'tray_deg',direction:1},
    i:{axis:'lift_mm',direction:1},
    k:{axis:'lift_mm',direction:-1},
    u:{axis:'holder_deg',direction:-1},
    o:{axis:'holder_deg',direction:1},
    n:{axis:'grip_deg',direction:-1},
    m:{axis:'grip_deg',direction:1}
  });
  const PRESET=Object.freeze({
    '1':{axis:'tray_deg',value:0},'2':{axis:'tray_deg',value:90},
    '3':{axis:'tray_deg',value:180},'4':{axis:'tray_deg',value:270},
    z:{axis:'grip_deg',value:0},x:{axis:'grip_deg',value:110},
    c:{axis:'grip_deg',value:140},
    t:{axis:'lift_mm',value:24}
  });
  const HOLD_KEYS=Object.freeze(Object.keys(HELD));
  function isEditable(element){
    if(!element)return false;
    if(element.isContentEditable)return true;
    const tag=String(element.tagName||'').toUpperCase();
    if(tag!=='INPUT')return tag==='TEXTAREA'||tag==='SELECT';
    // Sliders/checkboxes remain keyboard-friendly control surfaces, just
    // like the existing WASDQE handler. Do not intercept text/number input.
    return !['range','checkbox','radio','button','color'].includes(
      String(element.type||'text').toLowerCase());
  }
  function direction(held,axis){
    let value=0;
    for(const k of held){
      const info=HELD[k];
      if(info&&info.axis===axis)value+=info.direction;
    }
    return Math.max(-1,Math.min(1,value));
  }
  function create(options){
    const {model,axes,setTarget,onChange}=options||{};
    if(!model||!axes||typeof setTarget!=='function')
      throw new TypeError('Mechanism MOCK keyboard requires an in-browser model');
    const win=options.windowTarget||
      (typeof window==='undefined'?null:window);
    const doc=options.documentTarget||
      (typeof document==='undefined'?null:document);
    if(!win||!doc)throw new TypeError('Keyboard bindings require event targets');
    const held=new Set();
    let last=null,disposed=false;
    const update=()=>{if(typeof onChange==='function')onChange([...held]);};
    function keydown(e){
      if(disposed||e.ctrlKey||e.metaKey||e.altKey||isEditable(e.target))return;
      if(doc.hidden)return;
      const key=String(e.key||'').toLowerCase();
      if(Object.prototype.hasOwnProperty.call(HELD,key)){
        e.preventDefault();
        if(!held.has(key)){held.add(key);update();}
      }else if(Object.prototype.hasOwnProperty.call(PRESET,key)){
        e.preventDefault();
        if(e.repeat)return;
        const p=PRESET[key];setTarget(model,p.axis,p.value);update();
      }else if(key==='r'){
        e.preventDefault();
        if(e.repeat)return;
        // Zero targets, with smooth animation; NEVER a hardware homing action.
        for(const axis of Object.keys(axes))setTarget(model,axis,0);
        held.clear();update();
      }
    }
    function keyup(e){
      const key=String(e.key||'').toLowerCase();
      if(held.delete(key)){e.preventDefault();update();}
    }
    function clear(){
      if(held.size){held.clear();update();}
      last=null;
    }
    function visibility(){if(doc.hidden)clear();}
    function tick(now){
      if(disposed||!Number.isFinite(now))return;
      if(doc.hidden){clear();return;}
      const dt=last===null?0:Math.min(.05,Math.max(0,(now-last)/1000));
      last=now;
      if(dt===0||held.size===0)return;
      let dirty=false;
      for(const [axis,spec] of Object.entries(axes)){
        const dir=direction(held,axis);
        if(dir===0)continue; // Opposite keys cancel each other.
        const rate=spec.previewRate;
        if(!Number.isFinite(rate)||rate<=0)continue;
        const next=Math.min(spec.max,
          Math.max(spec.min,model.target[axis]+dir*rate*dt));
        if(Math.abs(next-model.target[axis])>1e-8){
          setTarget(model,axis,next);dirty=true;
        }
      }
      if(dirty)update();
    }
    function destroy(){
      if(disposed)return;
      disposed=true;held.clear();
      win.removeEventListener('keydown',keydown);
      win.removeEventListener('keyup',keyup);
      win.removeEventListener('blur',clear);
      doc.removeEventListener('visibilitychange',visibility);
    }
    win.addEventListener('keydown',keydown);
    win.addEventListener('keyup',keyup);
    win.addEventListener('blur',clear);
    doc.addEventListener('visibilitychange',visibility);
    return Object.freeze({tick,clear,destroy,heldKeys:()=>[...held]});
  }
  return Object.freeze({HELD,PRESET,HOLD_KEYS,isEditable,direction,create});
});
