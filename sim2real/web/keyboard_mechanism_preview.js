/* CAD-derived mechanism MOCK, shared across keyboard/SIM/TEST/REAL silhouettes.
 * Mechanical wiring: PC7 turntable, PC6/PC8/PC9 lift, PB15 holder, PB14 jaw.
 * No network, ROS2, serial, motor or physical position feedback is generated.
 */
(function(root,factory){
  'use strict';
  const api=factory();
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
  if(root)root.CarMechanismPreview=api;
})(typeof window!=='undefined'?window:null,function(){
  'use strict';
  // firmwareRate is documentation only; previewRate/previewAccel are visual
  // comfort settings. These never affect the robot or STM32 timing.
  const AXES=Object.freeze({
    tray_deg:{min:0,max:270,firmwareRate:60,previewRate:45,previewAccel:100,
      label:'PC7 · 车载三工位转盘',unit:'°'},
    lift_mm:{min:0,max:175,firmwareRate:null,previewRate:55,previewAccel:140,
      label:'PC6 / PC8 / PC9 · 夹爪升降',unit:'mm'},
    holder_deg:{min:0,max:180,firmwareRate:127.764,previewRate:85,previewAccel:220,
      label:'PB15 · 夹持机构旋转',unit:'°'},
    grip_deg:{min:0,max:140,firmwareRate:720,previewRate:165,previewAccel:470,
      label:'PB14 · 夹爪开合',unit:'°'}
  });
  const PRESETS=Object.freeze({
    tray_deg:[0,90,180,270],
    lift_mm:[0,24,80,141,170,175],
    holder_deg:[0,180],
    grip_deg:[0,110,140]
  });
  const NAMES=Object.keys(AXES);
  function clamp(name,val){
    const a=AXES[name];
    if(!a||typeof val!=='number'||!Number.isFinite(val))
      throw new RangeError('invalid MOCK mechanism target '+name);
    return Math.max(a.min,Math.min(a.max,val));
  }
  function fresh(){
    const target={},shown={},velocity={};
    for(const n of NAMES){target[n]=0;shown[n]=0;velocity[n]=0;}
    return {target,shown,velocity};
  }
  function setTarget(model,name,val){model.target[name]=clamp(name,val);return model.target[name];}
  function reset(model){
    for(const n of NAMES){
      model.target[n]=0;model.shown[n]=0;model.velocity[n]=0;
    }
  }
  function advance(model,elapsed){
    // Maximum 150ms of elapsed real time is consumed per frame. Long tab
    // suspension cannot skip an entire gripper stroke on resume.
    let remaining=Number.isFinite(elapsed)?Math.max(0,Math.min(.15,elapsed)):0;
    while(remaining>1e-8){
      const dt=Math.min(remaining,1/120);
      for(const n of NAMES){
        const a=AXES[n],distance=model.target[n]-model.shown[n];
        let v=model.velocity[n];
        if(!Number.isFinite(v))v=0;
        if(Math.abs(distance)<.015 && Math.abs(v)<.08){
          model.shown[n]=model.target[n];model.velocity[n]=0;continue;
        }
        // Braking-distance limit gives soft start, soft stop, and reversible
        // retargeting even if a slider is dragged while the part is moving.
        const desired=Math.sign(distance)*
          Math.min(a.previewRate,Math.sqrt(2*a.previewAccel*Math.abs(distance)));
        const delta=Math.max(-a.previewAccel*dt,
          Math.min(a.previewAccel*dt,desired-v));
        const nextV=v+delta;
        const next=model.shown[n]+nextV*dt;
        if((distance>0 && next>=model.target[n])||
           (distance<0 && next<=model.target[n])){
          model.shown[n]=model.target[n];model.velocity[n]=0;
        }else{
          model.shown[n]=clamp(n,next);
          model.velocity[n]=nextV;
        }
      }
      remaining-=dt;
    }
    return model.shown;
  }
  function computeMechanismPose(s){
    const r=clamp('grip_deg',s.grip_deg)/140;
    return Object.freeze({
      trayRotate:clamp('tray_deg',s.tray_deg),
      liftOffset:-clamp('lift_mm',s.lift_mm)/175*104,
      liftTopOffset:-clamp('lift_mm',s.lift_mm)/175*3.8,
      holderRotate:-clamp('holder_deg',s.holder_deg),
      jawOffset:r*13,
      sideJawScale:1-.35*r
    });
  }
  function element(tag,parent,label,klass){
    const n=document.createElement(tag);
    if(label!==undefined)n.textContent=label;
    if(klass)n.className=klass;
    parent.appendChild(n);return n;
  }
  function svgNode(tag,parent,attrs){
    const n=document.createElementNS('http://www.w3.org/2000/svg',tag);
    for(const [key,val] of Object.entries(attrs||{}))n.setAttribute(key,String(val));
    parent.appendChild(n);return n;
  }
  // Single CAD-derived TOP footprint generator for large figure and ALL map
  // thumbnails. No duplicate wheel/arm/tray geometry exists elsewhere.
  function buildTopRig(parent){
    const base=svgNode('g',parent,{'data-km-top-rig':'CAD_SHARED'});
    svgNode('rect',base,{x:-63,y:-58,width:126,height:116,rx:12,
      fill:'#243c57',stroke:'#91b7d3','stroke-width':2});
    const wheels=svgNode('g',base,{fill:'#232937',stroke:'#dba45c','stroke-width':3});
    for(const [x,y] of [[-74,-54],[-74,26],[53,-54],[53,26]])
      svgNode('rect',wheels,{x,y,width:21,height:29,rx:7});
    svgNode('rect',base,{x:-49,y:-33,width:86,height:10,rx:3,fill:'#a6bfce'});
    svgNode('path',base,{d:'M -51 -9 L -106 -9 M -51 9 L -106 9',
      stroke:'#9fbbd0','stroke-width':7,'stroke-linecap':'round'});
    svgNode('rect',base,{x:-46,y:-28,width:18,height:58,rx:5,
      fill:'#6889a2',stroke:'#d9f0fb'});
    const front=svgNode('g',base,{transform:'translate(-110 0)'});
    svgNode('rect',front,{x:-29,y:-15,width:28,height:30,rx:5,
      fill:'#a68ed4',stroke:'#e9d9ff','stroke-width':2});
    const left=svgNode('g',front,{});
    svgNode('path',left,{d:'M -29 -14 H -50 V -5',fill:'none',
      stroke:'#ece8ff','stroke-width':7,'stroke-linecap':'round'});
    const right=svgNode('g',front,{});
    svgNode('path',right,{d:'M -29 14 H -50 V 5',fill:'none',
      stroke:'#ece8ff','stroke-width':7,'stroke-linecap':'round'});
    // PB15 rotates the holder about the arm axis; it must NOT swing the whole
    // gripper in the top-view plane. A dial depicts its rotation instead.
    const dial=svgNode('g',front,{transform:'translate(-14 0)'});
    svgNode('circle',dial,{r:7,fill:'#2c3655',stroke:'#ede1ff','stroke-width':1.4});
    svgNode('path',dial,{d:'M 0 -5 V -1',stroke:'#eac6ff',
      'stroke-width':2,'stroke-linecap':'round'});
    const turret=svgNode('g',base,{transform:'translate(37 16)'});
    svgNode('circle',turret,{r:47,fill:'#15283d',stroke:'#b4c4d4','stroke-width':3});
    const tray=svgNode('g',turret,{});
    svgNode('path',tray,{d:'M 0 -38 L 34 19 L -34 19 Z',
      fill:'#72849a',stroke:'#e6edf5','stroke-width':2});
    for(const [x,y] of [[0,-25],[-22,14],[22,14]]){
      svgNode('circle',tray,{cx:x,cy:y,r:13,fill:'#dfe9ef',stroke:'#91abc0','stroke-width':2});
      svgNode('circle',tray,{cx:x,cy:y,r:7,fill:'#283b50'});
    }
    svgNode('circle',turret,{r:5,fill:'#c5a9f6'});
    return {base,left,right,dial,tray,front};
  }
  function applyTopPose(rig,pose){
    rig.tray.setAttribute('transform','rotate('+pose.trayRotate.toFixed(3)+')');
    rig.dial.setAttribute('transform','translate(-14 0) rotate('+pose.holderRotate.toFixed(3)+')');
    rig.left.setAttribute('transform','translate(0 '+pose.jawOffset.toFixed(3)+')');
    rig.right.setAttribute('transform','translate(0 '+(-pose.jawOffset).toFixed(3)+')');
    rig.front.setAttribute('transform','translate(-110 '+pose.liftTopOffset.toFixed(3)+')');
  }
  function decorateChassis(parent,options){
    const opts=options||{};
    const x=Number.isFinite(opts.centerX)?opts.centerX:0;
    const y=Number.isFinite(opts.centerY)?opts.centerY:0;
    const scale=Number.isFinite(opts.scale)?Math.max(.05,Math.min(.5,opts.scale)):.21;
    const mount=svgNode('g',parent,{'data-km-shared-cad':'true',
      'data-mechanism-source':'UNMEASURED_SCHEMATIC_ONLY',
      'pointer-events':'none',
      transform:'translate('+x+' '+y+') rotate(90) scale('+scale+')'});
    const rig=buildTopRig(mount);
    applyTopPose(rig,computeMechanismPose(opts.pose||fresh().shown));
    return Object.freeze({mount,rig,update:pose=>applyTopPose(rig,computeMechanismPose(pose))});
  }
  function create(host){
    const model=fresh();
    const panel=element('section',host,undefined,'keyboard-mechanism-panel');
    panel.id='carKeyboardMechanismPreview';panel.dataset.mock='true';
    element('div',panel,'04 / ROBOT MECHANISM · 纯网页模拟','keyboard-mechanism-eyebrow');
    element('h3',panel,'车载转盘 · 升降滑台 · 旋转夹爪','keyboard-mechanism-title');
    element('p',panel,
      '大图、地图小车采用同一套机械轮廓。机构数值仅为网页预览，不是 STM32 位置回执。',
      'keyboard-mechanism-note');
    const figure=element('div',panel,undefined,'keyboard-mechanism-figure');
    const svg=svgNode('svg',figure,{viewBox:'0 0 560 246',role:'img',
      'aria-label':'结构示意：四轮麦克纳姆底盘、三工位转盘、升降滑台与旋转夹爪'});
    svg.innerHTML=[
      '<text x="18" y="22" class="km-view-label">俯视 · 车头向左</text>',
      '<text x="317" y="22" class="km-view-label">侧视 · 导轨升降</text>',
      '<g data-km-top-mount transform="translate(177 123)"></g>',
      '<g><rect x="326" y="184" width="219" height="17" rx="6" fill="#3c536b" stroke="#a1b3c4"/>',
      '<circle cx="358" cy="202" r="19" fill="#b88648" stroke="#253545" stroke-width="4"/>',
      '<circle cx="516" cy="202" r="19" fill="#b88648" stroke="#253545" stroke-width="4"/>',
      '<rect x="409" y="45" width="18" height="140" rx="4" fill="#a5afc1" stroke="#dce8fa"/>',
      '<rect x="429" y="44" width="9" height="141" fill="#314860"/>',
      '<g data-lift><rect x="398" y="161" width="51" height="15" rx="4" fill="#9bb2ca" stroke="#fff"/>',
      '<path d="M 401 167 H 376 V 142" fill="none" stroke="#c7cfe0" stroke-width="8"/>',
      '<g data-side-holder transform="translate(373 140)"><path d="M 0 0 H -25" fill="none" stroke="#c6a9ec" stroke-width="8"/>',
      '<g data-side-jaw><path d="M -25 -17 V 17" stroke="#eadcf9" stroke-width="6"/></g></g></g>',
      '<ellipse cx="491" cy="180" rx="40" ry="6" fill="#d4dfe9" stroke="#8b9eae"/>',
      '<path d="M 454 177 L 462 189 L 483 189 L 488 177" fill="#9faebc"/>',
      '<path d="M 489 177 L 495 189 L 514 189 L 521 177" fill="#c3cedd"/>',
      '<text x="438" y="225" class="km-view-small">车载三工位托盘</text>',
      '</g><text x="18" y="235" class="km-view-small">MOCK / 非实车位置测量</text>'
    ].join('');
    const top=buildTopRig(svg.querySelector('[data-km-top-mount]'));
    const lift=svg.querySelector('[data-lift]');
    const sideHolder=svg.querySelector('[data-side-holder]');
    const sideJaw=svg.querySelector('[data-side-jaw]');
    const controls=element('div',panel,undefined,'keyboard-mechanism-controls');
    const sliders={},outputs={};
    const presetNames={
      tray_deg:{0:'0° 原点',90:'90°',180:'180°',270:'270°'},
      lift_mm:{0:'零位',24:'抓取低位',80:'接近',141:'转运',170:'上位',175:'最高预设'},
      holder_deg:{0:'朝前',180:'翻转 180°'},
      grip_deg:{0:'张开',110:'预夹',140:'夹紧'}
    };
    for(const name of NAMES){
      const a=AXES[name];
      const box=element('div',controls,undefined,'keyboard-mechanism-axis');
      box.dataset.axis=name;
      const head=element('div',box,undefined,'keyboard-mechanism-axis-head');
      element('strong',head,a.label);
      outputs[name]=element('output',head,'0'+a.unit,'keyboard-mechanism-value');
      const range=element('input',box);
      range.type='range';range.min=a.min;range.max=a.max;range.step='1';
      range.value='0';sliders[name]=range;
      range.setAttribute('aria-label',a.label+' · MOCK 示意');
      range.addEventListener('input',()=>{
        setTarget(model,name,Number(range.value));syncTargets();
      });
      const buttons=element('div',box,undefined,'keyboard-mechanism-presets');
      for(const val of PRESETS[name]){
        const label=presetNames[name][val]||val+a.unit;
        const b=element('button',buttons,label);b.type='button';
        b.dataset.target=String(val);
        b.title=String(val)+a.unit+' · 模拟预览，非真实机构指令';
        b.addEventListener('click',()=>{setTarget(model,name,val);syncTargets();});
      }
    }
    const foot=element('div',panel,undefined,'keyboard-mechanism-footer');
    const home=element('button',foot,'全部机构回演示零位','keyboard-mechanism-home');
    home.type='button';
    home.addEventListener('click',()=>{
      for(const name of NAMES)setTarget(model,name,0);
      syncTargets();
    });
    element('span',foot,'MOCK ONLY · STM32 TX OFF','keyboard-mechanism-only');
    element('p',panel,
      'PC7 转盘；PC6 升降脉冲、PC8/PC9 方向；PB15 机构翻转；PB14 夹爪。界面平滑速度独立于固件参数。升降 0 mm 为人工零位，尚未做限位寻零与机械干涉检测。',
      'keyboard-mechanism-note keyboard-mechanism-footnote');

    let mapThumb=null;
    function syncTargets(){
      for(const name of NAMES){
        sliders[name].value=model.target[name];
        controls.querySelectorAll('[data-axis="'+name+'"] button[data-target]').forEach(b=>{
          const active=Number(b.dataset.target)===model.target[name];
          b.classList.toggle('selected',active);
          b.setAttribute('aria-pressed',String(active));
        });
      }
    }
    function draw(){
      const s=model.shown;
      const pose=computeMechanismPose(s);
      applyTopPose(top,pose);
      lift.setAttribute('transform','translate(0 '+pose.liftOffset.toFixed(3)+')');
      sideHolder.setAttribute('transform',
        'translate(373 140) rotate('+pose.holderRotate.toFixed(3)+')');
      sideJaw.setAttribute('transform','scale(1 '+pose.sideJawScale.toFixed(3)+')');
      if(mapThumb&&mapThumb.mount.isConnected)mapThumb.update(s);
      for(const name of NAMES)
        outputs[name].textContent=s[name].toFixed(name==='lift_mm'?0:1)+AXES[name].unit;
    }
    let previous=null;
    function tick(now){
      if(!Number.isFinite(now))return;
      const dt=previous===null?0:Math.max(0,Math.min(.15,(now-previous)/1000));
      previous=now;
      if(document.hidden){previous=null;return;}
      const moving=NAMES.some(n=>
        Math.abs(model.target[n]-model.shown[n])>.005 ||
        Math.abs(model.velocity[n])>.05);
      if(!moving)return; // No redundant SVG DOM writes while idle.
      advance(model,dt);
      draw();
    }
    function attachMap(car){
      if(mapThumb&&mapThumb.mount.isConnected)mapThumb.mount.remove();
      mapThumb=decorateChassis(car,{scale:.23,pose:model.shown});
      draw();
    }
    syncTargets();draw();
    return Object.freeze({root:panel,model,tick,attachMap});
  }
  return Object.freeze({AXES,PRESETS,clamp,fresh,setTarget,advance,reset,
    computeMechanismPose,buildTopRig,applyTopPose,decorateChassis,create});
});
