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
  function buildTopRig(parent,options){
    const compact=!!options?.compact;
    const base=svgNode('g',parent,{'data-km-top-rig':'CAD_SHARED',
      'data-km-visual-version':'2'});
    // Layered chassis: shadow, magnesium-grey frame, raised centre cover.
    svgNode('rect',base,{x:-68,y:-61,width:136,height:124,rx:16,
      fill:'#040f1d',opacity:.52});
    svgNode('rect',base,{x:-64,y:-58,width:128,height:116,rx:13,
      fill:'#243c57',stroke:'#86aaca','stroke-width':3});
    svgNode('rect',base,{x:-59,y:-53,width:118,height:106,rx:10,
      fill:'#172b42',stroke:'#486a87','stroke-width':1});
    svgNode('path',base,{d:'M -55 -41 L -43 -49 H 47 L 56 -39 V 39 L 46 49 H -43 L -55 39 Z',
      fill:'#2a4865',stroke:'#567d99','stroke-width':1.6});
    svgNode('path',base,{d:'M -49 -36 H 42 L 51 -28 V 27 L 42 36 H -49 Z',
      fill:'#1d344d',stroke:'#668ba7','stroke-width':1.1});
    if(!compact)for(const [x,y] of [[-50,-43],[48,-43],[-50,42],[48,42]]){
      svgNode('circle',base,{cx:x,cy:y,r:3.1,fill:'#0d1d30',
        stroke:'#9cb9cb','stroke-width':1.3});
      svgNode('circle',base,{cx:x,cy:y,r:.9,fill:'#cad7e2'});
    }
    // Four mecanum tires with conspicuous diagonal rollers and gold hubs.
    const wheels=svgNode('g',base,{});
    for(const [x,y,flip] of [[-75,-54,1],[-75,26,-1],[54,-54,-1],[54,26,1]]){
      svgNode('rect',wheels,{x:x-1,y:y-1,width:23,height:32,rx:7,
        fill:'#0a111d',stroke:'#d5a15e','stroke-width':2});
      svgNode('rect',wheels,{x:x+3,y:y+2,width:15,height:26,rx:4,
        fill:'#334054',stroke:'#617181','stroke-width':1});
      const rollers=svgNode('g',wheels,{});
      if(!compact)for(let n=0;n<3;n++){
        const cy=y+5+n*8;
        svgNode('path',rollers,{d:flip===1?
          'M '+(x+5)+' '+(cy+5)+' L '+(x+15)+' '+cy:
          'M '+(x+5)+' '+cy+' L '+(x+15)+' '+(cy+5),
          stroke:'#b4c3d2','stroke-width':2.7,'stroke-linecap':'round'});
      }
      svgNode('circle',wheels,{cx:x+10,cy:y+15,r:2.4,
        fill:'#edbb74',stroke:'#fff0ca','stroke-width':.6});
    }
    // Front twin slide rails, carriage and powered gripper wrist.
    svgNode('path',base,{d:'M -50 -11 H -109 M -50 11 H -109',
      stroke:'#081a2a','stroke-width':10,'stroke-linecap':'round'});
    svgNode('path',base,{d:'M -50 -11 H -109 M -50 11 H -109',
      stroke:'#b3d0e5','stroke-width':5.3,'stroke-linecap':'round'});
    svgNode('rect',base,{x:-49,y:-31,width:19,height:62,rx:4,
      fill:'#577a96',stroke:'#c8e9ff','stroke-width':2});
    svgNode('path',base,{d:'M -43 -26 V 25 M -35 -26 V 25',
      stroke:'#d3edfa','stroke-width':1.1,opacity:.72});
    svgNode('rect',base,{x:-117,y:-20,width:18,height:40,rx:5,
      fill:'#7b96b3',stroke:'#dceafa','stroke-width':1.7});
    const front=svgNode('g',base,{transform:'translate(-110 0)'});
    svgNode('rect',front,{x:-29,y:-16,width:31,height:32,rx:5,
      fill:'#9176c2',stroke:'#e7d8ff','stroke-width':2.2});
    svgNode('rect',front,{x:-25,y:-12,width:22,height:24,rx:4,
      fill:'#c5b5e4',stroke:'#f4efff','stroke-width':1});
    svgNode('circle',front,{cx:-14,cy:0,r:10,fill:'#433a6f',
      stroke:'#f1e5ff','stroke-width':1.5});
    const left=svgNode('g',front,{});
    svgNode('path',left,{d:'M -29 -14 H -50 V -5',fill:'none',
      stroke:'#e4e5f4','stroke-width':6.8,'stroke-linecap':'round',
      'stroke-linejoin':'round'});
    svgNode('path',left,{d:'M -49 -7 L -52 -4',fill:'none',
      stroke:'#c5adfc','stroke-width':3.2,'stroke-linecap':'round'});
    const right=svgNode('g',front,{});
    svgNode('path',right,{d:'M -29 14 H -50 V 5',fill:'none',
      stroke:'#e4e5f4','stroke-width':6.8,'stroke-linecap':'round',
      'stroke-linejoin':'round'});
    svgNode('path',right,{d:'M -49 7 L -52 4',fill:'none',
      stroke:'#c5adfc','stroke-width':3.2,'stroke-linecap':'round'});
    // PB15 is axial gripper rotation, visualised as a wrist dial, not yaw.
    const dial=svgNode('g',front,{transform:'translate(-14 0)'});
    svgNode('circle',dial,{r:6,fill:'#1a2139',stroke:'#d9c5ff','stroke-width':1});
    svgNode('path',dial,{d:'M 0 -4 V -1',stroke:'#fbdeff',
      'stroke-width':2.2,'stroke-linecap':'round'});
    // Machined three-pocket tray with bearing race and a visible spindle.
    const turret=svgNode('g',base,{transform:'translate(36 15)'});
    svgNode('circle',turret,{r:50,fill:'#091827',
      stroke:'#7e9caf','stroke-width':2.8});
    svgNode('circle',turret,{r:45,fill:'#22384f',
      stroke:'#c3d9e8','stroke-width':1.7});
    svgNode('circle',turret,{r:40,fill:'#142739',
      stroke:'#5b7c98','stroke-width':2});
    if(!compact)for(let n=0;n<12;n++){
      const theta=2*Math.PI*n/12;
      svgNode('path',turret,{d:'M '+(44*Math.cos(theta)).toFixed(2)+
        ' '+(44*Math.sin(theta)).toFixed(2)+' L '+
        (47*Math.cos(theta)).toFixed(2)+' '+(47*Math.sin(theta)).toFixed(2),
        stroke:'#c5dce9','stroke-width':1.3,opacity:.85});
    }
    const tray=svgNode('g',turret,{});
    svgNode('path',tray,{d:'M 0 -39 L 34 20 L -34 20 Z',
      fill:'#607e9c',stroke:'#e4f2ff','stroke-width':2.7,
      'stroke-linejoin':'round'});
    svgNode('path',tray,{d:'M 0 -31 L 27 16 L -27 16 Z',
      fill:'#a5b9ca',stroke:'#d3e7f5','stroke-width':1.4});
    for(const [x,y] of [[0,-25],[-22,14],[22,14]]){
      svgNode('circle',tray,{cx:x,cy:y,r:13.5,fill:'#d9e4ee',
        stroke:'#697f95','stroke-width':1.6});
      svgNode('circle',tray,{cx:x,cy:y,r:10.1,fill:'#425d75',
        stroke:'#f2f7fc','stroke-width':1.1});
      svgNode('circle',tray,{cx:x,cy:y,r:6.6,fill:'#17293c',
        stroke:'#8397ae','stroke-width':1});
      if(!compact)svgNode('path',tray,{d:'M '+(x-3.2)+' '+(y-2.3)+' h 5.1',
        stroke:'#a5bdd1','stroke-width':1,'stroke-linecap':'round'});
    }
    svgNode('circle',turret,{r:6.5,fill:'#d9d4f6',
      stroke:'#6d5a9c','stroke-width':2});
    svgNode('circle',turret,{r:2.1,fill:'#7e69be'});
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
    const rig=buildTopRig(mount,{compact:true});
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
      '<g><rect x="325" y="186" width="220" height="18" rx="6" fill="#0b1525" opacity=".65"/>',
      '<rect x="326" y="183" width="219" height="17" rx="6" fill="#3c536b" stroke="#a1b3c4" stroke-width="2"/>',
      '<path d="M 340 187 H 530" stroke="#7899b5" stroke-width="2"/>',
      '<path d="M 349 196 H 524" stroke="#20364b" stroke-width="2"/>',
      '<circle cx="358" cy="202" r="19" fill="#121f30" stroke="#c3995f" stroke-width="5"/>',
      '<circle cx="358" cy="202" r="10" fill="#35465c" stroke="#a1b3c8" stroke-width="2"/>',
      '<circle cx="516" cy="202" r="19" fill="#121f30" stroke="#c3995f" stroke-width="5"/>',
      '<circle cx="516" cy="202" r="10" fill="#35465c" stroke="#a1b3c8" stroke-width="2"/>',
      '<path d="M 351 195 l 12 12 M 351 208 l 12 -12 M 509 195 l 12 12 M 509 208 l 12 -12" stroke="#d2b184" stroke-width="2"/>',
      '<rect x="403" y="40" width="41" height="7" rx="3" fill="#8dabc5" stroke="#e1eafa"/>',
      '<rect x="409" y="45" width="18" height="140" rx="4" fill="#9bb1c5" stroke="#dce8fa" stroke-width="2"/>',
      '<rect x="429" y="44" width="9" height="141" fill="#314860" stroke="#6e8cab" stroke-width="1"/>',
      '<path d="M 416 51 V 179" stroke="#e3eff6" stroke-width="2" stroke-dasharray="5 4"/>',
      '<path d="M 425 50 V 179" stroke="#5c7998" stroke-width="2"/>',
      '<g fill="#d7e6f3"><circle cx="417" cy="53" r="2"/><circle cx="417" cy="176" r="2"/></g>',
      '<g data-lift><rect x="396" y="159" width="55" height="18" rx="4" fill="#8ba6c0" stroke="#e6f5ff" stroke-width="2"/>',
      '<rect x="401" y="163" width="41" height="10" rx="3" fill="#d3e2ed" stroke="#7892ad"/>',
      '<circle cx="405" cy="168" r="2" fill="#425a78"/><circle cx="437" cy="168" r="2" fill="#425a78"/>',
      '<path d="M 401 167 H 376 V 142" fill="none" stroke="#c7cfe0" stroke-width="8"/>',
      '<g data-side-holder transform="translate(373 140)"><path d="M 0 0 H -25" fill="none" stroke="#c6a9ec" stroke-width="8"/>',
      '<rect x="-8" y="-7" width="10" height="14" rx="3" fill="#a899ce" stroke="#f4e4ff"/>',
      '<circle cx="-18" cy="0" r="7" fill="#40385e" stroke="#ead7ff" stroke-width="1.5"/>',
      '<g data-side-jaw><path d="M -25 -17 V 17" stroke="#eadcf9" stroke-width="6"/>',
      '<path d="M -25 -17 l -7 -4 M -25 17 l -7 4" stroke="#be99fa" stroke-width="3" fill="none"/></g></g></g>',
      '<ellipse cx="491" cy="180" rx="42" ry="8" fill="#c6d4e0" stroke="#859aac" stroke-width="2"/>',
      '<ellipse cx="491" cy="178" rx="34" ry="4" fill="#536e87" stroke="#e1eafa"/>',
      '<path d="M 454 177 L 462 189 L 483 189 L 488 177" fill="#9faebc"/>',
      '<path d="M 489 177 L 495 189 L 514 189 L 521 177" fill="#c3cedd"/>',
      '<text x="438" y="225" class="km-view-small">车载三工位托盘</text>',
      '</g><text x="18" y="235" class="km-view-small">MOCK / 非实车位置测量</text>'
    ].join('');
    const top=buildTopRig(svg.querySelector('[data-km-top-mount]'));
    const lift=svg.querySelector('[data-lift]');
    const sideHolder=svg.querySelector('[data-side-holder]');
    const sideJaw=svg.querySelector('[data-side-jaw]');
    const keyboardHelp=element('div',panel,undefined,'keyboard-mechanism-shortcuts');
    element('strong',keyboardHelp,'MOCK 机构键盘 · 当前控制：紫色主车');
    const keyboardLegend=element('div',keyboardHelp,undefined,'keyboard-mechanism-shortcut-list');
    for(const text of [
      'J / L · 转盘 − / +', 'I / K · 升 / 降',
      'U / O · 翻转 − / +','N / M · 张开 / 夹紧',
      '1–4 · 转盘角度','Z / X / C · 夹爪预设',
      'T · 抓取低位','R · 演示零位'
    ])element('span',keyboardLegend,text);
    const keyboardStatus=element('div',keyboardHelp,
      '机构按键：未按下 · 仅网页演示，SIM / TEST / REAL 不接收机构按键',
      'keyboard-mechanism-key-state');
    keyboardStatus.setAttribute('role','status');
    keyboardStatus.setAttribute('aria-live','polite');
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
    home.setAttribute('aria-keyshortcuts','R');
    home.addEventListener('click',()=>{
      for(const name of NAMES)setTarget(model,name,0);
      syncTargets();
    });
    element('span',foot,'MOCK ONLY · STM32 TX OFF','keyboard-mechanism-only');
    element('p',panel,
      'PC7 转盘；PC6 升降脉冲、PC8/PC9 方向；PB15 机构翻转；PB14 夹爪。界面平滑速度独立于固件参数。升降 0 mm 为人工零位，尚未做限位寻零与机械干涉检测。',
      'keyboard-mechanism-note keyboard-mechanism-footnote');

    let mapThumb=null;
    let keyboardController=null;
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
      for(const name of NAMES){
        outputs[name].textContent=s[name].toFixed(name==='lift_mm'?0:1)+AXES[name].unit;
        const box=controls.querySelector('[data-axis="'+name+'"]');
        if(box)box.dataset.moving=String(
          Math.abs(model.target[name]-s[name])>.04 ||
          Math.abs(model.velocity[name])>.08);
      }
    }
    let previous=null;
    function tick(now){
      if(!Number.isFinite(now))return;
      const dt=previous===null?0:Math.max(0,Math.min(.15,(now-previous)/1000));
      previous=now;
      if(document.hidden){previous=null;return;}
      // Held mechanism keys change ONLY the browser model targets. They
      // never call the existing WASDQE or ROS2 control pathway.
      if(keyboardController)keyboardController.tick(now);
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
    function keyboardChanged(keys){
      keyboardStatus.textContent=keys.length?
        '机构按键中：'+keys.map(k=>k.toUpperCase()).join(' + ')+' · MOCK ONLY':
        '机构按键：未按下 · 仅网页演示，SIM / TEST / REAL 不接收机构按键';
      for(const name of NAMES){
        const block=controls.querySelector('[data-axis="'+name+'"]');
        if(block&&window.CarMechanismKeyboard){
          const active=keys.some(k=>
            window.CarMechanismKeyboard.HELD[k]?.axis===name);
          block.dataset.keyActive=String(active);
        }
      }
      syncTargets();
    }
    if(window.CarMechanismKeyboard){
      keyboardController=window.CarMechanismKeyboard.create({
        model,axes:AXES,setTarget,onChange:keyboardChanged
      });
    }else{
      keyboardStatus.textContent='机构快捷键未加载 · 滑块和预设按钮仍可使用';
    }
    syncTargets();draw();
    return Object.freeze({root:panel,model,tick,attachMap,
      keyboard:keyboardController});
  }
  return Object.freeze({AXES,PRESETS,clamp,fresh,setTarget,advance,reset,
    computeMechanismPose,buildTopRig,applyTopPose,decorateChassis,create});
});
