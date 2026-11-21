/* Browser-only mechanism MOCK; not a motor command or position feedback.
 * CAD: Hardware_mechanical/mechanical/{俯视图,前视图,侧视图}.png
 * STM32: PC7 tray, PC6+PC8/PC9 lift, PB15 holder, PB14 grip.
 */
(function(root,make){
  'use strict';
  const api=make();
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
  if(root)root.CarMechanismPreview=api;
})(typeof window!=='undefined'?window:null,function(){
  'use strict';
  const AXES=Object.freeze({
    tray_deg:{min:0,max:270,rate:60,label:'PC7 · 车载三工位转盘',unit:'°'},
    lift_mm:{min:0,max:175,rate:65,label:'PC6 / PC8 / PC9 · 夹爪升降',unit:'mm'},
    holder_deg:{min:0,max:180,rate:127.764,label:'PB15 · 夹持机构旋转',unit:'°'},
    grip_deg:{min:0,max:140,rate:720,label:'PB14 · 夹爪开合',unit:'°'}
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
      throw new RangeError('invalid MOCK mechanism axis '+name);
    return Math.max(a.min,Math.min(a.max,val));
  }
  function fresh(){
    const target={},shown={};
    for(const n of NAMES)target[n]=shown[n]=0;
    return {target,shown};
  }
  function setTarget(model,name,value){
    model.target[name]=clamp(name,value);
    return model.target[name];
  }
  function advance(model,dt){
    const seconds=Number.isFinite(dt)?Math.max(0,Math.min(.05,dt)):0;
    for(const n of NAMES){
      const delta=model.target[n]-model.shown[n];
      model.shown[n]=clamp(n,model.shown[n]+Math.sign(delta)*Math.min(Math.abs(delta),AXES[n].rate*seconds));
    }
    return model.shown;
  }
  function reset(model){for(const n of NAMES)model.target[n]=model.shown[n]=0;}
  function element(tag,parent,label,klass){
    const node=document.createElement(tag);
    if(label!==undefined)node.textContent=label;
    if(klass)node.className=klass;
    parent.appendChild(node);return node;
  }
  function svgNode(tag,parent,attrs){
    const node=document.createElementNS('http://www.w3.org/2000/svg',tag);
    for(const [n,v] of Object.entries(attrs||{}))node.setAttribute(n,String(v));
    parent.appendChild(node);return node;
  }
  function create(host){
    const model=fresh();
    const panel=element('section',host,undefined,'keyboard-mechanism-panel');
    panel.id='carKeyboardMechanismPreview';panel.dataset.mock='true';
    element('div',panel,'04 / ROBOT MECHANISM · 纯网页模拟','keyboard-mechanism-eyebrow');
    element('h3',panel,'车载转盘 · 升降滑台 · 旋转夹爪','keyboard-mechanism-title');
    element('p',panel,'依据仓库俯视/侧视机械结构绘制。数值是浏览器目标与演示姿态，不是 STM32 反馈。','keyboard-mechanism-note');
    const figure=element('div',panel,undefined,'keyboard-mechanism-figure');
    const svg=svgNode('svg',figure,{viewBox:'0 0 560 246',role:'img',
      'aria-label':'工创小车机械示意：四轮麦克纳姆底盘、三工位转盘、升降滑台与可旋转夹爪'});
    svg.innerHTML=[
      '<text x="18" y="22" class="km-view-label">俯视 · 车头向左</text>',
      '<text x="317" y="22" class="km-view-label">侧视 · 导轨升降</text>',
      '<g transform="translate(177,123)">',
      '<rect x="-63" y="-58" width="126" height="116" rx="12" fill="#243c57" stroke="#91b7d3" stroke-width="2"/>',
      '<g fill="#232937" stroke="#dba45c" stroke-width="3">',
      '<rect x="-74" y="-54" width="21" height="29" rx="7"/><rect x="-74" y="26" width="21" height="29" rx="7"/>',
      '<rect x="53" y="-54" width="21" height="29" rx="7"/><rect x="53" y="26" width="21" height="29" rx="7"/>',
      '</g><rect x="-49" y="-33" width="86" height="10" rx="3" fill="#a6bfce"/>',
      '<path d="M -51 -9 L -106 -9 M -51 9 L -106 9" stroke="#9fbbd0" stroke-width="7" stroke-linecap="round"/>',
      '<rect x="-46" y="-28" width="18" height="58" rx="5" fill="#6889a2" stroke="#d9f0fb"/>',
      '<g data-holder transform="translate(-110 0)">',
      '<rect x="-29" y="-15" width="28" height="30" rx="5" fill="#a68ed4" stroke="#e9d9ff" stroke-width="2"/>',
      '<g data-jaw-left><path d="M -29 -14 H -50 V -5" fill="none" stroke="#ece8ff" stroke-width="7" stroke-linecap="round"/></g>',
      '<g data-jaw-right><path d="M -29 14 H -50 V 5" fill="none" stroke="#ece8ff" stroke-width="7" stroke-linecap="round"/></g></g>',
      '<g transform="translate(37 16)"><circle r="47" fill="#15283d" stroke="#b4c4d4" stroke-width="3"/>',
      '<g data-tray><path d="M 0 -38 L 34 19 L -34 19 Z" fill="#72849a" stroke="#e6edf5" stroke-width="2"/>',
      '<g fill="#dfe9ef" stroke="#91abc0" stroke-width="2"><circle cx="0" cy="-25" r="13"/><circle cx="-22" cy="14" r="13"/><circle cx="22" cy="14" r="13"/></g>',
      '<g fill="#283b50"><circle cx="0" cy="-25" r="7"/><circle cx="-22" cy="14" r="7"/><circle cx="22" cy="14" r="7"/></g></g>',
      '<circle r="5" fill="#c5a9f6"/></g></g>',
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
    const tray=svg.querySelector('[data-tray]');
    const holder=svg.querySelector('[data-holder]');
    const jawLeft=svg.querySelector('[data-jaw-left]');
    const jawRight=svg.querySelector('[data-jaw-right]');
    const lift=svg.querySelector('[data-lift]');
    const sideHolder=svg.querySelector('[data-side-holder]');
    const sideJaw=svg.querySelector('[data-side-jaw]');

    const controls=element('div',panel,undefined,'keyboard-mechanism-controls');
    const sliders={},outputs={};
    for(const name of NAMES){
      const a=AXES[name];
      const box=element('div',controls,undefined,'keyboard-mechanism-axis');
      box.dataset.axis=name;
      const head=element('div',box,undefined,'keyboard-mechanism-axis-head');
      element('strong',head,a.label);
      outputs[name]=element('output',head,'0'+a.unit,'keyboard-mechanism-value');
      const range=element('input',box);
      range.type='range';range.min=a.min;range.max=a.max;
      range.step='1';range.value='0';sliders[name]=range;
      range.setAttribute('aria-label',a.label+' · MOCK 模拟');
      range.addEventListener('input',()=>{
        setTarget(model,name,Number(range.value));syncTargets();
      });
      const buttons=element('div',box,undefined,'keyboard-mechanism-presets');
      for(const val of PRESETS[name]){
        const label=name==='grip_deg'?({0:'张开',110:'预夹',140:'夹紧'}[val]):val+a.unit;
        const b=element('button',buttons,label);
        b.type='button';b.dataset.target=String(val);
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
      '固件映射：PC7 转盘伺服 · PC6 升降脉冲、PC8/PC9 方向 · PB15 夹持机构旋转 · PB14 夹爪。升降 0 mm 为人工零位；没有限位寻零和机械干涉检测。',
      'keyboard-mechanism-note keyboard-mechanism-footnote');
    const map={g:null,tray:null,holder:null,lift:null,left:null,right:null};
    function syncTargets(){
      for(const name of NAMES){
        sliders[name].value=model.target[name];
        controls.querySelectorAll('[data-axis="'+name+'"] button[data-target]').forEach(b=>{
          const selected=Number(b.dataset.target)===model.target[name];
          b.classList.toggle('selected',selected);
          b.setAttribute('aria-pressed',String(selected));
        });
      }
    }
    function draw(){
      const s=model.shown,r=s.grip_deg/140;
      for(const name of NAMES)outputs[name].textContent=Math.round(s[name])+AXES[name].unit;
      tray.setAttribute('transform','rotate('+s.tray_deg.toFixed(2)+')');
      holder.setAttribute('transform','translate(-110 0) rotate('+(-s.holder_deg).toFixed(2)+')');
      jawLeft.setAttribute('transform','translate(0 '+(r*13).toFixed(2)+')');
      jawRight.setAttribute('transform','translate(0 '+(-r*13).toFixed(2)+')');
      lift.setAttribute('transform','translate(0 '+(-s.lift_mm/175*104).toFixed(2)+')');
      sideHolder.setAttribute('transform','translate(373 140) rotate('+(-s.holder_deg).toFixed(2)+')');
      sideJaw.setAttribute('transform','scale(1 '+(1-.35*r).toFixed(2)+')');
      if(map.g && map.g.isConnected){
        map.tray.setAttribute('transform','rotate('+s.tray_deg.toFixed(2)+')');
        map.holder.setAttribute('transform','rotate('+(-s.holder_deg).toFixed(2)+')');
        map.lift.setAttribute('transform','translate(0 '+(-s.lift_mm/175*5).toFixed(2)+')');
        map.left.setAttribute('transform','translate(0 '+(r*3).toFixed(2)+')');
        map.right.setAttribute('transform','translate(0 '+(-r*3).toFixed(2)+')');
      }
    }
    let prev=null;
    function tick(now){
      if(!Number.isFinite(now))return;
      const seconds=prev===null?0:Math.max(0,Math.min(.05,(now-prev)/1000));
      prev=now;
      if(document.hidden)return;
      advance(model,seconds);draw();
    }
    function attachMap(car){
      if(map.g&&map.g.isConnected)map.g.remove();
      const g=svgNode('g',car,{'data-km-mechanism':'MOCK','pointer-events':'none'});
      svgNode('rect',g,{x:-13,y:-19,width:26,height:38,rx:4,fill:'#7047af',stroke:'#e9cfff','stroke-width':1.5});
      for(const [x,y] of [[-17,-15],[-17,7],[13,-15],[13,7]])
        svgNode('rect',g,{x,y,width:4,height:9,rx:1,fill:'#b88b56',stroke:'#f9ce96','stroke-width':.6});
      svgNode('path',g,{d:'M -4 -18 V -30',stroke:'#e0d5f6','stroke-width':3});
      const carLift=svgNode('g',g,{});
      svgNode('rect',carLift,{x:-8,y:-24,width:8,height:11,rx:2,fill:'#b6d8e6',stroke:'#fff','stroke-width':.7});
      const carHolder=svgNode('g',carLift,{});
      svgNode('path',carHolder,{d:'M -4 -23 V -36',stroke:'#f4dfff','stroke-width':2.8});
      const left=svgNode('path',carHolder,{d:'M -4 -36 L -12 -38 L -13 -32',fill:'none',stroke:'#fff','stroke-width':2});
      const right=svgNode('path',carHolder,{d:'M -4 -36 L 4 -38 L 5 -32',fill:'none',stroke:'#fff','stroke-width':2});
      const disk=svgNode('g',g,{transform:'translate(7 10)'});
      svgNode('circle',disk,{r:10,fill:'#1c2b48',stroke:'#e6d4ff','stroke-width':1});
      const carTray=svgNode('g',disk,{});
      for(let i=0;i<3;i++){
        const a=i*Math.PI*2/3-Math.PI/2;
        svgNode('circle',carTray,{cx:(6*Math.cos(a)).toFixed(2),
          cy:(6*Math.sin(a)).toFixed(2),r:3.5,
          fill:['#ff9d96','#8ad8f4','#f1c27f'][i],stroke:'#fff','stroke-width':.6});
      }
      Object.assign(map,{g,tray:carTray,lift:carLift,holder:carHolder,left,right});
      draw();
    }
    syncTargets();draw();
    return Object.freeze({root:panel,model,tick,attachMap});
  }
  return Object.freeze({AXES,PRESETS,clamp,fresh,setTarget,advance,reset,create});
});
