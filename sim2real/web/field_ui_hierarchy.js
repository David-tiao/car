/* Promote the existing interactive 2.4 m field to the first viewport.
 * Move DOM nodes; do not clone them or change map logic / element IDs.
 */
(() => {
  'use strict';
  function mount() {
    const map=document.querySelector('.maincol > .map-panel');
    const nav=document.getElementById('carModeSwitch');
    if (!map || !nav) return;
    map.classList.add('field-hero-panel');
    nav.after(map);
    const stage=map.querySelector('.map-stage');
    if(stage && !map.querySelector('.field-stage-layout')){
      const body=document.createElement('div');
      body.className='field-stage-layout';
      stage.before(body);
      body.append(stage);
      if(window.CarWebMode?.mode==='test'){
        const slot=document.createElement('div');
        slot.id='fieldWorkbenchSlot';
        body.append(slot);
      }
    }
    // Keep existing map event listeners bound by app.js: append moves the node,
    // preserving the active SVG, stage controls, physics and references.
    const existing=map.querySelector('.map-panel-heading-badge');
    if(!existing){
      const subtitle=document.createElement('div');
      subtitle.className='map-panel-heading-badge';
      subtitle.textContent='赛场总览 · 先看地图，再进行通讯与任务实验';
      map.querySelector('.panel-heading')?.prepend(subtitle);
    }
  }
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',mount);
  else mount();
})();
