/* Static regression checks for visual polish and HiDPI ROS2 UI.
 * No browser/ROS2/STM32 required. Source assertions are not screenshot tests.
 */
'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

const web = path.join(__dirname,'..','web');
const read = file => fs.readFileSync(path.join(web,file),'utf8');
const html = read('index.html');
const css = read('operator_polish.css');
const mapping = read('keyboard_mapping_panel.js');

function balancedCss(text) {
  // Strip comments first; braces in CSS url/string contents are not used by
  // this stylesheet, so a simple structural check catches truncation.
  const stripped = text.replace(/\/\*[\s\S]*?\*\//g,'');
  let depth=0;
  for(const ch of stripped) {
    if(ch === '{') depth++;
    if(ch === '}') depth--;
    assert.ok(depth>=0,'unexpected CSS closing brace');
  }
  assert.equal(depth,0,'unbalanced CSS rules');
}

balancedCss(css);
assert.match(html, /href="operator_polish\.css\?v=15\.0\.0"/);
assert.ok(
  html.indexOf('keyboard_mapping.css') < html.indexOf('operator_polish.css'),
  'polish CSS must override the previous keyboard styles'
);
assert.equal((html.match(/operator_polish\.css/g)||[]).length,1);
for(const selector of [
  '#carModeSwitch a[aria-current=page]',
  '.field-hero-panel .map-stage',
  '.keyboard-signal-chip[data-online="true"]',
  '.keyboard-key.pressed',
  '.keyboard-metric[data-metric="ros_observed"]',
  'button:focus-visible',
  'prefers-reduced-motion:reduce',
  'max-width:600px'
]) {
  assert.ok(css.includes(selector),'missing rendering selector: '+selector);
}
assert.match(mapping,/ctx\.setTransform\(ratio,0,0,ratio,0,0\)/);
assert.match(mapping,/Math\.min\(2,Math\.max\(1,window\.devicePixelRatio\|\|1\)\)/);
assert.match(mapping,/r\.online===true && r\.source===routeMode/);
assert.match(mapping,/signalChip\.dataset\.online=String\(receiptOnline\)/);
assert.match(mapping,/signalChip\.setAttribute\('aria-live','polite'\)/);
assert.match(mapping,/setAttribute\('aria-keyshortcuts',key\.toUpperCase\(\)\)/);
assert.ok(
  !/serial\.Serial\(|\/dev\/ttyUSB|create_publisher\(Twist,["']\/cmd_vel/.test(css),
  'presentation stylesheet must not include hardware command paths'
);
console.log('PASS: render stylesheet, responsive selectors, source-truth status and HiDPI canvas');
