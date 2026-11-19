/* Run from sim2real: node tests/test_mode_router.js */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const code = fs.readFileSync(path.join(__dirname, '../web/mode_router.js'), 'utf8');
for (const mode of ['sim', 'test', 'real', 'keyboard']) {
  const events = {};
  const sandbox = {window: {}, location: {pathname: '/' + mode},
    document: {readyState: 'loading', addEventListener: (name, cb) => events[name] = cb}};
  vm.runInNewContext(code, sandbox);
  const config = sandbox.window.CarWebMode;
  assert.equal(config.mode, mode);
  assert.equal(config.channel, mode === 'keyboard' ? 'real' : mode);
  assert.equal(config.runSource, mode === 'sim' ? 'simulation' : 'live');
  assert.equal(config.telemetryUrl, mode === 'sim' ? null : `/api/${config.channel}/telemetry`);
  assert.equal(typeof events.DOMContentLoaded, 'function');
  console.log(`[PASS] ${mode}: ${config.channel} feed / ${config.runSource}`);
}
console.log('FOUR MODE ROUTER: PASS');
