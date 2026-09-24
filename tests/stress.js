#!/usr/bin/env node
// Headless stress test for sim-3d's navigation core.
//
// Extracts the nav core from sim-3d/rover-sim-3d.html (everything between
// "use strict" and the @@END_NAV_CORE@@ marker — pure logic, no DOM or
// Three.js), runs it in a Node vm with a seeded Math.random, and drives
// simStep() at 60 Hz for many randomized trials.
//
// Usage: node tests/stress.js [trials=200] [maxSimSeconds=200] [scenario=all]
//   scenario: default | random | multistop (4 stops) | multistop7 | grover | qaoa | all

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const HTML = process.env.SIM_HTML || path.join(__dirname, '..', 'sim-3d', 'rover-sim-3d.html');
const [TRIALS = 200, MAX_T = 200, SCENARIO = 'all'] = process.argv.slice(2).map((a, i) => (i < 2 ? Number(a) : a));

function extractCore() {
  const html = fs.readFileSync(HTML, 'utf8');
  const start = html.indexOf('"use strict";');
  const end = html.indexOf('// @@END_NAV_CORE@@');
  if (start < 0 || end < 0) throw new Error('nav core markers not found in ' + HTML);
  return html.slice(start + '"use strict";'.length, end);
}

// mulberry32 — small, fast, good enough for reproducible trials
function seeded(seed) {
  return function () {
    seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const DRIVER = `
  let __minCharge = 100;
  function missionInfo() {
    return { recharges: mission.legs.filter(l => l.kind === 'base' && !l.final).length,
             rtbs: mission.rtbs || 0, minCharge: __minCharge };
  }
  return {
    // Embedded quantum-chosen orders must still be optimal under this
    // planner — fails if the map or cost constants changed without
    // re-running the Qiskit scripts (the stale-embed trap the README warns about).
    quantumChecks() {
      return [['grover', GROVER_DELIVERY], ['qaoa', QAOA_DELIVERY]].map(([name, Q]) => {
        const dist = distanceMatrix([BASE, ...Q.stops]);
        const best = walkTour(bestOrder(dist, Q.stops.length, 100).order, dist, 100).time;
        const got = walkTour(Q.order, dist, 100).time;
        return { name, best, got, ok: got <= best * 1.01 };
      });
    },
    run(maxT, scenario) {
      reset();
      if (scenario === 'random') pickRandomDestination();
      if (scenario === 'grover') planMission(GROVER_DELIVERY.stops.map(p => ({ ...p })), { order: GROVER_DELIVERY.order.slice(), method: 'grover' });
      if (scenario === 'qaoa') planMission(QAOA_DELIVERY.stops.map(p => ({ ...p })), { order: QAOA_DELIVERY.order.slice(), method: 'qaoa' });
      if (scenario.startsWith('multistop')) {
        const stops = [], n = Number(scenario.slice(9)) || 4;
        for (let i = 0; i < n; i++) stops.push(randomFreePoint());
        planMission(stops);
      }
      const FRAME = 1 / 60;
      let t = 0;
      while (t < maxT) {
        simStep(FRAME); t += FRAME;
        if (!Number.isFinite(rover.x) || !Number.isFinite(rover.y) || !Number.isFinite(rover.heading))
          return { ok: false, nonFinite: true, t };
        __minCharge = Math.min(__minCharge, battery.charge);
        if (missionComplete) {
          const allDelivered = mission.kind !== 'delivery' || mission.stops.every(s => s.delivered);
          return { ok: allDelivered, t, stats: { ...stats }, m: missionInfo() };
        }
        if (battery.depleted) return { ok: false, depleted: true, t, stats: { ...stats }, m: missionInfo() };
      }
      return { ok: false, timeout: true, t, stats: { ...stats }, m: missionInfo() };
    }
  };
`;

function makeSim(seed) {
  const ctx = vm.createContext({ console });
  vm.runInContext(`Math.random = (${seeded.toString()})(${seed});`, ctx);
  return vm.runInContext(`(function(){ "use strict"; ${extractCore()} ${DRIVER} })()`, ctx);
}

function runScenario(name) {
  let ok = 0, timeouts = 0, nonFinite = 0, depleted = 0, hits = 0, bumps = 0, timeSum = 0, recharges = 0, rtbs = 0, minCharge = 100;
  const started = Date.now();
  for (let i = 0; i < TRIALS; i++) {
    const r = makeSim(1000 + i).run(MAX_T, name);
    if (r.ok) { ok++; timeSum += r.t; }
    if (r.timeout) timeouts++;
    if (r.nonFinite) nonFinite++;
    if (r.depleted) depleted++;
    if (r.stats) { hits += r.stats.hits; bumps += r.stats.collisions; }
    if (r.m) { recharges += r.m.recharges; rtbs += r.m.rtbs; minCharge = Math.min(minCharge, r.m.minCharge); }
  }
  const pct = (n) => ((n / TRIALS) * 100).toFixed(1) + '%';
  console.log(`[${name}] ${TRIALS} trials, cap ${MAX_T}s  (${((Date.now() - started) / 1000).toFixed(1)}s wall)`);
  console.log(`  success   ${ok}/${TRIALS} (${pct(ok)})   mean time ${(timeSum / Math.max(1, ok)).toFixed(1)}s`);
  console.log(`  timeouts  ${timeouts}   depleted ${depleted}   non-finite ${nonFinite}`);
  console.log(`  threat hits/trial ${(hits / TRIALS).toFixed(2)}   wall bumps/trial ${(bumps / TRIALS).toFixed(2)}`);
  console.log(`  planned recharges/trial ${(recharges / TRIALS).toFixed(2)}   live RTB diversions ${rtbs}   lowest battery seen ${minCharge.toFixed(1)}%`);
  return { ok, nonFinite, depleted };
}

const scenarios = SCENARIO === 'all' ? ['default', 'random', 'multistop', 'multistop7', 'grover', 'qaoa'] : [SCENARIO];
let failed = false;
for (const c of makeSim(1).quantumChecks()) {
  console.log(`[${c.name}-check] embedded order ${c.got.toFixed(2)}s vs planner optimum ${c.best.toFixed(2)}s -> ${c.ok ? 'OK' : 'STALE: re-run qiskit/' + c.name + '_delivery_order.py'}`);
  if (!c.ok) failed = true;
}
for (const s of scenarios) {
  const r = runScenario(s);
  if (r.nonFinite > 0 || r.depleted > 0) failed = true;
}
process.exit(failed ? 1 : 0);
