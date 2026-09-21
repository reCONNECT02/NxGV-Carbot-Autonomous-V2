// V4 guidance.js corridor/branchCheck + local-planner.js candidates on a scripted
// situation (ideal camera render, no memory). Input JSON on argv[2]:
//   {pose: {x,y,a}, index: route index, route: 0|1, speed, steer}
// Prints {corridor: {...}, candidates: [...]}. Used by test_v4_equivalence.py.
const path = require('path');
const ref = path.join(__dirname, '..', '..', 'docs', 'reference', 'v4_simulator');
for (const f of ['core.js', 'reeds-shepp.js', 'vehicle.js', 'recovery.js', 'guidance.js', 'local-planner.js'])
  require(path.join(ref, f));
const C = globalThis.CarbotCore;
const inp = JSON.parse(process.argv[2]);
const c = {...C.DEFAULTS};
const course = new C.Course(c);
const mission = C.buildMission(course, c);
const pose = inp.pose;
// ideal camera grid (same rule as carbot_planning.sim_core.render_grid)
const n = 100, res = .018, x0 = -.65, y0 = -.90, kind = new Uint8Array(n * n), grown = new Uint8Array(n * n);
for (let i = 0; i < n * n; i++) {
  const p = {x: x0 + (Math.floor(i / n) + .5) * res, y: y0 + ((i % n) + .5) * res}, w = C.toWorld(pose, p);
  const d = course.clearance(w.x, w.y);
  kind[i] = d >= 0 ? 1 : d >= -.10 ? 2 : 3; grown[i] = kind[i] === 1 ? 1 : 0;
}
const perception = {n, res, x0, y0, kind, grown, stamp: 10,
  support(p) { let x = Math.floor((p.y - y0) / res), y = Math.floor((p.x - x0) / res);
    if (x < 0 || y < 0 || x >= n || y >= n) return 0; return grown[y * n + x] ? 1 : kind[y * n + x] === 2 ? -1 : 0; }};
const sim = {t: 10, c, course, path: mission.routes[inp.route], ctrl: {index: inp.index},
  estimate: {pose, odom: {...pose}, transform: {x: 0, y: 0}, distance: 0, globalOffset: {x: 0, y: 0}, globalSigma: .01, visualRank: 1},
  cameraOdom: {...pose}, perception, memory: {query: () => null}, plant: {speed: inp.speed}, steeringEstimate: inp.steer, lidar: []};
const G = globalThis.CarbotGuidance;
sim.guidance = G.corridor(sim);
const branch = G.branchCheck(sim);
const cands = globalThis.CarbotLocal.candidates(sim);
process.stdout.write(JSON.stringify({
  corridor: {offset: sim.guidance.offset, observed: sim.guidance.observed, mode: sim.guidance.mode,
             guide: sim.guidance.points.map(p => [p.x, p.y, p.a]), centres: sim.guidance.centres.map(q => [q.x, q.y, q.width])},
  branch,
  candidates: cands.map(q => ({id: q.id, offset: q.offset, cost: q.cost, valid: q.valid, reject: q.reject,
                               seen: q.seen, steer: q.commandSteer, minClear: q.minClear, end: q.points.at(-1)}))}));
