// V4 reeds-shepp.js candidates + core.js parkingPlan on the V4 course. Input JSON on argv[2]:
//   {what: 'rs', start: {x,y,a}, goal: {x,y,a}, r, step}
//   {what: 'park', start: {x,y,a}}               (goal = course.parkGoal(), V4 defaults)
// Prints JSON. Used by src/carbot_planning/test/test_parking_v4.py.
const path = require('path');
const ref = path.join(__dirname, '..', '..', 'docs', 'reference', 'v4_simulator');
for (const f of ['core.js', 'reeds-shepp.js'])
  require(path.join(ref, f));
const C = globalThis.CarbotCore, RS = globalThis.CarbotRS;
const inp = JSON.parse(process.argv[2]);
const pts = p => p.map(q => [q.x, q.y, q.a, q.dir, q.k]);
if (inp.what === 'rs') {
  const out = RS.candidates(inp.start, inp.goal, inp.r, inp.step || .006);
  process.stdout.write(JSON.stringify(out.map(q => ({types: q.types, lengths: q.lengths, cost: q.cost,
    variant: q.variant, points: pts(q.path)}))));
} else if (inp.what === 'park') {
  const c = {...C.DEFAULTS}, course = new C.Course(c), goal = course.parkGoal();
  const r = C.parkingPlan(inp.start, goal, course, c);
  process.stdout.write(JSON.stringify({goal, stage: r.stage, reason: r.reason, cost: r.cost ?? null,
    types: r.types ?? '', n: r.path.length, points: pts(r.path.map(p => ({k: 0, ...p}))),
    evaluated: r.evaluated.length, valid: r.evaluated.filter(q => q.valid).length}));
}
