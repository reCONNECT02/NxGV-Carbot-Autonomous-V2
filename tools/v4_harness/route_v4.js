// Runs V4 core.js buildMission in Node and prints the two routes as JSON.
// Used by src/carbot_planning/test/test_v4_equivalence.py (skipped without node).
const path = require('path');
const ref = path.join(__dirname, '..', '..', 'docs', 'reference', 'v4_simulator');
require(path.join(ref, 'core.js'));
const C = globalThis.CarbotCore;
const course = new C.Course(C.DEFAULTS);
const m = C.buildMission(course, C.DEFAULTS);
const out = m.routes.map(r => r.map(p => [p.x, p.y, p.a, p.dir || 1]));
process.stdout.write(JSON.stringify({routes: out, expanded: m.expanded}));
