"""Carbot browser GUI server.

Phase 1: a status page that proves the skeleton is alive: mode, every node's
heartbeat (block, level, state, input ages), mission state, preflight / wizard
state, scoreboard. Phase 7 replaces the page with the main tab + 11 diagnostic
tabs by extending the base dashboard (risabot_automode/dashboard.py), with
lazy per-tab subscriptions and throttled, downscaled streams (gui.yaml).
"""
import http.server
import json
import socketserver
import threading
import time

import rclpy
from carbot_common import topics as T
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED
from carbot_interfaces.msg import (CalibrationState, MissionState, NodeStatus, PreflightReport,
                                   Scoreboard)
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool

LEVELS = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STUB'}

PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Carbot - %MODE%</title><style>
:root{--bg:#f4f6f8;--fg:#1b2430;--card:#fff;--line:#dde3ea;--blue:#1e6fff;--ok:#1f9d55;--warn:#d69e2e;--err:#e53e3e;--stub:#718096}
@media (prefers-color-scheme:dark){:root{--bg:#10151c;--fg:#e6edf5;--card:#18202a;--line:#2a3542}}
body{margin:0;font-family:system-ui,sans-serif;background:var(--bg);color:var(--fg)}
header{display:flex;gap:12px;align-items:center;padding:10px 16px;background:var(--card);border-bottom:1px solid var(--line)}
.mode{font-weight:700;padding:4px 10px;border-radius:6px;color:#fff;background:var(--blue)}
.race .mode{background:#b7791f}
main{padding:12px 16px;display:grid;gap:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px;overflow-x:auto}
table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:4px 6px;border-bottom:1px solid var(--line);text-align:left;white-space:nowrap}
.l0{color:var(--ok)}.l1{color:var(--warn)}.l2{color:var(--err)}.l3{color:var(--stub)}.stale{opacity:.45}
#estop{margin-left:auto;background:var(--err);color:#fff;border:0;border-radius:8px;padding:10px 14px;font-weight:700}
.banner{padding:8px 12px;border-radius:8px;background:#fff3cd;color:#5c4400}
</style></head><body class="%MODE%"><header><span class="mode">%MODE_UP%</span>
<b>Carbot V4</b><span id="summary"></span><button id="estop">%ESTOP%</button></header>
<main><div id="banner"></div>
<div class="card"><b>Mission</b><div id="mission">-</div></div>
<div class="card"><b id="opstitle">Ops</b><div id="ops">-</div></div>
<div class="card"><b>Nodes</b><table id="nodes"></table></div>
<div class="card"><b>Scoreboard</b><table id="score"></table></div></main>
<script>
const esc=s=>String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
document.getElementById('estop').onclick=()=>{if(confirm('%ESTOP%?'))fetch('/api/estop',{method:'POST'})};
async function tick(){try{const s=await (await fetch('/api/state')).json();
 const n=s.nodes;let ok=0;let rows='<tr><th>block</th><th>node</th><th>level</th><th>state</th><th>detail</th><th>inputs (age s)</th></tr>';
 for(const x of n){if(x.level==0||x.level==3)ok++;rows+=`<tr class="${x.stale?'stale':''}"><td>${esc(x.block)}</td><td>${esc(x.node)}</td><td class="l${x.level}">${esc(x.level_name)}${x.stale?' (stale)':''}</td><td>${esc(x.state)}</td><td>${esc(x.detail)}</td><td>${esc(x.inputs)}</td></tr>`}
 document.getElementById('nodes').innerHTML=rows;
 document.getElementById('summary').textContent=`${n.length} nodes, ${ok} alive without error`;
 const m=s.mission;document.getElementById('mission').textContent=m?`${m.mode} | source ${m.active_source} | challenge ${m.challenge_id} ${m.challenge_name}`:'no mission state';
 document.getElementById('banner').innerHTML=m&&m.banner?`<div class="banner">${esc(m.banner)}</div>`:'';
 document.getElementById('opstitle').textContent=s.mode=='race'?'Preflight':'Calibration wizard';
 document.getElementById('ops').innerHTML=esc(s.ops||'-');
 let sc='<tr><th>#</th><th>challenge</th><th>marks</th><th>state</th></tr>';for(const c of s.score)sc+=`<tr><td>${c.id}</td><td>${esc(c.name)}</td><td>${c.max}</td><td>${esc(c.state)}</td></tr>`;
 document.getElementById('score').innerHTML=sc;}catch(e){}}
setInterval(tick,1000);tick();
</script></body></html>"""


class GuiServer(CarbotNode):

    def __init__(self):
        super().__init__('gui_server', '', ['host', 'port', 'mode', 'race.estop_label'])
        self.mode = str(self.p('mode'))
        self.lock = threading.Lock()
        self.nodes = {}
        self.mission = None
        self.ops = ''
        self.score = []
        self.sub(NodeStatus, T.STATUS, self._status, 100)
        self.sub(MissionState, T.MISSION_STATE, self._mission, LATCHED)
        self.sub(Scoreboard, T.SCOREBOARD, self._score, LATCHED)
        if self.mode == 'race':
            self.sub(PreflightReport, T.RACE_PREFLIGHT, self._preflight, 10)
        else:
            self.sub(CalibrationState, T.CALIBRATION_STATE, self._calib, LATCHED)
        self.estop_pub = self.create_publisher(Bool, T.E_STOP, 10)
        self.set_status(3, 'STUB', f'phase-1 status page, mode={self.mode}')

    def _status(self, m):
        with self.lock:
            self.nodes[m.node] = (time.monotonic(), m)

    def _mission(self, m):
        with self.lock:
            self.mission = {'mode': m.mode, 'active_source': m.active_source,
                            'challenge_id': m.challenge_id, 'challenge_name': m.challenge_name,
                            'banner': m.banner}

    def _score(self, m):
        with self.lock:
            self.score = [{'id': c.id, 'name': c.name, 'max': c.max_marks, 'state': c.state}
                          for c in m.challenges]

    def _preflight(self, m):
        with self.lock:
            self.ops = m.summary

    def _calib(self, m):
        with self.lock:
            self.ops = ' | '.join(f'{s.index}. {s.title}: {s.status}' for s in m.steps)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            nodes = []
            for name, (t, m) in sorted(self.nodes.items(), key=lambda kv: (kv[1][1].block or 'zz', kv[0])):
                inputs = ', '.join(f'{tp}={a:.1f}' if a >= 0 else f'{tp}=never'
                                   for tp, a in zip(m.input_topics, m.input_age_s))
                nodes.append({'node': name, 'block': m.block, 'level': m.level,
                              'level_name': LEVELS.get(m.level, '?'), 'state': m.state,
                              'detail': m.detail, 'inputs': inputs, 'stale': now - t > 3.0})
            return {'mode': self.mode, 'nodes': nodes, 'mission': self.mission,
                    'ops': self.ops, 'score': self.score}

    def estop(self):
        self.get_logger().warn('E-STOP pressed in GUI (counts as manual intervention)')
        self.estop_pub.publish(Bool(data=True))


def serve(node: GuiServer):
    mode = node.mode
    label = str(node.p('race.estop_label')) if mode == 'race' else 'E-STOP'
    page = (PAGE.replace('%MODE_UP%', mode.upper()).replace('%MODE%', mode)
            .replace('%ESTOP%', label)).encode()

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.startswith('/api/state'):
                self._send(200, json.dumps(node.snapshot()).encode(), 'application/json')
            elif self.path == '/' or self.path.startswith('/?'):
                self._send(200, page, 'text/html; charset=utf-8')
            else:
                self._send(404, b'not found', 'text/plain')

        def do_POST(self):
            if self.path == '/api/estop':
                node.estop()
                self._send(200, b'{"ok":true}', 'application/json')
            else:
                self._send(404, b'not found', 'text/plain')

    class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    srv = Server((str(node.p('host')), int(node.p('port'))), Handler)
    node.get_logger().info(f'GUI ({mode}) at http://<robot_ip>:{int(node.p("port"))}/')
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def main(args=None):
    rclpy.init(args=args)
    node = GuiServer()
    srv = serve(node)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
