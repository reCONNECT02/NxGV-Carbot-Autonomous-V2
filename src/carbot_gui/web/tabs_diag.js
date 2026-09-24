/* tabs_diag.js — table-style diagnostic tabs (phase 7). Uses H, ImgLoop, camTile from tabs.js. */
'use strict';

/* ============================================================ DETECTIONS */
TABS.det = {
  rate: c => Math.max(1, c.rates.image),
  create(el, ctx) {
    el.innerHTML = H.head('Detections', 'BPU model output and debounced states', '', ctx) +
      `<div class="grid g-side"><div class="panel">${camTile('Detector debug · boxes drawn by bpu_detector')}</div>
      <div class="side" data-k="side"></div></div>
      <div class="panel" style="margin-top:14px"><h3>Gate vs route mismatch log <small>logged and shown, the route is never changed</small></h3><div class="scroll" data-k="mm"></div></div>`;
    const q = k => el.querySelector(`[data-k="${k}"]`), tile = el.querySelector('.cam');
    const loop = ImgLoop(tile.querySelector('img'), () => 'det', ctx.cfg.rates.image, ok => { tile.querySelector('.none').style.display = ok ? 'none' : ''; });
    return {
      update(d) {
        const x = d.det;
        const cls = s => s === 'RED' || s === 'CLOSED' ? 'bad-t' : s === 'GREEN' || s === 'OPEN' ? 'ok-t' : 'muted';
        q('side').innerHTML = H.panel('Debounced states', x ? H.kv([['Traffic light', x.light, cls(x.light)], ['Boom gate', x.gate, cls(x.gate)],
          ['Speed bump sign', x.bump ? 'seen' : 'no'], ['Inference', D.f(x.ms, 1) + ' ms']]) : '<div class="empty">No /carbot/detections: is bpu_detector running?</div>') +
          H.panel('This frame', x && x.items.length ? `<table><tr><th>Class</th><th class="r">Conf.</th><th>Camera</th><th class="r">Dist.</th></tr>` +
            x.items.map(i => `<tr><td>${D.esc(i.cls)}</td><td class="r">${D.f(i.conf)}</td><td>${D.esc(i.cam)}</td><td class="r">${i.dist >= 0 ? D.f(i.dist) + ' m' : '—'}</td></tr>`).join('') + '</table>'
            : '<div class="muted">Nothing detected</div>');
        const mm = d.mismatch || [];
        q('mm').innerHTML = mm.length ? `<table><tr><th>Time</th><th>Visit</th><th>Gate seen</th><th class="r">Conf.</th><th>Planned exit</th><th>Note</th></tr>` +
          mm.slice().reverse().map(m => `<tr><td>${D.f(m.t, 1)} s</td><td>${m.visit}</td><td class="warn-t">${D.esc(m.gate)}</td><td class="r">${D.f(m.conf)}</td><td>${D.esc(m.planned)}</td><td class="muted">${D.esc(m.note || 'kept route')}</td></tr>`).join('') + '</table>'
          : '<div class="muted">No mismatch this run</div>';
      },
      destroy() { loop.destroy(); },
    };
  },
};

/* ============================================================ CONTROL + SAFETY */
TABS.control = {
  rate: c => Math.max(2, c.rates.cand),
  create(el, ctx) {
    el.innerHTML = H.head('Control + safety', 'What was asked, what was sent, and who vetoed', '', ctx) +
      `<div class="grid g3" style="margin-bottom:14px" data-k="cards"></div>
      <div class="grid g2"><div class="panel"><h3>Speed <small>m/s</small></h3><div class="cv" data-k="spd"></div>
        <div class="legend"><span><i style="background:var(--muted)"></i>Requested</span><span><i style="background:var(--lane)"></i>Commanded</span><span><i style="background:var(--ok)"></i>Measured</span></div></div>
      <div class="panel"><h3>Steering <small>rad, + = left</small></h3><div class="cv" data-k="str"></div>
        <div class="legend"><span><i style="background:var(--muted)"></i>Requested</span><span><i style="background:var(--lane)"></i>Commanded</span></div></div></div>
      <div class="grid g-side" style="margin-top:14px"><div class="panel"><h3>Safety checks <small>the first failing check vetoes motion</small></h3><div class="scroll" data-k="checks"></div></div>
      <div class="side"><div class="panel"><h3>Requests <small>age of each /carbot/request/*</small></h3><div data-k="req"></div></div></div></div>`;
    const q = k => el.querySelector(`[data-k="${k}"]`);
    let d = null;
    const oS = D.canvas(q('spd'), 600 / 220, () => draw()), oT = D.canvas(q('str'), 600 / 220, () => draw());
    function draw() {
      if (!d) return;
      const hs = d.history_s || 12, h = d.hist || [];
      const col = i => h.map(r => [r[0], r[i]]);
      D.chart(oS, [{ pts: col(1), color: D.css('--muted') }, { pts: col(2), color: D.css('--lane'), width: 2.5 }, { pts: col(3), color: D.css('--ok') }], -0.2, 0.8, hs, 0);
      D.chart(oT, [{ pts: col(4), color: D.css('--muted') }, { pts: col(5), color: D.css('--lane'), width: 2.5 }], -0.5, 0.5, hs, 0);
    }
    return {
      update(data) {
        d = data; const o = d.owner || {}, s = d.safety;
        const bad = ['SAFETY_STOP', 'WATCHDOG'].includes(o.winner);
        q('cards').innerHTML = H.panel('Command owner', H.kv([['Mission source', D.esc(o.source || '—')], ['Winner', D.esc(o.winner || '—'), bad ? 'bad-t' : 'ok-t'],
          ['Reason', D.esc(o.reason || '—')], ['Request age', o.req_age != null && o.req_age >= 0 ? D.f(o.req_age * 1000, 0) + ' ms' : '—']])) +
          H.panel('Output to servo_controller', H.kv([['linear.x (duty)', D.f(o.lin, 3)], ['angular.z (norm.)', D.f(o.ang, 3)],
            ['Commanded', `${D.f(o.cmd_v)} m/s · ${D.f(o.cmd_s, 3)} rad`], ['Measured', D.f(o.meas) + ' m/s']])) +
          H.panel('Watchdog', H.kv([['State', o.watchdog == null ? '—' : o.watchdog ? 'OK' : 'TRIPPED', H.okc(o.watchdog)], ['Armed', o.armed ? 'yes' : 'no']]));
        q('checks').innerHTML = s ? `<table><tr><th>Check</th><th>Status</th><th class="r">Value</th><th class="r">Limit</th><th>Detail</th></tr>` +
          s.checks.map(c => `<tr ${c.name === s.veto ? 'style="background:var(--bad-bg)"' : ''}><td>${c.name === s.veto ? '<b>' + D.esc(c.name) + '</b>' : D.esc(c.name)}</td>` +
            `<td class="${c.ok ? 'ok-t' : 'bad-t'}">${c.ok ? 'ok' : c.name === s.veto ? 'VETO' : 'fail'}</td><td class="r">${D.f(c.value, 3)}</td><td class="r">${D.f(c.limit, 3)}</td><td class="muted">${D.esc(c.detail)}</td></tr>`).join('') + '</table>' +
          (s.allowed ? '' : `<p class="bad-t" style="margin:10px 0 0">Motion vetoed by ${D.esc(s.veto)}: ${D.esc(s.reason)}</p>`) : '<div class="empty">No /carbot/safety/status: safety_monitor silent (the owner stops the car)</div>';
        q('req').innerHTML = `<table><tr><th>Source</th><th class="r">Age</th><th class="r">m/s</th><th class="r">rad</th></tr>` +
          Object.entries(d.requests || {}).map(([k, r]) => `<tr ${k === o.source ? 'class="cur"' : ''}><td>${k}</td><td class="r ${r && r.age < 0.3 ? 'ok-t' : 'muted'}">${r ? D.f(r.age) + ' s' : 'never'}</td><td class="r">${r ? D.f(r.v) : '—'}</td><td class="r">${r ? D.f(r.s, 3) : '—'}</td></tr>`).join('') + '</table>';
        draw();
      },
    };
  },
};

/* ============================================================ SYSTEM HEALTH */
TABS.health = {
  rate: c => c.rates.health,
  create(el, ctx) {
    el.innerHTML = H.head('System health', 'Topics, compute, power, processes and every node', '', ctx) + '<div data-k="body"></div>';
    const q = k => el.querySelector(`[data-k="${k}"]`);
    const bar = (v, warn) => `<div class="bar" style="margin:4px 0 10px"><i class="${v >= warn ? 'warn' : ''}" style="width:${Math.min(100, v || 0)}%"></i></div>`;
    return {
      update(d) {
        const s = d.sys, levels = ['OK', 'WARN', 'ERROR', 'STUB'];
        let h = '';
        if (s) {
          h += `<div class="grid g3" style="margin-bottom:14px">` +
            H.panel('CPU per core', s.cores.map((v, i) => `<div class="kv"><span>Core ${i}</span><b>${D.f(v, 0)} %</b></div>${bar(v, 85)}`).join('') || '—') +
            H.panel('Compute', `<div class="kv"><span>CPU total</span><b>${D.f(s.cpu, 0)} %</b></div>${bar(s.cpu, 85)}<div class="kv"><span>BPU</span><b>${D.f(s.bpu, 0)} %</b></div>${bar(s.bpu, 90)}` +
              `<div class="kv"><span>RAM</span><b>${D.f(s.ram, 0)} %</b></div>${bar(s.ram, 85)}<div class="kv"><span>SoC temperature</span><b class="${s.temp > 80 ? 'bad-t' : s.temp > 70 ? 'warn-t' : ''}">${D.f(s.temp, 1)} °C</b></div>${bar(s.temp, 75)}`) +
            H.panel('Power and links', H.kv([['Battery', D.f(s.battery) + ' V', H.okc(s.battery_ok)], ['micro-ROS agent', s.agent ? 'running' : 'NOT running', H.okc(s.agent)]]) +
              `<h3 style="margin-top:12px">Camera processes</h3><table><tr><th>Process</th><th class="r">PID</th></tr>${s.procs.map(([n, p]) => `<tr><td>${D.esc(n)}</td><td class="r">${p}</td></tr>`).join('')}</table>` +
              (() => { const c = {}; s.procs.forEach(([n]) => { c[n] = (c[n] || 0) + 1; }); const dup = Object.entries(c).filter(([n, k]) => k > 1);
                return dup.length ? '<p class="bad-t" style="margin:8px 0 0">Duplicate camera processes: stale launch, restart the stack</p>' : '<p class="ok-t" style="margin:8px 0 0">No duplicates</p>'; })()) + '</div>';
          h += H.panel('Topics', `<div class="scroll"><table><tr><th>Topic</th><th class="r">Hz</th><th class="r">Expected</th><th class="r">Age</th><th class="r">Latency</th></tr>` +
            s.topics.map(t => `<tr><td>${D.esc(t.topic)}</td><td class="r ${t.ok ? 'ok-t' : 'bad-t'}">${D.f(t.hz, 1)}</td><td class="r">${D.f(t.expected, 0)}</td><td class="r">${t.age >= 0 ? D.f(t.age) + ' s' : 'never'}</td><td class="r">${t.latency >= 0 ? t.latency + ' ms' : '—'}</td></tr>`).join('') + '</table></div>');
        } else h += '<div class="panel empty" style="margin-bottom:14px">No /carbot/system/health: is system_monitor running?</div>';
        h += `<div style="margin-top:14px">` + H.panel('Nodes', `<div class="scroll"><table><tr><th>Node</th><th>Block</th><th>Level</th><th>State</th><th>Message</th><th class="r">Oldest input</th><th>Never received</th></tr>` +
          d.nodes.map(n => `<tr><td>${D.esc(n.node)}</td><td>${D.esc(n.block || '—')}</td><td class="${n.stale ? 'bad-t' : ['ok-t', 'warn-t', 'bad-t', 'muted'][n.level]}">${n.stale ? 'SILENT' : levels[n.level]}</td>` +
            `<td>${D.esc(n.state)}</td><td class="muted">${D.esc(n.detail)}</td><td class="r">${n.oldest_input_s >= 0 ? D.f(n.oldest_input_s) + ' s' : '—'}</td><td class="${n.never.length ? 'warn-t' : 'muted'}">${D.esc(n.never.join(', ') || '—')}</td></tr>`).join('') +
          '</table></div>') + '</div>';
        q('body').innerHTML = h;
      },
    };
  },
};

/* ============================================================ EVENTS + LOG */
TABS.events = {
  rate: () => 1,
  create(el, ctx) {
    let filter = 'all', seq = 0;
    const all = [];
    el.innerHTML = H.head('Events + log', 'What happened, in order (newest first)', H.seg('f', [['all', 'All'], ['mission', 'Mission'], ['safety', 'Safety'], ['warn', 'Warnings']], filter), ctx) +
      `<div class="grid g2"><div class="panel"><h3>Timeline <small>run time, s</small></h3><div class="tl" data-k="tl"></div></div>
      <div class="panel"><h3>Node log <small>WARN and above from /rosout</small></h3><div class="scroll" data-k="log"></div></div></div>`;
    const q = k => el.querySelector(`[data-k="${k}"]`);
    const render = () => {
      const keep = e => filter === 'all' || (filter === 'mission' && ['mission', 'ops'].includes(e.kind)) ||
        (filter === 'safety' && ['safety', 'owner', 'manual'].includes(e.kind)) || (filter === 'warn' && ['warn', 'bad'].includes(e.level));
      const tl = all.filter(e => e.kind !== 'log' && keep(e)).slice(-150).reverse();
      q('tl').innerHTML = tl.length ? tl.map(e => `<div class="ev ${e.level}"><time>${D.f(e.t, 1)}</time><b>${D.esc(e.source)}</b> ${D.esc(e.text)}</div>`).join('') : '<div class="muted">No events yet</div>';
      const lg = all.filter(e => e.kind === 'log' && keep(e)).slice(-100).reverse();
      q('log').innerHTML = lg.length ? `<table><tr><th>t</th><th>Level</th><th>Node</th><th>Message</th></tr>${lg.map(e => `<tr><td>${D.f(e.t, 1)}</td><td class="${e.level === 'bad' ? 'bad-t' : 'warn-t'}">${e.level === 'bad' ? 'ERROR' : 'WARN'}</td><td>${D.esc(e.source)}</td><td style="white-space:normal">${D.esc(e.text)}</td></tr>`).join('')}</table>` : '<div class="muted">Nothing above INFO</div>';
    };
    H.wireSeg(el, 'f', v => { filter = v; render(); });
    return {
      query: () => `since=${seq}`,
      update(d) {
        (d.events || []).forEach(e => { if (e.seq > seq) { all.push(e); seq = e.seq; } });
        if (all.length > 600) all.splice(0, all.length - 600);
        render();
      },
    };
  },
};

/* CALIBRATION overview + step pages: tabs_calib.js (phase 8) */

/* ============================================================ TUNING (calibrate only) */
TABS.tuning = {
  poll: false,
  rate: () => 0.5,
  create(el, ctx) {
    let rows = [], file = null, node = null, session = '';
    el.innerHTML = H.head('Tuning', 'Apply live, then save into the active calibration session', '', ctx) +
      `<div class="grid g-side editable"><div class="panel" data-k="tbl"><div class="empty">Loading parameters…</div></div>
      <div class="side"><div class="panel"><h3>Files</h3><div class="filelist" data-k="files"></div></div>
      <div class="panel"><h3>Session</h3><div class="muted" data-k="sess"></div></div></div></div>`;
    const q = k => el.querySelector(`[data-k="${k}"]`);
    async function load() {
      const r = await ctx.api('/api/params');
      if (!r || r.error) { q('tbl').innerHTML = `<div class="empty">${D.esc(r ? r.error : 'No answer')}</div>`; return; }
      rows = r.rows; session = r.session;
      q('sess').textContent = session || 'No session loaded: Save is refused until a calibration session exists.';
      const files = [...new Set(rows.map(x => x.file))];
      file = file || files[0];
      q('files').innerHTML = files.map(f => `<button data-f="${f}" aria-current="${f === file}">${f}</button>`).join('');
      q('files').querySelectorAll('button').forEach(b => b.addEventListener('click', () => { file = b.dataset.f; node = null; renderFile(); load(); }));
      renderFile();
    }
    async function renderFile() {
      const nodes = [...new Set(rows.filter(x => x.file === file).map(x => x.node))];
      node = node && nodes.includes(node) ? node : nodes[0];
      const mine = rows.filter(x => x.file === file && x.node === node);
      q('tbl').innerHTML = `<h3>${D.esc(file)} <small>${nodes.map(n => `<a href="#" data-n="${n}" style="${n === node ? 'font-weight:700' : ''}">${n}</a>`).join(' · ')}</small></h3>` +
        `<div class="scroll"><table><tr><th>Key</th><th class="r">Live</th><th class="r">Saved</th><th></th><th></th></tr>` +
        mine.map((x, i) => `<tr data-i="${i}"><td>${D.esc(x.key)}</td><td class="r"><input type="text" value="${D.esc(JSON.stringify(x.saved).replace(/^"|"$/g, ''))}" aria-label="${D.esc(x.key)}"></td>` +
          `<td class="r muted">${D.esc(JSON.stringify(x.saved))}${x.overlay ? ' ●' : ''}</td><td><button class="btn" data-a="set">Apply live</button></td><td><button class="btn primary" data-a="save">Save</button></td></tr>`).join('') +
        '</table></div><p class="muted" style="font-size:12px">● = value comes from this session\'s params_overlay.yaml. Nodes that read parameters only at start need a restart.</p>';
      q('tbl').querySelectorAll('[data-n]').forEach(a => a.addEventListener('click', e => { e.preventDefault(); node = a.dataset.n; renderFile(); }));
      q('tbl').querySelectorAll('tr[data-i]').forEach(tr => {
        const x = mine[+tr.dataset.i], inp = tr.querySelector('input');
        tr.querySelectorAll('button').forEach(b => b.addEventListener('click', async () => {
          const r = await ctx.api('/api/params/' + b.dataset.a, { node: x.node, key: x.key, value: inp.value, default: x.default });
          ctx.toast(r ? `${x.key}: ${r.message}` : 'No answer');
          tr.classList.toggle('diff', !!(r && r.ok && b.dataset.a === 'set'));
          if (r && r.ok && b.dataset.a === 'save') load();
        }));
      });
      const live = await ctx.api('/api/params/get', { node, keys: mine.map(x => x.key) });
      if (live && live.ok) q('tbl').querySelectorAll('tr[data-i]').forEach(tr => {
        const x = mine[+tr.dataset.i], v = live.values[x.key];
        if (v !== undefined && v !== null) { tr.querySelector('input').value = typeof v === 'string' ? v : JSON.stringify(v); tr.classList.toggle('diff', JSON.stringify(v) !== JSON.stringify(x.saved)); }
      });
      else if (live) ctx.toast(live.message || `${node} not running`);
    }
    load();
    return { update() {} };
  },
};

/* ============================================================ STEERING TEST (calibrate only) */
/* Drive in circles at full lock, read the angle the steering servo is commanded to, then type the safe
   minimum / maximum. Servo angle: below the centre = LEFT, above the centre = RIGHT (servo_controller). */
TABS.steering = {
  rate: c => c.rates.core,
  create(el, ctx) {
    const NODE = 'servo_controller', KEYS = ['servo_center', 'servo_range_left', 'servo_range_right'];
    el.innerHTML = H.head('Steering test', 'Live steering servo angle, then set the minimum and maximum it may reach', '', ctx) +
      `<div class="grid g-side"><div>
        <div class="panel"><h3>Steering servo now</h3>
          <div style="font-size:56px;font-weight:700;line-height:1.1" data-k="now">—</div>
          <div class="muted" data-k="side" style="margin-bottom:10px"></div>
          <div data-k="gauge" style="position:relative;height:46px;background:rgba(127,127,127,.18);border-radius:8px;margin:6px 0 4px"></div>
          <div style="display:flex;justify-content:space-between;font-size:12px" class="muted"><span>0 (left)</span><span>90</span><span>180 (right)</span></div>
        </div>
        <div class="panel" style="margin-top:14px"><h3>Seen since reset</h3>
          <div class="kv"><span>Lowest angle (most LEFT)</span><b data-k="smin">—</b></div>
          <div class="kv"><span>Highest angle (most RIGHT)</span><b data-k="smax">—</b></div>
          <p style="margin:10px 0 0"><button class="btn" data-k="reset">Reset min / max</button></p>
          <p class="muted" style="font-size:12px;margin:8px 0 0">Hold full lock (drive a full circle each way), read the two numbers, and reset before trying again.</p>
        </div></div>
      <div class="side"><div class="panel"><h3>Allowed steering angle</h3>
        <p class="muted" style="font-size:12px;margin-top:0">Stops the servo from turning further, for the joystick and for automatic driving. If the belt slips at full lock, type a smaller range.</p>
        <table>
          <tr><td>Minimum angle (left limit)</td><td class="r"><input type="text" size="5" data-k="imin"></td></tr>
          <tr><td>Centre (straight ahead)</td><td class="r"><input type="text" size="5" data-k="icen"></td></tr>
          <tr><td>Maximum angle (right limit)</td><td class="r"><input type="text" size="5" data-k="imax"></td></tr>
        </table>
        <p style="margin:10px 0 4px"><button class="btn" data-k="apply">Apply live</button> <button class="btn primary" data-k="save">Save to session</button> <button class="btn" data-k="load">Reload current</button></p>
        <p class="muted" style="font-size:12px;margin:6px 0 0" data-k="msg">Apply live is lost at the next launch; Save writes it into the calibration session (params_overlay.yaml).</p>
      </div></div></div>`;
    const q = k => el.querySelector(`[data-k="${k}"]`);
    const num = k => Number(q(k).value);
    async function load() {
      const r = await ctx.api('/api/params/get', { node: NODE, keys: KEYS });
      if (!r || !r.ok) { q('msg').textContent = (r && r.message) || 'servo_controller not answering'; return; }
      const v = r.values;
      q('icen').value = v.servo_center; q('imin').value = v.servo_center - v.servo_range_left; q('imax').value = v.servo_center + v.servo_range_right;
      q('msg').textContent = 'Loaded the values servo_controller is using now.';
    }
    function plan() {
      const mn = num('imin'), c = num('icen'), mx = num('imax');
      if (![mn, c, mx].every(Number.isFinite) || !(0 <= mn && mn < c && c < mx && mx <= 180)) return null;
      return { servo_center: c, servo_range_left: c - mn, servo_range_right: mx - c };
    }
    async function push(action) {
      const p = plan();
      if (!p) { ctx.toast('Need 0 <= minimum < centre < maximum <= 180'); return; }
      const order = KEYS.slice(1).concat(['servo_center']);          // ranges first; the centre moves the wheels, so only when it changed
      const cur = await ctx.api('/api/params/get', { node: NODE, keys: KEYS });
      let bad = '';
      for (const k of order) {
        if (action === 'set' && k === 'servo_center' && cur && cur.ok && cur.values.servo_center === p.servo_center) continue;
        const r = await ctx.api('/api/params/' + action, { node: NODE, key: k, value: String(p[k]), default: 0 });
        if (!r || !r.ok) { bad = `${k}: ${(r && r.message) || 'no answer'}`; break; }
      }
      q('msg').textContent = bad || (action === 'set' ? `Applied live: ${num('imin')} .. ${num('imax')} (centre ${num('icen')}).` : 'Saved to the calibration session.');
      ctx.toast(q('msg').textContent);
    }
    q('apply').addEventListener('click', () => push('set'));
    q('save').addEventListener('click', () => push('save'));
    q('load').addEventListener('click', load);
    q('reset').addEventListener('click', async () => { await ctx.api('/api/steering/reset', {}); ctx.toast('Min / max reset'); });
    load();
    const pos = a => `${Math.max(0, Math.min(180, a)) / 180 * 100}%`;
    return {
      update(d) {
        const s = d.live;
        if (!s) { q('now').textContent = '—'; q('side').textContent = 'No /carbot/vehicle/steering: is servo_controller running (with the extension)?'; return; }
        const off = s.angle - s.center;
        q('now').textContent = `${s.angle}°`;
        q('side').textContent = off === 0 ? 'straight ahead' : `${Math.abs(off)}° ${off < 0 ? 'LEFT' : 'RIGHT'} of the centre (${s.center}°). Current limits ${s.limit_min}° .. ${s.limit_max}°`;
        q('smin').textContent = s.seen_min === null ? '—' : `${s.seen_min}°  (${s.center - s.seen_min}° left of centre)`;
        q('smax').textContent = s.seen_max === null ? '—' : `${s.seen_max}°  (${s.seen_max - s.center}° right of centre)`;
        const seen = s.seen_min === null ? '' : `<div style="position:absolute;top:30px;height:8px;background:#4c8bf5;opacity:.55;border-radius:4px;left:${pos(s.seen_min)};width:calc(${pos(s.seen_max)} - ${pos(s.seen_min)})"></div>`;
        q('gauge').innerHTML =
          `<div style="position:absolute;top:0;height:100%;left:${pos(s.limit_min)};width:calc(${pos(s.limit_max)} - ${pos(s.limit_min)});background:rgba(60,180,90,.22);border-radius:8px"></div>` +
          `<div style="position:absolute;top:0;height:100%;width:2px;background:#888;left:${pos(s.center)}"></div>` + seen +
          `<div style="position:absolute;top:2px;height:24px;width:6px;margin-left:-3px;background:#f5a623;border-radius:3px;left:${pos(s.angle)}"></div>`;
      },
    };
  },
};
