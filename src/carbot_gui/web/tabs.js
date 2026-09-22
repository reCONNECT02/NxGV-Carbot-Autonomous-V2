/* tabs.js — Drive + map-like diagnostic tabs (phase 7). Each tab: create(el, ctx) -> {update(d, core), query(), destroy()}.
 * rate(cfg) = polls per second. All drawing is canvas (draw.js); no libraries. */
'use strict';
const TABS = {};
const H = {
  panel: (title, inner, small) => `<div class="panel"><h3>${title}${small ? ` <small>${small}</small>` : ''}</h3>${inner}</div>`,
  kv: rows => `<div class="kv">${rows.map(([k, v, cls]) => `<span>${k}</span><b class="${cls || ''}">${v}</b>`).join('')}</div>`,
  head: (title, sub, right, ctx) => `<div class="tabhead"><h1>${title}</h1><p>${sub}</p><div class="right">${right || ''}` +
    `${ctx && ctx.cfg.read_only ? '<span class="chip grey ro">Read-only</span>' : ''}</div></div>`,
  okc: v => v === true ? 'ok-t' : v === false ? 'bad-t' : 'muted',
  seg: (name, opts, cur) => `<div class="seg" role="group" data-seg="${name}">${opts.map(([v, t]) =>
    `<button data-v="${v}" aria-pressed="${v === cur}">${t}</button>`).join('')}</div>`,
  wireSeg: (el, name, cb) => el.querySelectorAll(`[data-seg="${name}"] button`).forEach(b => b.addEventListener('click', () => {
    b.parentElement.querySelectorAll('button').forEach(x => x.setAttribute('aria-pressed', String(x === b))); cb(b.dataset.v);
  })),
};

/* ------------------------------------------------------------ image loop (lazy JPEG pulls) */
function ImgLoop(img, keyFn, fps, onState) {
  let url = null, stop = false, h = null;
  const tick = async () => {
    if (stop) return;
    const key = keyFn();
    if (key && !document.hidden) {
      try {
        const r = await fetch(`/api/img/${key}`);
        if (r.status === 200) {
          const u = URL.createObjectURL(await r.blob());
          img.src = u; if (url) URL.revokeObjectURL(url); url = u; onState && onState(true);
        } else onState && onState(false);
      } catch (e) { onState && onState(false); }
    }
    h = setTimeout(tick, 1000 / Math.max(fps, 0.2));
  };
  tick();
  return { destroy() { stop = true; clearTimeout(h); if (url) URL.revokeObjectURL(url); } };
}
function camTile(label) {
  return `<div class="cam"><img alt="${label}"><span class="none">No image yet</span><span class="lbl">${label}</span></div>`;
}

/* ------------------------------------------------------------ shared track + route cache */
const Track = { fp: '', track: null, routeN: '', route: null, info: null,
  query() { return `have=${encodeURIComponent(this.fp)}&have_route=${encodeURIComponent(this.routeN)}`; },
  take(d) {
    if (d.track) { this.track = d.track; this.fp = d.track_fp; }
    if (d.route) { this.route = d.route; this.routeN = d.route_n; }
    if (d.info) this.info = d.info;
  },
};
function drawTrack(ctx, v, t) {
  if (!t) return;
  const lw = (t.lane_width_m || 0.38) * v.s;
  Object.values(t.areas || {}).forEach(a => {
    if (!a.poly || a.poly.length < 3) return;
    D.path(ctx, a.poly, p => v.X(p[0]), p => v.Y(p[1])); ctx.closePath();
    ctx.fillStyle = a.kind === 'tunnel' ? 'rgba(22,27,38,.16)' : 'rgba(29,99,224,.05)'; ctx.fill();
    ctx.setLineDash(a.kind === 'tunnel' ? [] : [4, 4]); ctx.strokeStyle = D.css('--muted'); ctx.lineWidth = 1; ctx.stroke(); ctx.setLineDash([]);
  });
  Object.values(t.sections || {}).forEach(p => D.line(ctx, p, q => v.X(q[0]), q => v.Y(q[1]), D.css('--road'), lw));
  if (t.ring) {
    ctx.beginPath(); ctx.arc(v.X(t.ring.x), v.Y(t.ring.y), t.ring.r * v.s, 0, 2 * Math.PI);
    ctx.strokeStyle = D.css('--road'); ctx.lineWidth = lw; ctx.stroke();
  }
  ctx.strokeStyle = D.css('--line'); ctx.lineWidth = 1;
  (t.paint || []).forEach(([x0, y0, x1, y1]) => { ctx.beginPath(); ctx.moveTo(v.X(x0), v.Y(y0)); ctx.lineTo(v.X(x1), v.Y(y1)); ctx.stroke(); });
}
function gridLines(ctx, v, b, step) {
  ctx.strokeStyle = D.css('--grid'); ctx.globalAlpha = .5; ctx.lineWidth = 1;
  for (let x = Math.floor(b[0]); x <= b[2]; x += step) { ctx.beginPath(); ctx.moveTo(v.X(x), v.Y(b[1])); ctx.lineTo(v.X(x), v.Y(b[3])); ctx.stroke(); }
  for (let y = Math.floor(b[1]); y <= b[3]; y += step) { ctx.beginPath(); ctx.moveTo(v.X(b[0]), v.Y(y)); ctx.lineTo(v.X(b[2]), v.Y(y)); ctx.stroke(); }
  ctx.globalAlpha = 1;
}
/* route drawn per leg: done grey, current blue, upcoming dashed; reverse orange */
function drawRoute(ctx, v, legs) {
  const r = Track.route, info = Track.info;
  if (!r || r.length < 2) return;
  const N = Number(Track.routeN) || r.length, sc = (r.length - 1) / Math.max(N - 1, 1);
  const cur = legs ? legs.leg : -1;
  const pieces = (info && info.pieces) || [{ leg: 0, start: 0, end: N - 1 }];
  pieces.forEach(p => {
    if (p.start < 0) return;
    const a = Math.round(p.start * sc), b = Math.round(p.end * sc), seg = r.slice(a, b + 1);
    const done = p.leg < cur, now = p.leg === cur;
    let run = [], dir = null;
    const flush = () => {
      if (run.length < 2) return;
      const color = dir < 0 ? '#E8871E' : done ? D.css('--muted') : D.css('--lane');
      ctx.globalAlpha = done ? .55 : now ? 1 : .55;
      D.line(ctx, run, q => v.X(q[0]), q => v.Y(q[1]), color, now ? 5 : 3, (!done && !now) ? [7, 7] : []);
      ctx.globalAlpha = 1;
    };
    seg.forEach(q => { const d = q[2] < 0 ? -1 : 1; if (dir !== null && d !== dir) { flush(); run = [run[run.length - 1]]; } dir = d; run.push(q); });
    flush();
  });
  // leg end points P0..Pn
  if (info && info.legs) {
    const pts = [];
    info.legs.forEach((lg, k) => {
      const ps = pieces.filter(p => p.leg === k && p.start >= 0);
      if (!ps.length) return;
      if (k === 0) pts.push(r[Math.round(ps[0].start * sc)]);
      pts.push(r[Math.min(r.length - 1, Math.round(ps[ps.length - 1].end * sc))]);
    });
    pts.forEach((q, k) => {
      if (!q) return;
      const done = cur >= 0 && k <= cur, next = k === cur + 1;
      D.dot(ctx, v.X(q[0]), v.Y(q[1]), 12, done ? D.css('--muted') : D.css('--raised'), next ? D.css('--lane') : D.css('--muted'));
      ctx.fillStyle = done ? D.css('--panel') : next ? D.css('--lane') : D.css('--muted');
      ctx.font = '700 11px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText('P' + k, v.X(q[0]), v.Y(q[1]) + 1);
    });
  }
  // planned roundabout exits
  const t = Track.track;
  if (info && info.visits && t && t.exits) {
    info.visits.forEach(vi => {
      const e = t.exits[vi.exit];
      if (!e || e.length < 2) return;
      const leg = pieces.find(p => p.index === vi.piece);
      const now = leg && leg.leg === cur;
      D.label(ctx, v.X(e[0]), v.Y(e[1]) - 16, `Visit ${vi.visit}: ${vi.label || vi.exit}`, now ? D.css('--lane') : D.css('--panel'), now ? '#fff' : D.css('--ink'));
    });
  }
}

/* ============================================================ DRIVE */
TABS.drive = {
  rate: c => Math.min(c.rates.core, 8),
  create(el, ctx) {
    const race = ctx.cfg.mode === 'race';
    el.innerHTML = `<div class="drive"><div class="scene"><div class="hud">
        <div class="speed"><b data-k="speed">0.00</b><span>m/s measured</span></div>
        <div class="maxring" title="Set max speed"><div><b data-k="max">—</b><br><span>max</span></div></div>
        <div class="zone" title="Active speed zone"><span data-k="zname">zone</span><b data-k="zmax">—</b></div>
        <div class="modechip"><span class="big" data-k="mode">—</span><div data-k="chal"></div></div></div>
        <div class="banner" data-k="banner" hidden><b></b><span></span></div></div>
      <div class="side">
        ${race ? `<div class="panel"><h3>Start line</h3><div data-k="pf"></div>
          <button class="startbtn" data-k="start" disabled>START</button>
          <p class="muted" data-k="pfnote" style="font-size:12px;margin:8px 0 0"></p></div>` : ''}
        <div class="panel"><h3>Mission</h3><div data-k="mis"></div><div class="progress" data-k="prog"></div></div>
        <div class="panel"><h3>In the scene</h3><div data-k="scene"></div></div>
      </div></div>`;
    const q = k => el.querySelector(`[data-k="${k}"]`);
    let d = null, c = null;
    const o = D.canvas(el.querySelector('.scene'), null, () => draw());
    if (race) q('start').addEventListener('click', async () => {
      q('start').disabled = true;
      const r = await ctx.api('/api/start', {}); ctx.toast(r ? r.message || (r.ok ? 'Started' : 'Refused') : 'No answer');
    });
    function project(w, h) {
      const f = w * 0.95, hy = h * 0.14, eye = 0.9, cam = 0.55, cx = w / 2;
      return (x, y) => { const z = x + eye; if (z < 0.25) return null; return [cx - f * y / z, hy + f * cam / z, f / z]; };
    }
    function draw() {
      const { ctx: g, w, h } = o; g.clearRect(0, 0, w, h);
      g.fillStyle = D.css('--scene'); g.fillRect(0, 0, w, h);
      const P = project(w, h), lanes = d && d.lanes;
      const L = lanes && lanes.left.length > 1 ? lanes.left : [[0, 0.19], [3, 0.19]];
      const R = lanes && lanes.right.length > 1 ? lanes.right : [[0, -0.19], [3, -0.19]];
      const pl = L.map(p => P(p[0], p[1])).filter(Boolean), pr = R.map(p => P(p[0], p[1])).filter(Boolean);
      if (pl.length > 1 && pr.length > 1) {
        g.beginPath(); pl.forEach((p, i) => i ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1]));
        pr.slice().reverse().forEach(p => g.lineTo(p[0], p[1])); g.closePath(); g.fillStyle = D.css('--road'); g.fill();
        const locked = lanes && lanes.locked;
        [pl, pr].forEach(pp => {
          D.line(g, pp, p => p[0], p => p[1], locked ? D.css('--lane-soft') : D.css('--line'), 16);
          D.line(g, pp, p => p[0], p => p[1], locked ? D.css('--lane') : D.css('--muted'), 6, lanes ? [] : [10, 10]);
        });
      }
      if (d && d.path && d.path.length > 1) {
        const pp = d.path.map(p => P(p[0], p[1])).filter(Boolean);
        D.line(g, pp, p => p[0], p => p[1], D.css('--lane'), 3, [2, 10]);
      }
      const det = d && d.det;
      (det ? det.items : []).forEach(it => {
        if (!(it.px > 0)) return;
        const p = P(it.px, it.py); if (!p) return;
        const k = p[2] * 0.06, x = p[0], y = p[1];
        if (it.cls.startsWith('traffic_light')) {
          const red = it.cls.endsWith('red');
          g.fillStyle = D.css('--muted'); g.fillRect(x - k * .06, y - k * 2.2, k * .12, k * 2.2);
          g.fillStyle = '#20252D'; D.roundRect(g, x - k * .4, y - k * 3.4, k * .8, k * 1.3, k * .15); g.fill();
          D.dot(g, x, y - k * 3.05, k * .25, red ? '#FF4B4B' : '#4A2226'); D.dot(g, x, y - k * 2.45, k * .25, red ? '#1F3A2A' : '#3BE08A');
          D.label(g, x + k * .5, y - k * 2.8, `${red ? 'RED' : 'GREEN'} ${D.f(it.conf)}`, red ? D.css('--bad') : D.css('--ok'), '#fff', 'left');
        } else if (it.cls.startsWith('boom_gate')) {
          const open = it.cls.endsWith('open');
          g.save(); g.translate(x, y - k * .8); g.rotate(open ? -1.1 : 0);
          g.fillStyle = '#fff'; g.fillRect(0, -k * .08, k * 2, k * .16); g.fillStyle = '#E0373D';
          for (let i = 0; i < 4; i++) g.fillRect(k * (.25 + i * .45), -k * .08, k * .2, k * .16); g.restore();
          D.label(g, x, y + 14, open ? 'Gate OPEN' : 'Gate CLOSED', D.css('--raised'), open ? D.css('--ok') : D.css('--bad'));
        } else if (it.cls === 'speed_bump_sign') {
          g.fillStyle = D.css('--muted'); g.fillRect(x - k * .05, y - k * 1.6, k * .1, k * 1.6);
          g.beginPath(); g.moveTo(x, y - k * 2.5); g.lineTo(x + k * .5, y - k * 1.6); g.lineTo(x - k * .5, y - k * 1.6); g.closePath();
          g.fillStyle = '#fff'; g.fill(); g.lineWidth = Math.max(2, k * .08); g.strokeStyle = '#E0373D'; g.stroke();
          D.label(g, x, y + 14, `Bump sign ${D.f(it.conf)}`, D.css('--raised'), D.css('--ink'));
        }
      });
      (d && d.obstacles || []).forEach(ob => {
        const p = P(ob.x, ob.y); if (!p) return; const k = Math.min(p[2] * 0.05, 60);
        g.fillStyle = '#8B93A1'; g.fillRect(p[0] - k, p[1] - k * 1.2, 2 * k, k * 1.2);
        g.strokeStyle = ob.dist < 0.2 ? D.css('--bad') : 'transparent'; g.lineWidth = 3; g.strokeRect(p[0] - k, p[1] - k * 1.2, 2 * k, k * 1.2);
        D.label(g, p[0], p[1] - k * 1.2 - 12, `Obstacle ${D.f(ob.dist)} m`, D.css('--raised'), ob.dist < 0.2 ? D.css('--bad') : D.css('--muted'));
      });
      // our car, rear view
      const s = Math.min(w, h) / 600, cx = w / 2, cy = h - 80 * s;
      g.save(); g.translate(cx, cy); g.scale(s, s);
      g.fillStyle = 'rgba(0,0,0,.12)'; g.beginPath(); g.ellipse(0, 66, 92, 12, 0, 0, 2 * Math.PI); g.fill();
      g.fillStyle = D.css('--car'); g.fill(new Path2D('M-78 58 Q-84 10 -60 -8 L-44 -48 Q-40 -58 -28 -60 L28 -60 Q40 -58 44 -48 L60 -8 Q84 10 78 58 Q74 66 62 66 L-62 66 Q-74 66 -78 58 Z'));
      g.fillStyle = '#1A1F2A'; g.fill(new Path2D('M-40 -44 Q-36 -54 -24 -54 L24 -54 Q36 -54 40 -44 L50 -12 L-50 -12 Z'));
      g.fillStyle = '#E0373D'; g.fill(new Path2D('M-70 8 L-38 12 L-38 20 L-72 18 Z')); g.fill(new Path2D('M70 8 L38 12 L38 20 L72 18 Z'));
      g.fillStyle = D.css('--car-dark'); g.fillRect(-36, 12, 72, 8); g.restore();
    }
    return {
      update(data, core) {
        d = data; c = core || {};
        const m = c.mission || {}, o2 = c.owner || {};
        q('speed').textContent = D.f(o2.speed);
        q('max').textContent = D.f(m.set_max);
        q('zname').textContent = m.speed_zone || 'zone'; q('zmax').textContent = D.f(m.zone_max);
        const stop = o2.winner === 'SAFETY_STOP', mode = c.manual ? 'MANUAL' : stop ? 'SAFETY STOP' : (m.mode || '—');
        q('mode').textContent = mode; q('mode').classList.toggle('stop', stop || m.mode === 'HOLD');
        q('chal').textContent = m.challenge_id ? `Challenge ${m.challenge_id} · ${m.challenge_name}` + (m.hold_reason ? ` · ${m.hold_reason}` : '') : (m.hold_reason || '');
        const b = q('banner');
        b.hidden = !m.banner;
        if (m.banner) { b.querySelector('b').textContent = m.banner; b.querySelector('span').textContent = ['Info', 'Warning', 'Alarm'][m.banner_level] || '';
          b.style.boxShadow = m.banner_level ? `0 0 0 2px ${m.banner_level > 1 ? D.css('--bad') : D.css('--warn')}` : ''; }
        const lg = c.legs;
        q('mis').innerHTML = H.kv([['Hold', m.hold_reason || 'none', m.hold_reason ? 'bad-t' : ''],
          ['Leg', lg && lg.legs ? `${lg.leg + 1} of ${lg.legs} · ${D.f(lg.pct, 0)} %` : '—'],
          ['Piece', lg && lg.piece_no ? `${lg.piece_no} of ${lg.pieces} · ${lg.piece_kind}` : '—'],
          ['Next roundabout exit', m.next_exit || '—'], ['Lane lock', c.lane_locked ? 'Locked' : 'Not locked', c.lane_locked ? 'ok-t' : 'warn-t']]);
        q('prog').innerHTML = Array.from({ length: 13 }, (_, i) => `<i class="${i + 1 < m.challenge_id ? 'done' : i + 1 === m.challenge_id ? 'act' : ''}">${i + 1}</i>`).join('');
        const det = d && d.det;
        q('scene').innerHTML = H.kv([['Traffic light', det ? det.light : '—', det && det.light === 'RED' ? 'bad-t' : det && det.light === 'GREEN' ? 'ok-t' : 'muted'],
          ['Boom gate', det ? det.gate : '—', det && det.gate === 'OPEN' ? 'ok-t' : det && det.gate === 'CLOSED' ? 'warn-t' : 'muted'],
          ['Bump sign', det ? (det.bump ? 'seen' : 'no') : '—'], ['Nearest obstacle ahead', d && d.obstacles && d.obstacles.length ? D.f(d.obstacles[0].dist) + ' m' : 'none'],
          ['Safety', c.safety ? (c.safety.motion_allowed ? 'Motion allowed' : 'Veto: ' + c.safety.veto_check) : '—', c.safety ? H.okc(c.safety.motion_allowed) : 'muted']]);
        if (race) {
          const pf = c.preflight, names = ['Loading', 'Calibration missing', 'Preflight', 'Not ready', 'READY', 'Running', 'Finished', 'E-stopped'];
          if (pf) {
            const bad = pf.checks.filter(x => !x.ok);
            q('pf').innerHTML = `<div class="check"><span>State</span><span class="${pf.state === 4 || pf.state === 5 ? 'ok-t' : 'warn-t'}">${names[pf.state] || pf.state}</span></div>` +
              (pf.missing.length ? `<div class="check"><span>Missing calibration</span><span class="bad-t">${D.esc(pf.missing.join(', '))}</span></div>` : '') +
              `<div class="check"><span>Preflight</span><span class="${bad.length ? 'bad-t' : 'ok-t'}">${pf.checks.length - bad.length} of ${pf.checks.length} pass</span></div>` +
              bad.slice(0, 5).map(x => `<div class="check"><span>${D.esc(x.name)}</span><span class="bad-t">${D.esc(x.value)} (${D.esc(x.expected)})</span></div>`).join('');
            q('start').disabled = pf.state !== 4 || c.manual;
            q('pfnote').textContent = pf.state === 5 ? 'Running. No further input is accepted.' : pf.state === 4 ? 'Press START once. The car then runs on its own.' : (pf.summary || '');
          } else { q('pf').innerHTML = '<div class="empty">Waiting for race_supervisor</div>'; }
        }
        draw();
      },
      destroy() {},
    };
  },
};

/* ============================================================ GLOBAL MAP */
TABS.map = {
  rate: c => c.rates.map,
  create(el, ctx) {
    el.innerHTML = H.head('Global map', 'Mission legs, planned route, exits and live pose', '', ctx) +
      `<div class="panel" style="margin-bottom:14px"><div class="legs" data-k="legs"><span class="empty">Waiting for the route</span></div></div>
      <div class="grid g-side"><div class="panel"><div class="cv" data-k="cv"></div>
        <div class="legend"><span><i style="background:var(--muted);opacity:.6"></i>Leg done</span><span><i style="background:var(--lane)"></i>Current leg</span><span><i style="background:var(--lane);opacity:.5"></i>Upcoming (dashed)</span><span><i style="background:#E8871E"></i>Reverse</span><span><i style="background:var(--ok)"></i>UWB-aided pose</span></div></div>
      <div class="side"><div data-k="legtab"></div><div data-k="now"></div><div data-k="files"></div><div data-k="mm"></div></div></div>`;
    const q = k => el.querySelector(`[data-k="${k}"]`);
    let d = null;
    const o = D.canvas(q('cv'), 900 / 560, () => draw());
    function draw() {
      const { ctx: g, w, h } = o; g.clearRect(0, 0, w, h); g.fillStyle = D.css('--raised'); g.fillRect(0, 0, w, h);
      const t = Track.track;
      if (!t) { g.fillStyle = D.css('--muted'); g.font = '13px system-ui'; g.textAlign = 'center'; g.fillText('Waiting for /carbot/map/track_json', w / 2, h / 2); return; }
      const v = D.fitView(t.bounds, w, h, 24);
      gridLines(g, v, t.bounds, 0.5); drawTrack(g, v, t); drawRoute(g, v, d && d.legs);
      if (d && d.trail && d.trail.length > 1) D.line(g, d.trail, p => v.X(p[0]), p => v.Y(p[1]), D.css('--lane'), 1.5, [3, 4]);
      if (d && d.global) { const [x, y, , e] = d.global; D.ellipse(g, v.X(x), v.Y(y), e[0], e[1], e[2], v.s, D.css('--ok')); }
      if (d && d.pose) { const [x, y, a] = d.pose; D.arrow(g, v.X(x), v.Y(y), a, 13, D.css('--lane'));
        if (d.legs && d.legs.legs) D.label(g, v.X(x), v.Y(y) - 24, `Leg ${d.legs.leg + 1} · ${D.f(d.legs.pct, 0)} %`, D.css('--ink'), D.css('--panel')); }
    }
    return {
      query: () => Track.query(),
      update(data) {
        d = data; Track.take(d);
        const lg = d.legs, info = Track.info;
        if (lg && lg.legs) {
          let h = '';
          for (let k = 0; k < lg.legs; k++) {
            const st = k < lg.leg ? 'done' : k === lg.leg ? 'act' : '';
            h += `<span class="pt ${k <= lg.leg ? 'done' : ''} ${k === lg.leg + 1 ? 'here' : ''}">P${k}</span>`;
            h += `<div class="leg ${st}"><b>Leg ${k + 1}${st === 'act' ? ' · now' : st === 'done' ? ' · done' : ''}</b><span>P${k} → P${k + 1} · ${D.f(lg.lengths[k], 1)} m` +
              (st === 'act' && lg.piece_no ? ` · piece ${lg.piece_no} of ${lg.pieces}, ${D.esc(lg.piece_kind)} · ${D.f(lg.pct, 0)} %` : '') +
              `</span><div class="bar"><i style="width:${st === 'done' ? 100 : st === 'act' ? lg.pct : 0}%"></i></div></div>`;
          }
          q('legs').innerHTML = h + `<span class="pt ${lg.leg + 1 === lg.legs ? 'here' : ''}">P${lg.legs}</span>`;
          q('legtab').innerHTML = H.panel('Legs', `<table><tr><th>Leg</th><th>Route</th><th class="r">m</th><th>Exits</th><th>State</th></tr>` +
            Array.from({ length: lg.legs }, (_, k) => {
              const L = info && info.legs && info.legs[k] || {};
              return `<tr class="${k === lg.leg ? 'cur' : ''}"><td>${k + 1}</td><td>P${k} → P${k + 1}${L.parking_bay ? ' · ' + D.esc(L.parking_bay) : ''}</td><td class="r">${D.f(lg.lengths[k], 1)}</td>` +
                `<td>${D.esc((L.exits || []).join(', ') || '—')}</td><td class="${k === lg.leg ? 'blue-t' : 'muted'}">${k < lg.leg ? 'done' : k === lg.leg ? D.f(lg.pct, 0) + ' %' : 'next'}</td></tr>`;
            }).join('') + '</table>');
          q('now').innerHTML = H.panel('Now', H.kv([['Piece', lg.piece_no ? `${lg.piece_no} of ${lg.pieces} · ${D.esc(lg.piece_kind)}` : '—'],
            ['Pose x, y, yaw', d.pose ? `${D.f(d.pose[0])}, ${D.f(d.pose[1])}, ${D.f(d.pose[2] * 180 / Math.PI, 1)}°` : '—']]));
        }
        q('files').innerHTML = H.panel('Route', H.kv([['Status', info ? (info.ok ? 'ROUTE_OK' : 'FAILED') : '—', info ? H.okc(info.ok) : 'muted'],
          ['Reason', D.esc(info && info.reason || '—')], ['Warnings', info && info.warnings ? info.warnings.length : '—'],
          ['Map fingerprint', D.esc((Track.fp || '').slice(0, 40) || '—')]]));
        q('mm').innerHTML = d.mismatch && d.mismatch.length ? H.panel('Gate vs route', `<table><tr><th>t</th><th>Visit</th><th>Gate</th><th>Planned</th></tr>` +
          d.mismatch.slice(-5).reverse().map(x => `<tr><td>${D.f(x.t, 1)}</td><td>${x.visit}</td><td class="warn-t">${D.esc(x.gate)}</td><td>${D.esc(x.planned)}</td></tr>`).join('') + '</table>', 'route is never changed') : '';
        draw();
      },
    };
  },
};

/* ============================================================ PERCEPTION + PLANNER */
TABS.percplan = {
  rate: c => c.rates.cand,
  create(el, ctx) {
    let src = 'local', layer = 'cam', d = null, hover = null;
    const on = { foot: true, mask: true, edge: true, rej: true, sel: true, cor: false };
    el.innerHTML = H.head('Perception + planner', 'Cameras, the stitched drivable area, and the candidates drawn on top of it',
      H.seg('src', [['local', 'Road'], ['parking', 'Parking'], ['recovery', 'Recovery']], src) + H.seg('lay', [['cam', 'Footage'], ['ov', 'Mask overlay']], layer), ctx) +
      `<div class="grid g3" style="margin-bottom:14px">${camTile('Front')}${camTile('Left rear')}${camTile('Right rear')}</div>
      <div class="grid g-side"><div class="panel" style="position:relative"><h3>Drivable area and candidates <small>top-down, base_link</small></h3>
        <div class="toggles" style="margin-bottom:10px">${[['foot', 'Camera cells'], ['mask', 'Road mask'], ['edge', 'Final drivable edge'], ['rej', 'Rejected candidates'], ['sel', 'Selected path'], ['cor', 'Corridor guide']]
          .map(([k, t]) => `<label><input type="checkbox" data-l="${k}" ${on[k] ? 'checked' : ''}> ${t}</label>`).join('')}
          <label><input type="checkbox" data-dbg> Debug images (stitched, mask)</label></div>
        <div class="cv" data-k="cv"></div><div class="tip hover" data-k="tip"></div>
        <div class="grid g2" data-k="dbg" hidden style="margin-top:12px">${camTile('Stitched')}${camTile('Road mask')}</div>
        <div class="legend"><span><i style="background:#3D82F5"></i>Road mask</span><span><i style="background:#fff;outline:1px solid var(--muted)"></i>Final drivable edge</span><span><i style="background:var(--lane)"></i>Selected</span><span><i style="background:#9AA3B2"></i>Rejected</span><span><i style="background:#F5B942"></i>Relaxed</span></div></div>
      <div class="side" data-k="side"></div></div>`;
    const q = k => el.querySelector(`[data-k="${k}"]`);
    H.wireSeg(el, 'src', v => { src = v; });
    H.wireSeg(el, 'lay', v => { layer = v; });
    el.querySelectorAll('[data-l]').forEach(cb => cb.addEventListener('change', () => { on[cb.dataset.l] = cb.checked; draw(); }));
    const loops = [];
    const roles = ['front', 'left_rear', 'right_rear'];
    el.querySelectorAll('.g3 .cam').forEach((tile, i) => loops.push(ImgLoop(tile.querySelector('img'), () => `${layer}_${roles[i]}`, ctx.cfg.rates.image,
      ok => { tile.querySelector('.none').style.display = ok ? 'none' : ''; })));
    let dbgOn = false;
    el.querySelector('[data-dbg]').addEventListener('change', e => { dbgOn = e.target.checked; q('dbg').hidden = !dbgOn; });
    q('dbg').querySelectorAll('.cam').forEach((tile, i) => loops.push(ImgLoop(tile.querySelector('img'), () => dbgOn ? ['stitched', 'mask'][i] : null,
      ctx.cfg.rates.image, ok => { tile.querySelector('.none').style.display = ok ? 'none' : ''; })));
    const o = D.canvas(q('cv'), 1, () => draw());
    let view = null;
    function draw() {
      const { ctx: g, w, h } = o; g.clearRect(0, 0, w, h); g.fillStyle = '#1D2229'; g.fillRect(0, 0, w, h);
      const v = view = D.carView(w, h, h / 1.95, 0.5, 0.64);
      const gr = d && d.grid;
      if (gr) {
        const bytes = D.b64bytes(gr.b64), res = gr.res, px = res * v.s + 0.6;
        const grown = (r, c) => r >= 0 && c >= 0 && r < gr.rows && c < gr.cols && (bytes[r * gr.cols + c] & 4);
        for (let r = 0; r < gr.rows; r++) for (let c = 0; c < gr.cols; c++) {
          const b = bytes[r * gr.cols + c], k = b & 3; if (!k) continue;
          const x = gr.x0 + (r + 1) * res, y = gr.y0 + (c + 1) * res, X = v.X(x, y), Y = v.Y(x, y);
          if (on.foot) { g.fillStyle = k === 2 ? '#C9CED6' : k === 3 ? '#343B45' : '#4A525D'; g.fillRect(X, Y, px, px); }
          if (on.mask && k === 1) { g.fillStyle = (b & 4) ? 'rgba(61,130,245,.75)' : 'rgba(61,130,245,.35)'; g.fillRect(X, Y, px, px); }
          if (on.edge && (b & 4)) {
            g.fillStyle = '#fff';
            if (!grown(r + 1, c)) g.fillRect(X, Y, px, 2); if (!grown(r - 1, c)) g.fillRect(X, Y + px - 2, px, 2);
            if (!grown(r, c + 1)) g.fillRect(X, Y, 2, px); if (!grown(r, c - 1)) g.fillRect(X + px - 2, Y, 2, px);
          }
        }
      } else { g.fillStyle = '#8B93A1'; g.font = '13px system-ui'; g.textAlign = 'center'; g.fillText('Waiting for /carbot/perception/road_grid', w / 2, h / 2); }
      const fx = p => v.X(p[0], p[1]), fy = p => v.Y(p[0], p[1]);
      if (on.cor && d && d.corridor) D.line(g, d.corridor.guide, fx, fy, '#3BE08A', 2.5, [8, 6]);
      (d && d.cands || []).forEach(c => {
        if (c.sel) return;
        if (!on.rej) return;
        g.globalAlpha = c.valid ? .85 : .45;
        D.line(g, c.pts, fx, fy, c.relaxed ? '#F5B942' : '#9AA3B2', hover === c ? 4 : 2.2, c.valid ? [] : [5, 5]); g.globalAlpha = 1;
      });
      (d && d.cands || []).filter(c => c.sel).forEach(c => { if (on.sel) D.line(g, c.pts, fx, fy, '#4C8DFF', 6); });
      if (on.sel && d && d.path && d.path.length > 1 && !(d.cands || []).some(c => c.sel)) D.line(g, d.path, fx, fy, '#4C8DFF', 4);
      g.fillStyle = '#4C8DFF'; g.strokeStyle = '#fff'; g.lineWidth = 2;
      const a = [v.X(0.25, 0.08), v.Y(0.25, 0.08)], b2 = [v.X(-0.05, -0.08), v.Y(-0.05, -0.08)];
      g.fillRect(a[0], a[1], b2[0] - a[0], b2[1] - a[1]); g.strokeRect(a[0], a[1], b2[0] - a[0], b2[1] - a[1]);
    }
    q('cv').addEventListener('mousemove', e => {
      if (!view || !d) return;
      const r = q('cv').getBoundingClientRect(), mx = e.clientX - r.left, my = e.clientY - r.top;
      let best = null, bd = 14;
      (d.cands || []).forEach(c => c.pts.forEach(p => { const dd = Math.hypot(view.X(p[0], p[1]) - mx, view.Y(p[0], p[1]) - my); if (dd < bd) { bd = dd; best = c; } }));
      hover = best; const tip = q('tip');
      if (best) {
        tip.style.display = 'block'; tip.style.left = Math.min(mx + 16, r.width - 230) + 'px'; tip.style.top = (my + 40) + 'px';
        tip.innerHTML = `<b>#${best.id} · offset ${D.f(best.offset)} m${best.sel ? ' · selected' : ''}</b><br>cost ${D.f(best.cost)} · min clear ${D.f(best.clear)} m` +
          (best.stage ? `<br>${D.esc(best.stage)}` : '') + (best.reject ? `<br><span style="color:#FFB3B3">Rejected: ${D.esc(best.reject)}</span>` : '');
      } else tip.style.display = 'none';
      draw();
    });
    q('cv').addEventListener('mouseleave', () => { hover = null; q('tip').style.display = 'none'; draw(); });
    return {
      query: () => `src=${src}`,
      update(data) {
        d = data;
        const cs = d.cands || [], sel = cs.find(c => c.sel), valid = cs.filter(c => c.valid).length, rej = {};
        cs.filter(c => !c.valid && c.reject).forEach(c => { const k = c.reject.split(/[0-9:(]/)[0].trim() || c.reject; rej[k] = (rej[k] || 0) + 1; });
        const cor = d.corridor, gr = d.grid, ps = d.perception_status;
        const st = src === 'parking' ? d.parking_state : src === 'recovery' ? d.recovery_state : null;
        q('side').innerHTML = H.panel('Planner this cycle', H.kv([['Source', src], ['Selected', sel ? `#${sel.id} · ${D.f(sel.offset)} m` : 'none valid', sel ? '' : 'bad-t'],
          ['Cost', sel ? D.f(sel.cost) : '—'], ['Steer command', sel ? D.f(sel.steer, 3) + ' rad' : '—'], ['Valid / total', `${valid} / ${cs.length}`],
          ['Corridor observed', cor ? cor.observed : '—'], ['Lane locked', cor ? (cor.locked ? 'yes' : 'no') : '—', cor ? H.okc(cor.locked) : ''], ['Corridor mode', D.esc(cor ? cor.mode : '—')]])) +
          H.panel('Rejections', Object.keys(rej).length ? `<table><tr><th>Reason</th><th class="r">Count</th></tr>${Object.entries(rej).map(([k, n]) => `<tr><td>${D.esc(k)}</td><td class="r">${n}</td></tr>`).join('')}</table>` : '<div class="muted">None this cycle</div>') +
          (st ? H.panel(src === 'parking' ? 'Parking session' : 'Recovery', H.kv(Object.entries(st).filter(([, v]) => typeof v !== 'object').slice(0, 8).map(([k, v]) => [D.esc(k), D.esc(v)]))) : '') +
          H.panel('Road mask', H.kv([['Coverage', gr ? D.f(gr.coverage * 100, 1) + ' %' : '—'], ['Connected road cells', gr ? gr.connected : '—'],
            ['road_perception', ps ? D.esc(ps.detail || ['OK', 'WARN', 'ERROR', 'STUB'][ps.level]) : 'silent', ps ? (ps.level === 0 ? 'ok-t' : ps.level === 2 ? 'bad-t' : 'warn-t') : 'bad-t']]));
        draw();
      },
      destroy() { loops.forEach(l => l.destroy()); },
    };
  },
};

/* ============================================================ MEMORY + LIDAR */
TABS.memory = {
  rate: c => c.rates.lidar,
  create(el, ctx) {
    el.innerHTML = H.head('Local memory + LiDAR', '3 m memory window, laser scan and tunnel walls', '', ctx) +
      `<div class="grid g-side"><div class="panel"><div class="cv" data-k="cv"></div>
      <div class="legend"><span><i style="background:var(--lane)"></i>Fresh road</span><span><i style="background:var(--lane);opacity:.3"></i>Aged road</span><span><i style="background:var(--bad)"></i>LiDAR hit</span><span><i style="background:var(--warn)"></i>Tunnel centreline / walls</span></div></div>
      <div class="side" data-k="side"></div></div>`;
    const q = k => el.querySelector(`[data-k="${k}"]`);
    let d = null;
    const o = D.canvas(q('cv'), 1, () => draw());
    function draw() {
      const { ctx: g, w, h } = o; g.clearRect(0, 0, w, h); g.fillStyle = D.css('--raised'); g.fillRect(0, 0, w, h);
      const v = D.carView(w, h, h / 3.6, 0.5, 0.62);
      g.strokeStyle = D.css('--grid'); g.lineWidth = 1;
      for (let r = 0.5; r <= 2; r += 0.5) { g.beginPath(); g.arc(v.X(0, 0), v.Y(0, 0), r * v.s, 0, 2 * Math.PI); g.stroke(); }
      const gr = d && d.grid, od = d && d.odom;
      if (gr && od) {
        const bytes = D.b64bytes(gr.b64), c = Math.cos(od[2]), s = Math.sin(od[2]), px = gr.res * v.s + 0.6;
        for (let r = 0; r < gr.rows; r++) for (let cc = 0; cc < gr.cols; cc++) {
          const b = bytes[r * gr.cols + cc], k = b & 3; if (!k) continue;
          const xo = gr.x0 + (r + .5) * gr.res - od[0], yo = gr.y0 + (cc + .5) * gr.res - od[1];
          const bx = c * xo + s * yo, by = -s * xo + c * yo, age = (b >> 4) / 15;
          g.fillStyle = k === 1 ? `rgba(29,99,224,${(0.9 - 0.75 * age).toFixed(2)})` : k === 2 ? 'rgba(160,168,180,.6)' : 'rgba(90,98,110,.25)';
          g.fillRect(v.X(bx, by) - px / 2, v.Y(bx, by) - px / 2, px, px);
        }
      }
      (d && d.scan || []).forEach(p => { g.fillStyle = D.css('--bad'); g.fillRect(v.X(p[0], p[1]) - 1.5, v.Y(p[0], p[1]) - 1.5, 3, 3); });
      const t = d && d.tunnel;
      if (t && t.cl && t.cl.length > 1) {
        D.line(g, t.cl.map(p => [p.x, p.y]), p => v.X(p[0], p[1]), p => v.Y(p[0], p[1]), D.css('--warn'), 3);
        if (t.l) D.line(g, [[0, t.l], [1, t.l]], p => v.X(p[0], p[1]), p => v.Y(p[0], p[1]), D.css('--warn'), 2, [6, 5]);
        if (t.r) D.line(g, [[0, -t.r], [1, -t.r]], p => v.X(p[0], p[1]), p => v.Y(p[0], p[1]), D.css('--warn'), 2, [6, 5]);
      }
      g.fillStyle = D.css('--ink'); const a = [v.X(0.25, 0.08), v.Y(0.25, 0.08)], b = [v.X(-0.05, -0.08), v.Y(-0.05, -0.08)];
      g.fillRect(a[0], a[1], b[0] - a[0], b[1] - a[1]);
    }
    return {
      update(data) {
        d = data; const t = d.tunnel || {}, gr = d.grid;
        q('side').innerHTML = H.panel('Tunnel (base code, wrapped)', H.kv([['LiDAR trigger /tunnel_detected', d.tunnel_trigger == null ? 'no data' : String(d.tunnel_trigger), H.okc(d.tunnel_trigger)],
          ['Mission TUNNEL active', d.tunnel_mode ? 'yes' : 'no', d.tunnel_mode ? 'ok-t' : 'muted'], ['Bridge forwarding', d.tunnel_forwarding ? 'yes' : 'no', d.tunnel_forwarding ? 'ok-t' : 'muted'],
          ['Left / right wall', t.l != null ? `${D.f(t.l)} / ${D.f(t.r)} m` : '—'], ['Lateral error', t.lat != null ? D.f(t.lat, 3) + ' m' : '—'],
          ['/tunnel_cmd_vel angular.z', d.tunnel_cmd ? D.f(d.tunnel_cmd[1]) : '—'], ['Converted steer', d.tunnel_steer != null ? D.f(d.tunnel_steer, 3) + ' rad' : '—']])) +
          H.panel('Memory', H.kv([['Cells held', gr ? gr.cells : '—'], ['Oldest', gr ? D.f(gr.oldest, 1) + ' s' : '—'], ['Travel since oldest', gr ? D.f(gr.travel) + ' m' : '—'], ['Frame', gr ? D.esc(gr.frame) : '—']])) +
          H.panel('LiDAR', H.kv([['Points', d.scan_n != null ? d.scan_n : 'no scan', d.scan_n ? '' : 'bad-t'], ['Closest', d.closest ? `${D.f(d.closest[0])} m @ ${d.closest[1]}°` : '—']]));
        draw();
      },
    };
  },
};

/* ============================================================ LOCALIZATION */
TABS.loc = {
  rate: c => c.rates.map,
  create(el, ctx) {
    let zoom = 'car', d = null;
    el.innerHTML = H.head('Localization', 'Local estimate vs UWB-aided vs raw UWB', H.seg('zoom', [['car', 'Around the car'], ['track', 'Whole track']], zoom), ctx) +
      `<div class="grid g-side"><div class="panel"><div class="cv" data-k="cv"></div>
      <div class="legend"><span><i style="background:var(--lane)"></i>Local (05)</span><span><i style="background:var(--ok)"></i>Global, UWB-aided (06)</span><span><i style="background:var(--warn)"></i>Raw UWB fix</span><span><i style="background:var(--muted)"></i>Anchor range (dashed = gated or stale)</span></div></div>
      <div class="side" data-k="side"></div></div>`;
    const q = k => el.querySelector(`[data-k="${k}"]`);
    H.wireSeg(el, 'zoom', v => { zoom = v; draw(); });
    const o = D.canvas(q('cv'), 700 / 520, () => draw());
    function draw() {
      const { ctx: g, w, h } = o; g.clearRect(0, 0, w, h); g.fillStyle = D.css('--raised'); g.fillRect(0, 0, w, h);
      const t = Track.track;
      let b = t ? t.bounds.slice() : [-1, -1, 8, 6];
      (d && d.anchors || []).forEach(a => { b = [Math.min(b[0], a.x - .3), Math.min(b[1], a.y - .3), Math.max(b[2], a.x + .3), Math.max(b[3], a.y + .3)]; });
      if (zoom === 'car' && d && d.pose) b = [d.pose[0] - 1.2, d.pose[1] - 0.9, d.pose[0] + 1.2, d.pose[1] + 0.9];
      const v = D.fitView(b, w, h, 20);
      gridLines(g, v, b, zoom === 'car' ? 0.1 : 0.5);
      g.globalAlpha = .7; drawTrack(g, v, t); g.globalAlpha = 1;
      (d && d.anchors || []).forEach(a => {
        if (a.corr) { g.beginPath(); g.arc(v.X(a.x), v.Y(a.y), a.corr * v.s, 0, 2 * Math.PI); g.setLineDash(a.gated || !a.seen ? [4, 6] : []);
          g.strokeStyle = D.css('--muted'); g.lineWidth = 1.2; g.stroke(); g.setLineDash([]); }
        const X = v.X(a.x), Y = v.Y(a.y);
        g.beginPath(); g.moveTo(X, Y - 11); g.lineTo(X + 10, Y + 7); g.lineTo(X - 10, Y + 7); g.closePath(); g.fillStyle = a.seen ? D.css('--ink') : D.css('--bad'); g.fill();
        g.font = '650 12px system-ui'; g.textAlign = 'left'; g.fillText(a.id, X + 13, Y + 4);
      });
      const tr = d && d.trails || {};
      [['raw', '--warn'], ['global', '--ok'], ['local', '--lane']].forEach(([k, c]) => { if (tr[k] && tr[k].length > 1) D.line(g, tr[k], p => v.X(p[0]), p => v.Y(p[1]), D.css(c), 1.6, [3, 4]); });
      if (d && d.raw) D.dot(g, v.X(d.raw[0]), v.Y(d.raw[1]), 5, D.css('--warn'));
      if (d && d.global) { const [x, y, , e] = d.global; D.ellipse(g, v.X(x), v.Y(y), e[0], e[1], e[2], v.s, D.css('--ok')); D.dot(g, v.X(x), v.Y(y), 5, D.css('--ok')); }
      if (d && d.pose) { const [x, y, a] = d.pose; if (d.local_ellipse) D.ellipse(g, v.X(x), v.Y(y), d.local_ellipse[0], d.local_ellipse[1], d.local_ellipse[2], v.s, D.css('--lane')); D.arrow(g, v.X(x), v.Y(y), a, 12, D.css('--lane')); }
      g.fillStyle = D.css('--muted'); g.font = '11px system-ui'; g.textAlign = 'right'; g.fillText(`grid ${zoom === 'car' ? '10 cm' : '50 cm'}`, w - 8, h - 8);
    }
    return {
      query: () => Track.query(),
      update(data) {
        d = data; Track.take(d);
        const e = d.est || {}, u = d.uwb || {};
        q('side').innerHTML = H.panel('Estimator', H.kv([['State', D.esc(e.state || '—'), e.state === 'RUNNING' ? 'ok-t' : 'warn-t'], ['Local σ', D.f(e.local_sigma_m, 3) + ' m'], ['Global σ', D.f(e.global_sigma_m, 3) + ' m'],
          ['Offset x / y', `${D.f(e.global_offset_x_m, 3)} / ${D.f(e.global_offset_y_m, 3)} m`], ['UWB residual', D.f(e.uwb_residual_m, 3) + ' m'], ['UWB gain', D.f(e.uwb_gain)],
          ['Accepted / rejected', `${e.uwb_accepted ?? '—'} / ${e.uwb_rejected ?? '—'}`], ['Re-acquires', e.uwb_reacquires ?? '—'], ['Visual updates', e.visual_updates ?? '—'],
          ['IMU', e.imu_ok == null ? '—' : e.imu_ok ? 'fresh' : 'stale: heading from /odom', H.okc(e.imu_ok)], ['Map aligned to UWB', d.aligned ? 'yes' : 'no (step 11)', H.okc(d.aligned)]])) +
          H.panel('Anchors', `<div class="scroll"><table><tr><th>ID</th><th class="r">Raw</th><th class="r">Corr.</th><th class="r">Age</th><th class="r">Samples</th><th>Gate</th></tr>` +
            (d.anchors || []).map(a => `<tr><td>${D.esc(a.id)}</td><td class="r">${D.f(a.raw, 3)}</td><td class="r">${D.f(a.corr, 3)}</td><td class="r ${a.seen ? 'ok-t' : 'bad-t'}">${a.age != null ? D.f(a.age) + ' s' : 'never'}</td>` +
              `<td class="r">${a.fresh_count ?? '—'}</td><td class="${a.gated ? 'warn-t' : 'ok-t'}">${a.gated ? 'gated' : 'in'}</td></tr>`).join('') + '</table></div>' +
            H.kv([['Agent link', u.link == null ? '—' : u.link ? `ok · ${D.f(u.hz, 1)} Hz` : 'DOWN', H.okc(u.link)], ['Tag reboots', u.reboots ?? '—'], ['Offsets calibrated', u.offsets == null ? '—' : u.offsets ? 'yes' : 'no: ranges read ~1 m long', H.okc(u.offsets)],
              ['WiFi latency', u.latency_ms != null ? u.latency_ms + ' ms' : '—'], ['Unknown anchor seen', D.esc(u.unknown || 'none'), u.unknown ? 'warn-t' : '']]));
        draw();
      },
    };
  },
};
