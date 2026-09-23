/* calib_lidar.js — calibration step 5 page: LiDAR-camera alignment (phase 8).
 * Live data: /carbot/calibration/live step.live from carbot_ops/step_lidar_camera.py
 *   {image_key, points [[u,v,range_m]], targets [{u,v,bearing_deg,range_m}], captures [...],
 *    estimate, base_to_laser, intrinsics, scan_age_s, error}   (u, v normalised 0..1)
 * The front image + overlay and the x / height inputs are ONE persistent node, moved back
 * into the page after every poll redraw: the image does not reload and typing is not lost.
 * Clicking the image sets the target foot; Capture sends STEP {"op":"CAPTURE",u,v}. */
'use strict';

const CalLidar = (() => {
  const S = { nCaps: null, node: null, ctx: null, click: null, x: '', z: '', live: null, focus: null, url: null, h: null };
  const f = (v, n) => (v == null || !isFinite(v) ? '—' : Number(v).toFixed(n));

  function build(ctx) {
    const n = document.createElement('div');
    n.innerHTML = `<div class="lcbox" style="position:relative;background:#000;border-radius:8px;overflow:hidden;cursor:crosshair;min-height:120px">
        <img data-lc="img" alt="Front camera" style="display:block;width:100%;height:auto">
        <svg data-lc="svg" style="position:absolute;inset:0;width:100%;height:100%;pointer-events:none" preserveAspectRatio="none"></svg>
        <span data-lc="none" style="position:absolute;left:12px;top:10px;color:#fff;font-size:13px">No front camera image yet</span></div>
      <div class="legend" style="margin-top:6px"><span><i style="background:#FF7A1A"></i>LiDAR near</span><span><i style="background:#2EC4FF"></i>LiDAR far</span>
        <span><i style="background:transparent;outline:2px solid #F5D90A"></i>Target the LiDAR sees</span><span><i style="background:#fff"></i>Your click</span>
        <span><i style="background:#4FC98A"></i>Captured</span></div>`;
    const img = n.querySelector('[data-lc="img"]');
    img.addEventListener('load', () => { n.querySelector('[data-lc="none"]').style.display = 'none'; draw(); });
    n.querySelector('.lcbox').addEventListener('click', e => {
      const r = img.getBoundingClientRect();
      if (!r.width || !r.height) return;
      S.click = { u: Math.min(1, Math.max(0, (e.clientX - r.left) / r.width)), v: Math.min(1, Math.max(0, (e.clientY - r.top) / r.height)) };
      draw(); syncArgs();
    });
    S.node = n; S.ctx = ctx;
    loop();
    return n;
  }

  /* one JPEG pull loop, active only while the page is shown */
  function loop() {
    clearTimeout(S.h);
    const tick = async () => {
      const on = S.node && document.body.contains(S.node) && !document.hidden;
      const key = S.live && S.live.image_key;
      if (on && key) {
        try {
          const r = await fetch(`/api/img/${key}`);
          if (r.status === 200) {
            const u = URL.createObjectURL(await r.blob());
            S.node.querySelector('[data-lc="img"]').src = u;
            if (S.url) URL.revokeObjectURL(S.url);
            S.url = u;
          }
        } catch (e) { /* header shows connection problems */ }
      }
      const fps = (S.ctx && S.ctx.cfg && S.ctx.cfg.rates && S.ctx.cfg.rates.image) || 3;
      S.h = setTimeout(tick, 1000 / Math.max(0.5, fps));
    };
    tick();
  }

  function draw() {
    if (!S.node) return;
    const img = S.node.querySelector('[data-lc="img"]'), svg = S.node.querySelector('[data-lc="svg"]');
    const W = img.naturalWidth || 640, H = img.naturalHeight || 480;
    svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
    const lv = S.live || {}, rmax = 1.6;
    const col = r => { const t = Math.min(1, Math.max(0, r / rmax)); return `rgb(${Math.round(255 - 209 * t)},${Math.round(122 + 74 * t)},${Math.round(26 + 229 * t)})`; };
    const px = (u, v) => [u * W, v * H];
    let s = '';
    const rad = Math.max(2, W / 220);
    (lv.points || []).forEach(([u, v, r]) => { const [x, y] = px(u, v); s += `<circle cx="${x}" cy="${y}" r="${rad}" fill="${col(r)}"/>`; });
    (lv.targets || []).forEach(t => { if (t.u == null) return; const [x, y] = px(t.u, t.v);
      s += `<circle cx="${x}" cy="${y}" r="${rad * 4}" fill="none" stroke="#F5D90A" stroke-width="${rad * 0.8}"/>`; });
    (lv.captures || []).forEach((c, i) => {
      const [x, y] = px(c.u, c.v);
      if (c.lu != null) { const [a, b] = px(c.lu, c.lv); s += `<line x1="${x}" y1="${y}" x2="${a}" y2="${b}" stroke="#4FC98A" stroke-width="${rad * 0.6}"/><circle cx="${a}" cy="${b}" r="${rad * 1.6}" fill="#4FC98A"/>`; }
      s += `<path d="M${x - rad * 3} ${y}H${x + rad * 3}M${x} ${y - rad * 3}V${y + rad * 3}" stroke="#4FC98A" stroke-width="${rad * 0.8}"/>` +
        `<text x="${x + rad * 3}" y="${y - rad * 3}" fill="#4FC98A" font-size="${rad * 6}" font-weight="700">${i + 1}</text>`;
    });
    if (S.click) { const [x, y] = px(S.click.u, S.click.v);
      s += `<circle cx="${x}" cy="${y}" r="${rad * 3}" fill="none" stroke="#fff" stroke-width="${rad * 0.8}"/><path d="M${x - rad * 5} ${y}H${x + rad * 5}M${x} ${y - rad * 5}V${y + rad * 5}" stroke="#fff" stroke-width="${rad * 0.5}"/>`; }
    svg.innerHTML = s;
  }

  /* buttons carry their argument in data-arg; refreshed on every click / keystroke */
  function runArg() {
    const a = {};
    if (S.x.trim() !== '') a.lidar_x_m = S.x.trim();
    if (S.z.trim() !== '') a.lidar_z_m = S.z.trim();
    return Object.keys(a).length ? JSON.stringify(a) : '';
  }
  function syncArgs() {
    document.querySelectorAll('[data-lccap]').forEach(b => {
      b.dataset.arg = S.click ? JSON.stringify({ op: 'CAPTURE', u: +S.click.u.toFixed(4), v: +S.click.v.toFixed(4) }) : '';
      b.disabled = !S.click;
    });
    document.querySelectorAll('[data-lcrun]').forEach(b => { b.dataset.arg = runArg(); });
  }

  /* inputs are persistent too (typing survives the redraw) */
  let ctlNode = null;
  function ctlBuild() {
    const n = document.createElement('div');
    n.innerHTML = `<div class="actions" style="margin:0 0 10px">
        <button class="btn primary" data-act="STEP" data-lccap disabled>Capture<small>the clicked target foot + last LiDAR scans</small></button>
        <button class="btn" data-act="STEP" data-lcre data-arg='{"op":"UNDO"}'>Undo last</button>
        <button class="btn" data-act="STEP" data-lcre data-arg='{"op":"CLEAR"}'>Clear all</button></div>
      <p class="muted" style="margin:0 0 6px;font-size:12.5px">Measured LiDAR position (optional, blank = keep the current TF):</p>
      <div style="display:flex;gap:10px;flex-wrap:wrap">
        <label style="flex:1;min-width:130px">x from the rear axle (m)<br><input data-lcin="x" inputmode="decimal" style="width:100%" placeholder=""></label>
        <label style="flex:1;min-width:130px">height above floor (m)<br><input data-lcin="z" inputmode="decimal" style="width:100%" placeholder=""></label></div>
      <p class="muted" data-lc="tf" style="margin:8px 0 0;font-size:12px"></p>`;
    n.querySelectorAll('[data-lcin]').forEach(inp => {
      inp.addEventListener('input', () => { S[inp.dataset.lcin] = inp.value; syncArgs(); });
      inp.addEventListener('focus', () => { S.focus = inp.dataset.lcin; });
      inp.addEventListener('blur', () => { setTimeout(() => { if (document.activeElement !== inp && ctlNode && ctlNode.isConnected) S.focus = null; }, 0); });
    });
    ctlNode = n;
    return n;
  }

  /* after the page's innerHTML redraw, put the persistent nodes back (and the caret) */
  function mount() {
    const box = document.querySelector('[data-lcslot="img"]');
    if (box && S.node && S.node.parentNode !== box) box.appendChild(S.node);
    const cb = document.querySelector('[data-lcslot="ctl"]');
    if (cb && ctlNode && ctlNode.parentNode !== cb) {
      cb.appendChild(ctlNode);
      if (S.focus) { const inp = ctlNode.querySelector(`[data-lcin="${S.focus}"]`); if (inp) { const p = inp.selectionStart; inp.focus(); try { inp.setSelectionRange(p, p); } catch (e) { /* number-ish input */ } } }
    }
    if (ctlNode) ctlNode.querySelectorAll('[data-lcre]').forEach(b => { b.disabled = false; });
    draw(); syncArgs();
  }

  function page(st, live, d, ctx) {
    const lv = st.live || {};
    S.live = lv;
    const nCaps = (lv.captures || []).length;
    if (S.nCaps != null && nCaps > S.nCaps) S.click = null;     // captured: the next one needs a new click
    S.nCaps = nCaps;
    if (!S.node) build(ctx);
    if (!ctlNode) ctlBuild();
    const m = lv.base_to_laser;
    if (m) {
      ctlNode.querySelector('[data-lcin="x"]').placeholder = `now ${f(m[0], 3)}`;
      ctlNode.querySelector('[data-lcin="z"]').placeholder = `now ${f(m[2], 3)}`;
      ctlNode.querySelector('[data-lc="tf"]').textContent = `Current TF base_link → laser_frame: x ${f(m[0], 3)} y ${f(m[1], 3)} z ${f(m[2], 3)} m, yaw ${f(m[3] * 180 / Math.PI, 2)}°. Front camera: ${lv.sensor || '?'}, ${lv.intrinsics || ''}.`;
    }
    queueMicrotask(mount);

    const caps = lv.captures || [], need = lv.min_captures || 3, est = lv.estimate;
    const scanOk = lv.scan_age_s != null && !lv.error && lv.scan_age_s <= 1.0;
    const seen = (lv.targets || []).some(t => t.u != null);
    const ins = Cal.list(st.meta.instructions);
    let stage = 0;
    if (st.saved_status && !st.unsaved && ['PASS', 'KEPT_PREVIOUS'].includes(st.status)) stage = ins.length;
    else if (st.status === 'PASS' && st.unsaved) stage = ins.length - 1;
    else if (caps.length >= need && est && est.spread_deg >= (lv.min_spread_deg || 20)) stage = 4;
    else if (caps.length) stage = 3;
    else if (S.click) stage = 2;
    else if (scanOk && seen) stage = 2;
    else if (scanOk) stage = 1;
    const todo = Cal.todo(ins.map((t, i) => [t, i < stage ? 'done' : i === stage ? 'now' : 'todo', st.status === 'FAIL' && i === stage]));

    let head = '';
    if (lv.error) head = Cal.alertBad('Step 5 cannot draw the camera view', lv.error, 'Steps 3 and 4 must have saved the front camera; check cameras.yaml roles.front and mounts.front, then relaunch.');
    else if (lv.scan_age_s == null) head = Cal.alertBad('No /scan from the LiDAR', 'calibration_wizard has not received a LaserScan.', 'Go back to step 1: the LiDAR row must be green.');
    else if (lv.scan_age_s > 1.0) head = Cal.alertBad('The LiDAR stopped', `Last scan ${f(lv.scan_age_s, 1)} s ago.`, 'Check the LiDAR USB cable and the ydlidar lines in the launch terminal.');
    else if (!seen) head = Cal.alert('info', 'No target in front yet', 'Stand a thin upright object (bottle, can, pole) 0.4-1.2 m ahead, in the camera view, nothing else within 20 cm of it.');

    const tbl = caps.length ? `<table class="metric" style="margin-top:10px"><tr><th>#</th><th class="r">Bearing</th><th class="r">LiDAR − camera</th><th class="r">After correction</th><th class="r">Distance diff</th></tr>` +
      caps.map((c, i) => `<tr><td>${i + 1}</td><td class="r">${f(c.bearing_deg, 1)}°</td><td class="r">${f(c.error_deg, 2)}°</td>` +
        `<td class="r ${Math.abs(c.residual_deg) > ((st.meta.pass || {}).max_bearing_error_deg || 2) ? 'bad-t' : ''}">${f(c.residual_deg, 2)}°</td><td class="r">${f(c.range_diff_m * 100, 1)} cm</td></tr>`).join('') + '</table>' +
      (est ? `<p class="muted" style="margin:6px 0 0;font-size:12.5px">Estimate: yaw correction <b>${f(est.offset_deg, 2)}°</b>, worst residual ${f(est.max_residual_deg, 2)}°, spread ${f(est.spread_deg, 0)}° over ${est.n} of ≥ ${need} positions.</p>` : '')
      : `<p class="muted" style="margin:8px 0 0">No captures yet: click the target's foot in the image, then Capture. Need ${need}, spread ≥ ${f(lv.min_spread_deg || 20, 0)}°.</p>`;
    const liveHtml = `<div class="panel"><h3>Front camera + LiDAR <small>${lv.scan_age_s != null ? `scan ${f(lv.scan_age_s, 2)} s old · ` : ''}${(lv.points || []).length} points · click the target's foot</small></h3>` +
      head + '<div data-lcslot="img"></div>' + tbl + '</div>';

    const res = st.result || {};
    let result = (st.message ? `<p class="muted" style="margin:0 0 10px">${Cal.esc(st.message)}</p>` : '') + calResult(st);
    if (res.warning) result += Cal.alert('info', 'Distance check (warning only)', res.warning);
    if (res.base_to_laser && st.status === 'PASS') result += `<p class="muted" style="font-size:12.5px;margin:8px 0 0">New TF yaw ${f(res.base_to_laser[3] * 180 / Math.PI, 2)}° (was ${f((res.base_to_laser_before || [])[3] * 180 / Math.PI, 2)}°). Save writes it to the session overlay; it takes effect on the next launch.</p>`;

    const running = st.status === 'RUNNING';
    const redo = !!(st.result && st.unsaved) || st.status === 'FAIL' || st.saved_status;
    const actions = `<button class="btn primary" data-act="${redo ? 'REDO' : 'RUN'}" data-lcrun ${running ? 'disabled' : ''}>Compute${redo ? ' again' : ''}<small>from the ${caps.length} capture(s)</small></button>` +
      calActions(st, 'Compute').replace(/<button class="btn(?: primary)?" data-act="(?:RUN|REDO|CANCEL)"[^>]*>.*?<\/button>/, '');
    return { todo, ctl: '<div data-lcslot="ctl"></div>', live: liveHtml, result, actions };
  }
  return { page };
})();

STEP_PAGES.lidar_camera = CalLidar.page;
