/* tabs_calib.js — calibration wizard pages (phase 8).
 *   TABS.calibration  Overview: every step, sessions + rollback.
 *   TABS.calstep      One page per step (tab id cal-N), data from /api/tab/calibration:
 *                     {steps (CalibrationState), live (the OPEN step, /carbot/calibration/live), wizard (heartbeat)}.
 * Built pages: sensor_health (step 1), camera_identity (step 2), camera_intrinsics (step 3),
 * extrinsics_ipm (step 4), lidar_camera (step 5), imu_odometry (step 6),
 * servo_steering (step 7), speed_pid (step 8),
 * venue_thresholds (step 9), uwb_survey (step 10), map_uwb_alignment (step 11),
 * mission_planner (step 12), practice_runs (step 13).
 * Unknown future steps use a placeholder with instructions and a terminal tool.
 * The layout is created once; only its slots are refreshed, so clicks, open <details>
 * and the embedded diagnostic tab survive each poll. Buttons use one delegated handler. */
'use strict';

const Cal = (() => {
  const CHIP = { PASS: ['Passed', 'ok'], FAIL: ['Failed', 'bad'], RUNNING: ['In progress', 'blue'], PENDING: ['Not run', 'grey'],
    KEPT_PREVIOUS: ['Kept previous', 'grey'], SKIPPED_OPTIONAL: ['Optional', 'grey'] };
  const chip = st => { const [t, c] = CHIP[st] || [st || '—', 'grey']; return `<span class="chip ${c}">${D.esc(t)}</span>`; };
  /* calibration_steps.yaml `tab:` ids -> GUI tabs (mirror of gui_core.YAML_TAB_ALIASES) */
  const TAB = { main: 'drive', map: 'map', perception: 'percplan', planner: 'percplan', memory_lidar: 'memory',
    localization: 'loc', detections: 'det', control: 'control', health: 'health', tuning: 'tuning', scoreboard: 'events', events: 'events' };
  const esc = v => D.esc(v == null ? '' : String(v));
  const list = v => (Array.isArray(v) ? v : (v ? [v] : [])).map(String);

  async function act(ctx, step, action, argument) {
    try {
      const r = await ctx.api('/api/calibration/action', { step: String(step || ''), action, argument: argument || '' });
      if (!r) return { ok: false, message: 'No answer from the car' };
      return r;
    } catch (e) { return { ok: false, message: 'Connection to the car failed: ' + e }; }
  }
  function problem(d) {
    if (!d) return 'No answer from gui_server';
    if (d.live && d.live.error) return d.live.error;
    if (d.wizard && !d.wizard.up) return 'calibration_wizard is not running (' + (d.wizard.detail || 'no heartbeat') + '). Look for its error in the launch terminal.';
    if (!d.steps || !d.steps.length) return 'No /carbot/calibration/state yet: calibration_wizard is starting…';
    return '';
  }
  const alertBad = (t, why, fix) => `<div class="alert bad"><b class="t">${esc(t)}</b>${why || fix ? `<dl>${why ? `<dt>Why</dt><dd>${esc(why)}</dd>` : ''}${fix ? `<dt>Fix</dt><dd>${esc(fix)}</dd>` : ''}</dl>` : ''}</div>`;
  const alert = (k, t, b) => `<div class="alert ${k}"><b class="t">${esc(t)}</b>${b ? esc(b) : ''}</div>`;
  const todo = items => items.map(([t, st, fail]) => `<li class="${st === 'done' ? 'done' : st === 'now' ? (fail ? 'fail now' : 'now') : ''}">` +
    (st === 'now' ? `<span><b class="nowtag">${fail ? 'Fix this' : 'Do this now'}</b>${esc(t)}</span>` : `<span>${esc(t)}</span>`) + '</li>').join('');
  return { chip, TAB, esc, list, act, problem, alertBad, alert, todo };
})();

/* ============================================================ OVERVIEW */
TABS.calibration = {
  rate: () => 1,
  create(el, ctx) {
    el.innerHTML = H.head('Calibration', 'Run the steps in order. Click any step to open it.', '', ctx) +
      '<div data-k="alerts"></div><div class="grid g-side"><div class="panel"><div class="scroll" data-k="steps"></div></div>' +
      '<div class="side"><div class="panel"><h3>Sessions</h3><div data-k="sess"></div><p class="muted" style="font-size:12px;margin:8px 0 0">Race mode loads the ACTIVE session. Rolling back makes an older one ACTIVE; nothing is deleted.</p></div></div></div>';
    const q = k => el.querySelector(`[data-k="${k}"]`);
    let armed = '';
    el.addEventListener('click', async e => {
      const b = e.target.closest('[data-roll]'); if (!b) return;
      if (armed !== b.dataset.roll) { armed = b.dataset.roll; b.textContent = 'Click again to confirm'; return; }
      armed = ''; b.disabled = true;
      const r = await Cal.act(ctx, '', 'ROLLBACK', b.dataset.roll);
      ctx.toast(r.message, 6000);
    });
    return {
      update(d) {
        const p = Cal.problem(d);
        const live = d.live || {};
        let a = p ? Cal.alertBad(p) : '';
        if (!p) {
          const miss = d.steps.filter(s => s.required && !['PASS', 'KEPT_PREVIOUS'].includes(s.status));
          a += miss.length ? `<div class="alert bad"><b class="t">This session is not complete yet</b>Not passed: ${miss.map(s => s.index).join(', ')}. ` +
            `Race mode keeps using ${Cal.esc(live.active || 'no session')} until every required step passes or keeps its previous value.</div>`
            : '<div class="alert ok"><b class="t">All required steps passed</b>This session is ACTIVE: race mode will load it.</div>';
          if (live.notice) a += Cal.alert('info', live.notice, '');
        }
        q('alerts').innerHTML = a;
        q('steps').innerHTML = (d.steps || []).length ? `<table class="ovtab"><tr><th>#</th><th>Step</th><th>Status</th><th>Result</th><th>Saved</th><th></th></tr>` +
          d.steps.map(s => `<tr class="${s.index === d.current ? 'cur' : ''}"><td>${s.index}</td><td>${Cal.esc(s.title)}${s.required ? '' : ' <span class="muted">(optional)</span>'}</td>` +
            `<td>${Cal.chip(s.status)}</td><td class="muted">${Cal.esc(s.summary || '—')}</td><td class="muted">${Cal.esc(s.file || (s.status === 'KEPT_PREVIOUS' ? 'kept' : '—'))}</td>` +
            `<td><button class="btn" data-go="cal-${s.index}">Open</button></td></tr>`).join('') + '</table>' : '<div class="empty">No steps yet</div>';
        const ss = live.sessions || [];
        q('sess').innerHTML = ss.length ? `<table><tr><th>Session</th><th>Passed</th><th></th></tr>` + ss.map(s =>
          `<tr><td><b>${Cal.esc(s.name)}</b><br><span class="muted">${[s.current ? 'this wizard' : '', s.active ? 'ACTIVE' : ''].filter(Boolean).join(' · ') || '&nbsp;'}</span></td>` +
          `<td>${s.passed} of ${s.required}</td><td>${s.active ? '' : `<button class="btn" data-roll="${Cal.esc(s.name)}">Roll back</button>`}</td></tr>`).join('') + '</table>'
          : `<div class="muted">No saved session yet. The first Save creates ${Cal.esc(d.session && d.session !== '(not started)' ? d.session : 'a new timestamped one')}.</div>`;
      },
    };
  },
};

/* ============================================================ STEP PAGES */
const STEP_PAGES = {};
/* Buttons with data-argfrom="<id>": the RUN/REDO argument is built at click time by
 * STEP_ARGS[<id>](pageEl, button) -> {arg: object|string} | {error: text} (forms, step 10) */
const STEP_ARGS = {};

TABS.calstep = {
  rate: () => 2,
  api: () => 'calibration',
  create(el, ctx, id) {
    const index = Number(String(id).slice(4));
    el.innerHTML = `<div data-k="head"></div><div data-k="alerts"></div>
      <div class="calib"><div class="side"><div data-k="need"></div>
        <div class="panel"><h3>What to do</h3><ol class="steps" data-k="todo"></ol></div>
        <div class="panel"><h3>Controls</h3><div class="ctl" data-k="ctl"></div></div></div>
      <div class="side"><div data-k="cams"></div><div class="panel" data-k="cambox" hidden><h3 data-k="camlbl"></h3>
          <div class="cam" data-k="cam"><img alt="camera"><span class="none">No image yet: is the camera preview running?</span>
          <svg data-k="camsvg" viewBox="0 0 1 1" preserveAspectRatio="none" style="position:absolute;inset:0;pointer-events:none"></svg></div></div>
        <div data-k="widget"></div>
        <div class="panel" data-k="mapbox" hidden><h3 data-k="maplbl"></h3><div class="cv" data-k="mapcv"></div><div class="legend" data-k="maplegend"></div></div>
        <div data-k="live"></div>
        <div class="panel"><h3>Result</h3><div data-k="result"></div><div class="actions" data-k="actions"></div></div>
        <details class="panel" data-k="embedbox" hidden><summary data-k="embedsum" style="cursor:pointer;font-weight:650"></summary><div data-k="embed" style="margin-top:12px"></div></details>
      </div></div>`;
    const q = k => el.querySelector(`[data-k="${k}"]`);
    let last = null, selectedAt = 0, embed = null, busy = false, lastErr = '';
    let camSig = null, camLoops = [];

    /* ---- camera tiles (out.cams = [{key, label, note, off}]): rebuilt only when the set of
     *      cameras changes, so the <img> loops keep running across polls; labels refresh every poll */
    function stopCams() { camLoops.forEach(l => l.destroy()); camLoops = []; }
    function setCams(cams) {
      const box = q('cams');
      const sig = cams ? cams.map(c => `${c.key || ''}|${c.off ? 1 : 0}`).join(',') : '';
      if (sig !== camSig) {
        stopCams(); camSig = sig;
        box.innerHTML = cams && cams.length ? `<div class="panel"><h3>Camera pictures</h3><div class="grid ${cams.length > 2 ? 'g3' : cams.length > 1 ? 'g2' : ''}">` +
          cams.map((c, i) => `<div class="cam${c.off ? ' off' : ''}" data-cam="${i}">${c.off ? '' : '<img alt="">'}<span class="none">${c.off ? 'Switched off' : 'No image yet'}</span>` +
            '<span class="lbl"></span><span class="lbl2"></span></div>').join('') + '</div></div>' : '';
        (cams || []).forEach((c, i) => {
          const tile = box.querySelector(`[data-cam="${i}"]`);
          if (c.off || !c.key) return;
          camLoops.push(ImgLoop(tile.querySelector('img'), () => c.key, ctx.cfg.rates.image,
            ok => { tile.querySelector('.none').style.display = ok ? 'none' : ''; }));
        });
      }
      (cams || []).forEach((c, i) => {
        const tile = box.querySelector(`[data-cam="${i}"]`); if (!tile) return;
        tile.querySelector('.lbl').textContent = c.label || '';
        const l2 = tile.querySelector('.lbl2'); l2.textContent = c.note || ''; l2.hidden = !c.note;
      });
    }

    /* ---- camera box: survives re-renders (a page returns cam: {key, label, size, points}) */
    let camKey = '';
    const camImg = q('cam').querySelector('img');
    const camLoop = ImgLoop(camImg, () => camKey, 4, ok => { q('cam').querySelector('.none').style.display = ok ? 'none' : ''; });
    camImg.style.objectFit = 'fill';
    function drawCam(cam) {
      const box = q('cambox');
      camKey = cam && cam.key ? cam.key : '';
      box.hidden = !camKey;
      if (!camKey) return;
      q('camlbl').textContent = cam.label || 'Camera';
      if (cam.size && cam.size[0] && cam.size[1]) q('cam').style.aspectRatio = `${cam.size[0]}/${cam.size[1]}`;
      const pts = cam.points || [];
      const col = cam.color || 'var(--ok)';
      q('camsvg').innerHTML = pts.map(([u, v]) => `<circle cx="${u}" cy="${v}" r="0.008" fill="${col}" />`).join('') +
        (pts.length > 1 ? `<polyline points="${pts.map(p => p.join(',')).join(' ')}" fill="none" stroke="${col}" stroke-width="0.003" />` : '');
    }

    /* ---- persistent page widget (out.widget = {id, create(box, ctx) -> {update(data), destroy()}, data}):
     *      created once per id, so canvases and edits survive the 2 Hz re-render (step 12 map) */
    let wid = null;
    function setWidget(w) {
      if (wid && (!w || wid.id !== w.id)) { if (wid.inst.destroy) wid.inst.destroy(); wid = null; q('widget').innerHTML = ''; }
      if (!w) return;
      if (!wid) wid = { id: w.id, inst: w.create(q('widget'), ctx) };
      wid.inst.update(w.data);
    }

    /* ---- track map box (step 11; a page returns map: {label, legend, path, fixes, anchors, poses, points,
     *      car, fix}, all in the TRACK frame): drawn with the Global map tab's renderer (drawTrack) on a
     *      canvas that survives re-renders. The track comes from /api/tab/map once (Track cache). */
    let mapData = null, mapFetchAt = 0;
    const mo = D.canvas(q('mapcv'), 900 / 560, () => drawMap());
    async function loadTrack() {
      if (Track.track || Date.now() - mapFetchAt < 5000) return;
      mapFetchAt = Date.now();
      try { const dd = await ctx.api('/api/tab/map?' + Track.query()); if (dd) { Track.take(dd); drawMap(); } } catch (err) { /* retried */ }
    }
    function drawMap() {
      const m = mapData; if (!m) return;
      const { ctx: g, w, h } = mo; g.clearRect(0, 0, w, h); g.fillStyle = D.css('--raised'); g.fillRect(0, 0, w, h);
      const t = Track.track;
      let b = t && t.bounds ? t.bounds.slice() : null;
      const ext = p => { if (!p) return; b = b ? [Math.min(b[0], p[0]), Math.min(b[1], p[1]), Math.max(b[2], p[0]), Math.max(b[3], p[1])] : [p[0], p[1], p[0], p[1]]; };
      Object.values(m.anchors || {}).forEach(ext);
      if (!t) { (m.path || []).forEach(ext); Object.values(m.poses || {}).forEach(ext); }
      if (!b) { g.fillStyle = D.css('--muted'); g.font = '13px system-ui'; g.textAlign = 'center'; g.fillText('Waiting for the track map', w / 2, h / 2); return; }
      const v = D.fitView(b, w, h, 24);
      if (t) { gridLines(g, v, t.bounds, 0.5); drawTrack(g, v, t); }
      Object.entries(m.poses || {}).forEach(([k, p]) => { D.arrow(g, v.X(p[0]), v.Y(p[1]), p[2], 7, D.css('--muted')); });
      if (m.path && m.path.length > 1) D.line(g, m.path, p => v.X(p[0]), p => v.Y(p[1]), D.css('--lane'), 2);
      (m.fixes || []).forEach(p => D.dot(g, v.X(p[0]), v.Y(p[1]), 2.5, D.css('--ok')));
      (m.points || []).forEach(p => { D.dot(g, v.X(p.xy[0]), v.Y(p.xy[1]), 6, null, D.css('--lane')); D.label(g, v.X(p.xy[0]), v.Y(p.xy[1]) - 16, p.pose, D.css('--ink'), D.css('--panel')); });
      Object.entries(m.anchors || {}).forEach(([k, p]) => { D.dot(g, v.X(p[0]), v.Y(p[1]), 6, D.css('--warn') || '#E8871E'); D.label(g, v.X(p[0]), v.Y(p[1]) - 16, k, D.css('--ink'), D.css('--panel')); });
      if (m.car) D.arrow(g, v.X(m.car[0]), v.Y(m.car[1]), m.car[2], 12, D.css('--lane'));
      if (m.fix) D.dot(g, v.X(m.fix[0]), v.Y(m.fix[1]), 6, D.css('--ok'), '#fff');
    }
    function setMap(m) {
      q('mapbox').hidden = !m;
      mapData = m || null;
      if (!m) return;
      q('maplbl').textContent = m.label || 'Track map';
      if (q('maplegend').dataset.h !== (m.legend || '')) { q('maplegend').innerHTML = m.legend || ''; q('maplegend').dataset.h = m.legend || ''; }
      loadTrack();
      drawMap();
    }

    /* ---- selecting this step on the wizard (its live view follows the open page) */
    const select = () => { selectedAt = Date.now(); Cal.act(ctx, index, 'SELECT'); };
    select();

    /* ---- one delegated click handler for every button on the page */
    el.addEventListener('click', async e => {
      const b = e.target.closest('[data-act]'); if (!b || b.disabled || busy) return;
      let arg = b.dataset.arg || '';
      if (b.dataset.argfrom) {
        const fn = STEP_ARGS[b.dataset.argfrom];
        const a = fn ? fn(el, b) : { error: 'no form handler for ' + b.dataset.argfrom };
        if (a.error) { lastErr = a.error; ctx.toast(a.error, 7000); if (last) render(last); return; }
        arg = typeof a.arg === 'string' ? a.arg : JSON.stringify(a.arg);
      }
      busy = true; b.disabled = true;
      const r = await Cal.act(ctx, index, b.dataset.act, arg);
      busy = false;
      lastErr = r.ok ? '' : r.message;
      ctx.toast(r.message, r.ok ? 3500 : 7000);
      if (last) render(last);
    });

    /* ---- form fields (data-keep): what the user typed survives the 2 Hz re-render; the slot is
     *      only rewritten when its HTML changes, then edited fields get their typed value back */
    el.addEventListener('input', e => { if (e.target.matches('[data-keep]')) e.target.dataset.dirty = '1'; });
    function setKeep(box, html) {
      if (box.dataset.h === html) return;
      const typed = {};
      const key = i => [...i.attributes].filter(a => a.name.startsWith('data-') && !['data-keep', 'data-dirty'].includes(a.name))
        .map(a => `${a.name}=${a.value}`).join('|') + '@' + ((i.closest('[data-row]') || { dataset: {} }).dataset.row || '');
      box.querySelectorAll('[data-keep][data-dirty]').forEach(i => { typed[key(i)] = i.value; });
      box.innerHTML = html; box.dataset.h = html;
      box.querySelectorAll('[data-keep]').forEach(i => { const k = key(i); if (k in typed) { i.value = typed[k]; i.dataset.dirty = '1'; } });
    }

    /* ---- embedded diagnostic tab: created and polled only while opened */
    function stopEmbed() { if (embed) { clearTimeout(embed.h); if (embed.inst && embed.inst.destroy) embed.inst.destroy(); embed = null; } q('embed').innerHTML = ''; }
    function startEmbed(tab) {
      stopEmbed();
      const T = TABS[tab]; if (!T) return;
      const box = document.createElement('section'); q('embed').appendChild(box);
      embed = { tab, inst: T.create(box, ctx), h: null };
      const tick = async () => {
        if (!embed || embed.tab !== tab) return;
        if (!document.hidden && T.poll !== false) {
          try { const dd = await ctx.api(`/api/tab/${tab}${embed.inst.query ? '?' + embed.inst.query() : ''}`); if (embed && embed.inst) embed.inst.update(dd || {}, ctx.core); } catch (err) { /* header shows it */ }
        }
        if (embed) embed.h = setTimeout(tick, 1000 / Math.max(0.2, T.rate ? T.rate(ctx.cfg) : 1));
      };
      tick();
    }
    q('embedbox').addEventListener('toggle', () => {
      const tab = q('embedbox').dataset.tab;
      if (q('embedbox').open && tab) startEmbed(tab); else stopEmbed();
    });

    function head(d, s, st) {
      const n = (d.steps || []).length || (ctx.cfg.calib_steps || []).length;
      const canNext = s && (s.can_advance || !s.required);
      const why = !s ? '' : st && st.blocked_by ? (st.blocked_by.unsaved_pass ? `Save step ${st.blocked_by.index} first` : `Finish step ${st.blocked_by.index} first`) : (canNext ? '' : 'Pass this step or keep the previous value to continue');
      return `<div class="stephead"><span class="num">Step ${index} of ${n}</span><h1>${Cal.esc(s ? s.title : 'Step ' + index)}</h1>${s ? Cal.chip(s.status) : ''}` +
        `<div class="nav"><button class="btn" data-go="${index > 1 ? 'cal-' + (index - 1) : 'calibration'}">Previous</button>` +
        (index < n ? `<button class="btn ${canNext ? 'primary' : ''}" ${canNext ? '' : 'disabled'} data-go="cal-${index + 1}">Next step</button>` : '') +
        (why ? `<span class="why">${Cal.esc(why)}</span>` : '') + '</div></div>';
    }

    function render(d) {
      last = d;
      const s = (d.steps || []).find(x => x.index === index);
      const live = d.live || {};
      // own view from live.pages (several open pages each get one); live.step = the last selected step
      const st = (live.pages || {})[String(index)] || (live.step && live.step.index === index ? live.step : null);
      q('head').innerHTML = head(d, s, st);
      const p = Cal.problem(d);
      let al = p ? Cal.alertBad(p) : '';
      if (lastErr) al += Cal.alertBad('Last action refused', lastErr);
      // stay watched: re-SELECT well inside page_watch_s, at once (3 s) while this page has no view.
      // Independent of problem(): a false "not running" banner must not stop the page asking.
      const every = Math.max(1000, (live.page_watch_s || 8) * 1000 / 3);
      if (Date.now() - selectedAt > (st ? every : 3000)) select();
      if (!p && !st) al += Cal.alert('info', 'Opening this step…', 'Waiting for calibration_wizard to switch its live view here.');
      if (st && st.blocked_by && st.status !== 'RUNNING') al += st.blocked_by.unsaved_pass
        ? `<div class="alert bad"><b class="t">Step ${st.blocked_by.index} (${Cal.esc(st.blocked_by.title)}) passed but is not saved</b>Open it and press Save; until then this step stays locked. <button class="btn" data-go="cal-${st.blocked_by.index}">Go to step ${st.blocked_by.index}</button></div>`
        : Cal.alert('info', `Step ${st.blocked_by.index} (${st.blocked_by.title}) is not passed yet`, 'You can read this page, but Run stays refused until the earlier steps pass and are saved, or keep their previous value.');
      q('alerts').innerHTML = al;
      if (!st) return;
      const page = st.built ? (STEP_PAGES[st.id] || STEP_PAGES._generic) : STEP_PAGES._placeholder;
      let out;
      try { out = page(st, live, d, ctx); } catch (err) { out = { live: Cal.alertBad('This page failed to draw', String(err), 'Reload the browser page (Ctrl+Shift+R). If it repeats, report the message.') }; }
      const need = st.meta && st.meta.need;
      q('need').innerHTML = need ? `<div class="need"><b>Before you start</b><span>${Cal.esc(need)}</span></div>` : '';
      q('todo').innerHTML = out.todo || '';
      setKeep(q('ctl'), out.ctl || '<span class="muted">Nothing to set for this step.</span>');
      setCams(out.cams);
      q('live').innerHTML = out.live || '';
      q('result').innerHTML = out.result || '';
      q('actions').innerHTML = out.actions || '';
      drawCam(out.cam);
      setWidget(out.widget);
      setMap(out.map);
      const tab = Cal.TAB[(st.meta && st.meta.tab) || ''];
      const box = q('embedbox');
      if (tab && TABS[tab]) {
        box.hidden = false;
        if (box.dataset.tab !== tab) { box.dataset.tab = tab; if (box.open) startEmbed(tab); }
        q('embedsum').textContent = `Live ${ctx.cfg.tabs.find(t => t.id === tab)?.title || tab} tab (opens its data only while expanded)`;
      } else box.hidden = true;
    }
    return { update(d) { render(d || {}); }, destroy() { stopEmbed(); stopCams(); camLoop.destroy(); setWidget(null); } };
  },
};

/* ---- shared Result / Actions blocks */
/* runLabel null: the page has its own Run buttons (in Controls); only Cancel is added here */
function calActions(st, runLabel) {
  const running = st.status === 'RUNNING';
  const hasResult = !!(st.result && st.unsaved) || st.status === 'FAIL';
  const run = running ? `<button class="btn" data-act="CANCEL">Cancel</button>` : runLabel == null ? ''
    : `<button class="btn primary" data-act="${hasResult || st.saved_status ? 'REDO' : 'RUN'}" ${st.built ? '' : 'disabled'}>${hasResult || st.saved_status ? 'Redo' : Cal.esc(runLabel)}</button>`;
  const save = `<button class="btn good" data-act="SAVE" ${st.status === 'PASS' && st.unsaved ? '' : 'disabled'}>Save</button>`;
  const keep = st.can_keep ? `<button class="btn" data-act="KEEP_PREVIOUS">Keep previous value<small>${Cal.esc(st.previous)} (passed)</small></button>`
    : `<button class="btn" disabled>Keep previous value<small>${st.previous ? (st.built ? 'switched off in ops.yaml' : 'available once this page is built') : 'No earlier PASS: this step must pass now'}</small></button>`;
  return run + save + (st.required ? keep : '') +
    `<span class="hint">Save writes timestamped YAML into the wizard session. Nothing is overwritten; older sessions stay for rollback.</span>`;
}

function calResult(st) {
  if (st.status === 'RUNNING' && st.run) {
    const r = st.run;
    return `<div class="alert info"><b class="t">Measuring… ${D.f(r.remaining_s, 0)} s left</b>${r.samples} health reports collected. Keep everything powered and still.</div>` +
      `<div class="bar"><i style="width:${Math.round((r.fraction || 0) * 100)}%"></i></div>`;
  }
  const res = st.result;
  if (st.status === 'KEPT_PREVIOUS') return Cal.alert('info', 'Using the previous value', `Kept from ${st.from_session || st.previous}. Press Redo to measure again.`);
  if (!res) return `<div class="muted">${st.saved_status ? 'Saved as ' + Cal.esc(st.saved_status) : 'Not run yet.'}</div>`;
  const checks = res.checks || [];
  const failed = checks.filter(c => !c.passed);
  let h = '';
  if (res.passed) h += Cal.alert('ok', st.unsaved ? 'All checks passed: press Save' : 'Passed and saved',
    st.unsaved ? res.summary : `${res.summary}. Saved to ${st.saved_file}.`);
  else {
    h += `<div class="alert bad"><b class="t">${Cal.esc(res.summary || 'Failed')}</b>` +
      (failed.length ? failed.map(c => `<div class="failitem"><b>${Cal.esc(c.label)}</b><div><b>Why:</b> ${Cal.esc(c.why)}</div>${c.fix ? `<div><b>Fix:</b> ${Cal.esc(c.fix)}</div>` : ''}</div>`).join('') : '') +
      ((res.problems || []).length ? `<ul>${res.problems.map(p => `<li>${Cal.esc(p)}</li>`).join('')}</ul>` : '') +
      '<p style="margin:8px 0 0">Fix the problems above, then press Redo.</p></div>';
  }
  if (checks.length) h += `<table class="metric"><tr><th>Check</th><th class="r">Measured</th><th class="r">Limit</th><th>Result</th></tr>` +
    checks.map(c => `<tr><td>${Cal.esc(c.label)}</td><td class="r">${Cal.esc(c.measured)}</td><td class="r">${Cal.esc(c.limit)}</td>` +
      `<td class="${c.passed ? 'ok-t' : 'bad-t'}">${c.passed ? 'pass' : 'fail'}${c.ok_fraction != null && c.ok_fraction < 1 ? ` <span class="muted">(${Math.round(c.ok_fraction * 100)} %)</span>` : ''}</td></tr>`).join('') + '</table>';
  if (res.time) h += `<p class="muted" style="font-size:12px;margin:8px 0 0">Measured ${Cal.esc(res.time)}${res.measure_s ? ` over ${res.measure_s} s` : ''}${res.samples != null ? `, ${res.samples} samples` : ''}.</p>`;
  return h;
}

function calTaskBox(task) {
  if (!task || task.state === 'idle') return '';
  const k = task.state === 'failed' ? 'bad' : task.state === 'done' ? 'ok' : 'info';
  return `<div class="alert ${k}" style="margin:8px 0 0"><b class="t">${task.state === 'running' ? 'Restarting camera drivers…' : task.state === 'done' ? 'Camera drivers restarted' : 'Camera restart failed'}</b>${Cal.esc(task.message)}` +
    (task.log && task.log.length ? `<details style="margin-top:6px"><summary>Log</summary><pre style="white-space:pre-wrap;font-size:11px;margin:6px 0 0">${Cal.esc(task.log.join('\n'))}</pre></details>` : '') + '</div>';
}

/* ---- step 1: sensor health */
STEP_PAGES.sensor_health = (st, live) => {
  const lv = st.live || {};
  const rows = lv.rows || [];
  const ins = Cal.list(st.meta.instructions);
  const allOk = rows.length && rows.every(r => r.state === 'ok');
  const anyHealth = rows.some(r => r.state !== 'wait');
  let stage = 0;
  if (st.saved_status && !st.unsaved && ['PASS', 'KEPT_PREVIOUS'].includes(st.status)) stage = ins.length;
  else if (st.status === 'PASS' && st.unsaved) stage = ins.length - 1;
  else if (st.status === 'RUNNING' || allOk) stage = ins.length - 1;
  else if (anyHealth && rows.some(r => r.state === 'bad')) stage = Math.min(2, ins.length - 1);
  else if (anyHealth) stage = 1;
  const failNow = st.status === 'FAIL' || (stage === 2 && !allOk);
  const todo = Cal.todo(ins.map((t, i) => [t, i < stage ? 'done' : i === stage ? 'now' : 'todo', failNow && i === stage]));

  const task = live.task;
  const ctl = `<button class="btn" data-act="RESTART_CAMERAS" ${task && task.state === 'running' || st.status === 'RUNNING' ? 'disabled' : ''}>Restart camera drivers` +
    `<small>Kills stale mipi_cam, hobot_codec, websocket and Astra processes, then restarts the 3 camera drivers (~10 s)</small></button>` + calTaskBox(task);

  let tbl;
  if (lv.error) tbl = Cal.alertBad('Step 1 cannot run', lv.error, 'Fix calibration_steps.yaml / cameras.yaml / uwb.yaml, then relaunch calibrate.launch.py.');
  else if (!rows.length) tbl = '<div class="empty">Waiting for the first health report…</div>';
  else tbl = `<table class="calcheck"><tr><th>Sensor</th><th class="r">Measured</th><th class="r">Limit</th><th>State</th></tr>` +
    rows.map(r => `<tr${r.state === 'bad' ? ' class="badrow"' : ''}><td><b>${Cal.esc(r.label)}</b><br><span class="muted" style="font-size:11.5px">${Cal.esc([r.topic, r.detail].filter(Boolean).join(' · '))}</span></td>` +
      `<td class="r">${Cal.esc(r.measured)}</td><td class="r">${Cal.esc(r.limit)}</td>` +
      `<td class="${r.state === 'ok' ? 'ok-t' : r.state === 'bad' ? 'bad-t' : 'muted'}">${r.state === 'ok' ? 'ok' : r.state === 'bad' ? 'fail' : 'waiting'}</td></tr>` +
      (r.state !== 'ok' && (r.why || r.fix) ? `<tr${r.state === 'bad' ? ' class="badrow"' : ''}><td colspan="4" class="whyfix">${r.why ? `<b>Why:</b> ${Cal.esc(r.why)}` : ''}${r.fix ? `<br><b>Fix:</b> ${Cal.esc(r.fix)}` : ''}</td></tr>` : '')).join('') + '</table>';
  const liveHtml = `<div class="panel"><h3>Live sensor check <small>${lv.n != null ? `${lv.n_ok} of ${lv.n} ok` : ''}${lv.health_age_s != null ? ` · health report ${D.f(lv.health_age_s, 1)} s old` : ''}</small></h3>${tbl}</div>`;
  return { todo, ctl, live: liveHtml, result: (st.message ? `<p class="muted" style="margin:0 0 10px">${Cal.esc(st.message)}</p>` : '') + calResult(st), actions: calActions(st, 'Run check') };
};

/* ---- step 2: camera identity (switched-off cameras are shown and skipped) */
STEP_PAGES.camera_identity = st => {
  const lv = st.live || {};
  const cams = lv.cameras || [];
  const on = cams.filter(c => c.enabled);
  const off = cams.filter(c => !c.enabled);
  const ins = Cal.list(st.meta.instructions);
  const running = st.status === 'RUNNING';
  const allLive = on.length > 0 && on.every(c => c.state === 'live');
  let stage = 0;
  if (st.saved_status && !st.unsaved && ['PASS', 'KEPT_PREVIOUS'].includes(st.status)) stage = ins.length;
  else if (st.status === 'PASS' && st.unsaved) stage = ins.length - 1;
  else if (running || st.status === 'FAIL') stage = Math.max(0, ins.length - 2);
  const todo = Cal.todo(ins.map((t, i) => [t, i < stage ? 'done' : i === stage ? 'now' : 'todo', st.status === 'FAIL' && i === stage]));

  const STATE = { live: ['live', 'ok-t'], stale: ['frozen', 'bad-t'], none: ['no frames', 'bad-t'], wait: ['waiting for health report', 'muted'], off: ['switched off, skipped', 'muted'] };
  const stateTxt = c => (STATE[c.state] || [c.state, 'muted'])[0] + (c.state === 'stale' && c.age != null ? ` (${D.f(c.age, 1)} s old)` : '');
  const tiles = cams.map(c => ({ key: c.enabled ? c.preview : null, off: !c.enabled, label: `${c.label} · ${c.sensor_label}`,
    note: c.enabled ? stateTxt(c) + (c.hz != null && c.state === 'live' ? ` · ${D.f(c.hz, 0)} fps` : '') : 'cameras.yaml enabled: false' }));

  const hasRun = !!(st.result && st.unsaved) || st.status === 'FAIL' || st.saved_status;
  const act = hasRun ? 'REDO' : 'RUN';
  const dis = running || !on.length || lv.error ? 'disabled' : '';
  const names = on.map(c => c.label.toLowerCase()).join(', ');
  let ctl = lv.error ? Cal.alertBad('Step 2 cannot run', lv.error, 'Fix cameras.yaml / calibration_steps.yaml, then relaunch calibrate.launch.py.') : '';
  ctl += `<button class="btn primary" data-act="${act}" data-arg="${Cal.esc(JSON.stringify({ confirm: true, swap: false }))}" ${dis}>` +
    `Confirm: the pictures are right<small>Confirms ${Cal.esc(names || 'nothing (every camera is off)')}</small></button>`;
  if (lv.sides_enabled) {
    ctl += `<button class="btn" data-act="${act}" data-arg="${Cal.esc(JSON.stringify({ confirm: true, swap: true }))}" ${dis}>` +
      'Confirm swapped<small>The left picture shows the right side (and the other way round): Save swaps the two side roles</small></button>';
  } else if (off.length) {
    ctl += `<p class="muted" style="margin:8px 0 0;font-size:12.5px">${Cal.esc(off.map(c => `${c.label} (${c.sensor_label})`).join(' and '))} ` +
      `${off.length > 1 ? 'are' : 'is'} switched off in cameras.yaml, so only ${Cal.esc(names || 'nothing')} is confirmed. ` +
      'Once a side camera is switched on again it counts as unconfirmed until this step runs again.</p>';
  }
  if (on.length && !allLive && !running) {
    ctl += Cal.alert('bad', 'A camera picture is not live', 'Confirm will fail while a switched-on camera is frozen or missing. Step 1 has a Restart camera drivers button.');
  }

  const rows = cams.map(c => `<tr${c.enabled && c.state !== 'live' && c.state !== 'wait' ? ' class="badrow"' : ''}><td><b>${Cal.esc(c.label)}</b></td>` +
    `<td>${Cal.esc(c.sensor_label)}<br><span class="muted" style="font-size:11.5px">${Cal.esc(c.topic)}</span></td>` +
    `<td class="${(STATE[c.state] || ['', 'muted'])[1]}">${Cal.esc(stateTxt(c))}</td>` +
    `<td class="r">${c.hz != null ? D.f(c.hz, 1) + ' Hz' : '—'}</td></tr>`).join('');
  const conf = lv.roles_confirmed ? `confirmed for ${(lv.roles_confirmed_for || []).join(', ') || 'no role'}` : 'not confirmed';
  const liveHtml = `<div class="panel"><h3>Cameras <small>roles as loaded: ${Cal.esc(conf)}</small></h3>` +
    (cams.length ? `<table class="calcheck"><tr><th>Role</th><th>Sensor</th><th>Picture</th><th class="r">Rate</th></tr>${rows}</table>` : '<div class="empty">Waiting for the wizard…</div>') +
    '<p class="muted" style="font-size:12px;margin:8px 0 0">Saved roles apply from the next launch of this session. Pictures here follow the roles the launch loaded.</p></div>';

  let result = (st.message ? `<p class="muted" style="margin:0 0 10px">${Cal.esc(st.message)}</p>` : '') + calResult(st);
  const r = st.result;
  if (r && r.roles) {
    result += `<table class="metric"><tr><th>Role</th><th>Sensor</th><th></th></tr>` + Object.keys(r.roles).map(k =>
      `<tr><td>${Cal.esc(k)}</td><td>${Cal.esc(r.roles[k])}</td><td class="muted">${(r.confirmed_roles || []).includes(k) ? 'confirmed' : 'skipped (switched off)'}</td></tr>`).join('') + '</table>';
  }
  return { todo, ctl, cams: tiles, live: liveHtml, result, actions: calActions(st, null) };
};

/* ---- step 3: camera intrinsics (one camera per Run; switched-off cameras shown as not detected) */
STEP_PAGES.camera_intrinsics = (st, live) => {
  const lv = st.live || {};
  if (lv.error) return { live: Cal.alertBad('Step 3 cannot run', lv.error, 'Fix calibration_steps.yaml / cameras.yaml, then relaunch calibrate.launch.py.'),
    result: calResult(st), actions: calActions(st, 'Run') };
  const sensors = lv.sensors || [];
  const cap = lv.capture;
  const running = st.status === 'RUNNING';
  const ins = Cal.list(st.meta.instructions);
  const board = lv.board || {};
  let stage = 1;
  if (st.saved_status && !st.unsaved && ['PASS', 'KEPT_PREVIOUS'].includes(st.status)) stage = ins.length;
  else if (st.status === 'PASS' && st.unsaved) stage = ins.length - 1;
  else if (running && cap) stage = cap.calibrating ? ins.length - 1 : (cap.views ? 3 : 2);
  const todo = Cal.todo(ins.map((t, i) => [t, i < stage ? 'done' : i === stage ? 'now' : 'todo', st.status === 'FAIL' && i === stage]));

  const SCHIP = { pass: ['Passed', 'ok'], fail: ['Failed', 'bad'], capturing: ['Capturing', 'blue'], todo: ['Not run', 'grey'], off: ['Not detected', 'grey'] };
  const schip = k => { const [t, c] = SCHIP[k] || [k, 'grey']; return `<span class="chip ${c}">${Cal.esc(t)}</span>`; };
  const ctl = `<table class="calcheck"><tr><th>Camera</th><th>State</th><th></th></tr>` + sensors.map(x => {
    const r = x.result || {};
    const detail = !x.enabled ? x.note : r.rms_px != null ? `${r.model} · ${D.f(r.rms_px, 3)} px · ${r.views} views · hfov ${D.f(r.hfov_deg, 1)}°` : x.topic;
    const btn = !x.enabled ? '' : running ? (cap && cap.sensor === x.name ? `<button class="btn" data-act="CANCEL">Cancel</button>` : '')
      : `<button class="btn ${r.status ? '' : 'primary'}" data-act="${r.status ? 'REDO' : 'RUN'}" data-arg="${Cal.esc(x.name)}">${r.status ? 'Redo' : 'Run'}</button>`;
    return `<tr${x.enabled ? '' : ' class="muted"'}><td><b>${Cal.esc(x.label)}</b><br><span class="muted" style="font-size:11.5px">${Cal.esc(detail || '')}</span></td>` +
      `<td>${schip(x.state)}</td><td class="r">${btn}</td></tr>`;
  }).join('') + '</table>' +
    `<p class="muted" style="font-size:12px;margin:8px 0 0">Board: ${Cal.esc((board.inner_corners || []).join(' x '))} inner corners, ${Cal.esc(board.square_mm)} mm squares ` +
    `(docs/calibration/intrinsics_board_A4.pdf, printed at 100 %).</p>`;

  let liveHtml, cam = null;
  if (cap) {
    const pct = Math.round(100 * Math.min(1, cap.views / Math.max(1, cap.target_views)));
    const grid = `<table class="covgrid" style="border-collapse:separate;border-spacing:3px;margin:6px 0">` + (cap.coverage || []).map(row =>
      '<tr>' + row.map(n => `<td style="width:46px;height:26px;text-align:center;border-radius:5px;font-size:11.5px;` +
        `background:var(${n ? '--ok-bg' : '--raised'});color:var(${n ? '--ok' : '--muted'});border:1px solid var(--line)">${n ? '✓' : ''}</td>`).join('') + '</tr>').join('') + '</table>';
    const stateCls = cap.state === 'new view captured' ? 'ok-t' : cap.state === 'no board in view' ? 'bad-t' : 'warn-t';
    liveHtml = `<div class="panel"><h3>Capturing ${Cal.esc(cap.label)} <small>${cap.frames} frames checked</small></h3>` +
      (cap.calibrating ? Cal.alert('info', 'Calibrating…', `Fitting both lens models to ${cap.views} views. This takes a few seconds; keep the page open.`)
        : `<p style="margin:0 0 6px;font-size:15px"><b class="${stateCls}">${Cal.esc(cap.state)}</b></p>`) +
      `<div class="kv"><span>Views</span><b>${cap.views} of ${cap.target_views} <span class="muted">(pass needs ${cap.min_views})</span></b>` +
      `<span>Image areas covered</span><b>${cap.coverage_cells} of 9</b></div><div class="bar" style="margin:8px 0"><i class="${pct >= 100 ? 'ok' : ''}" style="width:${pct}%"></i></div>` +
      grid + `<p class="muted" style="font-size:12px;margin:0">Grey squares: move the board there. It calibrates by itself once enough views cover the whole image` +
      `${st.run ? `, or after ${D.f(st.run.remaining_s, 0)} s` : ''}.</p></div>`;
    if (cap.preview) cam = { key: 'cam_' + cap.preview, label: `${cap.label} · live`, size: cap.size,
      points: cap.corners || [], color: cap.state === 'new view captured' ? '#2ecc71' : '#f1c40f' };
  } else {
    liveHtml = `<div class="panel"><h3>Live capture</h3>${Cal.alert('info', 'Not capturing', 'Press Run next to a camera. The camera feed and the detected board corners appear here.')}</div>`;
    const first = sensors.find(x => x.enabled);
    if (first) cam = { key: 'cam_front', label: 'Front camera · aim check' };
  }
  let result;
  if (running) result = Cal.alert('info', cap && cap.calibrating ? 'Calibrating…' : 'Capturing views…', 'The result appears here when the fit is done.');
  else {
    result = calResult(st);
    const rs = st.result && st.result.sensors ? Object.values(st.result.sensors).filter(r => r.rms_px != null) : [];
    if (rs.length) result += `<table class="metric" style="margin-top:8px"><tr><th>Camera</th><th>Model</th><th class="r">fx / fy</th><th class="r">cx / cy</th><th class="r">hfov</th></tr>` +
      rs.map(r => `<tr><td>${Cal.esc(r.label)}</td><td>${Cal.esc(r.model)}</td><td class="r">${D.f(r.fx, 1)} / ${D.f(r.fy, 1)}</td>` +
        `<td class="r">${D.f(r.cx, 1)} / ${D.f(r.cy, 1)}</td><td class="r">${D.f(r.hfov_deg, 1)}°</td></tr>`).join('') + '</table>';
  }
  return { todo, ctl, live: liveHtml, cam, result: (st.message ? `<p class="muted" style="margin:0 0 10px">${Cal.esc(st.message)}</p>` : '') + result,
    actions: calActions(st, 'Run next camera') };
};

/* ---- step 4: camera mounts from the printed floor boards (existing calib_extrinsics solver) */
STEP_PAGES.extrinsics_ipm = (st) => {
  const lv = st.live || {};
  const roles = lv.roles || [];
  const running = st.status === 'RUNNING';
  const todo = Cal.todo(Cal.list(st.meta.instructions).map((instruction, i) =>
    [instruction, running && i === 2 ? 'now' : 'todo']));
  const cams = roles.map(r => ({ key: r.preview, label: r.role,
    note: r.captured ? 'Picture captured for this run' : 'Keep the floor board fully visible' }));
  const ctl = `<p class="muted">${Cal.esc(st.meta.need || '')}</p>` +
    (running ? `<div class="alert info"><b class="t">${Cal.esc(lv.state || 'Capturing pictures…')}</b>` +
      'The result appears here when the camera mounts are solved.</div>' :
      '<p>Check the live camera pictures, then run the measurement.</p>');
  const result = running ? '<div class="muted">Waiting for pictures and mount calculation…</div>' : calResult(st);
  return { todo, ctl, cams, result, actions: calActions(st, 'Run'), live: '' };
};

/* ---- step 6: IMU + wheel odometry. The car is moved BY HAND. Distance + spin are page operations
 *      (action STEP {"op"}), Run = the 60 s drift test, which produces the step result. */
STEP_PAGES.imu_odometry = st => {
  const lv = st.live || {};
  const ins = Cal.list(st.meta.instructions);
  const running = st.status === 'RUNNING';
  const a = lv.active;
  const tests = lv.tests || {};
  const dist = tests.distance || { status: 'NOT_RUN', runs: [] };
  const spin = tests.spin || { status: 'NOT_RUN', runs: [] };
  const v = lv.values;
  const busy = lv.busy || '';
  const saved = st.saved_status && !st.unsaved && ['PASS', 'KEPT_PREVIOUS'].includes(st.status);
  const opBtn = (op, label, sub, cls, off) => `<button class="btn ${cls || ''}" data-act="STEP" data-arg="${Cal.esc(JSON.stringify({ op }))}" ${off ? 'disabled' : ''}>${Cal.esc(label)}${sub ? `<small>${Cal.esc(sub)}</small>` : ''}</button>`;

  /* what to do: tape (0), distance (1), spin (2), drift (3), save (4) */
  let stage;
  if (saved) stage = ins.length;
  else if (st.status === 'PASS' && st.unsaved) stage = 4;
  else if (lv.drift_ready || running || st.status === 'FAIL') stage = 3;
  else if (dist.status === 'PASS') stage = lv.spin_test ? 2 : 3;
  else stage = dist.n_runs || (a && a.test === 'distance') ? 1 : 0;
  const lastBad = t => t.runs && t.runs.length && !['PASS'].includes(t.runs[t.runs.length - 1].status);
  const failNow = (stage === 1 && lastBad(dist)) || (stage === 2 && lastBad(spin)) || (stage === 3 && st.status === 'FAIL');
  const todo = Cal.todo(ins.map((t, i) => [t, i < stage ? 'done' : i === stage ? 'now' : 'todo', failNow && i === stage]));

  /* controls */
  let ctl = '';
  if (lv.error) ctl += Cal.alertBad('Step 6 cannot run', lv.error, 'Fix calibration_steps.yaml (imu_odometry), then relaunch calibrate.launch.py.');
  if (lv.link_error) ctl += Cal.alertBad('servo_controller not reachable', lv.link_error, '');
  ctl += `<div class="calvals"><b>servo_controller now</b> ` + (v
    ? `ticks_per_meter <b>${D.f(v.ticks_per_meter, 1)}</b> · reverse polarity <b>${v.odom_reverse_polarity ? 'on' : 'off'}</b> · imu_yaw_scale <b>${D.f(v.imu_yaw_scale, 4)}</b>`
    : '<span class="muted">reading…</span>') +
    (busy ? `<br><span class="muted">${Cal.esc(busy)}…</span>` : '') + '</div>';
  const any = !!a;
  const noGo = running || !v || !!busy;
  const d = a && a.test === 'distance';
  const sp = a && a.test === 'spin';
  ctl += '<h4 style="margin:6px 0">1. Distance</h4>' + (d
    ? opBtn('distance_stop', 'Stop: rear axle on the second mark', `odom so far ${D.f(a.distance_m, 3)} m`, 'primary') + opBtn('abort', 'Abort run', '')
    : opBtn('distance_start', dist.status === 'NOT_RUN' ? 'Start distance run' : 'Start another distance run',
      `rear axle on the first mark, then push ${D.f(lv.straight_run_m, 2)} m straight`, dist.status === 'PASS' ? '' : 'primary', noGo || any));
  if (lv.spin_test) {
    ctl += '<h4 style="margin:12px 0 6px">2. Spin</h4>' + (sp
      ? (a.phase === 'arming' ? '<div class="muted">Setting the IMU scale for the measurement…</div>' + opBtn('abort', 'Abort spin', '')
        : opBtn('spin_stop', 'Stop: back on the tape mark', `IMU so far ${D.f(a.yaw_deg, 1)} deg${a.settling ? ' (settling, wait before turning)' : ''}`, 'primary') + opBtn('abort', 'Abort spin', ''))
      : opBtn('spin_start', spin.status === 'NOT_RUN' ? 'Start spin' : 'Start another spin', 'then ONE full turn to the LEFT by hand',
        dist.status === 'PASS' && spin.status !== 'PASS' ? 'primary' : '', noGo || any));
  }
  const hasRun = !!(st.result && st.unsaved) || st.status === 'FAIL' || st.saved_status;
  ctl += `<h4 style="margin:12px 0 6px">${lv.spin_test ? '3' : '2'}. Drift</h4>` + (running ? '<div class="muted">Hands off: measuring…</div>'
    : `<button class="btn ${lv.drift_ready ? 'primary' : ''}" data-act="${hasRun ? 'REDO' : 'RUN'}" ${lv.drift_ready && !any && !busy ? '' : 'disabled'}>Run drift test` +
      `<small>${lv.drift_ready ? `hands off, the car stays still for ${D.f(lv.drift_test_s, 0)} s` : `needs the distance${lv.spin_test ? ' and spin tests' : ' test'} passed first`}</small></button>`);
  ctl += opBtn('reread', 'Re-read servo_controller values', '', '', running || !!busy || any);

  /* live measurement + run history */
  const ages = lv.ages || {};
  const ageTxt = x => x == null ? 'never' : x > 1 ? `${D.f(x, 1)} s old` : 'live';
  let liveHtml = `<div class="panel"><h3>Live <small>/odom ${Cal.esc(ageTxt(ages.odom))} · /imu/rpy ${Cal.esc(ageTxt(ages.imu))}</small></h3>`;
  if (d) {
    const f = Math.max(0, Math.min(1, a.distance_m / lv.straight_run_m));
    liveHtml += `<div class="calbig">${D.f(a.distance_m, 3)} m <span class="muted">of ${D.f(lv.straight_run_m, 2)} m tape</span></div><div class="bar"><i style="width:${Math.round(f * 100)}%"></i></div>` +
      `<p class="muted" style="font-size:12px">${a.odom_msgs} odom messages in ${D.f(a.elapsed_s, 0)} s. Push straight; stop exactly on the mark.</p>`;
  } else if (sp) {
    const f = Math.max(0, Math.min(1, Math.abs(a.yaw_deg) / 360));
    liveHtml += a.phase === 'arming' ? '<div class="muted">Setting imu_yaw_scale to ±1 for the measurement…</div>'
      : `<div class="calbig">${D.f(a.yaw_deg, 1)}° <span class="muted">of +360° (measured at unit scale)</span></div><div class="bar"><i style="width:${Math.round(f * 100)}%"></i></div>` +
        `<p class="muted" style="font-size:12px">${a.settling ? 'Settling: wait a moment before turning. ' : ''}Turn LEFT (counter-clockwise seen from above). Negative = the IMU turns the wrong way; the page fixes the sign.</p>`;
  } else if (running && st.run) {
    liveHtml += `<div class="calbig">${D.f(st.run.drift_deg, 2)}° <span class="muted">drift so far</span></div><div class="bar"><i style="width:${Math.round((st.run.fraction || 0) * 100)}%"></i></div>` +
      `<p class="muted" style="font-size:12px">${D.f(st.run.remaining_s, 0)} s left, ${st.run.samples} IMU messages. Nobody touches the car or the table.</p>`;
  } else liveHtml += '<div class="muted">Nothing is being measured. Use the buttons under Controls.</div>';
  const STAT = { PASS: ['pass', 'ok-t'], CORRECTED: ['corrected, verify', 'bad-t'], FAIL: ['fail', 'bad-t'], NO_DATA: ['no data', 'bad-t'] };
  const runsTbl = (title, t, cols) => {
    if (!t.runs || !t.runs.length) return `<h4 style="margin:12px 0 4px">${title}</h4><div class="muted">No run yet.</div>`;
    const off = t.n_runs - t.runs.length;
    const last = t.runs[t.runs.length - 1];
    return `<h4 style="margin:12px 0 4px">${title} <small class="muted">${t.n_runs} run${t.n_runs > 1 ? 's' : ''}</small></h4>` +
      `<table class="calruns"><tr><th>#</th>${cols.map(c => `<th class="r">${c[0]}</th>`).join('')}<th>Result</th></tr>` +
      t.runs.map((r, i) => `<tr><td>${off + i + 1}</td>${cols.map(c => `<td class="r">${Cal.esc(c[1](r))}</td>`).join('')}` +
        `<td class="${(STAT[r.status] || ['', 'muted'])[1]}">${Cal.esc((STAT[r.status] || [r.status])[0])}</td></tr>`).join('') + '</table>' +
      (last.status !== 'PASS' && (last.why || last.fix) ? `<div class="calwhy">${last.why ? `<b>Why:</b> ${Cal.esc(last.why)}` : ''}${last.fix ? `<br><b>Fix:</b> ${Cal.esc(last.fix)}` : ''}</div>` : '') +
      (t.failed_runs >= lv.max_runs && last.status !== 'PASS' ? Cal.alert('bad', `${t.failed_runs} runs without a pass`, 'Something mechanical is off (wheel slipping, car not pushed straight, IMU loose). Check it before more runs.') : '');
  };
  liveHtml += runsTbl('Distance runs', dist, [['Odom', r => r.odom_m != null ? D.f(r.odom_m, 3) + ' m' : '—'],
    ['Error', r => r.error_pct != null ? (r.error_pct > 0 ? '+' : '') + D.f(r.error_pct, 1) + ' %' : '—'], ['ticks/m', r => D.f(r.ticks_per_meter, 1)]]);
  if (lv.spin_test) liveHtml += runsTbl('Spins', spin, [['IMU', r => (r.yaw_at_scale_deg != null ? D.f(r.yaw_at_scale_deg, 1) : D.f(r.yaw_change_deg, 1)) + '°'],
    ['Error', r => r.error_pct != null ? (r.error_pct > 0 ? '+' : '') + D.f(r.error_pct, 1) + ' %' : '—'], ['scale', r => D.f(r.imu_yaw_scale, 4)]]);
  const lim = lv.limits || {};
  liveHtml += `<p class="muted" style="font-size:12px;margin:8px 0 0">Limits: distance ±${D.f(lim.distance_pct, 1)} %, spin ±${D.f(lim.spin_pct, 1)} %, drift ${D.f(lim.drift_deg_per_min, 1)} °/min. ` +
    'A corrected run never passes: the next run with the new value verifies it.</p></div>';

  /* result */
  let result = (st.message ? `<p class="muted" style="margin:0 0 10px">${Cal.esc(st.message)}</p>` : '');
  result += running ? Cal.alert('info', `Drift test: ${D.f(st.run ? st.run.remaining_s : 0, 0)} s left`, 'Hands off. STOP MOTORS cancels it.') : calResult(st);
  const r = st.result;
  if (r && r.values && !running) {
    result += `<table class="metric"><tr><th>servo_controller</th><th class="r">${st.unsaved ? 'to save' : 'saved'}</th></tr>` +
      Object.keys(r.values).map(k => `<tr><td>${Cal.esc(k)}</td><td class="r">${Cal.esc(String(r.values[k]))}</td></tr>`).join('') + '</table>';
    (r.warnings || []).forEach(w => { result += Cal.alert('info', 'Note', w); });
  }
  return { todo, ctl, live: liveHtml, result, actions: calActions(st, null) };
};

/* ---- step 7: servo centre + steering. The car DRIVES ITSELF (calibrate mode, through the command owner).
 *      Run starts the sequence; it waits at Go (action STEP {"op":"go"}, allowed while RUNNING) before every
 *      segment: left circle, right circle, straight runs. Cancel / STOP MOTORS stop it at once. */
STEP_PAGES.servo_steering = st => {
  const lv = st.live || {};
  const ins = Cal.list(st.meta.instructions);
  const running = st.status === 'RUNNING';
  const r = lv.run;
  const v = lv.values;
  const busy = lv.busy || '';
  const pr = lv.procedure || {};
  const saved = st.saved_status && !st.unsaved && ['PASS', 'KEPT_PREVIOUS'].includes(st.status);
  const SEG = { left: 1, right: 2, straight: 3 };
  let stage = 0;
  if (saved) stage = ins.length;
  else if (st.status === 'PASS' && st.unsaved) stage = 4;
  else if (running && r) stage = SEG[r.segment] || 0;
  const todo = Cal.todo(ins.map((t, i) => [t, i < stage ? 'done' : i === stage ? 'now' : 'todo', st.status === 'FAIL' && i === stage]));

  /* controls */
  let ctl = '';
  if (lv.error) ctl += Cal.alertBad('Step 7 cannot run', lv.error, 'Fix calibration_steps.yaml (servo_steering) / common.yaml vehicle.wheelbase_m, then relaunch calibrate.launch.py.');
  if (lv.link_error) ctl += Cal.alertBad('servo_controller / command_owner not reachable', lv.link_error, '');
  ctl += '<div class="calvals">' + (v
    ? `servo_center <b>${v.servo_center}</b> · range left <b>${v.servo_range_left}</b> / right <b>${v.servo_range_right}</b><br>` +
      `steer_sign <b>${D.f(v.steer_sign, 0)}</b> · imu_yaw_scale <b>${D.f(v.imu_yaw_scale, 4)}</b> · command_owner mode <b>${Cal.esc(v.mode)}</b>`
    : '<span class="muted">reading servo_controller / command_owner…</span>') +
    (busy ? `<br><span class="muted">${Cal.esc(busy)}…</span>` : '') + '</div>';
  if (v && v.mode !== 'calibrate') ctl += Cal.alertBad('Not in calibrate mode', `command_owner runs in ${v.mode} mode`, 'Start ros2 launch carbot_bringup calibrate.launch.py.');
  if (!running) {
    const hasRun = !!(st.result && st.unsaved) || st.status === 'FAIL' || st.saved_status;
    const ok = v && v.mode === 'calibrate' && !busy;
    ctl += `<button class="btn primary" data-act="${hasRun ? 'REDO' : 'RUN'}" ${ok ? '' : 'disabled'}>${hasRun ? 'Redo calibration drive' : 'Start calibration drive'}` +
      '<small>Nothing moves yet: the car waits for Go before every segment</small></button>' +
      `<button class="btn" data-act="STEP" data-arg="${Cal.esc(JSON.stringify({ op: 'reread' }))}" ${busy ? 'disabled' : ''}>Re-read values</button>`;
  } else if (r) {
    const what = r.segment === 'straight' ? `straight run ${r.straight_n} of up to ${r.max_runs}` : r.label;
    const where = r.segment === 'left' ? 'Put the car on the clear floor, about 1 m of free space on every side.'
      : r.segment === 'right' ? 'Move the car back to the middle of the clear floor if needed.'
        : `Line the car up with ${D.f(pr.straight_run_m, 1)} m free straight ahead${r.straight_n > 1 ? ` (servo_center is now ${r.servo_center})` : ''}.`;
    if (r.phase === 'ready') {
      ctl += Cal.alert('info', `Next: ${what}`, where + ' Keep a hand near STOP MOTORS.') +
        `<button class="btn primary" data-act="STEP" data-arg="${Cal.esc(JSON.stringify({ op: 'go' }))}" ${busy ? 'disabled' : ''}>Go: drive the ${Cal.esc(what)}` +
        `<small>duty ${D.f(pr.raw_duty, 2)}, stops by itself</small></button>`;
    } else if (r.phase === 'correcting') ctl += Cal.alert('info', 'Setting the new servo_center…', `${r.servo_center}, live on servo_controller.`);
    else ctl += Cal.alert('bad', `Driving: ${what}`, 'Press Cancel below or STOP MOTORS (top right) to stop at once.');
    if (r.note) ctl += Cal.alert('info', 'Note', r.note);
  }

  /* live */
  const ages = lv.ages || {};
  const ageTxt = x => x == null ? 'never' : x > 1 ? `${D.f(x, 1)} s old` : 'live';
  let liveHtml = `<div class="panel"><h3>Live <small>/odom ${Cal.esc(ageTxt(ages.odom))} · /imu/rpy ${Cal.esc(ageTxt(ages.imu))}</small></h3>`;
  if (r && ['driving', 'stopping', 'settling'].includes(r.phase)) {
    const f = r.segment === 'straight' ? Math.abs(r.distance_m) / pr.straight_run_m : Math.abs(r.yaw_deg) / pr.circle_yaw_deg;
    liveHtml += `<div class="calbig">${r.segment === 'straight' ? `${D.f(r.distance_m, 2)} m <span class="muted">of ${D.f(pr.straight_run_m, 1)} m</span>`
      : `${D.f(r.yaw_deg, 0)}° <span class="muted">of ${D.f(pr.circle_yaw_deg, 0)}°, ${D.f(r.distance_m, 2)} m driven</span>`}</div>` +
      `<div class="bar"><i style="width:${Math.round(Math.min(1, f) * 100)}%"></i></div>` +
      `<p class="muted" style="font-size:12px">${r.phase === 'settling' ? 'Settling the IMU…' : r.phase === 'stopping' ? 'Stopping…' : `${D.f(r.elapsed_s, 0)} s of at most ${D.f(pr.timeout_s, 0)} s`}</p>`;
  } else liveHtml += `<div class="muted">${running ? 'Waiting for Go.' : 'Not driving.'}</div>`;
  const circ = (r && r.circles) || {};
  const res = st.result || {};
  const cl = Object.keys(circ).length ? circ : (res.left ? { left: { radius_m: res.left.radius_m, yaw_deg: res.left.yaw_change_deg, distance_m: res.left.distance_m },
    right: { radius_m: res.right.radius_m, yaw_deg: res.right.yaw_change_deg, distance_m: res.right.distance_m } } : {});
  liveHtml += '<h4 style="margin:12px 0 4px">Full-lock circles</h4>' + (Object.keys(cl).length
    ? '<table class="calruns"><tr><th>Side</th><th class="r">Driven</th><th class="r">Turned</th><th class="r">Radius</th></tr>' +
      Object.keys(cl).map(k => `<tr><td>${k}</td><td class="r">${D.f(cl[k].distance_m, 2)} m</td><td class="r">${D.f(cl[k].yaw_deg, 0)}°</td>` +
        `<td class="r ${cl[k].radius_m <= lv.limits.min_radius_m_max ? 'ok-t' : 'bad-t'}">${D.f(cl[k].radius_m, 3)} m</td></tr>`).join('') + '</table>'
    : '<div class="muted">Not driven yet.</div>');
  const sr = (r && r.straights) || (res.straight_runs || []).map(x => ({ distance_m: x.distance_m, drift_cm_per_m: x.drift_m_per_m * 100 }));
  liveHtml += '<h4 style="margin:12px 0 4px">Straight runs</h4>' + (sr.length
    ? '<table class="calruns"><tr><th>#</th><th class="r">Driven</th><th class="r">servo_center</th><th class="r">Drift</th></tr>' +
      sr.map((x, i) => `<tr><td>${i + 1}</td><td class="r">${D.f(x.distance_m, 2)} m</td><td class="r">${x.servo_center != null ? x.servo_center : '—'}</td>` +
        `<td class="r ${x.drift_cm_per_m <= lv.limits.straight_drift_cm_per_m ? 'ok-t' : 'bad-t'}">${D.f(x.drift_cm_per_m, 2)} cm/m</td></tr>`).join('') + '</table>'
    : '<div class="muted">Not driven yet.</div>');
  liveHtml += `<p class="muted" style="font-size:12px;margin:8px 0 0">Limits: radius ≤ ${D.f((lv.limits || {}).min_radius_m_max, 2)} m each side, drift ≤ ${D.f((lv.limits || {}).straight_drift_cm_per_m, 1)} cm/m. ` +
    `Wheelbase ${D.f(lv.wheelbase_m, 3)} m (common.yaml) converts the radius to the steering limit.</p></div>`;

  /* result */
  let result = (st.message ? `<p class="muted" style="margin:0 0 10px">${Cal.esc(st.message)}</p>` : '');
  result += running ? Cal.alert('info', 'Calibration drive in progress', 'The result appears after the last straight run.') : calResult(st);
  if (res.steering && !running) {
    result += `<table class="metric"><tr><th>Value</th><th class="r">${st.unsaved ? 'to save' : 'saved'}</th></tr>` +
      `<tr><td>servo_controller.servo_center</td><td class="r">${res.servo_center}</td></tr>` +
      Object.keys(res.steering).map(k => `<tr><td>command_owner.steering.${Cal.esc(k)}</td><td class="r">${Cal.esc(String(res.steering[k]))}</td></tr>`).join('') +
      `<tr><td>vehicle.min_turning_radius_m</td><td class="r">${res.min_turning_radius_m}</td></tr></table>` +
      '<p class="muted" style="font-size:12px;margin:6px 0 0">servo_center is live now; the steering limits apply from the next launch of this session.</p>';
  }
  return { todo, ctl, live: liveHtml, result, actions: calActions(st, null) };
};

/* ---- step 9: venue colour / lighting thresholds (road_perception classify.*, measured from its own output) */
function calVenueMap(m) {
  /* mini map of road_perception's grid (seen part): forward up, left on the left (V4 bev_view flip),
   * one rect per run of equal cells per row; sample boxes outlined */
  if (!m || !m.rows) return '<div class="empty">No road grid from road_perception yet.</div>';
  const COL = ['rgb(24,33,48)', 'rgb(40,110,160)', 'rgb(249,240,202)', 'rgb(69,80,95)'];
  const R = m.rows, C = m.cols, k = m.kinds || '';
  let rects = '';
  for (let r = 0; r < R; r++) {
    const y = R - 1 - r;
    let c = 0;
    while (c < C) {
      const v = k[r * C + c]; let e = c + 1;
      while (e < C && k[r * C + e] === v) e++;
      rects += `<rect x="${C - e}" y="${y}" width="${e - c}" height="1" fill="${COL[+v] || COL[0]}"/>`;
      c = e;
    }
  }
  const BOX = { road: ['#2ecc71', 'open road box'], tunnel: ['#f1c40f', 'tunnel box'] };
  let boxes = '';
  Object.entries(m.boxes || {}).forEach(([p, b]) => {
    if (!b) return;
    const [r0, r1, c0, c1] = b;
    boxes += `<rect x="${C - 1 - c1}" y="${R - 1 - r1}" width="${c1 - c0 + 1}" height="${r1 - r0 + 1}" fill="none" stroke="${(BOX[p] || ['#fff'])[0]}" stroke-width="0.6"${p === 'tunnel' ? ' stroke-dasharray="1.5 1"' : ''}/>`;
  });
  const legend = Object.keys(m.boxes || {}).map(p => `<span style="color:${(BOX[p] || ['#fff'])[0]}">▭ ${Cal.esc((BOX[p] || ['', p])[1])}</span>`).join(' · ');
  return `<svg viewBox="0 0 ${C} ${R}" style="width:100%;max-height:260px;background:rgb(24,33,48);border-radius:8px" preserveAspectRatio="xMidYMid meet" shape-rendering="crispEdges">${rects}${boxes}</svg>` +
    `<p class="muted" style="font-size:12px;margin:4px 0 0">Forward is up, the car is just below the picture. Blue = road, cream = tape, grey = other. ${legend}</p>`;
}

STEP_PAGES.venue_thresholds = (st) => {
  const lv = st.live || {};
  if (lv.error && !lv.phases) return { live: Cal.alertBad('Step 9 cannot run', lv.error, 'Fix calibration_steps.yaml / cameras.yaml, then relaunch calibrate.launch.py.'),
    result: calResult(st), actions: calActions(st, 'Run') };
  const ins = Cal.list(st.meta.instructions);
  const running = st.status === 'RUNNING';
  const run = lv.run || (st.run ? { phase: st.run.phase, state: st.run.state } : null);
  const samples = lv.samples || {};
  const phases = lv.phases || ['road'];
  const pct = v => v == null ? '—' : `${D.f(v * 100, 1)} %`;
  const lim = lv.limits || {};
  const sampled = phases.every(p => samples[p]);
  let stage = 0;
  if (st.saved_status && !st.unsaved && ['PASS', 'KEPT_PREVIOUS'].includes(st.status)) stage = ins.length;
  else if (st.status === 'PASS' && st.unsaved) stage = ins.length - 1;
  else if (sampled) stage = lv.applied_ok ? ins.length - 1 : 3;
  else if (samples.road) stage = 2;
  else if (running && run && run.phase === 'road') stage = 1;
  const todo = Cal.todo(ins.map((t, i) => [t, i < stage ? 'done' : i === stage ? 'now' : 'todo', st.status === 'FAIL' && i === stage]));

  /* controls (every button is a RUN with the phase as argument; Cancel is in the actions row) */
  const btn = (arg, label, small, primary, off) => `<button class="btn ${primary ? 'primary' : ''}" data-act="RUN" data-arg="${arg}" ${running || off ? 'disabled' : ''}>${Cal.esc(label)}<small>${Cal.esc(small)}</small></button>`;
  let ctl = lv.error ? Cal.alertBad('Last action failed', lv.error) : '';
  ctl += btn('road', samples.road ? 'Sample open road again' : 'Sample open road', 'Car in the middle of a straight lane, green box on bare road', lv.next === 'road');
  if (lv.tunnel_check) ctl += btn('tunnel', samples.tunnel ? 'Sample tunnel again' : 'Sample tunnel', 'Car in the tunnel, yellow box on the tunnel floor', lv.next === 'tunnel');
  ctl += btn('apply', 'Apply live', 'Sets the proposed values on road_perception now (Save keeps them)', sampled && !lv.applied_ok, !lv.proposal);
  if (lv.original && lv.current && JSON.stringify(lv.original) !== JSON.stringify(lv.current)) {
    ctl += btn('revert', 'Put back the old values', 'What road_perception had before this step changed it', false, false);
  }
  const P = ['road_max_luma', 'road_max_chroma', 'paint_min_luma'];
  const cur = lv.current || {}, org = lv.original || {}, prop = lv.proposal || {};
  const val = v => Cal.esc(v == null ? '—' : v);
  ctl += `<table class="calcheck" style="margin-top:10px"><tr><th>road_perception classify.</th><th class="r">Before</th><th class="r">Live now</th><th class="r">Proposed</th></tr>` +
    P.map(k => `<tr><td>${k}</td><td class="r">${val(org[k])}</td><td class="r">${val(cur[k])}</td><td class="r"><b>${val(prop[k])}</b></td></tr>`).join('') + '</table>' +
    `<p class="muted" style="font-size:12px;margin:6px 0 0">${lv.applied_ok ? 'The proposed values are running on road_perception.' : 'Live now = what road_perception uses at this moment.'} ` +
    'Rule (V4): road if luma &lt; road_max_luma and colour &lt; road_max_chroma, else tape if luma &gt; paint_min_luma.</p>' +
    (lv.bpu_note ? `<p class="muted" style="font-size:12px;margin:6px 0 0">${Cal.esc(lv.bpu_note)}</p>` : '');

  /* live: road_perception's grid now, and the samples (before / after) */
  const now = lv.now || {};
  const LAB = { road: 'Open road', tunnel: 'Tunnel' };
  let liveHtml = `<div class="panel"><h3>Road grid (road_perception) <small>${lv.grid_age_s != null ? `${D.f(lv.grid_age_s, 1)} s old` : 'waiting'} · cameras: ${Cal.esc((lv.cameras || []).join(', '))}</small></h3>` +
    calVenueMap(lv.map) +
    `<table class="calcheck" style="margin-top:8px"><tr><th>Box, right now</th><th class="r">Cells</th><th class="r">Road</th><th class="r">Tape</th></tr>` +
    phases.map(p => {
      const n = now[p] || {};
      return `<tr><td>${LAB[p] || p}</td><td class="r">${n.cells == null ? '—' : n.cells}</td>` +
        `<td class="r ${n.road != null && n.road < (lim.min_road_coverage || 0) ? 'bad-t' : ''}">${pct(n.road)}</td>` +
        `<td class="r ${n.paint != null && n.paint > (lim.max_false_paint_ratio == null ? 1 : lim.max_false_paint_ratio) ? 'bad-t' : ''}">${pct(n.paint)}</td></tr>`;
    }).join('') + '</table>' +
    `<p class="muted" style="font-size:12px;margin:6px 0 0">Only the box the car is placed for means anything. Limits: road ≥ ${pct(lim.min_road_coverage)}, tape ≤ ${pct(lim.max_false_paint_ratio)} of the box.</p></div>`;
  const rows = phases.filter(p => samples[p]).map(p => samples[p]);
  if (rows.length) {
    liveHtml += `<div class="panel"><h3>Samples <small>before = thresholds live while sampling · after = proposed</small></h3>` +
      `<table class="metric"><tr><th>Sample</th><th class="r">Road before → after</th><th class="r">Tape in box before → after</th><th class="r">Road luma mean / bright end</th><th class="r">Tape luma dark end</th></tr>` +
      rows.map(s => `<tr><td>${Cal.esc(s.label)}<br><span class="muted" style="font-size:11.5px">${s.frames} frames · ${s.box_cells} cells · decode ${pct(s.decode_agreement)}</span></td>` +
        `<td class="r">${pct(s.before.road)} → <b>${pct(s.after.road)}</b></td><td class="r">${pct(s.before.paint)} → <b>${pct(s.after.paint)}</b></td>` +
        `<td class="r">${D.f(s.road_luma_mean, 0)} / ${D.f(s.road_luma_hi, 0)}</td>` +
        `<td class="r">${s.tape_luma_lo != null ? D.f(s.tape_luma_lo, 0) : '—'} <span class="muted">(${s.tape_cells} cells)</span></td></tr>`).join('') + '</table>' +
      (lv.notes || []).map(n => Cal.alert('info', 'Note', n)).join('') + '</div>';
  }

  let result;
  if (running && run) {
    const r = st.run || {};
    const what = run.phase === 'apply' ? 'Applying the values on road_perception…' : run.phase === 'revert' ? 'Putting back the old values…'
      : run.state === 'reading' ? 'Reading road_perception\'s current values…' : `Sampling ${LAB[run.phase] || run.phase}… ${D.f(r.remaining_s, 0)} s left`;
    result = `<div class="alert info"><b class="t">${Cal.esc(what)}</b>${run.frames || r.samples || 0} stitched frames. Keep everyone out of the camera picture.</div>` +
      `<div class="bar"><i style="width:${Math.round((r.fraction || 0) * 100)}%"></i></div>`;
  } else result = calResult(st);
  return { todo, ctl, live: liveHtml, result: (st.message ? `<p class="muted" style="margin:0 0 10px">${Cal.esc(st.message)}</p>` : '') + result,
    actions: calActions(st, null) };
};

/* ---- step 12: mission planner. Map (Global map renderer: gridLines + drawTrack) with the poses
 *      P0..Pn placed by click (drag = heading), planned routes and their roundabout exits.
 *      Poses are drafts in the browser until Run sends them: {"poses": [[x, y, yaw_deg], ...]}. */
const MP_COL = ['#1D63E0', '#B04FD6', '#0FA3A3', '#4C9F38', '#C0392B'];
function mpWrap(a) { return Math.atan2(Math.sin(a), Math.cos(a)); }
function mpInPoly(x, y, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [ax, ay] = poly[i], [bx, by] = poly[j];
    if ((ay > y) !== (by > y) && x < (bx - ax) * (y - ay) / (by - ay) + ax) inside = !inside;
  }
  return inside;
}
/* = mission_planner.py Guide.place with snap on: inside a bay -> centred in it (Road.bay_pose),
 *   else the nearest lane-centre point, heading along the lane closest to the car's (Road.lane_snap) */
function mpSnap(p, t, car) {
  const [x, y, yd] = p, a = yd * Math.PI / 180;
  for (const ar of Object.values(t.areas || {})) {
    if (ar.kind !== 'bay' || ar.heading == null || !mpInPoly(x, y, ar.poly)) continue;
    const cx = ar.poly.reduce((s, q) => s + q[0], 0) / ar.poly.length, cy = ar.poly.reduce((s, q) => s + q[1], 0) / ar.poly.length;
    let h = ar.heading;
    if (Math.abs(mpWrap(a - h)) > Math.PI / 2) h = mpWrap(h + Math.PI);
    const sh = (car.front - car.rear) / 2;
    return [cx - sh * Math.cos(h), cy - sh * Math.sin(h), h * 180 / Math.PI];
  }
  let best = null;
  for (const pts of Object.values(t.sections || {})) {
    for (let i = 0; i + 1 < pts.length; i++) {
      const [ax, ay] = pts[i], [bx, by] = pts[i + 1], dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
      if (L2 < 1e-12) continue;
      const u = Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / L2)), qx = ax + u * dx, qy = ay + u * dy;
      const d = Math.hypot(x - qx, y - qy);
      if (!best || d < best.d) best = { d, x: qx, y: qy, h: Math.atan2(dy, dx) };
    }
  }
  if (!best) return p;
  let h = best.h;
  if (Math.abs(mpWrap(h - a)) > Math.PI / 2) h = mpWrap(h + Math.PI);
  return [best.x, best.y, h * 180 / Math.PI];
}
function mpBody(p, car) {
  const [x, y, yd] = p, a = yd * Math.PI / 180, c = Math.cos(a), s = Math.sin(a);
  return [[-car.rear, -car.half_w], [car.front, -car.half_w], [car.front, car.half_w], [-car.rear, car.half_w]]
    .map(([u, v]) => [x + u * c - v * s, y + u * s + v * c]);
}
const mpSame = (a, b) => a && b && a.length === b.length && a.every((p, i) => p.every((v, k) => Math.abs(v - b[i][k]) < 1e-3));

function MissionMap(box, ctx) {
  box.innerHTML = `<div class="panel"><h3>Mission map <small data-k="mpsrc"></small></h3>
    <div style="display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin-bottom:8px"><div class="seg" data-k="mpsel"></div>
      <button class="btn" data-mp="rot:5" title="Turn left 5°">⟲ 5°</button><button class="btn" data-mp="rot:-5" title="Turn right 5°">⟳ 5°</button>
      <button class="btn" data-mp="flip" title="Turn the car round on the spot">Flip</button>
      <label style="font-size:12.5px"><input type="checkbox" data-k="mpsnap"> Snap to lane / bay</label>
      <button class="btn" data-mp="reset" title="Back to the saved / mission.yaml poses">Reset poses</button></div>
    <div data-k="mpcv" style="touch-action:none;cursor:crosshair"></div>
    <p class="muted" style="font-size:12px;margin:6px 0 8px">Pick a pose above (P0 start … finish), then click on the map to place it; press and drag to set its heading (the drag direction).
      Click a placed car to select it, drag to move it. Poses are rear-axle centre + heading, like mission.yaml.</p>
    <div data-k="mptab"></div>
    <div class="actions" style="margin-top:8px"><button class="btn primary" data-k="mprun" data-act="RUN">Plan routes</button><span class="hint" data-k="mphint"></span></div></div>`;
  const q = k => box.querySelector(`[data-k="${k}"]`);
  let poses = null, touched = false, sel = 0, data = null, drag = null, snap = null, o = null, v = null;

  function send() {
    const b = q('mprun');
    b.dataset.arg = poses ? JSON.stringify({ poses: poses.map(p => p.map(x => Math.round(x * 10000) / 10000)) }) : '';
  }
  function fromServer(lv, st) {
    const res = st.result || {};
    if (lv.running && lv.running.poses) return lv.running.poses;
    if (res.poses && res.poses.length) return res.poses.map(p => [p.x, p.y, p.yaw_deg]);
    if (lv.plan && lv.plan.poses && lv.plan.poses.length) return lv.plan.poses;
    return lv.default_poses || [];
  }
  function world(e) {
    const r = o.cv.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  }
  function draw() {
    if (!o) return;
    const { ctx: g, w, h } = o; g.clearRect(0, 0, w, h); g.fillStyle = D.css('--raised'); g.fillRect(0, 0, w, h);
    const lv = data && data.lv, t = lv && lv.track;
    if (!t) return;
    v = D.fitView(t.bounds, w, h, 16);
    gridLines(g, v, t.bounds, 0.5); drawTrack(g, v, t);
    Object.entries(t.exits || {}).forEach(([k, p]) => D.label(g, v.X(p[0]), v.Y(p[1]), k, D.css('--panel'), D.css('--muted')));
    const plan = lv.plan, stale = !plan || plan.stale || (touched && !mpSame(poses, plan.poses));
    if (plan) (plan.routes || []).forEach((r, i) => (r.pieces || []).forEach(pc => {
      let run = [], dir = null;
      const flush = () => { if (run.length > 1) { g.globalAlpha = stale ? .35 : 1;
        D.line(g, run, p => v.X(p[0]), p => v.Y(p[1]), dir < 0 ? '#E8871E' : MP_COL[i % MP_COL.length], stale ? 2 : 3.5); g.globalAlpha = 1; } };
      pc.pts.forEach(p => { if (dir !== null && p[2] !== dir) { flush(); run = [run[run.length - 1]]; } dir = p[2]; run.push(p); });
      flush();
    }));
    const res = (data.st.result || {}).poses || [];
    (poses || []).forEach((p, i) => {
      const r = res[i], checked = r && Math.abs(r.x - p[0]) < 1e-3 && Math.abs(r.y - p[1]) < 1e-3 && Math.abs(r.yaw_deg - p[2]) < .05;
      const col = checked ? (r.fits ? D.css('--ok') : D.css('--bad')) : D.css('--muted');
      const body = mpBody(p, lv.car);
      D.path(g, body, q2 => v.X(q2[0]), q2 => v.Y(q2[1])); g.closePath();
      g.globalAlpha = .18; g.fillStyle = col; g.fill(); g.globalAlpha = 1;
      g.lineWidth = i === sel ? 3 : 1.5; g.strokeStyle = i === sel ? D.css('--lane') : col; g.stroke();
      D.arrow(g, v.X(p[0]), v.Y(p[1]), p[2] * Math.PI / 180, i === sel ? 10 : 8, i === sel ? D.css('--lane') : col);
      D.label(g, v.X(p[0]) + 18, v.Y(p[1]) - 16, 'P' + i, i === sel ? D.css('--lane') : D.css('--ink'), '#fff');
    });
  }
  function table() {
    const lv = data.lv, st = data.st, res = st.result || {}, rp = res.poses || [];
    const n = lv.legs || 0;
    q('mpsel').innerHTML = (poses || []).map((p, i) => `<button data-mp="sel:${i}" aria-pressed="${i === sel}">P${i}${i === 0 ? ' start' : i === n ? ' finish' : ''}</button>`).join('');
    q('mptab').innerHTML = poses && poses.length ? `<table class="calcheck"><tr><th>Pose</th><th class="r">x m</th><th class="r">y m</th><th class="r">heading</th><th>On the road</th></tr>` +
      poses.map((p, i) => {
        const r = rp[i], checked = r && Math.abs(r.x - p[0]) < 1e-3 && Math.abs(r.y - p[1]) < 1e-3 && Math.abs(r.yaw_deg - p[2]) < .05;
        return `<tr class="${i === sel ? 'cur' : ''}"><td><b>P${i}</b>${i === 0 ? ' start' : i === n ? ' finish' : ''}</td><td class="r">${D.f(p[0], 3)}</td><td class="r">${D.f(p[1], 3)}</td><td class="r">${D.f(p[2], 1)}°</td>` +
          `<td class="${checked ? (r.fits ? 'ok-t' : 'bad-t') : 'muted'}">${checked ? (r.fits ? 'fits' : 'OFF ROAD') + ` (${D.f(r.margin_m * 100, 1)} cm)${r.bay ? ' · ' + Cal.esc(r.bay) : ''}` : 'checked on Run'}</td></tr>`;
      }).join('') + '</table>' : '<div class="empty">No poses yet</div>';
  }
  function changed() { touched = true; send(); table(); draw(); }
  box.addEventListener('click', e => {
    const b = e.target.closest('[data-mp]'); if (!b || !poses) return;
    const [k, a] = b.dataset.mp.split(':');
    const car = data.lv.car;
    if (k === 'sel') { sel = Number(a); table(); draw(); return; }
    if (k === 'reset') { touched = false; poses = fromServer(data.lv, data.st).map(p => p.slice()); send(); table(); draw(); return; }
    const p = poses[sel]; if (!p) return;
    if (k === 'rot') p[2] = mpWrap((p[2] + Number(a)) * Math.PI / 180) * 180 / Math.PI;
    if (k === 'flip') { const d = car.front - car.rear, r = p[2] * Math.PI / 180;
      poses[sel] = [p[0] + d * Math.cos(r), p[1] + d * Math.sin(r), mpWrap(r + Math.PI) * 180 / Math.PI]; }
    changed();
  });
  q('mpsnap').addEventListener('change', e => { snap = e.target.checked; });

  return {
    update(dd) {
      data = dd;
      const lv = dd.lv, st = dd.st;
      if (snap === null) { snap = !!lv.snap; q('mpsnap').checked = snap; }
      if (!o && lv.track) {
        const b = lv.track.bounds;
        o = D.canvas(q('mpcv'), Math.max(1.2, Math.min(2.4, (b[2] - b[0]) / Math.max(b[3] - b[1], 1e-3))), () => draw());
        const cv = o.cv;
        cv.addEventListener('pointerdown', e => {
          if (!v || !poses || data.st.status === 'RUNNING') return;
          const [px, py] = world(e);
          let near = -1, dmin = 16;
          poses.forEach((p, i) => { const d = Math.hypot(v.X(p[0]) - px, v.Y(p[1]) - py); if (d < dmin) { dmin = d; near = i; } });
          const w0 = v.inv(px, py);
          if (near >= 0) { sel = near; const p = poses[near]; drag = { mode: 'move', dx: w0[0] - p[0], dy: w0[1] - p[1], px, py }; }
          else { poses[sel] = [w0[0], w0[1], poses[sel] ? poses[sel][2] : 0]; drag = { mode: 'rotate', px, py }; }
          cv.setPointerCapture(e.pointerId); changed();
        });
        cv.addEventListener('pointermove', e => {
          if (!drag || !v) return;
          const [px, py] = world(e), wq = v.inv(px, py), p = poses[sel];
          if (drag.mode === 'move') { if (Math.hypot(px - drag.px, py - drag.py) < 3) return; p[0] = wq[0] - drag.dx; p[1] = wq[1] - drag.dy; }
          else if (Math.hypot(px - drag.px, py - drag.py) > 10) p[2] = Math.atan2(wq[1] - p[1], wq[0] - p[0]) * 180 / Math.PI;
          else return;
          drag.moved = true; draw(); table();
        });
        const up = () => {
          if (!drag) return;
          if (snap && (drag.mode === 'rotate' || drag.moved)) poses[sel] = mpSnap(poses[sel], data.lv.track, data.lv.car);
          drag = null; changed();
        };
        cv.addEventListener('pointerup', up); cv.addEventListener('pointercancel', up);
      }
      const srv = fromServer(lv, st);
      if (!touched || !poses || poses.length !== srv.length) { poses = srv.map(p => p.slice()); if (!srv.length) poses = null; touched = false; }
      if (sel >= (poses || []).length) sel = 0;
      const running = st.status === 'RUNNING';
      const hasRun = !!(st.result && st.unsaved) || st.status === 'FAIL' || st.saved_status;
      const b = q('mprun');
      b.dataset.act = hasRun ? 'REDO' : 'RUN';
      b.textContent = running ? 'Planning…' : hasRun ? 'Plan again with these poses' : 'Plan routes';
      b.disabled = running || !poses || !!lv.error;
      q('mphint').textContent = running ? 'Cancel is below the result.' : touched && lv.plan && !mpSame(poses, lv.plan.poses)
        ? 'Poses moved since the last plan: the routes drawn are old (faded). Press Plan again.' : '';
      q('mpsrc').textContent = lv.map ? `${lv.map.source === 'session' ? 'this session\'s' : 'repo'} track_map.yaml · sha1 ${String(lv.map.sha1).slice(0, 10)}` : '';
      send(); table(); draw();
    },
    destroy() { o = null; },
  };
}

STEP_PAGES.mission_planner = st => {
  const lv = st.live || {};
  if (lv.error) return { live: Cal.alertBad('Step 12 cannot run', lv.error, 'Fix calibration_steps.yaml (mission_planner.procedure) or the map file, then relaunch calibrate.launch.py.'),
    result: calResult(st), actions: calActions(st, null) };
  const ins = Cal.list(st.meta.instructions);
  const running = st.status === 'RUNNING';
  let stage = 1;
  if (st.saved_status && !st.unsaved && ['PASS', 'KEPT_PREVIOUS'].includes(st.status)) stage = ins.length;
  else if (st.status === 'PASS' && st.unsaved) stage = ins.length - 1;
  else if (running) stage = ins.length - 1;
  const todo = Cal.todo(ins.map((t, i) => [t, i < stage ? 'done' : i === stage ? 'now' : 'todo', st.status === 'FAIL' && i === stage]));
  const map = lv.map || {};
  const ctl = `<div class="kv"><span>Map</span><b>${Cal.esc(map.source === 'session' ? 'this session (step 11)' : 'repo default')}</b>` +
    `<span>File</span><b style="word-break:break-all">${Cal.esc(map.path || '—')}</b><span>sha1</span><b>${Cal.esc(String(map.sha1 || '').slice(0, 12))}</b>` +
    `<span>Start poses from</span><b>${Cal.esc(lv.default_from || 'planner defaults')}</b></div>` +
    '<p class="muted" style="font-size:12px;margin:8px 0 0">The routes and roundabout exits are chosen by the planner from these poses and saved in mission.yaml. ' +
    'Race mode checks them against mission_rules.yaml roundabout_visits; the boom gate never changes them.</p>';

  // routes + exits (from the run in progress, the last plan, or the saved mission.yaml)
  const plan = lv.plan, legs = (st.result && st.result.legs) || [];
  const run = lv.running;
  let liveHtml = '';
  if (running && run) {
    const done = (run.done || []).filter(x => x !== null).length;
    liveHtml = `<div class="panel"><h3>Planning <small>${Cal.esc(run.stage)}</small></h3>` +
      Cal.alert('info', `Planning leg ${Math.max(1, (run.leg || 0) + 1)} of ${lv.legs}`, 'About a minute per leg on the car (a leg whose poses did not move is reused). Keep this page open or come back later.') +
      `<div class="bar" style="margin-top:8px"><i style="width:${Math.round(100 * done / Math.max(1, lv.legs))}%"></i></div></div>`;
  } else if (plan) {
    liveHtml = `<div class="panel"><h3>Routes <small>${Cal.esc(plan.from)}${plan.stale ? ' · planned on a different map: press Plan again' : ''}</small></h3>` +
      `<table class="calcheck"><tr><th>Leg</th><th>From → to</th><th class="r">Length</th><th>Roundabout exits</th><th>State</th></tr>` +
      (plan.routes || []).map((r, i) => {
        const L = legs[i] || {};
        return `<tr${r.ok ? '' : ' class="badrow"'}><td><b style="color:${MP_COL[i % MP_COL.length]}">leg${i + 1}</b></td><td>P${i} → P${i + 1}${r.pieces && r.pieces.some(p => p.kind === 'manoeuvre') ? ' · parking' : ''}</td>` +
          `<td class="r">${r.ok ? D.f(r.length_m, 2) + ' m' : '—'}</td><td>${r.ok ? Cal.esc((r.exits || []).join(', ') || 'none') : '—'}</td>` +
          `<td class="${r.ok ? 'ok-t' : 'bad-t'}">${r.ok ? 'route' + (L.seconds ? ` (${D.f(L.seconds, 0)} s)` : '') : Cal.esc(r.reason || L.reason || 'no route')}</td></tr>`;
      }).join('') + '</table><p class="muted" style="font-size:12px;margin:8px 0 0">Orange = reversing (parking manoeuvre preview; block 11 replans it from the observed bay).</p></div>';
  }
  const b07 = st.result && st.result.block07;
  let result = (st.message ? `<p class="muted" style="margin:0 0 10px">${Cal.esc(st.message)}</p>` : '') + calResult(st);
  if (!running && b07 && b07.visits && b07.visits.length) result += `<table class="metric" style="margin-top:8px"><tr><th>Roundabout visit</th><th>Leg</th><th>Exit</th></tr>` +
    b07.visits.map(x => `<tr><td>${x.visit}</td><td>${Cal.esc(x.leg)}</td><td>${Cal.esc(x.exit)}</td></tr>`).join('') + '</table>';
  if (running) result = Cal.alert('info', 'Planning…', 'The result appears here when every leg is planned and checked.');
  return { todo, ctl, live: liveHtml, result, actions: calActions(st, null),
    widget: { id: 'mission_map', create: MissionMap, data: { st, lv } } };
};

/* ---- step 13: per-challenge practice runs (optional). The page never drives: it records attempts
 *      from /carbot/mission/state + events; the user grades them. RUN argument JSON {op: start|grade|done}. */
STEP_PAGES.practice_runs = st => {
  const lv = st.live || {};
  if (lv.error) return { live: Cal.alertBad('Step 13 cannot run', lv.error, 'Fix calibration_steps.yaml / challenges.yaml, then relaunch calibrate.launch.py.'),
    result: calResult(st), actions: calActions(st, null) };
  const running = st.status === 'RUNNING';
  const cur = lv.current;
  const rows = lv.challenges || [];
  const atts = lv.attempts || [];
  const levels = lv.levels || [];
  const blocked = !!st.blocked_by;
  const ins = Cal.list(st.meta.instructions);
  const act = (st.result && st.unsaved) || st.status === 'FAIL' || st.saved_status ? 'REDO' : 'RUN';
  const arg = o => Cal.esc(JSON.stringify(o));
  const t = v => v == null ? '—' : D.f(v, 1) + ' s';
  const LV = { EXCELLENT: 'ok-t', COMPLETED: 'ok-t', PARTIAL: 'warn-t', FAIL: 'bad-t' };
  const lvl = v => v ? `<span class="${LV[v] || ''}">${Cal.esc(v.toLowerCase())}</span>` : '<span class="muted">not graded</span>';

  let stage = 0;
  if (st.saved_status === 'PASS' && !st.unsaved) stage = ins.length;
  else if (st.status === 'PASS') stage = ins.length - 1;
  else if (running) stage = 2;
  else if ((lv.ungraded || []).length) stage = 3;
  else if (lv.n_attempts) stage = ins.length - 1;
  const todo = Cal.todo(ins.map((x, i) => [x, i < stage ? 'done' : i === stage ? 'now' : 'todo', false]));

  /* controls: current attempt (Stop) or one Start button per challenge, then Practice done */
  let ctl = '';
  if (running && cur) {
    ctl = `<div class="alert info"><b class="t">Recording attempt ${cur.n}: ${cur.challenge} · ${Cal.esc(cur.name)}</b>` +
      `${D.f(cur.elapsed_s, 0)} s${cur.entered_s != null ? ` · in the challenge since ${D.f(cur.entered_s, 0)} s` : ' · mission has not entered it yet'}` +
      `${cur.left_s != null ? ` · left at ${D.f(cur.left_s, 0)} s` : ''}</div>` +
      `<button class="btn" data-act="CANCEL">Stop attempt<small>Ends and keeps the attempt (outcome: stopped). STOP MOTORS also ends it.</small></button>`;
  } else if (running) {
    ctl = '<div class="muted">Working…</div>';
  } else {
    ctl = `<table class="calcheck"><tr><th>#</th><th>Challenge</th><th class="r"></th></tr>` + rows.map(r =>
      `<tr><td>${r.id}</td><td>${Cal.esc(r.name)}${r.mandatory ? '' : ' <span class="muted">(bonus)</span>'}</td>` +
      `<td class="r"><button class="btn ${r.attempts ? '' : 'primary'}" data-act="${act}" data-arg="${arg({ op: 'start', challenge: r.id })}" ${blocked ? 'disabled' : ''}>Start attempt</button></td></tr>`).join('') + '</table>' +
      `<button class="btn good" style="margin-top:10px" data-act="${act}" data-arg="${arg({ op: 'done' })}" ${lv.n_attempts && !blocked ? '' : 'disabled'}>Practice done` +
      `<small>${lv.n_attempts ? `${lv.n_attempts} attempt(s) recorded; failed ones are kept` : 'Record at least one attempt first'}</small></button>`;
  }

  /* live: warnings, mission line, per-challenge table, current attempt timeline */
  let liveHtml = (lv.warnings || []).map(w => Cal.alert('info', 'Note', w)).join('');
  const m = lv.mission;
  liveHtml += `<div class="panel"><h3>Mission <small>${m ? `state ${D.f(m.age_s, 1)} s old` : 'no /carbot/mission/state'}</small></h3>` +
    (m ? `<div class="kv"><span>Mode</span><b>${Cal.esc(m.mode || '—')}</b><span>Challenge</span><b>${m.challenge_id ? `${m.challenge_id} ${Cal.esc(m.challenge_name)}` : 'between challenges'}</b>` +
      `<span>Hold</span><b>${Cal.esc(m.hold_reason || '—')}</b><span>Armed</span><b>${lv.armed === true ? 'yes' : lv.armed === false ? 'no' : 'unknown'}</b>` +
      `<span>Manual control</span><b class="${lv.manual ? 'bad-t' : ''}">${lv.manual ? 'ON' : 'off'}</b></div>` : '<div class="muted">mission_logic is not publishing.</div>') + '</div>';
  const practised = rows.filter(r => r.attempts);
  liveHtml += `<div class="panel"><h3>Practice so far <small>${lv.n_attempts || 0} attempts</small></h3>` +
    (practised.length ? `<table class="calcheck"><tr><th>Challenge</th><th class="r">Attempts</th><th>Last result</th><th class="r">Last time</th><th class="r">Best time</th><th class="r">Best marks</th></tr>` +
      practised.map(r => `<tr${r.last_level === 'FAIL' ? ' class="badrow"' : ''}><td><b>${r.id}</b> ${Cal.esc(r.name)}</td><td class="r">${r.attempts}${r.failed ? ` <span class="muted">(${r.failed} failed)</span>` : ''}</td>` +
        `<td>${lvl(r.last_level)}</td><td class="r">${t(r.last_time_s)}</td><td class="r">${t(r.best_time_s)}</td><td class="r">${r.best_marks == null ? '—' : `${r.best_marks} / ${r.max_marks}`}</td></tr>`).join('') + '</table>'
      : '<div class="empty">No attempt yet. Press Start attempt next to a challenge.</div>') + '</div>';
  if (cur) {
    const tl = (cur.transitions || []).map(x => `<tr><td class="r">${D.f(x.t_s, 1)} s</td><td>mode ${Cal.esc(x.mode)}${x.challenge_id ? ` · challenge ${x.challenge_id}` : ''}${x.hold_reason ? ` · ${Cal.esc(x.hold_reason)}` : ''}</td></tr>`)
      .concat((cur.events || []).map(e => `<tr><td class="r">${D.f(e.t_s, 1)} s</td><td><b>${Cal.esc(e.name)}</b> ${Cal.esc(e.detail)}</td></tr>`));
    liveHtml += `<div class="panel"><h3>This attempt</h3>${tl.length ? `<table class="metric">${tl.join('')}</table>` : '<div class="muted">No mission change yet.</div>'}</div>`;
  }

  /* result: recent attempts with grade buttons */
  let result = st.message ? `<p class="muted" style="margin:0 0 10px">${Cal.esc(st.message)}</p>` : '';
  if (st.status === 'PASS') result += Cal.alert('ok', st.unsaved ? 'Practice done: press Save' : 'Practice saved', (st.result || {}).summary || '');
  else if (st.status === 'KEPT_PREVIOUS') result += Cal.alert('info', 'Using the previous practice record', `Kept from ${st.from_session || st.previous}.`);
  result += atts.length ? `<table class="metric"><tr><th>#</th><th>Challenge</th><th>Ended</th><th class="r">Time</th><th>Grade</th></tr>` + atts.map(a =>
    `<tr><td>${a.n}</td><td>${a.challenge} ${Cal.esc(a.name)}</td><td class="muted">${Cal.esc(a.outcome_text)}</td>` +
    `<td class="r">${t(a.challenge_time_s != null ? a.challenge_time_s : a.duration_s)}</td><td>${lvl(a.level)}${a.marks != null ? ` · ${a.marks} marks` : ''}` +
    (running ? '' : `<div style="margin-top:4px">${levels.map(L => `<button class="btn" style="padding:2px 8px;font-size:11.5px" data-act="${act}" data-arg="${arg({ op: 'grade', attempt: a.n, level: L })}">${Cal.esc(L.toLowerCase())}</button>`).join(' ')}</div>`) +
    '</td></tr>').join('') + '</table>' : '<div class="muted">Attempts appear here; grade each one.</div>';
  result += `<p class="muted" style="font-size:12px;margin:8px 0 0">The scoreboard node is not running, so grades are yours; only a manual intervention is graded FAIL automatically (rulebook: 0 marks).</p>`;

  const save = `<button class="btn good" data-act="SAVE" ${st.status === 'PASS' && st.unsaved ? '' : 'disabled'}>Save</button>`;
  const keep = st.can_keep && !running ? `<button class="btn" data-act="KEEP_PREVIOUS">Keep previous practice<small>${Cal.esc(st.previous)}</small></button>` : '';
  const actions = save + keep + '<span class="hint">Optional step: it never blocks race mode. Save writes practice/&lt;challenge&gt;.yaml + practice/summary.yaml into the session.</span>';
  return { todo, ctl, live: liveHtml, result, actions };
};

/* ---- step 8: speed feedforward + PID. The car DRIVES ITSELF straight, forward and back alternately.
 *      Run starts the sequence; it waits at Go (action STEP {"op":"go"}) before every segment:
 *      duty sweep (CALIBRATION_RAW), then closed-loop speed steps (CALIBRATION). */
const calSpark = (pts, target) => {
  if (!pts || pts.length < 2) return '';
  const W = 300, Hh = 70, t1 = pts[pts.length - 1][0] || 1;
  const vs = pts.map(p => p[1]).concat(target != null ? [target, 0] : [0]);
  const lo = Math.min(...vs), hi = Math.max(...vs), span = hi - lo || 1;
  const X = t => (t / t1 * W).toFixed(1), Y = v => (Hh - 4 - (v - lo) / span * (Hh - 8)).toFixed(1);
  const tl = target != null ? `<line x1="0" x2="${W}" y1="${Y(target)}" y2="${Y(target)}" stroke="currentColor" stroke-dasharray="4 3" opacity=".45"/>` : '';
  return `<svg viewBox="0 0 ${W} ${Hh}" style="width:100%;height:70px;display:block;margin:6px 0;color:var(--muted)">${tl}` +
    `<polyline fill="none" stroke="var(--lane)" stroke-width="2" points="${pts.map(p => X(p[0]) + ',' + Y(p[1])).join(' ')}"/></svg>`;
};
STEP_PAGES.speed_pid = st => {
  const lv = st.live || {};
  const ins = Cal.list(st.meta.instructions);
  const running = st.status === 'RUNNING';
  const r = lv.run;
  const v = lv.values;
  const busy = lv.busy || '';
  const pr = lv.procedure || {};
  const lim = lv.limits || {};
  const saved = st.saved_status && !st.unsaved && ['PASS', 'KEPT_PREVIOUS'].includes(st.status);
  let stage = 0;
  if (saved) stage = ins.length;
  else if (st.status === 'PASS' && st.unsaved) stage = 3;
  else if (running && r) stage = r.kind === 'sweep' ? 1 : 2;
  const todo = Cal.todo(ins.map((t, i) => [t, i < stage ? 'done' : i === stage ? 'now' : 'todo', st.status === 'FAIL' && i === stage]));
  const cms = x => x == null ? '—' : `${D.f(x * 100, 1)} cm/s`;

  /* controls */
  let ctl = '';
  if (lv.error) ctl += Cal.alertBad('Step 8 cannot run', lv.error, 'Fix calibration_steps.yaml (speed_pid), then relaunch calibrate.launch.py.');
  if (lv.link_error) ctl += Cal.alertBad('command_owner not reachable', lv.link_error, '');
  ctl += '<div class="calvals">' + (v
    ? `feedforward <b>${D.f(v.ff.static_duty, 3)}</b> + <b>${D.f(v.ff.duty_per_mps, 3)}</b> × |v| · ` +
      `kp <b>${D.f(v.pid.kp, 3)}</b> ki <b>${D.f(v.pid.ki, 3)}</b> kd <b>${D.f(v.pid.kd, 3)}</b><br>command_owner mode <b>${Cal.esc(v.mode)}</b>`
    : '<span class="muted">reading command_owner…</span>') +
    (busy ? `<br><span class="muted">${Cal.esc(busy)}…</span>` : '') + '</div>';
  if (v && v.mode !== 'calibrate') ctl += Cal.alertBad('Not in calibrate mode', `command_owner runs in ${v.mode} mode`, 'Start ros2 launch carbot_bringup calibrate.launch.py.');
  if (!running) {
    const hasRun = !!(st.result && st.unsaved) || st.status === 'FAIL' || st.saved_status;
    const ok = v && v.mode === 'calibrate' && !busy;
    ctl += `<button class="btn primary" data-act="${hasRun ? 'REDO' : 'RUN'}" ${ok ? '' : 'disabled'}>${hasRun ? 'Redo speed calibration' : 'Start speed calibration'}` +
      '<small>Nothing moves yet: the car waits for Go before every step</small></button>' +
      `<button class="btn" data-act="STEP" data-arg="${Cal.esc(JSON.stringify({ op: 'reread' }))}" ${busy ? 'disabled' : ''}>Re-read values</button>`;
  } else if (r) {
    const sg = r.segment;
    if (r.phase === 'ready' && sg) {
      ctl += Cal.alert('info', `Next: ${sg.label}`, `The car drives ${sg.direction.toUpperCase()} for ${D.f(pr.hold_s, 0)} s. ` +
        'Keep about 1.5 m free that way (move it back to the middle if needed) and a hand near STOP MOTORS.') +
        `<button class="btn primary" data-act="STEP" data-arg="${Cal.esc(JSON.stringify({ op: 'go' }))}" ${busy ? 'disabled' : ''}>Go: ${Cal.esc(sg.direction)}` +
        `<small>${sg.source === 'CALIBRATION_RAW' ? `raw duty ${D.f(sg.command, 2)}` : `${D.f(sg.command, 3)} m/s closed loop`}, stops by itself</small></button>`;
    } else if (r.phase === 'applying') ctl += Cal.alert('info', 'Setting command_owner parameters…', r.kind === 'verify' && r.fit ? `feedforward ${D.f(r.fit.static_duty, 3)} + ${D.f(r.fit.duty_per_mps, 3)} × |v|, kp ${D.f(r.pid.kp, 3)} ki ${D.f(r.pid.ki, 3)}` : '');
    else ctl += Cal.alert('bad', `Driving: ${sg ? sg.label : ''}`, 'Press Cancel below or STOP MOTORS (top right) to stop at once.');
    if (r.note) ctl += Cal.alert('info', 'Note', r.note);
  }

  /* live */
  const ages = lv.ages || {};
  const ageTxt = x => x == null ? 'never' : x > 1 ? `${D.f(x, 1)} s old` : 'live';
  let liveHtml = `<div class="panel"><h3>Live <small>/odom ${Cal.esc(ageTxt(ages.odom))} · speed ${D.f(lv.speed_mps, 3)} m/s</small></h3>`;
  if (r && ['driving', 'stopping'].includes(r.phase) && r.segment) {
    const sg = r.segment;
    const tgt = sg.source === 'CALIBRATION' ? sg.command : null;
    liveHtml += `<div class="calbig">${D.f(r.speed_mps, 3)} m/s <span class="muted">${tgt != null ? `target ${D.f(tgt, 3)}` : `duty ${D.f(sg.command, 2)}`} · ${D.f(r.distance_m, 2)} m</span></div>` +
      `<div class="bar"><i style="width:${Math.round(Math.min(1, r.elapsed_s / (r.hold_s || 1)) * 100)}%"></i></div>` + calSpark(r.trace, tgt) +
      `<p class="muted" style="font-size:12px">${r.phase === 'stopping' ? 'Stopping…' : `${D.f(r.elapsed_s, 1)} s of ${D.f(r.hold_s, 0)} s; steady = mean of the last ${D.f(pr.steady_s, 1)} s`}</p>`;
  } else liveHtml += `<div class="muted">${running ? (r && r.phase === 'applying' ? 'Setting parameters…' : 'Waiting for Go.') : 'Not driving.'}</div>`;
  const res = st.result || {};
  const cap = res.capture || {};
  const sweep = (r && r.sweep) || (cap.sweep || []).map(x => ({ duty: x[0], speed_mps: x[1] }));
  liveHtml += `<h4 style="margin:12px 0 4px">Duty sweep <small class="muted">${sweep.length} of ${(pr.sweep_duties || []).length}</small></h4>` + (sweep.length
    ? '<table class="calruns"><tr><th>#</th><th class="r">Duty</th><th class="r">Steady speed</th></tr>' +
      sweep.map((x, i) => `<tr><td>${i + 1}</td><td class="r">${D.f(x.duty, 2)}</td><td class="r">${D.f(x.speed_mps, 3)} m/s</td></tr>`).join('') + '</table>'
    : '<div class="muted">Not driven yet.</div>');
  const fit = (r && r.fit) || (res.feedforward && res.feedforward.duty_per_mps > 0 ? { static_duty: res.feedforward.static_duty, duty_per_mps: res.feedforward.duty_per_mps, residual_mps: res.feedforward.fit_residual_mps } : null);
  if (fit) liveHtml += `<p style="margin:6px 0">Fit: duty = <b>${D.f(fit.static_duty, 3)}</b> + <b>${D.f(fit.duty_per_mps, 3)}</b> × |v| <span class="muted">(residual ${cms(fit.residual_mps)})</span></p>`;
  const runs = (r && r.runs ? r.runs.map(x => ({ pid: x.pid, metrics: x.metrics })) : (res.verify_runs || []).map(x => ({ pid: x.pid, metrics: x.metrics })));
  if (r && r.kind === 'verify' && r.metrics.length) runs.push({ pid: r.pid, metrics: r.metrics, current: true });
  liveHtml += '<h4 style="margin:12px 0 4px">Speed steps</h4>' + (runs.length
    ? '<table class="calruns"><tr><th>Run</th><th class="r">Target</th><th class="r">Steady</th><th class="r">Error</th><th class="r">Overshoot</th></tr>' +
      runs.map((u, k) => u.metrics.map((m, i) => `<tr><td>${i ? '' : `${k + 1}${u.current ? '*' : ''} <span class="muted">kp ${D.f(u.pid.kp, 2)} ki ${D.f(u.pid.ki, 2)}</span>`}</td>` +
        `<td class="r">${D.f(m.target, 3)}</td><td class="r">${D.f(m.steady, 3)}</td>` +
        `<td class="r ${m.steady_error * 100 <= lim.max_steady_error_cm_s ? 'ok-t' : 'bad-t'}">${cms(m.steady_error)}</td>` +
        `<td class="r ${m.overshoot_pct <= lim.max_overshoot_pct ? 'ok-t' : 'bad-t'}">${D.f(m.overshoot_pct, 0)} %</td></tr>`).join('')).join('') + '</table>'
    : '<div class="muted">Not driven yet.</div>');
  const retunes = (r && r.retunes) || res.retunes || [];
  if (retunes.length) liveHtml += `<p class="muted" style="font-size:12px;margin:6px 0 0">Retuned: ${retunes.map(Cal.esc).join('; ')}</p>`;
  liveHtml += `<p class="muted" style="font-size:12px;margin:8px 0 0">Limits: steady error ≤ ${D.f(lim.max_steady_error_cm_s, 1)} cm/s, overshoot ≤ ${D.f(lim.max_overshoot_pct, 0)} %, ` +
    `slowest moving sweep speed ≤ ${D.f(lim.creep_limit_cm_s, 1)} cm/s. Up to ${Cal.esc(pr.max_runs)} runs of the speed steps.</p></div>`;

  /* result (a terminal-tool result has checks as a {name: bool} dict: show it as rows) */
  let stc = st;
  if (res.checks && !Array.isArray(res.checks)) {
    stc = Object.assign({}, st, { result: Object.assign({}, res, { checks: Object.keys(res.checks).map(k => ({ label: k, measured: '', limit: '', passed: res.checks[k], why: '' })) }) });
  }
  let result = (st.message ? `<p class="muted" style="margin:0 0 10px">${Cal.esc(st.message)}</p>` : '');
  result += running ? Cal.alert('info', 'Speed calibration in progress', 'The result appears after the last speed step.') : calResult(stc);
  if (res.feedforward && res.pid && !running && res.feedforward.duty_per_mps > 0) {
    result += `<table class="metric"><tr><th>command_owner</th><th class="r">${st.unsaved ? 'to save' : 'saved'}</th></tr>` +
      `<tr><td>feedforward.static_duty</td><td class="r">${D.f(res.feedforward.static_duty, 4)}</td></tr>` +
      `<tr><td>feedforward.duty_per_mps</td><td class="r">${D.f(res.feedforward.duty_per_mps, 4)}</td></tr>` +
      Object.keys(res.pid).map(k => `<tr><td>speed_pid.${Cal.esc(k)}</td><td class="r">${D.f(res.pid[k], 4)}</td></tr>`).join('') + '</table>' +
      '<p class="muted" style="font-size:12px;margin:6px 0 0">Live on command_owner now (a failed run puts the old values back). Save also writes the tunnel_bridge copy and captures/speed.json.</p>';
  }
  return { todo, ctl, live: liveHtml, result, actions: calActions(st, null) };
};

/* ---- step 10: UWB anchor survey + offsets. One stage per Run; the argument is built from the
 *      form when the button is clicked (STEP_ARGS.uwb_survey), typed values survive the re-render */
STEP_ARGS.uwb_survey = (el, b) => {
  const stage = b.dataset.stage;
  const num = (sel, what) => {
    const i = el.querySelector(sel); const v = i ? String(i.value).trim().replace(',', '.') : '';
    if (v === '' || !isFinite(Number(v))) throw new Error(`${what}: enter a number (metres)`);
    return Number(v);
  };
  try {
    if (stage === 'link') return { arg: { stage } };
    if (stage === 'survey') {
      const anchors = [];
      el.querySelectorAll('[data-uwb-anchor]').forEach(tr => {
        const k = tr.dataset.uwbAnchor;
        const id = String(tr.querySelector('[data-f="id"]').value || '').trim().toUpperCase();
        if (!id) { if (tr.dataset.extra) return; throw new Error('an anchor has no id'); }
        anchors.push({ id, xyz_m: ['x', 'y', 'z'].map(f => num(`[data-uwb-anchor="${k}"] [data-f="${f}"]`, `anchor ${id} ${f === 'z' ? 'height' : f}`)) });
      });
      return { arg: { stage, anchors, tag: { z_m: num('[data-uwb-tag="z"]', 'tag antenna height'),
        mount_xy_m: [num('[data-uwb-tag="fwd"]', 'tag position forward'), num('[data-uwb-tag="left"]', 'tag position left')] } } };
    }
    return { arg: { stage, spot: [num(`[data-uwb-spot="${stage}-x"]`, `${stage} spot x`), num(`[data-uwb-spot="${stage}-y"]`, `${stage} spot y`)] } };
  } catch (err) { return { error: String(err.message || err) }; }
};

STEP_PAGES.uwb_survey = (st, live) => {
  const lv = st.live || {};
  if (lv.error) return { live: Cal.alertBad('Step 10 cannot run', lv.error, 'Fix calibration_steps.yaml / uwb.yaml, then relaunch calibrate.launch.py.'),
    result: calResult(st), actions: calActions(st, null) };
  const running = st.status === 'RUNNING';
  const stages = lv.stages || [];
  const S = Object.fromEntries(stages.map(s => [s.key, s]));
  const lim = lv.limits || {};
  const form = lv.form || { anchors: [], tag: {} };
  const ins = Cal.list(st.meta.instructions);
  const saved = st.saved_status && !st.unsaved && ['PASS', 'KEPT_PREVIOUS'].includes(st.status);
  /* what to do: the YAML instructions, ticked off by the stage that covers them */
  const done = k => S[k] && S[k].state === 'pass';
  /* instructions 0-4 = layout / survey / tag (form), 5 = OFFSETS, 6 = VERIFY */
  let stage = saved ? ins.length : done('verify') ? ins.length - 1 : done('offsets') ? 6 : done('survey') ? 5 : 0;
  stage = Math.min(stage, ins.length);
  const todo = Cal.todo(ins.map((t, i) => [t, i < stage ? 'done' : i === stage ? 'now' : 'todo', st.status === 'FAIL' && i === stage]));

  /* controls: 4 stage boxes. The HTML only changes when the form defaults or button states change */
  const dis = running ? 'disabled' : '';
  const act = st.saved_status || st.result ? 'REDO' : 'RUN';
  const v = x => (x == null || x === '' ? '' : Cal.esc(x));
  const inp = (attr, val, ro) => `<input type="text" inputmode="decimal" data-keep ${attr} value="${v(val)}" ${ro ? 'readonly' : ''} style="width:64px">`;
  const conf = new Set(lv.configured || []);
  const rows = (form.anchors || []).map((a, i) => `<tr data-uwb-anchor="${i}" data-row="a${i}"><td>${inp('data-f="id"', a.id, conf.has(a.id))}</td>` +
    ['x', 'y', 'z'].map((f, j) => `<td>${inp(`data-f="${f}"`, a.xyz_m ? a.xyz_m[j] : '')}</td>`).join('') + '</tr>').join('') +
    `<tr data-uwb-anchor="new" data-row="anew" data-extra="1"><td>${inp('data-f="id" placeholder="4th id"', '')}</td>` +
    ['x', 'y', 'z'].map(f => `<td>${inp(`data-f="${f}"`, '')}</td>`).join('') + '</tr>';
  const btn = (stg, label, sub, primary) => `<button class="btn ${primary ? 'primary' : ''}" data-act="${act}" data-argfrom="uwb_survey" data-stage="${stg}" ${dis}>${Cal.esc(label)}<small>${Cal.esc(sub)}</small></button>`;
  const nxt = lv.next || '';
  const tag = form.tag || {};
  const mxy = tag.mount_xy_m || ['', ''];
  const spot = (k, d) => `<div class="row"><label>x (m)${inp(`data-uwb-spot="${k}-x"`, d ? d[0] : '')}</label><label>y (m)${inp(`data-uwb-spot="${k}-y"`, d ? d[1] : '')}</label></div>`;
  const box = (k, n, body) => `<div style="border:1px solid var(--line);border-radius:8px;padding:8px 10px"><b>${n}. ${Cal.esc(S[k] ? S[k].label : k)}</b> ` +
    `${S[k] ? `<span class="chip ${S[k].state === 'pass' ? 'ok' : S[k].state === 'fail' ? 'bad' : S[k].state === 'running' ? 'blue' : 'grey'}">${Cal.esc(S[k].state)}</span>` : ''}<div class="ctl" style="margin-top:6px">${body}</div></div>`;
  const ctl = box('link', 1, btn('link', 'Link check', `Listens ${lim.link_check_s || 5} s: every anchor must be heard`, nxt === 'link')) +
    box('survey', 2, `<p class="muted" style="margin:0;font-size:12px">Tape-measured antenna positions in METRES from anchor 1782 (+x towards 1786, +y towards 1783); z = height above the floor. ` +
      'Leave the last row empty unless you add a 4th anchor (its id must also be in RangeProtocol.h).</p>' +
      `<table class="calcheck uwbform"><tr><th>Anchor</th><th>x</th><th>y</th><th>height</th></tr>${rows}</table>` +
      `<div class="row"><label>Tag height (m)${inp('data-uwb-tag="z"', tag.z_m)}</label><label>Tag fwd of rear axle (m)${inp('data-uwb-tag="fwd"', mxy[0])}</label>` +
      `<label>Tag left (m)${inp('data-uwb-tag="left"', mxy[1])}</label></div>` + btn('survey', 'Save survey', 'Checks spacing, angles and accuracy coverage (instant)', nxt === 'survey')) +
    box('offsets', 3, spot('offsets', lv.offset_spot_m) + btn('offsets', 'Measure offsets', `Tag STILL at this spot, > ${lim.min_distance_from_anchor_m || 1} m from every anchor, ${lim.seconds || 20} s`, nxt === 'offsets')) +
    box('verify', 4, spot('verify', lv.verify && lv.verify.spot_m) + btn('verify', 'Verify', `A different spot (>= ${lim.min_verify_separation_m || 0.5} m away), ${lim.seconds || 20} s, error <= ${D.f((lim.max_verify_error_m || 0.15) * 100, 0)} cm`, nxt === 'verify'));

  /* live: feed, per-anchor table, geometry, stage why/fix */
  const feed = lv.feed;
  const g = lv.geometry || {};
  let h = '';
  if (!feed || !feed.reports) h += Cal.alert('bad', 'No UWB tag reports yet', 'Tag powered and on WiFi? micro-ROS agent running (step 1)? ROS_LOCALHOST_ONLY=0, ROS_DOMAIN_ID=1.');
  else if (feed.age_s != null && feed.age_s > 2) h += Cal.alert('bad', `Last tag report ${D.f(feed.age_s, 1)} s ago`, 'The tag stopped reporting: check its power / WiFi.');
  if (feed && feed.unknown_ids && feed.unknown_ids.length) h += Cal.alert('info', `Heard anchor ids not in the survey: ${feed.unknown_ids.join(', ')}`, 'Add them in the survey (and RangeProtocol.h), or switch them off.');
  const run = lv.run;
  if (running && run && st.run) {
    h += `<div class="alert info"><b class="t">${Cal.esc(S[run.stage] ? S[run.stage].label : run.stage)}… ${D.f(st.run.remaining_s, 0)} s left</b>` +
      `${run.reports} tag reports${run.spot_m ? ` · tag at (${D.f(run.spot_m[0], 2)}, ${D.f(run.spot_m[1], 2)})` : ''}. Keep the tag still and stay out of the line of sight.</div>` +
      `<div class="bar"><i style="width:${Math.round((st.run.fraction || 0) * 100)}%"></i></div>`;
  }
  const f3 = x => (x == null ? '—' : D.f(x, 3));
  const arows = (lv.anchors || []).map(a => {
    const stale = a.age_s == null || a.age_s > 1.0;
    return `<tr${stale ? ' class="badrow"' : ''}><td><b>${Cal.esc(a.id)}</b>${a.configured ? '' : ' <span class="muted">(new)</span>'}<br><span class="muted" style="font-size:11.5px">${a.xyz_m ? a.xyz_m.map(q => D.f(q, 2)).join(', ') : '—'}</span></td>` +
      `<td class="r ${stale ? 'bad-t' : ''}">${a.age_s == null ? 'never' : D.f(a.age_s, 1) + ' s'}</td><td class="r">${a.rate_hz == null ? '—' : D.f(a.rate_hz, 1) + ' Hz'}</td>` +
      `<td class="r">${f3(a.last_raw_m)}</td><td class="r">${a.samples == null ? '—' : a.samples}</td>` +
      `<td class="r ${a.noisy ? 'warn-t' : ''}">${a.spread_m == null ? '—' : D.f(a.spread_m * 100, 1) + ' cm'}</td>` +
      `<td class="r">${a.offset_m == null ? '—' : (a.offset_m >= 0 ? '+' : '') + D.f(a.offset_m, 3)}</td></tr>`;
  }).join('');
  h += `<div class="panel"><h3>UWB anchors <small>${feed ? `${D.f(feed.hz, 1)} Hz tag reports · boot ${Cal.esc(feed.boot_id || '—')}` : 'no feed'}</small></h3>` +
    `<table class="calcheck"><tr><th>Anchor</th><th class="r">Age</th><th class="r">Rate</th><th class="r">Raw range (m)</th><th class="r">Samples</th><th class="r">Spread</th><th class="r">Offset (m)</th></tr>${arows}</table>` +
    `<p class="muted" style="font-size:12px;margin:8px 0 0">Raw = uncorrected 3-D range. Spread above ${D.f((lim.max_spread_m || 0.08) * 100, 0)} cm = multipath: raise the anchor / clear the line of sight.` +
    `${lv.saved_flags ? ` Loaded uwb.yaml: surveyed ${lv.saved_flags.anchors_surveyed ? 'yes' : 'no'}, offsets ${lv.saved_flags.offsets_calibrated ? 'yes' : 'no'}.` : ''}</p></div>`;
  h += `<div class="panel"><h3>Layout ${lv.entered ? '(your survey)' : '(uwb.yaml as loaded)'} <small>HDOP ≤ ${Cal.esc(lim.hdop_limit)} over ${D.f((g.hdop_coverage || 0) * 100, 0)} % of the ${Cal.esc(g.hdop_area || '')} (need ${D.f((lim.min_hdop_coverage || 0) * 100, 0)} %)</small></h3>` +
    ((g.errors || []).map(e => Cal.alert('bad', 'Layout problem', e)).join('') + (g.warnings || []).map(w => Cal.alert('info', 'Warning', w)).join('') || '<div class="muted">No layout problems.</div>') + '</div>';
  const vf = lv.verify;
  if (vf && vf.error_m != null) {
    h += `<div class="panel"><h3>Verify</h3><div class="kv"><span>Tape spot</span><b>${D.f(vf.spot_m[0], 2)}, ${D.f(vf.spot_m[1], 2)}</b>` +
      `<span>Median fix</span><b>${D.f(vf.median_fix_m[0], 2)}, ${D.f(vf.median_fix_m[1], 2)}</b>` +
      `<span>Error</span><b class="${vf.status === 'PASS' ? 'ok-t' : 'bad-t'}">${D.f(vf.error_m * 100, 1)} cm (limit ${D.f((lim.max_verify_error_m || 0.15) * 100, 0)})</b>` +
      `<span>Jitter</span><b>${D.f(vf.jitter_m * 100, 1)} cm</b><span>Flip-flop</span><b>${vf.flip_flop_m > 0 ? D.f(vf.flip_flop_m * 100, 0) + ' cm (multipath)' : 'none'}</b></div></div>`;
  }
  const bad = stages.filter(s => s.state === 'fail');
  if (bad.length && !running) h += bad.map(s => Cal.alertBad(`${s.label} failed`, s.why, s.fix)).join('');

  let result = (st.message ? `<p class="muted" style="margin:0 0 10px">${Cal.esc(st.message)}</p>` : '');
  result += running ? Cal.alert('info', 'Measuring…', 'The result appears here when this stage is done.') : calResult(st);
  const u = st.result && st.result.uwb;
  if (u && u.anchors) result += `<table class="metric" style="margin-top:8px"><tr><th>Anchor</th><th class="r">x, y, z (m)</th><th class="r">Offset (m)</th></tr>` +
    u.anchors.map(a => `<tr><td>${Cal.esc(a.id)}</td><td class="r">${a.xyz_m.map(q => D.f(q, 2)).join(', ')}</td><td class="r">${D.f(a.range_offset_m, 3)}</td></tr>`).join('') + '</table>' +
    `<p class="muted" style="font-size:12px;margin:6px 0 0">Save writes these, tag z ${D.f(u.tag.z_m, 2)} m and mount ${u.tag.mount_xy_m.map(q => D.f(q, 2)).join(', ')} m into the session's data/uwb.yaml (track alignment is step 11).</p>`;
  return { todo, ctl, live: h, result, actions: calActions(st, null) };
};

/* ---- step 11: map-to-UWB alignment. Lap (Start lap / Stop lap) or points (one named map pose per
 *      Record point). The fit, fixes and anchors are drawn on the track map (out.map). */
STEP_ARGS.map_uwb_alignment = (el, b) => {
  if (b.dataset.mode === 'lap') return { arg: { mode: 'lap' } };
  const s = el.querySelector('[data-uwb11="pose"]');
  const pose = s ? String(s.value || '') : '';
  if (!pose) return { error: 'Points: choose the map pose the car is parked on' };
  return { arg: { mode: 'points', pose } };
};

STEP_PAGES.map_uwb_alignment = st => {
  const lv = st.live || {};
  if (lv.error) return { live: Cal.alertBad('Step 11 cannot run', lv.error, 'Fix calibration_steps.yaml (map_uwb_alignment) / the map files, then relaunch calibrate.launch.py.'),
    result: calResult(st), actions: calActions(st, null) };
  const running = st.status === 'RUNNING';
  const run = lv.run;
  const lim = lv.limits || {};
  const modes = lv.modes || ['lap', 'points'];
  const ins = Cal.list(st.meta.instructions);
  const saved = st.saved_status && !st.unsaved && ['PASS', 'KEPT_PREVIOUS'].includes(st.status);
  /* instructions: 0 needs, 1 lap, 2 points, 3 check */
  let stage = saved ? ins.length : st.status === 'PASS' ? 3 : !lv.ready ? 0 : (run && run.mode === 'points') || (lv.points || []).length ? 2 : 1;
  stage = Math.min(stage, ins.length);
  const todo = Cal.todo(ins.map((t, i) => [t, i < stage ? 'done' : i === stage ? 'now' : 'todo', st.status === 'FAIL' && i === stage]));

  /* controls */
  const act = st.saved_status || st.result ? 'REDO' : 'RUN';
  const off = !lv.ready || running ? 'disabled' : '';
  let ctl = '';
  if (!lv.ready) ctl += `<div class="alert bad"><b class="t">Step 10 is not saved in this session</b>${Cal.esc(lv.blocked)} <button class="btn" data-go="cal-10">Go to step 10</button></div>`;
  if (modes.includes('lap')) {
    ctl += '<h4 style="margin:6px 0">Lap (best)</h4>' + (running && run && run.mode === 'lap'
      ? `<button class="btn primary" data-act="STEP" data-arg="${Cal.esc(JSON.stringify({ op: 'stop' }))}">Stop lap<small>after one full slow lap (at least ${D.f(lim.lap_min_s, 0)} s)</small></button>`
      : `<button class="btn primary" data-act="${act}" data-argfrom="map_uwb_alignment" data-mode="lap" ${off}>Start lap<small>car EXACTLY on the start pose, facing west</small></button>`);
  }
  if (modes.includes('points')) {
    const poses = Object.keys(lv.poses || {}).sort();
    const rec = lv.points || [];
    ctl += '<h4 style="margin:12px 0 6px">Points (quick)</h4>' +
      `<div class="row"><label>Car parked on <select data-keep data-uwb11="pose">${poses.map(p => `<option value="${Cal.esc(p)}">${Cal.esc(p)}</option>`).join('')}</select></label></div>` +
      `<button class="btn" data-act="${act}" data-argfrom="map_uwb_alignment" data-mode="points" ${off}>Record point<small>${D.f(lim.points_seconds, 0)} s still; ${lim.min_points || 2}+ poses at least ${D.f(lim.points_min_extent_m, 1)} m apart</small></button>` +
      (rec.length ? `<p class="muted" style="font-size:12px;margin:6px 0">Recorded: ${rec.map(p => `${Cal.esc(p.pose)} (${p.ranges} ranges)`).join(', ')}</p>` +
        `<button class="btn" data-act="STEP" data-arg="${Cal.esc(JSON.stringify({ op: 'clear_points' }))}" ${running ? 'disabled' : ''}>Clear points</button>` : '');
  }

  /* live */
  const feed = lv.feed;
  const ages = lv.inputs || {};
  const age = a => (a == null ? 'never' : D.f(a, 1) + ' s');
  const stale = a => a == null || a > (lim.max_input_age_s || 1);
  let h = '';
  if (!feed || !feed.reports) h += Cal.alert('bad', 'No UWB tag reports yet', 'Tag powered and on WiFi? micro-ROS agent running (step 1)?');
  h += `<div class="panel"><h3>Inputs</h3><div class="kv"><span>UWB tag</span><b>${feed ? `${D.f(feed.hz, 1)} Hz, last ${age(feed.age_s)}` : 'no feed'}</b>` +
    `<span>/odom</span><b class="${stale(ages.odom) ? 'bad-t' : ''}">${age(ages.odom)}</b><span>/imu/rpy</span><b class="${stale(ages.imu) ? 'bad-t' : ''}">${age(ages.imu)}</b>` +
    `<span>Step 10 data</span><b class="${lv.ready ? 'ok-t' : 'bad-t'}">${lv.ready ? `${lv.step10.anchors} anchors, offsets calibrated` : 'missing'}</b></div></div>`;
  const fit = run ? run.fit : lv.last && lv.last.fit;
  if (run) {
    const secs = run.mode === 'lap' ? lim.lap_min_s : lim.points_seconds;
    h += `<div class="alert info"><b class="t">${run.mode === 'lap' ? 'Recording the lap' : `Recording ${Cal.esc(run.pose)}`}… ${D.f(run.elapsed_s, 0)} s</b>` +
      (run.mode === 'lap' ? 'Push / drive SLOWLY around the whole track, then press Stop lap.' : 'Keep the car still on the pose.') + '</div>' +
      `<div class="bar"><i style="width:${Math.min(100, Math.round(100 * run.elapsed_s / Math.max(secs || 1, 1)))}%"></i></div>`;
    if (run.mode === 'lap') {
      h += `<div class="panel"><h3>Lap so far</h3><div class="kv"><span>Tag reports</span><b>${run.reports}${run.lost ? ` <span class="bad-t">(${run.lost} lost)</span>` : ''}</b>` +
        `<span>Ranges matched</span><b>${run.samples}</b>` +
        `<span>Area covered</span><b class="${run.extent_m >= lim.min_extent_m ? 'ok-t' : ''}">${D.f(run.extent_m, 1)} m <span class="muted">(need ${D.f(lim.min_extent_m, 1)})</span></b>` +
        `<span>Map pose (tag)</span><b>${run.tag_xy ? run.tag_xy.map(v => D.f(v, 2)).join(', ') : '—'}</b>` +
        `<span>UWB fix (track)</span><b>${run.fix_xy ? run.fix_xy.map(v => D.f(v, 2)).join(', ') : '—'}</b>` +
        `<span>Fix vs map pose</span><b class="${run.fix_error_m != null && run.fix_error_m > lim.inlier_m ? 'warn-t' : ''}">${run.fix_error_m == null ? '—' : D.f(run.fix_error_m * 100, 0) + ' cm'} <span class="muted">${Cal.esc(run.fix_frame || '')}</span></b></div></div>`;
    }
  }
  if (fit) {
    h += `<div class="panel"><h3>${run ? 'Fit so far' : 'Last fit'}</h3><div class="kv"><span>track → venue</span><b>x ${D.f(fit.x_m, 3)} m, y ${D.f(fit.y_m, 3)} m, yaw ${D.f(fit.yaw_deg, 2)}°</b>` +
      `<span>Range RMS</span><b class="${fit.rms_m <= lim.max_rms_m ? 'ok-t' : 'bad-t'}">${D.f(fit.rms_m * 100, 1)} cm <span class="muted">(limit ${D.f(lim.max_rms_m * 100, 0)})</span></b>` +
      `<span>Inliers</span><b class="${fit.inlier_frac >= lim.min_inlier_frac ? 'ok-t' : 'bad-t'}">${D.f(fit.inlier_frac * 100, 0)} % of ${fit.ranges} <span class="muted">(min ${D.f(lim.min_inlier_frac * 100, 0)})</span></b></div></div>`;
  }

  /* map overlay (track frame) */
  const ov = (run && run.overlay) || (lv.last && lv.last.overlay) || { poses: lv.poses };
  const map = { label: run ? 'Track map · live' : 'Track map · UWB fixes moved with the fitted transform',
    path: ov.path, fixes: ov.fixes, anchors: ov.anchors, poses: ov.poses || lv.poses, points: ov.points,
    car: run && run.pose, fix: run && run.fix_xy,
    legend: '<span><i style="background:var(--lane)"></i>Car path (odometry)</span><span><i style="background:var(--ok)"></i>UWB fixes</span>' +
      '<span><i style="background:var(--warn)"></i>Anchors</span><span><i style="background:var(--muted)"></i>Named poses</span>' };

  let result = (st.message ? `<p class="muted" style="margin:0 0 10px">${Cal.esc(st.message)}</p>` : '');
  result += running ? Cal.alert('info', 'Recording…', 'The result appears here when the lap / point is done.') : calResult(st);
  const t = st.result && st.result.track_to_venue;
  if (t) result += `<p class="muted" style="font-size:12px;margin:6px 0 0">Save writes ONLY track_to_venue (x ${D.f(t.x_m, 3)}, y ${D.f(t.y_m, 3)}, yaw ${D.f(t.yaw_deg, 2)}°, aligned) into the session's data/uwb.yaml; step 10's anchors and offsets stay. Check that the green fixes lie on the road before saving.</p>`;
  return { todo, ctl, live: h, map, result, actions: calActions(st, null) };
};

/* ---- a built step without a custom page (fallback) */
STEP_PAGES._generic = st => ({
  todo: Cal.todo(Cal.list(st.meta.instructions).map(t => [t, 'todo'])),
  live: st.live && st.live.error ? Cal.alertBad('This step cannot run', st.live.error) : '',
  result: calResult(st), actions: calActions(st, 'Run'),
});

/* ---- steps whose page is not built yet */
STEP_PAGES._placeholder = (st, live, d) => {
  const ins = Cal.list(st.meta.instructions);
  const tool = st.meta.tool || '';
  const cmd = st.meta.tool_cmd || '';
  const tab = Cal.TAB[st.meta.tab || ''];
  let ctl = '<span class="muted">No controls until this page is built.</span>';
  if (tool) {
    ctl = cmd ? `<p style="margin:0 0 6px">Until then, run it in a terminal on the car (it saves into this wizard session and this page picks up the result):</p><pre style="white-space:pre-wrap;background:var(--raised);border:1px solid var(--line);border-radius:8px;padding:8px;font-size:12px;margin:0">${Cal.esc(cmd)}</pre>`
      : `<p style="margin:0">Terminal tool: <code>${Cal.esc(tool)}</code>.<br>Save step 1 first, so a session exists for it to write into.</p>`;
  }
  return {
    todo: ins.length ? Cal.todo(ins.map(t => [t, 'todo'])) : '<li><span class="muted">Instructions come with this step\'s page.</span></li>',
    ctl,
    live: `<div class="panel"><h3>Live view</h3>${Cal.alert('info', 'This step\'s wizard page is not built yet', 'It is next in the build order. The Run button stays off until then.')}` +
      (tab ? `<button class="btn" data-go="${tab}">Open the diagnostic tab for this step</button>` : '') + '</div>',
    result: st.saved_status ? calResult(st) : `<div class="muted">${st.previous ? `Previous PASS in ${Cal.esc(st.previous)}.` : 'No result yet.'}</div>`,
    actions: calActions(st, 'Run'),
  };
};
