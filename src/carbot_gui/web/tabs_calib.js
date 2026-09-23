/* tabs_calib.js — calibration wizard pages (phase 8).
 *   TABS.calibration  Overview: every step, sessions + rollback.
 *   TABS.calstep      One page per step (tab id cal-N), data from /api/tab/calibration:
 *                     {steps (CalibrationState), live (the OPEN step, /carbot/calibration/live), wizard (heartbeat)}.
 * Built pages: sensor_health (step 1), camera_identity (step 2), camera_intrinsics (step 3), mission_planner (step 12). Every other step is a placeholder that shows its
 * instructions and terminal tool until its page is built.
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

    /* ---- selecting this step on the wizard (its live view follows the open page) */
    const select = () => { selectedAt = Date.now(); Cal.act(ctx, index, 'SELECT'); };
    select();

    /* ---- one delegated click handler for every button on the page */
    el.addEventListener('click', async e => {
      const b = e.target.closest('[data-act]'); if (!b || b.disabled || busy) return;
      busy = true; b.disabled = true;
      const r = await Cal.act(ctx, index, b.dataset.act, b.dataset.arg || '');
      busy = false;
      lastErr = r.ok ? '' : r.message;
      ctx.toast(r.message, r.ok ? 3500 : 7000);
      if (last) render(last);
    });

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
      const st = live.step && live.step.index === index ? live.step : null;
      q('head').innerHTML = head(d, s, st);
      const p = Cal.problem(d);
      let al = p ? Cal.alertBad(p) : '';
      if (lastErr) al += Cal.alertBad('Last action refused', lastErr);
      if (!p && !st) {
        al += Cal.alert('info', 'Opening this step…', 'Waiting for calibration_wizard to switch its live view here.');
        if (Date.now() - selectedAt > 3000) select();
      }
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
      q('ctl').innerHTML = out.ctl || '<span class="muted">Nothing to set for this step.</span>';
      setCams(out.cams);
      q('live').innerHTML = out.live || '';
      q('result').innerHTML = out.result || '';
      q('actions').innerHTML = out.actions || '';
      drawCam(out.cam);
      setWidget(out.widget);
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
