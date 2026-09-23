/* tabs_calib.js — calibration wizard pages (phase 8).
 *   TABS.calibration  Overview: every step, sessions + rollback.
 *   TABS.calstep      One page per step (tab id cal-N), data from /api/tab/calibration:
 *                     {steps (CalibrationState), live (the OPEN step, /carbot/calibration/live), wizard (heartbeat)}.
 * Built pages: sensor_health (step 1), camera_identity (step 2), camera_intrinsics (step 3). Every other step is a placeholder that shows its
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
      const tab = Cal.TAB[(st.meta && st.meta.tab) || ''];
      const box = q('embedbox');
      if (tab && TABS[tab]) {
        box.hidden = false;
        if (box.dataset.tab !== tab) { box.dataset.tab = tab; if (box.open) startEmbed(tab); }
        q('embedsum').textContent = `Live ${ctx.cfg.tabs.find(t => t.id === tab)?.title || tab} tab (opens its data only while expanded)`;
      } else box.hidden = true;
    }
    return { update(d) { render(d || {}); }, destroy() { stopEmbed(); stopCams(); camLoop.destroy(); } };
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

/* ---- step 7: servo centre + steering limits (the wizard drives the car through the command owner) */
STEP_PAGES.servo_steering = st => {
  const lv = st.live || {};
  if (lv.error) return { live: Cal.alertBad('Step 7 cannot run', lv.error, 'Fix calibration_steps.yaml / ops.yaml calibration_wizard.drive, then relaunch calibrate.launch.py.'),
    result: calResult(st), actions: calActions(st, 'Run circles') };
  const running = st.status === 'RUNNING';
  const lim = lv.limits || {};
  const circ = lv.circles || {};
  const runs = lv.straight_runs || [];
  const drv = lv.drive || {};
  const hasCircles = !!(circ.left && circ.right);
  const r = st.result;
  /* circles done, no straight run yet: not a failure, just the next run to do (no red box) */
  const interim = !running && r && st.unsaved && !r.passed && !r.aborted && r.next === 'straight' && !(r.straight_runs || []).length;
  const ins = Cal.list(st.meta.instructions);
  let stage = hasCircles ? 2 : 1;
  if (st.saved_status && !st.unsaved && ['PASS', 'KEPT_PREVIOUS'].includes(st.status)) stage = ins.length;
  else if (st.status === 'PASS' && st.unsaved) stage = ins.length - 1;
  const todo = Cal.todo(ins.map((t, i) => [t, i < stage ? 'done' : i === stage ? 'now' : 'todo', st.status === 'FAIL' && !interim && i === stage]));

  /* controls: e-stop reminder, input state, the two runs */
  const act = (st.result || st.saved_status) ? 'REDO' : 'RUN';
  const blocked = running || !!lv.input_problem;
  const runsLeft = Math.max(0, (lv.max_runs || 0) - runs.length);
  let ctl = Cal.alert('bad', 'The car drives by itself', 'Clear flat floor of about 1.5 x 2.5 m. Keep STOP MOTORS (header) under your finger: one press stops the car and cancels the run.');
  const okTxt = (ok, name) => `<span class="${ok ? 'ok-t' : 'bad-t'}">${name} ${ok ? 'ok' : 'missing'}</span>`;
  ctl += `<p style="margin:8px 0;font-size:12.5px">${okTxt(drv.odom_ok, '/odom')} · ${okTxt(drv.imu_ok, '/imu/rpy')} · ` +
    `${drv.estop ? '<span class="bad-t">STOP MOTORS pressed</span>' : '<span class="ok-t">STOP MOTORS released</span>'}` +
    ` · owner: <b>${Cal.esc(drv.owner_winner || '—')}</b></p>`;
  if (lv.input_problem && !running) ctl += Cal.alert('bad', 'Cannot drive yet', lv.input_problem);
  ctl += `<button class="btn ${hasCircles ? '' : 'primary'}" data-act="${act}" data-arg="circles" ${blocked ? 'disabled' : ''}>` +
    `Run circles<small>Full LEFT lock, then full RIGHT lock, until ${Cal.esc(lim.circle_yaw_deg)}° each (starts the step over)</small></button>`;
  ctl += `<button class="btn ${hasCircles ? 'primary' : ''}" data-act="${act}" data-arg="straight" ${blocked || !hasCircles || !runsLeft ? 'disabled' : ''}>` +
    `Straight run<small>${hasCircles ? `${Cal.esc(lim.straight_run_m)} m straight ahead; ${runsLeft} of ${Cal.esc(lv.max_runs)} runs left` : 'Run circles first'}</small></button>`;

  /* live view */
  const seg = lv.segment;
  const run = st.run || {};
  const PH = { params: 'Reading servo_controller / command_owner parameters', left: 'Left circle (full LEFT lock)', right: 'Right circle (full RIGHT lock)',
    straight: `Straight run ${runs.length + 1}`, apply: 'Setting servo_center live' };
  let liveHtml = `<div class="panel"><h3>Live drive <small>raw duty ${Cal.esc(lim.raw_duty)}${lv.servo_center != null ? ` · servo_center ${Cal.esc(lv.servo_center)}` : ''}${lv.steer_sign != null ? ` · steer_sign ${Cal.esc(lv.steer_sign)}` : ''}</small></h3>`;
  if (running) {
    liveHtml += `<p style="margin:0 0 6px;font-size:15px"><b>${Cal.esc(PH[lv.phase] || lv.phase || 'Starting')}</b></p>` +
      `<div class="bar" style="margin:6px 0"><i style="width:${Math.round((run.fraction || 0) * 100)}%"></i></div>`;
    if (seg) liveHtml += `<div class="kv"><span>Yaw turned</span><b>${D.f(seg.yaw_deg, 1)}°${lv.phase !== 'straight' ? ` of ${Cal.esc(lim.circle_yaw_deg)}°` : ''}</b>` +
      `<span>Distance</span><b>${D.f(seg.distance_m, 2)} m${lv.phase === 'straight' ? ` of ${Cal.esc(lim.straight_run_m)} m` : ` (max ${Cal.esc(lim.circle_max_distance_m)} m)`}</b>` +
      (lv.phase !== 'straight' ? `<span>Radius so far</span><b>${seg.radius_m != null ? D.f(seg.radius_m, 3) + ' m' : '—'}</b>`
        : `<span>Drift so far</span><b>${seg.drift_m_per_m != null ? D.f(seg.drift_m_per_m * 100, 2) + ' cm/m' : '—'}</b>`) +
      `<span>Owner</span><b class="${seg.accepted ? 'ok-t' : 'warn-t'}">${seg.accepted ? 'driving (CALIBRATION_RAW)' : 'waiting for the command owner'}</b>` +
      `<span>State</span><b>${Cal.esc(seg.state)} · ${D.f(seg.elapsed_s, 1)} s</b></div>`;
  }
  const yes = ok => `<td class="${ok ? 'ok-t' : 'bad-t'}">${ok ? 'ok' : 'fail'}</td>`;
  liveHtml += `<table class="calcheck" style="margin-top:8px"><tr><th>Circle</th><th class="r">Distance</th><th class="r">Yaw</th><th class="r">Radius</th><th></th></tr>` +
    ['left', 'right'].map(s => { const c = circ[s]; return c ? `<tr><td>${s === 'left' ? 'Left lock' : 'Right lock'}</td><td class="r">${D.f(c.distance_m, 2)} m</td>` +
      `<td class="r">${D.f(c.yaw_deg, 0)}°</td><td class="r">${D.f(c.radius_m, 3)} m</td><td class="${c.ok ? 'ok-t' : 'bad-t'}">${c.ok ? 'ok' : 'wrong way'}</td></tr>`
      : `<tr class="muted"><td>${s === 'left' ? 'Left lock' : 'Right lock'}</td><td class="r" colspan="4">not run</td></tr>`; }).join('') +
    `<tr><td colspan="5" class="muted" style="font-size:11.5px">Pass: larger radius <= ${Cal.esc(lim.min_radius_m_max)} m</td></tr></table>`;
  liveHtml += `<table class="calcheck" style="margin-top:8px"><tr><th>Straight run</th><th class="r">Distance</th><th class="r">Yaw</th><th class="r">Drift</th><th class="r">servo_center</th><th></th></tr>` +
    (runs.length ? runs.map((r, i) => `<tr><td>Run ${i + 1}</td><td class="r">${D.f(r.distance_m, 2)} m</td><td class="r">${D.f(r.yaw_deg, 1)}°</td>` +
      `<td class="r">${D.f(r.drift_m_per_m * 100, 2)} cm/m</td><td class="r">${Cal.esc(r.servo_center)}</td>${yes(r.ok)}</tr>`).join('')
      : '<tr class="muted"><td colspan="6">none yet</td></tr>') +
    `<tr><td colspan="6" class="muted" style="font-size:11.5px">Pass: drift <= ${D.f((lim.straight_drift_m_per_m || 0) * 100, 1)} cm/m. A failed run corrects servo_center live.</td></tr></table></div>`;

  /* result: the CLI's checks dict stays in the file; the table uses check_rows */
  let result = st.message && !interim ? `<p class="muted" style="margin:0 0 10px">${Cal.esc(st.message)}</p>` : '';
  if (running) result += Cal.alert('info', 'Driving…', 'The result appears here when the car has stopped.');
  else {
    if (interim) result += Cal.alert('info', 'Circles measured: next, the straight run', r.summary) +
      `<table class="metric"><tr><th>Check</th><th class="r">Measured</th><th class="r">Limit</th><th>Result</th></tr>` +
      (r.check_rows || []).map(c => `<tr><td>${Cal.esc(c.label)}</td><td class="r">${Cal.esc(c.measured)}</td><td class="r">${Cal.esc(c.limit)}</td>` +
        `<td class="${c.passed ? 'ok-t' : 'muted'}">${c.passed ? 'pass' : 'to do'}</td></tr>`).join('') + '</table>';
    else result += calResult(r ? Object.assign({}, st, { result: Object.assign({}, r, { checks: r.check_rows || [] }) }) : st);
    if (r && r.note) result += `<p style="margin:8px 0 0"><b>Last run:</b> ${Cal.esc(r.note)}</p>`;
    if (r && r.steering) result += `<table class="metric" style="margin-top:8px"><tr><th>Writes</th><th class="r">Value</th></tr>` +
      `<tr><td>servo_controller.servo_center</td><td class="r">${Cal.esc(r.servo_center)}</td></tr>` +
      `<tr><td>command_owner.steering.steer_sign</td><td class="r">${Cal.esc(r.steering.steer_sign)}</td></tr>` +
      `<tr><td>steering.left_max_rad / right_max_rad</td><td class="r">${D.f(r.steering.left_max_rad, 3)} / ${D.f(r.steering.right_max_rad, 3)}</td></tr>` +
      `<tr><td>vehicle.min_turning_radius_m</td><td class="r">${D.f(r.min_turning_radius_m, 3)}</td></tr></table>`;
  }
  return { todo, ctl, live: liveHtml, result, actions: calActions(st, null) };
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
