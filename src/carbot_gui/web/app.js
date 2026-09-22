/* app.js — GUI shell (phase 7): navigation, split view, header, manual control, e-stop.
 * Each open tab polls /api/tab/<id> at its own rate; the header polls /api/core.
 * Nothing polls while the browser tab is hidden, so an idle laptop costs the car nothing. */
'use strict';
const App = (() => {
  const $ = id => document.getElementById(id);
  let cfg = null, core = null, split = false, focus = 0, fails = 0;
  const cur = [null, null];
  const panes = [...document.querySelectorAll('.pane')].map(el => ({ el, body: el.querySelector('.panebody'),
    sel: el.querySelector('select'), inst: null, timer: null, tab: null }));

  async function api(path, body) {
    const opt = body === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body) };
    const r = await fetch(path, opt);
    if (r.status === 204) return null;
    const ct = r.headers.get('Content-Type') || '';
    return ct.includes('json') ? r.json() : r.blob();
  }
  function toast(msg, ms = 3500) {
    const t = $('toast'); t.textContent = msg; t.style.display = 'block';
    clearTimeout(toast.h); toast.h = setTimeout(() => { t.style.display = 'none'; }, ms);
  }

  /* ------------------------------------------------------------ navigation */
  const title = id => (cfg.tabs.find(t => t.id === id) || { title: id }).title;
  function buildNav() {
    const race = cfg.mode === 'race';
    let h = '';
    if (race && cfg.split_view) h += `<button class="splitbtn" id="splitbtn" aria-pressed="${split}">${split ? 'Single view' : 'Split view'}</button>`;
    h += `<h2>${race ? 'Tabs' : 'Calibration mode'}</h2>`;
    let n = 0;
    cfg.tabs.forEach(t => {
      const num = (t.id === 'drive' || t.id === 'calibration' || t.id === 'tuning') ? '' : ++n;
      if (!race && t.id === 'map') h += '<h2>Diagnostics</h2>';
      h += `<button class="navbtn" data-tab="${t.id}"><span class="n">${num}</span>${D.esc(t.title)}<span class="flag" data-flag="${t.id}"></span></button>`;
    });
    $('rail').innerHTML = h;
    panes.forEach(p => { p.sel.innerHTML = cfg.tabs.map(t => `<option value="${t.id}">${D.esc(t.title)}</option>`).join(''); });
    markNav();
  }
  function markNav() {
    document.querySelectorAll('#rail .navbtn').forEach(b =>
      b.setAttribute('aria-current', String(b.dataset.tab === cur[0] || (split && b.dataset.tab === cur[1]))));
  }

  function rateOf(id) { const t = TABS[id]; return t && t.rate ? Math.max(0.2, t.rate(cfg)) : 1; }

  function mount(pi, id) {
    const p = panes[pi];
    if (p.tab === id && p.inst) return;
    unmount(pi);
    p.tab = id; cur[pi] = id; p.sel.value = id;
    const sec = document.createElement('section');
    p.body.innerHTML = ''; p.body.appendChild(sec);
    const T = TABS[id];
    if (!T) { sec.innerHTML = `<div class="empty">Unknown tab ${D.esc(id)}</div>`; return; }
    p.inst = T.create(sec, ctx);
    const tick = async () => {
      if (!document.hidden && p.inst) {
        try {
          if (T.poll !== false) {
            const q = p.inst.query ? p.inst.query() : '';
            const d = await api(`/api/tab/${id}${q ? '?' + q : ''}`);
            if (p.inst && p.tab === id) p.inst.update(d || {}, core);
          } else if (p.inst.update) p.inst.update(null, core);
        } catch (e) { /* the header shows the connection state */ }
      }
      if (p.tab === id) p.timer = setTimeout(tick, 1000 / rateOf(id));
    };
    tick();
    markNav();
  }
  function unmount(pi) {
    const p = panes[pi];
    clearTimeout(p.timer); p.timer = null;
    if (p.inst && p.inst.destroy) p.inst.destroy();
    p.inst = null; p.tab = null;
  }
  function setSplit(on) {
    split = on;
    document.getElementById('panes').classList.toggle('split', split);
    panes[1].el.hidden = !split;
    if (split) {
      const other = cfg.tabs.map(t => t.id).find(t => t !== cur[0] && t !== 'drive') || cfg.tabs[0].id;
      mount(1, cur[1] && cur[1] !== cur[0] ? cur[1] : other);
    } else { unmount(1); focus = 0; }
    panes.forEach((p, i) => p.el.classList.toggle('focus', split && i === focus));
    buildNav();
  }
  function go(id) {
    if (!cfg.tabs.some(t => t.id === id)) return;
    const pi = split ? focus : 0;
    if (split && cur[1 - pi] === id) { const mine = cur[pi]; mount(1 - pi, mine); }
    mount(pi, id);
  }

  /* ------------------------------------------------------------ header */
  function renderRunning(lines) {
    $('running').innerHTML = (lines || []).map(l => `<div class="rline"><span class="lab">${D.esc(l.label)}</span>` +
      l.items.map((it, i) => (l.chain && i ? '<span class="arrow">›</span>' : '') +
        `<button class="blk ${it.kind || ''}" ${it.tab ? `data-go="${it.tab}"` : ''}><span class="id">${D.esc(it.id)}</span>${D.esc(it.text)}</button>`).join('') +
      '</div>').join('');
  }
  function renderHeader(c) {
    const m = c.mission || {}, o = c.owner || {};
    let mode = c.manual ? 'MANUAL' : (m.mode || '—');
    if (!c.manual && o.winner === 'SAFETY_STOP') mode = 'SAFETY STOP';
    const chip = $('modeChip');
    chip.textContent = mode;
    chip.className = 'modebig' + (mode === 'SAFETY STOP' ? ' stop' : (c.manual || mode === 'HOLD' ? ' hold' : ''));
    const arm = $('armedChip');
    arm.className = 'chip ' + (c.estopped ? 'bad' : c.armed ? 'ok' : 'grey');
    arm.innerHTML = `<span class="dot"></span>${c.estopped ? 'E-stopped' : c.armed ? 'Armed' : 'Not armed'}`;
    $('battery').textContent = c.battery_v == null ? '— V' : c.battery_v.toFixed(2) + ' V';
    renderRunning(c.running);
    document.querySelectorAll('[data-flag]').forEach(f => { f.className = 'flag'; });
    (c.running || []).forEach(l => l.items.forEach(it => {
      if (!it.tab || !(it.kind === 'bad' || it.kind === 'warn')) return;
      const f = document.querySelector(`[data-flag="${it.tab}"]`);
      if (f && !f.classList.contains('bad')) f.className = 'flag ' + it.kind;
    }));
    const mb = $('manualBtn');
    mb.classList.toggle('on', !!c.manual);
    mb.setAttribute('aria-pressed', String(!!c.manual));
    mb.querySelector('b').textContent = c.manual ? 'Hand back' : 'Manual control';
    $('manualSub').textContent = c.manual ? 'Return to autonomous control'
      : (cfg.mode === 'race' && c.armed ? 'Counts as manual intervention = 0 marks'
        : (cfg.mode === 'race' ? 'Position the car before START' : 'Drive with the controller'));
    $('manualBar').hidden = !c.manual;
    if (c.manual && c.manual_cmd) $('manualCmd').textContent = `Applied: throttle ${D.f(c.manual_cmd.lin)} · steer ${D.f(c.manual_cmd.ang)}`;
    document.body.dataset.manual = c.manual ? 'on' : 'off';
  }
  async function pollCore() {
    if (!document.hidden) {
      try {
        core = await api('/api/core');
        fails = 0; $('conn').classList.remove('on');
        renderHeader(core);
      } catch (e) {
        if (++fails >= 3) $('conn').classList.add('on');
      }
    }
    setTimeout(pollCore, 1000 / Math.max(1, cfg.rates.core));
  }

  /* ------------------------------------------------------------ actions */
  async function setManual(on, confirm) {
    const r = await api('/api/manual', { on, confirm: !!confirm });
    if (r && !r.ok) toast(r.message); else if (r) toast(r.message);
  }
  function onManual() {
    if (core && core.manual) return setManual(false);
    if (cfg.manual_confirm && core && core.armed) return $('takeover').showModal();
    return setManual(true);
  }
  function wire() {
    document.addEventListener('click', e => {
      const nb = e.target.closest('#rail .navbtn'); if (nb) return go(nb.dataset.tab);
      if (e.target.closest('#splitbtn')) return setSplit(!split);
      const g = e.target.closest('[data-go]'); if (g) return go(g.dataset.go);
    });
    panes.forEach((p, i) => {
      p.el.addEventListener('pointerdown', () => { if (!split) return; focus = i; panes.forEach((q, j) => q.el.classList.toggle('focus', j === i)); });
      p.sel.addEventListener('change', () => { focus = i; go(p.sel.value); });
    });
    $('manualBtn').addEventListener('click', onManual);
    $('tkCancel').addEventListener('click', () => $('takeover').close());
    $('tkGo').addEventListener('click', () => { $('takeover').close(); setManual(true, true); });
    $('estopBtn').addEventListener('click', () => {
      if (cfg.mode !== 'race' && core && core.estopped) {
        return api('/api/estop_release', {}).then(r => toast(r.message));
      }
      $('estopText').textContent = cfg.mode === 'race'
        ? 'The car stops immediately. This counts as manual intervention: the run scores 0 marks.'
        : 'The motors stop and the running step is cancelled.';
      $('estopDlg').showModal();
    });
    $('esCancel').addEventListener('click', () => $('estopDlg').close());
    $('esGo').addEventListener('click', () => { $('estopDlg').close(); api('/api/estop', {}).then(() => toast('E-STOP sent')); });
  }

  const ctx = { api, toast, go, get cfg() { return cfg; }, get core() { return core; } };

  async function boot() {
    try { cfg = await api('/api/config'); } catch (e) { $('conn').classList.add('on'); setTimeout(boot, 2000); return; }
    document.body.dataset.mode = cfg.mode;
    $('modeLabel').textContent = cfg.mode === 'race' ? 'Race mode' : 'Calibration mode' + (cfg.session ? ` · ${cfg.session}` : '');
    $('estopSub').textContent = cfg.mode === 'race' ? 'Counts as manual intervention = 0 marks' : 'Stop motors (press again to release)';
    buildNav(); wire();
    const first = cfg.tabs[0].id;
    mount(0, first);
    pollCore();
    document.addEventListener('visibilitychange', () => { if (!document.hidden) toast('Reconnected', 1200); });
  }
  return { boot, api, toast, go };
})();
App.boot();
