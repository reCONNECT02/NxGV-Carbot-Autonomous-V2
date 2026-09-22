/* draw.js — canvas helpers shared by the tabs (no libraries: the car may be offline). */
'use strict';
const D = (() => {
  const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim() || '#888';

  /** Canvas that follows its box size (device-pixel sharp). onResize() redraws. */
  function canvas(parent, aspect, onResize) {
    const cv = document.createElement('canvas');
    parent.appendChild(cv);
    const o = { cv, ctx: cv.getContext('2d'), w: 0, h: 0 };
    const fit = (init) => {
      const w = parent.clientWidth || 300, h = aspect ? Math.round(w / aspect) : (parent.clientHeight || 300);
      const r = window.devicePixelRatio || 1;
      if (w === o.w && h === o.h) return;
      o.w = w; o.h = h; cv.width = Math.round(w * r); cv.height = Math.round(h * r);
      if (aspect) cv.style.height = h + 'px';
      o.ctx.setTransform(r, 0, 0, r, 0, 0);
      if (onResize && init !== true) onResize();
    };
    new ResizeObserver(() => fit()).observe(parent);
    fit(true);   // first sizing: the caller's draw() may not exist yet
    return o;
  }

  /** World (x right, y up) fitted into the canvas with a margin. */
  function fitView(b, w, h, pad = 20) {
    const [x0, y0, x1, y1] = b;
    const s = Math.min((w - 2 * pad) / Math.max(x1 - x0, 1e-3), (h - 2 * pad) / Math.max(y1 - y0, 1e-3));
    const ox = (w - s * (x1 - x0)) / 2, oy = (h - s * (y1 - y0)) / 2;
    return { s, X: x => ox + (x - x0) * s, Y: y => h - (oy + (y - y0) * s), inv: (px, py) => [x0 + (px - ox) / s, y0 + (h - py - oy) / s] };
  }

  /** base_link view: +x forward = up, +y left = left; car at (cx, cy) in pixels, s px per metre. */
  function carView(w, h, s, cxFrac = 0.5, cyFrac = 0.8) {
    const cx = w * cxFrac, cy = h * cyFrac;
    return { s, X: (x, y) => cx - y * s, Y: (x, y) => cy - x * s, inv: (px, py) => [(cy - py) / s, (cx - px) / s] };
  }

  function path(ctx, pts, fx, fy) {
    ctx.beginPath();
    pts.forEach((p, i) => { const X = fx(p), Y = fy(p); i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y); });
  }

  function line(ctx, pts, fx, fy, color, width, dash) {
    if (!pts || pts.length < 2) return;
    ctx.save(); ctx.strokeStyle = color; ctx.lineWidth = width || 2; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    ctx.setLineDash(dash || []); path(ctx, pts, fx, fy); ctx.stroke(); ctx.restore();
  }

  function dot(ctx, X, Y, r, fill, stroke) {
    ctx.beginPath(); ctx.arc(X, Y, r, 0, 2 * Math.PI);
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 2; ctx.stroke(); }
  }

  /** Arrow for a pose; screen angle a (rad, 0 = +X screen, counter-clockwise = up). */
  function arrow(ctx, X, Y, a, size, fill) {
    ctx.save(); ctx.translate(X, Y); ctx.rotate(-a);
    ctx.beginPath(); ctx.moveTo(size, 0); ctx.lineTo(-size * .65, -size * .65); ctx.lineTo(-size * .3, 0); ctx.lineTo(-size * .65, size * .65); ctx.closePath();
    ctx.fillStyle = fill; ctx.fill(); ctx.lineWidth = 2; ctx.strokeStyle = '#fff'; ctx.stroke(); ctx.restore();
  }

  function ellipse(ctx, X, Y, a, b, ang, s, color) {
    if (!(a > 0)) return;
    ctx.save(); ctx.translate(X, Y); ctx.rotate(-ang);
    ctx.beginPath(); ctx.ellipse(0, 0, Math.max(a * s, 2), Math.max(b * s, 2), 0, 0, 2 * Math.PI);
    ctx.globalAlpha = .15; ctx.fillStyle = color; ctx.fill(); ctx.globalAlpha = 1; ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.stroke(); ctx.restore();
  }

  function label(ctx, X, Y, text, bg, fg, align) {
    ctx.save(); ctx.font = '600 12px system-ui, sans-serif'; ctx.textAlign = 'left';
    const w = ctx.measureText(text).width + 12, h = 20;
    let x = align === 'left' ? X : X - w / 2;
    ctx.fillStyle = bg; roundRect(ctx, x, Y - h / 2, w, h, 6); ctx.fill();
    ctx.fillStyle = fg; ctx.textBaseline = 'middle'; ctx.fillText(text, x + 6, Y + 1); ctx.restore();
  }

  function roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath(); ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath();
  }

  function b64bytes(s) {
    const bin = atob(s || ''); const u = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i);
    return u;
  }

  /** Draw a LocalGrid payload. cellXY(r,c) -> [x,y] metres (cell lower corner) in the view frame.
   *  colour(byte) -> css colour or null (skip). Cells drawn as rotated-free squares (view handles rotation). */
  function grid(ctx, g, place, colour) {
    if (!g) return;
    const bytes = b64bytes(g.b64);
    for (let r = 0; r < g.rows; r++) for (let c = 0; c < g.cols; c++) {
      const b = bytes[r * g.cols + c], col = colour(b);
      if (!col) continue;
      ctx.fillStyle = col; place(r, c);
    }
  }

  /** Time chart. series: [{pts:[[t,v]...], color, width, dash}] */
  function chart(o, series, yMin, yMax, tSpan, now) {
    const { ctx, w, h } = o, L = 44, R = 8, T = 8, B = 18;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = css('--raised'); ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = css('--grid'); ctx.fillStyle = css('--muted'); ctx.font = '10px system-ui'; ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = T + i * (h - T - B) / 4; ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(w - R, y); ctx.stroke();
      ctx.textAlign = 'right'; ctx.fillText((yMax - (yMax - yMin) * i / 4).toFixed(2), L - 6, y + 3);
    }
    ctx.textAlign = 'center'; ctx.fillText(`last ${tSpan.toFixed(0)} s`, (L + w) / 2, h - 4);
    const X = t => L + (1 - (now - t) / tSpan) * (w - L - R), Y = v => T + (yMax - v) / (yMax - yMin) * (h - T - B);
    series.forEach(s => {
      const pts = s.pts.filter(p => p[1] !== null && now - p[0] <= tSpan);
      line(ctx, pts, p => X(p[0]), p => Y(Math.max(yMin, Math.min(yMax, p[1]))), s.color, s.width || 2, s.dash);
    });
  }

  const esc = s => String(s == null ? '' : s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  const f = (v, d = 2, unit = '') => (v === null || v === undefined || Number.isNaN(v)) ? '—' : Number(v).toFixed(d) + unit;

  return { css, canvas, fitView, carView, path, line, dot, arrow, ellipse, label, roundRect, b64bytes, grid, chart, esc, f };
})();
