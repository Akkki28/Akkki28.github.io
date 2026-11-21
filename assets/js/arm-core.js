/* arm-core.js â€” pure logic for the 2-link arm: kinematics, collision model,
   RRT-Connect joint-space planner, trajectory smoothing. No DOM access. */
(function (root) {
"use strict";

const C = {
  WW: 600, WH: 460, TABLE: 420,
  S: { x: 80, y: 320 },            // shoulder joint
  L1: 230, L2: 200,                // link lengths
  PALM_H: 12, FINGER_L: 38, FINGER_T: 9,
  G: 50,                           // wrist joint -> fingertip (vertical, gripper always points down)
  BLOCK: 36, GAP_OPEN: 62, GAP_MIN: 6,
  A1: [-1.72, 0.6],                 // shoulder limits (rad)
  REL: [0.3, 2.75],                // elbow limits (rad); rel > 0 == elbow-up branch
  BASKET: { cx: 470, inner: 116, wall: 10, h: 110, floor: 10 },
  MARGIN: 3,
  HOME_TIP: { x: 250, y: 255 }
};

const B = C.BASKET;
const basketRects = {
  left:  { x0: B.cx - B.inner / 2 - B.wall, x1: B.cx - B.inner / 2, y0: C.TABLE - B.h, y1: C.TABLE, m: C.MARGIN },
  right: { x0: B.cx + B.inner / 2, x1: B.cx + B.inner / 2 + B.wall, y0: C.TABLE - B.h, y1: C.TABLE, m: C.MARGIN },
  floor: { x0: B.cx - B.inner / 2 - B.wall, x1: B.cx + B.inner / 2 + B.wall, y0: C.TABLE - B.floor, y1: C.TABLE, m: C.MARGIN }
};
const STATIC = [
  { x0: -200, x1: C.WW + 200, y0: C.TABLE + 1.5, y1: C.TABLE + 300, m: 0 },               // table
  { x0: C.S.x - 34, x1: C.S.x + 34, y0: C.S.y + 24, y1: C.TABLE, m: C.MARGIN },            // pedestal
  basketRects.left, basketRects.right, basketRects.floor
];

const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const BASKET_TOP = C.TABLE - B.h;
const INNER_L = B.cx - B.inner / 2, INNER_R = B.cx + B.inner / 2;

/* ---------- kinematics ---------- */
function fk(q) {
  const a2 = q[0] + q[1];
  const E = { x: C.S.x + C.L1 * Math.cos(q[0]), y: C.S.y + C.L1 * Math.sin(q[0]) };
  const W = { x: E.x + C.L2 * Math.cos(a2), y: E.y + C.L2 * Math.sin(a2) };
  return { E, W, tip: { x: W.x, y: W.y + C.G } };
}
const dMin = Math.sqrt(C.L1 ** 2 + C.L2 ** 2 + 2 * C.L1 * C.L2 * Math.cos(C.REL[1]));
const dMax = Math.sqrt(C.L1 ** 2 + C.L2 ** 2 + 2 * C.L1 * C.L2 * Math.cos(C.REL[0]));
function ik(tip) {
  let dx = tip.x - C.S.x, dy = tip.y - C.G - C.S.y;
  let d = Math.hypot(dx, dy) || 1e-6;
  const dc = clamp(d, dMin + 1e-3, dMax - 1e-3);
  dx *= dc / d; dy *= dc / d; d = dc;
  const c = clamp((d * d - C.L1 ** 2 - C.L2 ** 2) / (2 * C.L1 * C.L2), -1, 1);
  const rel = Math.acos(c);
  const a1 = Math.atan2(dy, dx) - Math.atan2(C.L2 * Math.sin(rel), C.L1 + C.L2 * c);
  return [clamp(a1, C.A1[0], C.A1[1]), clamp(rel, C.REL[0], C.REL[1])];
}

/* ---------- collision ---------- */
function ptRectDist(px, py, r) {
  const dx = Math.max(r.x0 - px, 0, px - r.x1), dy = Math.max(r.y0 - py, 0, py - r.y1);
  return Math.hypot(dx, dy);
}
function rectsOverlap(a, b, m) {
  return a.x0 < b.x1 + m && a.x1 > b.x0 - m && a.y0 < b.y1 + m && a.y1 > b.y0 - m;
}
function blockRect(b) {
  const h = C.BLOCK / 2;
  return { x0: b.x - h, x1: b.x + h, y0: b.y - h, y1: b.y + h, m: C.MARGIN };
}
function gripperRects(W, gap) {
  const T = C.FINGER_T, y0 = W.y + C.PALM_H, y1 = W.y + C.G;
  return [
    { x0: W.x - gap / 2 - T, x1: W.x + gap / 2 + T, y0: W.y, y1: y0 },       // palm
    { x0: W.x - gap / 2 - T, x1: W.x - gap / 2, y0: y0, y1: y1 },            // left finger
    { x0: W.x + gap / 2, x1: W.x + gap / 2 + T, y0: y0, y1: y1 }             // right finger
  ];
}
function heldRect(W) {
  const h = C.BLOCK / 2, cy = W.y + C.G - 14;
  return { x0: W.x - h, x1: W.x + h, y0: cy - h, y1: cy + h };
}
function sampleSeg(a, b, step, fn) {
  const n = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / step));
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    if (fn(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)) return true;
  }
  return false;
}
/* o: { gap, held (bool), blocks: [{x,y}] obstacle blocks } */
function collides(q, o) {
  if (q[0] < C.A1[0] || q[0] > C.A1[1] || q[1] < C.REL[0] || q[1] > C.REL[1]) return true;
  const { E, W } = fk(q);
  const obs = STATIC.concat((o.blocks || []).map(blockRect));
  const R1 = 16, R2 = 13;
  // links vs world
  let hit = sampleSeg(C.S, E, 7, (x, y) => { for (const r of obs) if (ptRectDist(x, y, r) < R1 + r.m) return true; return false; });
  if (hit) return true;
  hit = sampleSeg(E, W, 7, (x, y) => { for (const r of obs) if (ptRectDist(x, y, r) < R2 + r.m) return true; return false; });
  if (hit) return true;
  // gripper (+held block) vs world
  const g = gripperRects(W, o.gap);
  const parts = o.held ? g.concat([heldRect(W)]) : g;
  for (const p of parts) for (const r of obs) if (rectsOverlap(p, r, r.m)) return true;
  // self collision: upper arm vs gripper / held block
  hit = sampleSeg(C.S, E, 7, (x, y) => { for (const p of parts) if (ptRectDist(x, y, p) < R1 + 2) return true; return false; });
  return hit;
}
function edgeValid(a, b, o, step) {
  step = step || 0.03;
  const n = Math.max(1, Math.ceil(Math.max(Math.abs(b[0] - a[0]), Math.abs(b[1] - a[1])) / step));
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    if (collides([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t], o)) return false;
  }
  return true;
}

/* ---------- RRT-Connect (joint space) ---------- */
function mulberry(seed) { return function () { seed |= 0; seed = seed + 0x6D2B79F5 | 0; let t = Math.imul(seed ^ seed >>> 15, 1 | seed); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

function rrtConnect(qs, qg, o, rng, maxIter) {
  const STEP = 0.14;
  const ta = [{ q: qs, p: -1 }], tb = [{ q: qg, p: -1 }];
  function nearest(t, q) { let bi = 0, bd = 1e9; for (let i = 0; i < t.length; i++) { const d = dist(t[i].q, q); if (d < bd) { bd = d; bi = i; } } return bi; }
  function steer(from, to) { const d = dist(from, to); if (d <= STEP) return to; const k = STEP / d; return [from[0] + (to[0] - from[0]) * k, from[1] + (to[1] - from[1]) * k]; }
  function extend(t, q) {
    const i = nearest(t, q), nq = steer(t[i].q, q);
    if (!edgeValid(t[i].q, nq, o)) return -1;
    t.push({ q: nq, p: i }); return t.length - 1;
  }
  function trace(t, i) { const out = []; while (i >= 0) { out.push(t[i].q); i = t[i].p; } return out; }
  let swapped = false, A = ta, Bt = tb;
  for (let it = 0; it < maxIter; it++) {
    const sample = rng() < 0.1 ? Bt[0].q : [C.A1[0] + rng() * (C.A1[1] - C.A1[0]), C.REL[0] + rng() * (C.REL[1] - C.REL[0])];
    const ia = extend(A, sample);
    if (ia >= 0) {
      // greedily connect other tree toward new node
      const target = A[ia].q; let ib;
      do { ib = extend(Bt, target); } while (ib >= 0 && dist(Bt[ib].q, target) > 1e-9);
      if (ib >= 0 && dist(Bt[ib].q, target) <= 1e-9) {
        let pa = trace(A, ia).reverse(), pb = trace(Bt, ib);
        let path = pa.concat(pb.slice(1));
        if (swapped) path.reverse();
        return { path, nodes: ta.length + tb.length };
      }
    }
    const t = A; A = Bt; Bt = t; swapped = !swapped;
  }
  return null;
}
function shortcut(path, o, rng) {
  let p = path.slice();
  for (let k = 0; k < 120 && p.length > 2; k++) {
    let i = Math.floor(rng() * (p.length - 1)), j = Math.floor(rng() * (p.length - 1)) + 1;
    if (i > j) { const t = i; i = j; j = t; }
    if (j - i < 2) continue;
    if (edgeValid(p[i], p[j], o)) p = p.slice(0, i + 1).concat(p.slice(j));
  }
  return p;
}
function tipLength(path) {
  let L = 0, prev = null;
  for (let i = 0; i < path.length - 1; i++) {
    const n = 12;
    for (let k = 0; k <= n; k++) {
      const t = k / n, q = [path[i][0] + (path[i + 1][0] - path[i][0]) * t, path[i][1] + (path[i + 1][1] - path[i][1]) * t];
      const tp = fk(q).tip;
      if (prev) L += Math.hypot(tp.x - prev.x, tp.y - prev.y);
      prev = tp;
    }
  }
  return L;
}
/* Plan the best of several RRT-Connect runs (shortest in task space). */
function planPath(qs, qg, o, seed) {
  if (collides(qg, o)) return null;
  let start = qs, prefix = [];
  if (collides(qs, o)) {            // gently lift out of a touching/contact start state
    const tip = fk(qs).tip; let ok = false;
    for (let dy = 2; dy <= 60; dy += 2) {
      const q2 = ik({ x: tip.x, y: tip.y - dy });
      if (!collides(q2, o)) { start = q2; prefix = [qs]; ok = true; break; }
    }
    if (!ok) return null;
  }
  if (edgeValid(start, qg, o)) return { path: prefix.concat([start, qg]), nodes: 2, tries: 0 };
  let best = null, bestLen = 1e9, nodes = 0;
  for (let k = 0; k < 6; k++) {
    const rng = mulberry((seed || 1) * 977 + k * 131);
    const r = rrtConnect(start, qg, o, rng, 2500);
    if (!r) continue;
    nodes += r.nodes;
    const sp = shortcut(r.path, o, rng), L = tipLength(sp);
    if (L < bestLen) { bestLen = L; best = sp; }
  }
  if (!best) return null;
  return { path: prefix.concat(best), nodes, tries: 6, length: bestLen };
}

/* ---------- smooth trajectory ---------- */
function catmull(p0, p1, p2, p3, t) {
  const t2 = t * t, t3 = t2 * t;
  return [0, 1].map(i => 0.5 * ((2 * p1[i]) + (-p0[i] + p2[i]) * t + (2 * p0[i] - 5 * p1[i] + 4 * p2[i] - p3[i]) * t2 + (-p0[i] + 3 * p1[i] - 3 * p2[i] + p3[i]) * t3));
}
function dedupe(w) { const o = [w[0]]; for (const q of w) if (dist(q, o[o.length - 1]) > 1e-4) o.push(q); return o; }
function splineDense(w) {
  w = dedupe(w);
  if (w.length < 3) return densifyPoly(w);
  const out = [];
  for (let i = 0; i < w.length - 1; i++) {
    const p0 = w[Math.max(0, i - 1)], p1 = w[i], p2 = w[i + 1], p3 = w[Math.min(w.length - 1, i + 2)];
    const n = Math.max(4, Math.ceil(dist(p1, p2) / 0.02));
    for (let k = 0; k < n; k++) out.push(catmull(p0, p1, p2, p3, k / n));
  }
  out.push(w[w.length - 1]);
  return out;
}
function densifyPoly(w) {
  const out = [];
  for (let i = 0; i < w.length - 1; i++) {
    const n = Math.max(1, Math.ceil(dist(w[i], w[i + 1]) / 0.02));
    for (let k = 0; k < n; k++) out.push([w[i][0] + (w[i + 1][0] - w[i][0]) * k / n, w[i][1] + (w[i + 1][1] - w[i][1]) * k / n]);
  }
  out.push(w[w.length - 1]);
  return out;
}
function denseValid(pts, o, skipFirst) {
  for (let i = skipFirst || 0; i < pts.length; i++) if (collides(pts[i], o)) return false;
  return true;
}
/* Waypoints -> time-parametrised trajectory. Smooth spline if it stays collision free, polyline otherwise. */
function makeTrajectory(waypoints, o, opts) {
  opts = opts || {};
  const skip = opts.skipCheckFirst || 0;
  let pts = splineDense(waypoints), smooth = true;
  if (!denseValid(pts, o, skip)) { pts = densifyPoly(dedupe(waypoints)); smooth = false; }
  const cum = [0];
  for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + dist(pts[i], pts[i - 1]));
  const L = cum[cum.length - 1];
  const T = clamp(L / (opts.speed || 1.5), opts.minT || 0.5, 8);
  return { pts, cum, L, T, smooth };
}
const ease = t => { t = clamp(t, 0, 1); return t * t * t * (t * (6 * t - 15) + 10); };
function sampleTrajectory(tr, t) {
  const s = ease(t / tr.T) * tr.L;
  let lo = 0, hi = tr.cum.length - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (tr.cum[m] <= s) lo = m; else hi = m; }
  const seg = tr.cum[hi] - tr.cum[lo], u = seg > 1e-9 ? (s - tr.cum[lo]) / seg : 0;
  return [tr.pts[lo][0] + (tr.pts[hi][0] - tr.pts[lo][0]) * u, tr.pts[lo][1] + (tr.pts[hi][1] - tr.pts[lo][1]) * u];
}
/* straight task-space line (for the vertical descent / lift) */
function linearTip(qFrom, tipTo, minT) {
  const a = fk(qFrom).tip, pts = [];
  const n = Math.max(2, Math.ceil(Math.hypot(tipTo.x - a.x, tipTo.y - a.y) / 3));
  for (let i = 0; i <= n; i++) { const t = i / n; pts.push(ik({ x: a.x + (tipTo.x - a.x) * t, y: a.y + (tipTo.y - a.y) * t })); }
  pts[0] = qFrom;
  const cum = [0]; for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + dist(pts[i], pts[i - 1]));
  return { pts, cum, L: cum[cum.length - 1], T: minT || 0.7, smooth: true };
}

const api = { C, STATIC, BASKET_TOP, INNER_L, INNER_R, fk, ik, collides, edgeValid, planPath, makeTrajectory, sampleTrajectory, linearTip, gripperRects, heldRect, blockRect, clamp, mulberry };
if (typeof module !== "undefined" && module.exports) module.exports = api; else root.ArmCore = api;
})(typeof window !== "undefined" ? window : globalThis);
