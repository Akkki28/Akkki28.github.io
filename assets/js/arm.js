/* arm.js — interactive arm demo UI: teleop (cursor + hold-click gripper) and RRT path-planned pick & place. */
(function () {
"use strict";
const canvas = document.getElementById("arm");
if (!canvas || !window.ArmCore) return;
const A = window.ArmCore, C = A.C, S = C.S, B = C.BASKET;
const ctx = canvas.getContext("2d");
const $ = id => document.getElementById(id);
const hint = $("arm-hint"), telemetry = $("telemetry"), statusEl = $("task-status");
const btnTele = $("btn-teleop"), btnAuto = $("btn-auto"), btnReset = $("btn-reset");
const CANCEL = { cancelled: true };
const HINT_TELE = "Teleop: move the cursor to guide the arm · hold click to close the gripper";
const HINT_AUTO = "Path planning: RRT-Connect in joint space · click Teleop to take over";

/* ---------------- state ---------------- */
let q = A.ik(C.HOME_TIP);
let gap = C.GAP_OPEN, wantClosed = false, pointerDown = false;
let mode = "teleop", runId = 0, play = null, preview = null, planInfo = "";
let cursor = { x: C.HOME_TIP.x, y: C.HOME_TIP.y }, cursorIn = false;
let blocks = [], held = null;
const COLORS = ["#55d9ff", "#ffb45e", "#69e59b"];
function initBlocks() {
  blocks = [200, 265, 330].map((x, i) => ({ id: i, x, y: C.TABLE - C.BLOCK / 2, vy: 0, color: COLORS[i] }));
  held = null;
}
initBlocks();

/* ---------------- helpers ---------------- */
const tipNow = () => A.fk(q).tip;
const obstacles = (exclude) => ({ gap, held: !!held, blocks: blocks.filter(b => b !== held && b !== exclude) });
const isBinned = b => b !== held && b.x - 18 >= A.INNER_L - 1 && b.x + 18 <= A.INNER_R + 1 && b.y > A.BASKET_TOP;
const binnedCount = () => blocks.filter(isBinned).length;
const wait = ms => new Promise(r => setTimeout(r, ms));

/* ---------------- canvas / input ---------------- */
let view = { s: 1, ox: 0, oy: 0, w: 0, h: 0 };
function resize() {
  const r = canvas.getBoundingClientRect(), dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.round(r.width * dpr); canvas.height = Math.round(r.height * dpr);
  const s = Math.min(r.width / C.WW, r.height / C.WH);
  view = { s, ox: (r.width - C.WW * s) / 2, oy: (r.height - C.WH * s) / 2 + 4, w: r.width, h: r.height, dpr };
}
function toWorld(e) {
  const r = canvas.getBoundingClientRect();
  return { x: (e.clientX - r.left - view.ox) / view.s, y: (e.clientY - r.top - view.oy) / view.s };
}
canvas.addEventListener("pointermove", e => { cursor = toWorld(e); cursorIn = true; });
canvas.addEventListener("pointerenter", e => { cursor = toWorld(e); cursorIn = true; });
canvas.addEventListener("pointerleave", () => { cursorIn = false; });
canvas.addEventListener("pointerdown", e => {
  e.preventDefault(); cursor = toWorld(e); cursorIn = true;
  if (mode !== "teleop") return;
  pointerDown = true; try { canvas.setPointerCapture(e.pointerId); } catch (_) {}
});
const up = () => { pointerDown = false; };
canvas.addEventListener("pointerup", up);
canvas.addEventListener("pointercancel", up);
canvas.addEventListener("contextmenu", e => e.preventDefault());

/* ---------------- buttons ---------------- */
function setMode(m) {
  mode = m;
  btnTele.classList.toggle("on", m === "teleop");
  btnAuto.classList.toggle("on", m === "auto");
  hint.textContent = m === "teleop" ? HINT_TELE : HINT_AUTO;
}
function cancelAuto() { runId++; if (play) { const p = play; play = null; p.reject(CANCEL); } preview = null; wantClosed = false; }
btnTele.onclick = () => { cancelAuto(); setMode("teleop"); };
btnAuto.onclick = () => { if (mode === "auto") { cancelAuto(); setMode("teleop"); } else startAuto(); };
btnReset.onclick = () => {
  cancelAuto(); setMode("teleop"); initBlocks(); q = A.ik(C.HOME_TIP); gap = C.GAP_OPEN; pointerDown = false; planInfo = "";
};

/* ---------------- gripper + block physics ---------------- */
function updateGripper(dt) {
  const closing = mode === "teleop" ? pointerDown : wantClosed;
  const target = closing ? C.GAP_MIN : C.GAP_OPEN;
  const rate = closing ? 170 : 240;
  gap += clamp(target - gap, -rate * dt, rate * dt);
  if (held) {
    if (closing) gap = Math.max(gap, C.BLOCK);
    else if (gap > C.BLOCK + 3) { held.vy = 0; held = null; }
  } else if (closing) {
    const tip = tipNow();
    for (const b of blocks) {
      const dx = b.x - tip.x, v = tip.y - b.y;
      if (v > -6 && v < 28 && Math.abs(dx) < gap / 2 && gap / 2 - Math.abs(dx) <= C.BLOCK / 2 + 1.5) { held = b; b.vy = 0; break; }
    }
  }
  if (held) {
    const tip = tipNow(), k = Math.min(1, 22 * dt);
    held.x += (tip.x - held.x) * k; held.y += (tip.y - 14 - held.y) * k;
  }
}
function clamp(v, a, b) { return Math.max(a, Math.min(b, v)); }
function physics(dt) {
  const h = C.BLOCK / 2, wl = [[A.INNER_L - B.wall, A.INNER_L], [A.INNER_R, A.INNER_R + B.wall]];
  for (const b of blocks) {
    if (b === held) continue;
    const prevBottom = b.y + h;
    b.vy = Math.min(b.vy + 1500 * dt, 900);
    let ny = b.y + b.vy * dt, support = C.TABLE;
    const inside = b.x - h >= A.INNER_L - 0.5 && b.x + h <= A.INNER_R + 0.5;
    if (inside) support = C.TABLE - B.floor;
    else for (const [x0, x1] of wl) {
      if (b.x + h > x0 && b.x - h < x1) {
        if (prevBottom <= A.BASKET_TOP + 4) { support = Math.min(support, A.BASKET_TOP); b.x += (b.x < (x0 + x1) / 2 ? -1 : 1) * 80 * dt; }
        else b.x = b.x < (x0 + x1) / 2 ? x0 - h : x1 + h;
      }
    }
    for (const o of blocks) {
      if (o === b || o === held) continue;
      if (Math.abs(o.x - b.x) < C.BLOCK - 1 && prevBottom <= o.y - h + 4) support = Math.min(support, o.y - h);
    }
    if (ny + h >= support) { ny = support - h; b.vy = 0; }
    b.y = ny;
  }
}

/* ---------------- teleop ---------------- */
function teleopStep(dt) {
  const cur = tipNow();
  const want = { x: clamp(cursor.x, S.x + 70, C.WW), y: Math.min(cursor.y, C.TABLE - 4) };
  const dx = want.x - cur.x, dy = want.y - cur.y;
  if (Math.hypot(dx, dy) < 0.2) return;
  const k = Math.min(1, 13 * dt);
  let sx = dx * k, sy = dy * k;
  const m = Math.hypot(sx, sy), maxStep = 900 * dt;
  if (m > maxStep) { sx *= maxStep / m; sy *= maxStep / m; }
  const o = obstacles(), stuck = A.collides(q, o);
  for (const [ax, ay] of [[sx, sy], [sx, 0], [0, sy]]) {
    if (!ax && !ay) continue;
    const nq = A.ik({ x: cur.x + ax, y: cur.y + ay });
    if (!A.collides(nq, o) || (stuck && A.fk(nq).tip.y <= cur.y + 0.01)) { q = nq; return; }
  }
}

/* ---------------- autonomous pick & place ---------------- */
function playTraj(tr) {
  return new Promise((resolve, reject) => {
    preview = tr.pts.filter((_, i) => i % 3 === 0).map(p => A.fk(p).tip);
    play = { tr, t: 0, resolve, reject };
  });
}
function advancePlay(dt) {
  if (!play) return;
  play.t += dt;
  q = A.sampleTrajectory(play.tr, Math.min(play.t, play.tr.T));
  if (play.t >= play.tr.T) { const p = play; play = null; preview = null; p.resolve(); }
}
async function startAuto() {
  cancelAuto(); pointerDown = false; setMode("auto");
  const id = ++runId;
  const chk = () => { if (id !== runId) throw CANCEL; };
  const fail = msg => { hint.textContent = msg; throw { failed: true }; };
  try {
    for (let guard = 0; guard < 8; guard++) {
      chk();
      let target = null;
      if (!held) {
        const cand = blocks.filter(b => !isBinned(b) && !blocks.some(o => o !== b && Math.abs(o.x - b.x) < C.BLOCK - 2 && o.y < b.y - 5));
        cand.sort((a, b) => b.x - a.x);
        target = cand[0];
        if (!target) break;
        // 1. plan free-space approach to a pre-grasp pose above the block
        const grasp = { x: target.x, y: target.y + 12 }, pre = { x: target.x, y: grasp.y - 100 };
        const qPre = A.ik(pre), qGrasp = A.ik(grasp);
        const t0 = performance.now();
        const p = A.planPath(q, qPre, { gap: C.GAP_OPEN, held: false, blocks: blocks.filter(b => b !== held) }, 7 + guard);
        if (!p) fail("Planner: no collision-free approach found");
        planInfo = `RRT ${p.nodes} nodes · ${(performance.now() - t0).toFixed(0)} ms`;
        wantClosed = false;
        await playTraj(A.makeTrajectory(p.path, { gap: C.GAP_OPEN, held: false, blocks }, { skipCheckFirst: 2, speed: 1.6, minT: 0.6 })); chk();
        // 2. straight descent, 3. close the fingers
        await playTraj(A.linearTip(q, grasp, 0.8)); chk();
        wantClosed = true;
        for (let i = 0; i < 60 && !held; i++) { await wait(30); chk(); }
        if (!held) { wantClosed = false; fail("Grasp failed"); }
        await wait(250); chk();
      }
      // 4. carry: lift, arc over the basket wall, stop above the drop slot — one continuous spline
      const inBin = blocks.filter(isBinned), slot = [-24, 24, 0].find(s => !inBin.some(b => Math.abs(b.x - (B.cx + s)) < 14)) ?? 0;
      const relTip = { x: B.cx + slot, y: A.BASKET_TOP - 24 }, qRel = A.ik(relTip);
      const oc = { gap: C.BLOCK, held: true, blocks: blocks.filter(b => b !== held) };
      const cur = tipNow(), lift = A.ik({ x: cur.x, y: cur.y - 70 });
      const useLift = !A.collides(lift, oc) && cur.y > C.TABLE - 60;
      const qStart = useLift ? lift : q;
      const t1 = performance.now();
      const p2 = A.planPath(qStart, qRel, oc, 21 + guard);
      if (!p2) fail("Planner: no collision-free carry path found");
      planInfo = `RRT ${p2.nodes} nodes · ${(performance.now() - t1).toFixed(0)} ms`;
      const wps = (useLift ? [q] : []).concat(p2.path);
      await playTraj(A.makeTrajectory(wps, oc, { skipCheckFirst: 2, speed: 1.35, minT: 1.2 })); chk();
      // 5. open and let the block drop
      wantClosed = false;
      for (let i = 0; i < 40 && held; i++) { await wait(30); chk(); }
      await wait(650); chk();
    }
    // 6. return home
    const qHome = A.ik(C.HOME_TIP);
    const ph = A.planPath(q, qHome, { gap: C.GAP_OPEN, held: false, blocks }, 99);
    if (ph) await playTraj(A.makeTrajectory(ph.path, { gap: C.GAP_OPEN, held: false, blocks }, { skipCheckFirst: 2, speed: 1.6, minT: 0.6 }));
    chk();
    cancelAuto(); setMode("teleop");
  } catch (e) {
    if (e === CANCEL) return;
    if (e && e.failed) { if (id === runId) { cancelAuto(); btnAuto.classList.remove("on"); mode = "teleop"; btnTele.classList.add("on"); } return; }
    throw e;
  }
}

/* ---------------- rendering ---------------- */
function rr(x, y, w, h, r) { ctx.beginPath(); ctx.roundRect(x, y, w, h, r); }
function limb(a, b, r1, r2, c1, c2) {
  const th = Math.atan2(b.y - a.y, b.x - a.x), nx = -Math.sin(th), ny = Math.cos(th);
  const g = ctx.createLinearGradient(a.x + nx * r1, a.y + ny * r1, a.x - nx * r1, a.y - ny * r1);
  g.addColorStop(0, c1); g.addColorStop(0.5, c2); g.addColorStop(1, "#16273a");
  ctx.beginPath();
  ctx.arc(a.x, a.y, r1, th + Math.PI / 2, th + Math.PI * 1.5);
  ctx.arc(b.x, b.y, r2, th - Math.PI / 2, th + Math.PI / 2);
  ctx.closePath();
  ctx.fillStyle = g; ctx.fill(); ctx.strokeStyle = "#0b1522"; ctx.lineWidth = 1.5; ctx.stroke();
  // panel line + highlight
  ctx.strokeStyle = "rgba(160,225,255,.35)"; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(a.x + nx * r1 * 0.55, a.y + ny * r1 * 0.55); ctx.lineTo(b.x + nx * r2 * 0.55, b.y + ny * r2 * 0.55); ctx.stroke();
  ctx.strokeStyle = "rgba(85,217,255,.55)"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(a.x - nx * r1 * 0.5, a.y - ny * r1 * 0.5); ctx.lineTo(b.x - nx * r2 * 0.5, b.y - ny * r2 * 0.5); ctx.stroke();
}
function joint(p, r, rot) {
  ctx.fillStyle = "#1b2e44"; ctx.strokeStyle = "#0b1522"; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, 7); ctx.fill(); ctx.stroke();
  ctx.fillStyle = "#cfe6f7"; ctx.beginPath(); ctx.arc(p.x, p.y, r * 0.62, 0, 7); ctx.fill();
  ctx.strokeStyle = "#55d9ff"; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(p.x, p.y, r * 0.62, 0, 7); ctx.stroke();
  ctx.fillStyle = "#1b2e44"; ctx.beginPath(); ctx.arc(p.x, p.y, r * 0.2, 0, 7); ctx.fill();
  ctx.strokeStyle = "#1b2e44"; ctx.lineWidth = 2; ctx.beginPath();
  ctx.moveTo(p.x, p.y); ctx.lineTo(p.x + Math.cos(rot) * r * 0.55, p.y + Math.sin(rot) * r * 0.55); ctx.stroke();
}
function drawBlock(b, alpha) {
  const h = C.BLOCK / 2;
  ctx.save(); ctx.translate(b.x, b.y); ctx.globalAlpha = alpha || 1;
  rr(-h, -h, C.BLOCK, C.BLOCK, 5); ctx.fillStyle = b.color; ctx.fill();
  ctx.strokeStyle = "rgba(0,0,0,.45)"; ctx.lineWidth = 1.5; ctx.stroke();
  rr(-h + 5, -h + 5, C.BLOCK - 10, C.BLOCK - 10, 3); ctx.strokeStyle = "rgba(255,255,255,.4)"; ctx.stroke();
  ctx.restore();
}
function drawArm() {
  const f = A.fk(q), E = f.E, W = f.W, tip = f.tip;
  // pedestal
  ctx.fillStyle = "#1a2c42"; ctx.strokeStyle = "#0b1522"; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(S.x - 46, C.TABLE); ctx.lineTo(S.x - 26, S.y + 18); ctx.lineTo(S.x + 26, S.y + 18); ctx.lineTo(S.x + 46, C.TABLE); ctx.closePath(); ctx.fill(); ctx.stroke();
  rr(S.x - 56, C.TABLE - 12, 112, 12, 4); ctx.fillStyle = "#23374f"; ctx.fill(); ctx.stroke();
  ctx.strokeStyle = "rgba(85,217,255,.5)"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(S.x - 16, S.y + 40); ctx.lineTo(S.x + 16, S.y + 40); ctx.stroke();
  // links
  limb(S, E, 22, 17, "#5f86ab", "#37577a");
  limb(E, W, 16, 12, "#5f86ab", "#37577a");
  joint(S, 21, q[0]); joint(E, 16, q[0] + q[1]);
  // held block sits between fingers (drawn under them)
  if (held) drawBlock(held);
  // wrist + gripper
  const x = W.x, y = W.y, T = C.FINGER_T;
  rr(x - 9, y - 6, 18, 14, 3); ctx.fillStyle = "#274260"; ctx.fill(); ctx.strokeStyle = "#0b1522"; ctx.stroke();
  rr(x - gap / 2 - T - 2, y + 4, gap + 2 * T + 4, C.PALM_H, 3); ctx.fillStyle = "#5f86ab"; ctx.fill(); ctx.stroke();
  for (const s of [-1, 1]) {
    const fx = s < 0 ? x - gap / 2 - T : x + gap / 2, y0 = y + 4 + C.PALM_H - 2;
    ctx.beginPath();
    ctx.moveTo(fx, y0); ctx.lineTo(fx + T, y0);
    ctx.lineTo(fx + T, y + C.G - 4); ctx.lineTo(s < 0 ? fx + T - 3 : fx + 3, y + C.G); ctx.lineTo(s < 0 ? fx + T - 3 : fx + 3, y + C.G);
    ctx.lineTo(s < 0 ? fx : fx + T, y + C.G - 4); ctx.lineTo(fx, y0); ctx.closePath();
    ctx.fillStyle = "#cfe6f7"; ctx.fill(); ctx.strokeStyle = "#0b1522"; ctx.stroke();
    // rubber pad on the inner face
    const px = s < 0 ? fx + T - 3 : fx;
    ctx.fillStyle = "#18283a"; ctx.fillRect(px, y0 + 6, 3, C.G - 4 - (y0 + 6 - y));
  }
  joint(W, 11, q[0] + q[1]);
}
function drawScene() {
  const w = view.w / view.s, h = view.h / view.s, x0 = -view.ox / view.s, y0 = -view.oy / view.s;
  // grid
  ctx.strokeStyle = "rgba(99,150,190,.07)"; ctx.lineWidth = 1; ctx.beginPath();
  for (let x = Math.floor(x0 / 30) * 30; x < x0 + w; x += 30) { ctx.moveTo(x, y0); ctx.lineTo(x, C.TABLE); }
  for (let y = Math.floor(y0 / 30) * 30; y < C.TABLE; y += 30) { ctx.moveTo(x0, y); ctx.lineTo(x0 + w, y); }
  ctx.stroke();
  // table
  ctx.fillStyle = "#172436"; ctx.fillRect(x0, C.TABLE, w, y0 + h - C.TABLE + 2);
  ctx.fillStyle = "#2a4259"; ctx.fillRect(x0, C.TABLE, w, 3);
  // basket back + blocks + front
  const bl = A.INNER_L - B.wall, br = A.INNER_R + B.wall;
  ctx.fillStyle = "rgba(85,217,255,.07)"; ctx.fillRect(A.INNER_L, A.BASKET_TOP, B.inner, B.h);
  for (const b of blocks) if (b !== held) drawBlock(b);
  ctx.fillStyle = "#2c4c68"; ctx.strokeStyle = "#55d9ff"; ctx.lineWidth = 2;
  for (const [a, b2] of [[bl, A.INNER_L], [A.INNER_R, br]]) { ctx.fillRect(a, A.BASKET_TOP, b2 - a, B.h); ctx.strokeRect(a, A.BASKET_TOP, b2 - a, B.h); }
  ctx.fillRect(bl, C.TABLE - B.floor, br - bl, B.floor); ctx.strokeRect(bl, C.TABLE - B.floor, br - bl, B.floor);
  ctx.fillStyle = "#55d9ff"; ctx.font = "10px 'JetBrains Mono',monospace"; ctx.textAlign = "center";
  ctx.fillText("BIN", B.cx, C.TABLE + 18);
  // planned path preview
  if (preview && preview.length > 1) {
    ctx.save(); ctx.setLineDash([5, 5]); ctx.strokeStyle = "rgba(105,229,155,.75)"; ctx.lineWidth = 2;
    ctx.beginPath(); preview.forEach((p, i) => i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)); ctx.stroke(); ctx.restore();
  }
  drawArm();
  // cursor reticle
  if (mode === "teleop" && cursorIn) {
    ctx.strokeStyle = pointerDown ? "#ffb45e" : "rgba(85,217,255,.8)"; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(cursor.x, Math.min(cursor.y, C.TABLE), 7, 0, 7); ctx.moveTo(cursor.x - 12, cursor.y); ctx.lineTo(cursor.x - 4, cursor.y); ctx.moveTo(cursor.x + 4, cursor.y); ctx.lineTo(cursor.x + 12, cursor.y); ctx.stroke();
  }
}
function hud() {
  const count = binnedCount();
  statusEl.textContent = `GOAL: ${count} / ${blocks.length} IN BIN`;
  statusEl.classList.toggle("complete", count === blocks.length);
  const g = held ? "HOLDING" : gap < C.GAP_OPEN - 3 ? "CLOSING" : "OPEN";
  const deg = r => (r * 180 / Math.PI).toFixed(0).padStart(4, " ");
  telemetry.textContent = `${mode.toUpperCase()} · θ1 ${deg(q[0])}° θ2 ${deg(q[1])}° · GRIP ${g}` + (mode === "auto" && planInfo ? " · " + planInfo : "");
}

/* ---------------- main loop ---------------- */
let last = performance.now();
function frame(now) {
  const dt = Math.min(0.033, (now - last) / 1000); last = now;
  if (mode === "teleop") teleopStep(dt); else advancePlay(dt);
  updateGripper(dt);
  physics(dt);
  ctx.setTransform(view.dpr, 0, 0, view.dpr, 0, 0);
  ctx.clearRect(0, 0, view.w, view.h);
  ctx.save(); ctx.translate(view.ox, view.oy); ctx.scale(view.s, view.s);
  drawScene();
  ctx.restore();
  hud();
  requestAnimationFrame(frame);
}
window.addEventListener("resize", resize);
setMode("teleop"); resize();
window.__arm = { A, get q() { return q; }, get blocks() { return blocks; }, get held() { return held; } };
requestAnimationFrame(frame);
})();
