'use strict';
// BoatLab dashboard - vanilla JS, polls /api/state (5 Hz) and draws the course map on a canvas.
const $ = id => document.getElementById(id);
const fmt = (v, n = 2) => Number.isFinite(v) ? v.toFixed(n) : '-';
const DEG = 180 / Math.PI;
const BUOY = {red: '#e5484d', green: '#46c46d', blue: '#3b82f6', orange: '#f97316', purple: '#a855f7', yellow: '#facc15',
  black: '#101010', zebra: '#f2f2f2', pink: '#ff69b4'};
const HULL = [[1.9812, 0], [1.5, .32], [.8, .4572], [-1.5, .4572], [-1.9812, .4], [-1.9812, -.4], [-1.5, -.4572], [.8, -.4572], [1.5, -.32]];
let course = null, boat = null, snap = null, lastCam = -1;
let view = {cx: 0, cy: 0, scale: 10}, follow = false, wpMode = false, waypoints = [], drag = null, fitted = false;

// ------------------------------------------------------------------ helpers
function post(msg) {
  return fetch('/api/command', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(msg)})
    .then(r => r.json()).then(r => { if (!r.ok) note(r.error || JSON.stringify(r)); return r; })
    .catch(e => note('Command failed: ' + e));
}
function note(msg) { const n = $('notice'); if (!msg) { n.classList.add('hidden'); return; } n.textContent = msg; n.classList.remove('hidden'); clearTimeout(note.t); note.t = setTimeout(() => n.classList.add('hidden'), 8000); }
function compass(h) { return ((90 - h * DEG) % 360 + 360) % 360; }
function canvas() {
  const el = $('map'), r = el.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  if (el.width !== Math.round(r.width * dpr) || el.height !== Math.round(r.height * dpr)) { el.width = Math.round(r.width * dpr); el.height = Math.round(r.height * dpr); }
  const ctx = el.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, r.width, r.height); return [ctx, r.width, r.height];
}
function fit() {
  if (!course) return; const w = course.water, el = $('map').getBoundingClientRect();
  view.cx = (w.xmin + w.xmax) / 2; view.cy = (w.ymin + w.ymax) / 2;
  view.scale = Math.min((el.width - 30) / (w.xmax - w.xmin), (el.height - 30) / (w.ymax - w.ymin)); fitted = true;
}
function S(x, y, W, H) { return [W / 2 + (x - view.cx) * view.scale, H / 2 - (y - view.cy) * view.scale]; }
function Winv(px, py) { const r = $('map').getBoundingClientRect(); return [view.cx + (px - r.width / 2) / view.scale, view.cy - (py - r.height / 2) / view.scale]; }
function hullPts(x, y, h) {
  const L = boat ? boat.hull.length_m / 3.9624 : 1, B = boat ? boat.hull.beam_m / 0.9144 : 1, c = Math.cos(h), s = Math.sin(h);
  return HULL.map(([bx, by]) => [x + c * bx * L - s * by * B, y + s * bx * L + c * by * B]);
}

// ------------------------------------------------------------------ map
function drawMap() {
  const [ctx, W, H] = canvas(); if (!course) return; if (!fitted) fit();
  const st = snap && snap.state, all = $('layer').value === 'all';
  if (follow && st) { view.cx = st.x; view.cy = st.y; }
  const P = (x, y) => S(x, y, W, H), sc = view.scale;
  const poly = (pts, fill, stroke, lw = 1, dash = []) => { ctx.beginPath(); pts.forEach((p, i) => { const q = P(...p); i ? ctx.lineTo(...q) : ctx.moveTo(...q); }); ctx.closePath(); if (fill) { ctx.fillStyle = fill; ctx.fill(); } if (stroke) { ctx.setLineDash(dash); ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.stroke(); ctx.setLineDash([]); } };
  const line = (pts, stroke, lw = 1.5, dash = []) => { if (!pts || pts.length < 2) return; ctx.beginPath(); pts.forEach((p, i) => { const q = P(p[0], p[1]); i ? ctx.lineTo(...q) : ctx.moveTo(...q); }); ctx.setLineDash(dash); ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.lineJoin = 'round'; ctx.stroke(); ctx.setLineDash([]); };
  const w = course.water;
  // shore + water
  ctx.fillStyle = '#16241a'; ctx.fillRect(0, 0, W, H);
  poly([[w.xmin, w.ymin], [w.xmax, w.ymin], [w.xmax, w.ymax], [w.xmin, w.ymax]], '#0b2533', '#3a6072', 1.2);
  // grid 10 m
  ctx.strokeStyle = '#12303f'; ctx.lineWidth = 1;
  for (let x = Math.ceil(w.xmin / 10) * 10; x <= w.xmax; x += 10) { const a = P(x, w.ymin), b = P(x, w.ymax); ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke(); }
  for (let y = Math.ceil(w.ymin / 10) * 10; y <= w.ymax; y += 10) { const a = P(w.xmin, y), b = P(w.xmax, y); ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke(); }
  (course.keepout || []).forEach(k => poly(k.polygon, '#6b5139', '#8b6b4a'));
  const objs = Object.fromEntries(course.objects.map(o => [o.id, o]));
  // gates
  course.gates.forEach(g => { const r = objs[g.red], gr = objs[g.green]; line([[r.x, r.y], [gr.x, gr.y]], '#6f8aa0', 1, [4, 4]);
    const m = P((r.x + gr.x) / 2, (r.y + gr.y) / 2); ctx.fillStyle = '#7f98ad'; ctx.font = '10px system-ui'; ctx.fillText(g.id, m[0] + 6, m[1] - 6); });
  // detector zones
  course.objects.filter(o => o.kind === 'detector').forEach(o => { const c = P(o.x, o.y); ctx.beginPath(); ctx.arc(c[0], c[1], (o.detect_radius || 6) * sc, 0, 2 * Math.PI); ctx.fillStyle = '#ff69b418'; ctx.fill(); ctx.setLineDash([5, 4]); ctx.strokeStyle = '#ff69b480'; ctx.stroke(); ctx.setLineDash([]); });
  const hidden = new Set((snap && snap.truth && snap.truth.hidden) || []);
  // objects
  course.objects.forEach(o => {
    if (hidden.has(o.id)) return; const c = P(o.x, o.y);
    if (o.kind === 'target') { ctx.beginPath(); ctx.arc(c[0], c[1], o.radius * sc, 0, 2 * Math.PI); ctx.fillStyle = '#f97316aa'; ctx.fill(); ctx.beginPath(); ctx.arc(c[0], c[1], o.radius * .55 * sc, 0, 2 * Math.PI); ctx.fillStyle = '#111'; ctx.fill(); return; }
    if (o.kind === 'case') { const s = Math.max(5, .45 * sc); ctx.fillStyle = '#f5b800'; ctx.fillRect(c[0] - s / 2, c[1] - s / 2, s, s * .78); ctx.strokeStyle = '#000'; ctx.strokeRect(c[0] - s / 2, c[1] - s / 2, s, s * .78); return; }
    const r = Math.max(4, (o.radius || .15) * sc); ctx.beginPath(); ctx.arc(c[0], c[1], r, 0, 2 * Math.PI);
    ctx.fillStyle = BUOY[o.color] || '#888'; ctx.fill(); ctx.lineWidth = 1.2; ctx.strokeStyle = o.color === 'black' ? '#ddd' : '#000'; ctx.stroke();
    if (o.color === 'zebra') { ctx.beginPath(); ctx.moveTo(c[0] - r, c[1]); ctx.lineTo(c[0] + r, c[1]); ctx.strokeStyle = '#000'; ctx.lineWidth = 2; ctx.stroke(); }
  });
  if (!snap) return;
  // map estimates & detections
  if (all && snap.map) snap.map.forEach(t => { const c = P(t.x, t.y), sz = 4; ctx.strokeStyle = t.prior_id ? '#e6edf3' : '#ff6478'; ctx.lineWidth = 1.3;
    ctx.beginPath(); ctx.moveTo(c[0] - sz, c[1]); ctx.lineTo(c[0] + sz, c[1]); ctx.moveTo(c[0], c[1] - sz); ctx.lineTo(c[0], c[1] + sz); ctx.stroke();
    if (t.sigma > .12 && t.hits > 0) { ctx.beginPath(); ctx.arc(c[0], c[1], Math.min(60, t.sigma * sc), 0, 2 * Math.PI); ctx.strokeStyle = '#e6edf340'; ctx.stroke(); } });
  if (all && snap.detections) snap.detections.forEach(d => { const c = P(d.x, d.y); ctx.beginPath(); ctx.arc(c[0], c[1], 2.5, 0, 2 * Math.PI); ctx.fillStyle = '#3fe0cd'; ctx.fill(); });
  // trail, path, look-ahead
  if (snap.trail) line(snap.trail, '#3fe0cd88', 2);
  if (snap.path) line(snap.path, '#9fb3c8', 2, [6, 4]);
  if (snap.approach) { const a = snap.approach; poly(hullPts(a.pose[0], a.pose[1], a.pose[2]), null, '#c38bff', 1.5, [4, 3]); const p = P(a.pre[0], a.pre[1]); ctx.fillStyle = '#c38bff'; ctx.fillRect(p[0] - 3, p[1] - 3, 6, 6); }
  (snap.actions || []).forEach(a => { if (!a.landing) return; const c = P(a.landing[0], a.landing[1]); star(ctx, c[0], c[1], 8, a.success ? '#ffd23f' : '#ff6478'); });
  // waypoints
  if (waypoints.length) { line(waypoints, '#ffffffaa', 1.5, [3, 3]); waypoints.forEach((p, i) => { const c = P(...p); ctx.beginPath(); ctx.arc(c[0], c[1], 7, 0, 2 * Math.PI); ctx.fillStyle = '#fff'; ctx.fill(); ctx.fillStyle = '#081018'; ctx.font = 'bold 10px system-ui'; ctx.fillText(i + 1, c[0] - 3, c[1] + 3.5); }); }
  if (!st) return;
  if (snap.pp && snap.pp.lookahead && snap.source === 'runner' && snap.pp.mode !== 'idle') { const a = P(st.x, st.y), b = P(...snap.pp.lookahead); ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.strokeStyle = '#ffffffcc'; ctx.lineWidth = 1; ctx.stroke(); ctx.beginPath(); ctx.arc(b[0], b[1], 4, 0, 2 * Math.PI); ctx.fillStyle = '#fff'; ctx.fill(); }
  const live = snap.status === 'live' || snap.status === 'ue_direct';
  poly(hullPts(st.x, st.y, st.heading), live ? '#3fe0cdcc' : '#8fa6b8aa', '#e8f1f8', 1.2);
  if (boat) Object.values(boat.mechanisms).forEach(m => { if (!m.mount_flu_m) return; const c = Math.cos(st.heading), s = Math.sin(st.heading); const q = P(st.x + c * m.mount_flu_m[0] - s * m.mount_flu_m[1], st.y + s * m.mount_flu_m[0] + c * m.mount_flu_m[1]); ctx.beginPath(); ctx.arc(q[0], q[1], 2.5, 0, 2 * Math.PI); ctx.fillStyle = '#ffab5e'; ctx.fill(); });
  // scale bar + north
  const bar = 10 * sc; ctx.strokeStyle = '#cfe0ec'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(14, H - 16); ctx.lineTo(14 + bar, H - 16); ctx.stroke(); ctx.fillStyle = '#cfe0ec'; ctx.font = '11px system-ui'; ctx.fillText('10 m', 18, H - 22);
  ctx.fillText('N ↑', W - 34, 20);
}
function star(ctx, x, y, r, col) { ctx.beginPath(); for (let i = 0; i < 10; i++) { const a = -Math.PI / 2 + i * Math.PI / 5, rr = i % 2 ? r * .45 : r; ctx.lineTo(x + rr * Math.cos(a), y + rr * Math.sin(a)); } ctx.closePath(); ctx.fillStyle = col; ctx.fill(); ctx.strokeStyle = '#000'; ctx.lineWidth = 1; ctx.stroke(); }

// ------------------------------------------------------------------ panels
function bars(id, cmd, thrust) { const el = $(id), v = Number.isFinite(cmd) ? Math.max(-1, Math.min(1, cmd)) : 0; el.style.left = `${50 + Math.min(0, v * 50)}%`; el.style.width = `${Math.abs(v * 50)}%`; el.className = v < 0 ? 'neg' : ''; }
function render() {
  const s = snap || {}, st = s.state, src = s.source;
  const status = s.status || 'offline';
  $('status').textContent = '● ' + ({live: 'LIVE · RUNNER', ue_direct: 'LIVE · UNREAL (read-only)', finished: 'MISSION FINISHED', offline: 'OFFLINE'}[status] || status.toUpperCase());
  $('status').className = 'pill ' + (status === 'live' || status === 'ue_direct' ? 'live' : status === 'finished' ? 'warn' : 'bad');
  $('source').textContent = `backend: ${s.backend || '-'}${s.nav_mode ? ' · nav ' + s.nav_mode : ''}`;
  $('clock').textContent = `t ${fmt(s.t, 1)} s`;
  $('launcher').classList.toggle('hidden', src === 'runner');
  if (st) {
    $('k_speed').textContent = fmt(st.speed ?? Math.abs(st.u)); $('k_speed2').textContent = `${fmt((st.speed ?? 0) * 1.94384, 1)} kn · surge ${fmt(st.u)} m/s`;
    $('k_head').textContent = fmt(compass(st.heading), 0); $('k_head2').textContent = `ENU ${fmt(st.heading * DEG, 0)}° · x ${fmt(st.x, 1)} y ${fmt(st.y, 1)} m`;
    $('pp_src').textContent = st.source || '-';
    $('tl').textContent = `${fmt(st.left_thrust_n, 0)} N`; $('tr').textContent = `${fmt(st.right_thrust_n, 0)} N`;
  }
  const cmd = s.cmd || [NaN, NaN]; bars('bl', cmd[0], st && st.left_thrust_n); bars('br', cmd[1], st && st.right_thrust_n);
  const pp = s.pp || {};
  $('k_xte').textContent = fmt(pp.cross_track_m); $('k_mode').textContent = `controller ${pp.mode || '-'} · cmd L ${fmt(cmd[0])} R ${fmt(cmd[1])}`;
  $('ppmode').textContent = pp.mode || '-';
  $('pp_a').textContent = `${fmt(pp.alpha_deg, 1)}°`; $('pp_ld').textContent = `${fmt(pp.lookahead_m, 1)} m`;
  $('pp_u').textContent = `want ${fmt(pp.u_des)} / is ${fmt(st && st.u)} m/s`; $('pp_r').textContent = `${fmt(pp.r_des, 3)} / ${fmt(st && st.r, 3)} rad/s`;
  $('pp_rem').textContent = `${fmt(pp.remaining_m, 1)} m`;
  const m = s.mission;
  if (m) {
    $('mname').textContent = m.name + (m.paused ? ' · PAUSED' : m.aborted ? ' · ABORTED' : m.finished ? ' · DONE' : '');
    const cur = m.tasks[m.index]; $('k_task').textContent = cur ? cur.label : (m.finished ? 'Mission complete' : '-'); $('k_phase').textContent = s.phase || '-';
    $('tasks').innerHTML = m.tasks.map(t => `<li class="${t.status}"><span class="dot"></span><span>${t.label}${t.challenge ? ` <span class="sub">C${t.challenge}</span>` : ''}</span><span class="st">${t.status}${t.duration_s != null ? ' · ' + t.duration_s + 's' : ''}</span></li>`).join('') || '<li><span></span><span class="sub">No tasks - click waypoints on the map</span><span></span></li>';
  } else { $('tasks').innerHTML = '<li><span></span><span class="sub">No mission runner. Launch one below or run BoatLab\\START.cmd.</span><span></span></li>'; $('k_task').textContent = '-'; $('k_phase').textContent = src === 'unreal_direct' ? 'Unreal read-only view' : '-'; }
  if (s.identify_color && document.activeElement !== $('color')) $('color').value = s.identify_color;
  // score
  const sc = s.score, names = {'1_gate': 'C1 Gate', '2_dodge': 'C2 Dodge', '3_evade': 'C3 Evade', '4_identify': 'C4 Identify', '5_deploy': 'C5 Deploy', '6_launch': 'C6 Launch', '7_recover': 'C7 Recover', '9_return': 'C9 Return'};
  if (sc) {
    $('k_score').textContent = sc.passed_count; $('k_score2').textContent = `contacts: ${Object.keys(sc.contacts || {}).join(', ') || 'none'}`;
    $('score').querySelector('tbody').innerHTML = Object.entries(names).map(([k, n]) => { const v = sc[k] || {}; let d = '';
      if (k === '2_dodge' && v.time_s) d = `${v.time_s}s`; if (k === '3_evade') d = `exposure ${v.detector_exposure_s ?? '-'}s${v.time_s ? ' · ' + v.time_s + 's' : ''}`;
      if (v.distance_m != null) d = `${fmt(v.distance_m)} m from target`; if (k === '4_identify') d = v.color || ''; if (k === '1_gate' && v.buoy_contacts && v.buoy_contacts.length) d = 'touched ' + v.buoy_contacts.join(',');
      return `<tr><td>${n}</td><td class="${v.passed ? 'ok' : 'pend'}">${v.passed ? 'PASS' : '-'}</td><td class="sub">${d}</td></tr>`; }).join('');
  } else { $('k_score').textContent = '-'; $('score').querySelector('tbody').innerHTML = '<tr><td class="sub">Scores appear while a simulated mission runs.</td></tr>'; }
  // events
  const ev = s.events || []; $('evcount').textContent = `${ev.length} recent`;
  const box = $('events'), atBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 8;
  box.innerHTML = ev.map(e => `<div class="${e.level}">[${fmt(e.t, 1)}] ${e.msg.replace(/</g, '&lt;')}</div>`).join(''); if (atBottom) box.scrollTop = box.scrollHeight;
  // camera
  const cam = s.camera || {}, seq = cam.sequence;
  if (seq != null && seq >= 0 && seq !== lastCam) { lastCam = seq; $('cam').src = '/camera.jpg?seq=' + seq + '&r=' + Date.now(); }
  $('camov').classList.toggle('hidden', seq != null && seq >= 0);
  $('camov').textContent = src === 'none' ? 'No runner and no Unreal Play session.' : 'Waiting for a camera frame (Unreal needs the BoatLab plugin v2 camera).';
  $('caminfo').textContent = cam.source ? `${cam.source} · frame ${seq}` : (seq >= 0 ? `frame ${seq}` : '-');
  $('camage').textContent = cam.stamp_s != null && s.t != null ? `${fmt(s.t - cam.stamp_s, 1)} s old` : '-';
  $('dets').innerHTML = (s.detections || []).slice(0, 12).map(d => `${d.label.padEnd(7)} ${fmt(d.range_m, 1)} m  ${fmt(d.bearing_deg, 0)}°  (${d.src})`).join('<br>');
  drawMap();
}

// ------------------------------------------------------------------ interaction
const mapEl = $('map');
mapEl.addEventListener('wheel', e => { e.preventDefault(); const r = mapEl.getBoundingClientRect(); const [wx, wy] = Winv(e.clientX - r.left, e.clientY - r.top); const f = e.deltaY < 0 ? 1.15 : 1 / 1.15; view.scale *= f; view.cx = wx - (wx - view.cx) / f; view.cy = wy - (wy - view.cy) / f; drawMap(); }, {passive: false});
mapEl.addEventListener('mousedown', e => { drag = {x: e.clientX, y: e.clientY, cx: view.cx, cy: view.cy, moved: false}; });
window.addEventListener('mouseup', e => { if (drag && !drag.moved && wpMode) { const r = mapEl.getBoundingClientRect(); waypoints.push(Winv(e.clientX - r.left, e.clientY - r.top).map(v => +v.toFixed(2))); drawMap(); } drag = null; mapEl.style.cursor = wpMode ? 'crosshair' : 'grab'; });
window.addEventListener('mousemove', e => { const r = mapEl.getBoundingClientRect(); if (e.target === mapEl) { const [x, y] = Winv(e.clientX - r.left, e.clientY - r.top); $('cursor').textContent = `x ${x.toFixed(1)}  y ${y.toFixed(1)} m`; }
  if (!drag) return; const dx = e.clientX - drag.x, dy = e.clientY - drag.y; if (Math.abs(dx) + Math.abs(dy) > 3) { drag.moved = true; follow = false; $('b_follow').classList.remove('on'); mapEl.style.cursor = 'grabbing'; view.cx = drag.cx - dx / view.scale; view.cy = drag.cy + dy / view.scale; drawMap(); } });
$('b_follow').onclick = () => { follow = !follow; $('b_follow').classList.toggle('on', follow); if (follow && view.scale < 14) view.scale = 18; drawMap(); };
$('b_fit').onclick = () => { follow = false; $('b_follow').classList.remove('on'); fit(); drawMap(); };
$('b_wp').onclick = () => { wpMode = !wpMode; $('b_wp').classList.toggle('on', wpMode); mapEl.style.cursor = wpMode ? 'crosshair' : 'grab'; };
$('b_wpclr').onclick = () => { waypoints = []; drawMap(); };
$('b_wpgo').onclick = () => { if (!waypoints.length) return note('Click "Click waypoints", then click points on the map.'); post({cmd: 'waypoints', points: waypoints}); };
$('layer').onchange = drawMap;
$('c_resume').onclick = () => post({cmd: 'resume'});
$('c_pause').onclick = () => post({cmd: 'pause'});
$('c_skip').onclick = () => post({cmd: 'skip'});
$('c_abort').onclick = () => post({cmd: 'abort'});
$('c_reset').onclick = () => post({cmd: 'reset'});
$('c_color').onclick = () => post({cmd: 'set_color', color: $('color').value});
$('c_judge').onclick = () => { const c = ['blue', 'orange', 'purple', 'yellow'][Math.floor(Math.random() * 4)]; $('color').value = c; note('Judge picked ' + c); post({cmd: 'set_color', color: c}); };
$('l_go').onclick = () => post({cmd: 'launch', backend: $('l_backend').value, mission: $('l_mission').value, nav: $('l_nav').value || undefined, paused: $('l_paused').checked, color: $('color').value})
  .then(r => r && r.ok && note('Runner starting (' + r.args.join(' ') + ')'));
$('stoprunner').onclick = () => post({cmd: 'stop_runner'});
$('e_go').onclick = () => post({cmd: 'env', current: [+$('e_cx').value, +$('e_cy').value], wind: [+$('e_wx').value, +$('e_wy').value]});
window.addEventListener('resize', drawMap);

async function poll() {
  try { const r = await fetch('/api/state', {cache: 'no-store'}); snap = await r.json(); render(); }
  catch (e) { $('status').textContent = '● DASHBOARD SERVER OFFLINE'; $('status').className = 'pill bad'; }
  finally { setTimeout(poll, 200); }
}
fetch('/api/course').then(r => r.json()).then(d => {
  course = d.course; boat = d.boat;
  $('l_mission').innerHTML = d.missions.map(m => `<option value="${m.name}" title="${m.description.replace(/"/g, '&quot;')}">${m.name} (${m.tasks} tasks)</option>`).join('');
  fit(); drawMap(); poll();
});
