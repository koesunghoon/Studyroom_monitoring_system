/* ============================================================
   STUDYCAM 대시보드 클라이언트 JS
   - 좌석/로그/상태: Flask API에서 fetch (실데이터 구조)
   - SLAM 맵 / 카메라 감지박스: 아직 ROS2·YOLO가 없어 목업 유지
     (연결되면 이 두 군데만 실데이터 바인딩으로 교체하면 됨)
   ============================================================ */

/* ---------------- 시계 ---------------- */
function tickClock() {
  const d = new Date();
  document.getElementById('clockTime').textContent = d.toLocaleTimeString('ko-KR', { hour12: false });
  document.getElementById('clockDate').textContent = d
    .toLocaleDateString('ko-KR', { year: 'numeric', month: '2-digit', day: '2-digit' })
    .replaceAll('. ', '.')
    .replace('.', '');
}
tickClock();
setInterval(tickClock, 1000);

/* ---------------- 로봇 상태 / KPI ---------------- */
async function refreshStatus() {
  try {
    const res = await fetch('/api/status');
    const s = await res.json();
    document.getElementById('kpiPatrol').innerHTML = `${s.patrol_count_today}<span> 회</span>`;
    document.getElementById('robotZone').textContent = s.zone;
    document.getElementById('robotBattery').textContent = s.battery + '%';
  } catch (e) {
    console.warn('status fetch 실패', e);
  }
}

/* ---------------- 좌석 현황 ---------------- */
const seatGrid = document.getElementById('seatGrid');

function seatMark(state) {
  if (state === 'occupied') return '✓';
  if (state === 'away') return '!';
  if (state === 'alert') return '✕';
  return '○';
}
function seatLabel(state) {
  if (state === 'occupied') return '착석';
  if (state === 'away') return '이석 12m';
  if (state === 'alert') return '위급상황';
  return '공석';
}

async function refreshSeats() {
  try {
    const res = await fetch('/api/seats');
    const seats = await res.json();
    seatGrid.innerHTML = '';
    seats.forEach((s) => {
      const el = document.createElement('div');
      el.className = 'seat ' + s.state;
      el.id = 'seat-' + s.no;
      el.innerHTML = `<span class="no">${String(s.no).padStart(2, '0')}</span>
        <span class="mark">${seatMark(s.state)}</span><span>${seatLabel(s.state)}</span>`;
      seatGrid.appendChild(el);
    });
  } catch (e) {
    console.warn('seats fetch 실패', e);
  }
}

/* ---------------- 로그 테이블 (DB에서 fetch) ---------------- */
function badgeFor(status) {
  if (['정상', '완료', '확인됨'].includes(status)) return { cls: 'good', icon: '✓' };
  if (['주의', '확인 대기'].includes(status)) return { cls: 'warn', icon: '!' };
  return { cls: 'danger', icon: '✕' };
}

async function refreshLogs(kind, tbodyId, rowFn) {
  try {
    const res = await fetch(`/api/logs/${kind}`);
    const rows = await res.json();
    const tbody = document.getElementById(tbodyId);
    tbody.innerHTML = rows.map(rowFn).join('');
  } catch (e) {
    console.warn(`${kind} logs fetch 실패`, e);
  }
}

function rowAttendance(r) {
  const b = badgeFor(r.status);
  return `<tr>
    <td class="mono-cell">${r.ts}</td><td>${r.student_id}</td><td>${r.seat}</td><td>${r.action}</td>
    <td><span class="check-badge ${b.cls}">${b.icon} ${r.status}</span></td>
  </tr>`;
}
function rowPatrol(r) {
  const b = badgeFor(r.status);
  return `<tr>
    <td class="mono-cell">${r.ts}</td><td>${r.zone}</td><td>${r.result}</td><td>${r.duration}</td>
    <td><span class="check-badge ${b.cls}">${b.icon} ${r.status}</span></td>
  </tr>`;
}
function rowWarning(r) {
  const b = badgeFor(r.handled);
  return `<tr>
    <td class="mono-cell">${r.ts}</td><td>${r.seat}</td><td>${r.wtype}</td><td>${r.confidence}</td>
    <td><span class="check-badge ${b.cls}">${b.icon} ${r.handled}</span></td>
  </tr>`;
}

function refreshAllLogs() {
  refreshLogs('attendance', 'attendBody', rowAttendance);
  refreshLogs('patrol', 'patrolBody', rowPatrol);
  refreshLogs('warning', 'warnBody', rowWarning);
}

/* ---------------- 탭 전환 ---------------- */
document.querySelectorAll('.tab-btn').forEach((btn) => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.tab-btn').forEach((b) => b.classList.remove('active'));
    document.querySelectorAll('.tab-panel').forEach((p) => p.classList.remove('active'));
    btn.classList.add('active');
    document.getElementById('tab-' + btn.dataset.tab).classList.add('active');
  });
});

/* ---------------- 카메라 감지 박스 (아직 목업 — YOLO 연동 전) ---------------- */
const camBody = document.getElementById('camBody');
function renderBoxes(boxes) {
  document.querySelectorAll('.bbox').forEach((b) => b.remove());
  (boxes || []).forEach((b) => {
    const el = document.createElement('div');
    el.className = 'bbox ' + (b.cls || '');
    el.style.left = b.l + '%';
    el.style.top = b.t + '%';
    el.style.width = b.w + '%';
    el.style.height = b.h + '%';
    el.innerHTML = `<div class="tag">${b.label}</div>`;
    camBody.appendChild(el);
  });
}

/* ---------------- SLAM 맵 (아직 목업 — Nav2/Cartographer 연동 전) ---------------- */
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
let W, H;
function resizeCanvas() {
  const r = canvas.getBoundingClientRect();
  canvas.width = r.width * devicePixelRatio;
  canvas.height = r.height * devicePixelRatio;
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  W = r.width;
  H = r.height;
}
window.addEventListener('resize', resizeCanvas);

function seeded(seed) {
  let s = seed;
  return () => {
    s = (s * 9301 + 49297) % 233280;
    return s / 233280;
  };
}
const rnd = seeded(3);
const walls = [];
for (let i = 0; i < 20; i++) {
  walls.push({ x: rnd() * 0.85 + 0.04, y: rnd() * 0.78 + 0.06, w: rnd() * 0.055 + 0.018, h: rnd() * 0.055 + 0.018 });
}
const path = [];
for (let i = 0; i < 7; i++) path.push({ x: rnd() * 0.75 + 0.1, y: rnd() * 0.7 + 0.12 });

let robotT = 0;
function drawMap() {
  if (!W) resizeCanvas();
  ctx.clearRect(0, 0, W, H);

  ctx.fillStyle = 'rgba(111,148,104,.14)';
  walls.forEach((w) => {
    ctx.beginPath();
    ctx.roundRect(w.x * W, w.y * H, w.w * W, w.h * H, 3);
    ctx.fill();
  });
  ctx.strokeStyle = '#a6a894';
  ctx.lineWidth = 1.3;
  walls.forEach((w) => {
    ctx.beginPath();
    ctx.roundRect(w.x * W, w.y * H, w.w * W, w.h * H, 3);
    ctx.stroke();
  });

  ctx.strokeStyle = 'rgba(111,148,104,.7)';
  ctx.lineWidth = 1.6;
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  path.forEach((p, i) => {
    const x = p.x * W, y = p.y * H;
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
  ctx.setLineDash([]);
  path.forEach((p) => {
    ctx.fillStyle = 'rgba(111,148,104,.45)';
    ctx.beginPath();
    ctx.arc(p.x * W, p.y * H, 2.6, 0, 7);
    ctx.fill();
  });

  robotT += 0.0032;
  const seg = Math.floor(robotT) % path.length;
  const nextSeg = (seg + 1) % path.length;
  const localT = robotT % 1;
  const a = path[seg], b = path[nextSeg];
  const rx = (a.x + (b.x - a.x) * localT) * W;
  const ry = (a.y + (b.y - a.y) * localT) * H;
  const heading = Math.atan2(b.y - a.y, b.x - a.x);

  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(heading);
  ctx.fillStyle = '#d99a2b';
  ctx.beginPath();
  ctx.moveTo(8, 0);
  ctx.lineTo(-5, 5);
  ctx.lineTo(-5, -5);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  ctx.strokeStyle = 'rgba(217,154,43,.4)';
  ctx.beginPath();
  ctx.arc(rx, ry, 10, 0, 7);
  ctx.stroke();

  requestAnimationFrame(drawMap);
}
resizeCanvas();
requestAnimationFrame(drawMap);

/* ---------------- 이벤트 피드 (스티키노트) ---------------- */
const eventFeed = document.getElementById('eventFeed');
function pushEvent(msg, time) {
  const el = document.createElement('div');
  el.className = 'sticky';
  el.innerHTML = `<div class="sticky-msg">${msg}</div><div class="sticky-time">${time || new Date().toLocaleTimeString('ko-KR', { hour12: false })}</div>`;
  eventFeed.prepend(el);
  while (eventFeed.children.length > 12) eventFeed.removeChild(eventFeed.lastChild);
}

/* ---------------- 위급상황 알림 카드 ---------------- */
function showPin() {
  document.getElementById('alertPin').classList.add('show');
}
function dismissAlert() {
  document.getElementById('alertPin').classList.remove('show');
}

/* ---------------- 데모 트리거 버튼: 실제로 서버(DB)에 이벤트를 적재함 ---------------- */
async function postEvent(type, seat) {
  await fetch('/api/event', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ type, seat }),
  });
  refreshLogs('warning', 'warnBody', rowWarning);
}

async function setSeatState(no, state) {
  await fetch(`/api/seats/${no}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ state }),
  });
  refreshSeats();
}

async function triggerFall() {
  document.getElementById('alertDesc').textContent = '7번 좌석 근처에서 쓰러짐이 의심돼요. 로봇이 지금 가고 있어요.';
  document.getElementById('alertMeta').textContent = 'CAM01 · 확신도 94%';
  showPin();
  renderBoxes([{ l: 63, t: 68, w: 16, h: 10, label: '위급! 🚨', cls: 'danger' }]);
  pushEvent('🚨 <b>7번 좌석 쓰러짐 의심</b> — 로봇 출동 중!');
  await postEvent('fall', '07번');
  await setSeatState(7, 'alert');
}
async function triggerAway() {
  pushEvent('16번 좌석 15분째 자리 비움 ⏰');
  await postEvent('away', '16번');
  await setSeatState(16, 'away');
}
async function triggerIntrusion() {
  pushEvent('출입 제한 구역에서 낯선 사람이 보여요 👀');
  renderBoxes([{ l: 20, t: 18, w: 14, h: 22, label: '미확인 인원', cls: 'warn' }]);
  await postEvent('intrusion', null);
}
function resetDemo() {
  dismissAlert();
  renderBoxes([]);
  refreshSeats();
  pushEvent('시스템이 초기화됐어요 🔄');
}

/* ---------------- 초기 로드 & 폴링 ---------------- */
refreshStatus();
refreshSeats();
refreshAllLogs();
pushEvent('로봇이 2F 열람실 순찰을 시작했어요 🚶');

setInterval(refreshStatus, 8000);
setInterval(refreshSeats, 15000);
