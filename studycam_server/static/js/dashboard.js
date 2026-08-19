/* ============================================================
   STUDYCAM 대시보드 클라이언트 JS
   - 좌석/로그/상태: Flask API에서 fetch (실데이터)
   - SLAM 맵: /api/map 실제 occupancy grid + 로봇 위치(tf) + 이동 궤적
   - 좌석 수, 착석 인원, 경고 건수 KPI: 서버 응답 길이/값 기준으로 계산 (하드코딩 없음)
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
    document.getElementById('robotZone').textContent = s.zone;
    document.getElementById('robotBattery').textContent = s.battery + '%';
    document.getElementById('kpiPatrolState').textContent = s.patrol_active ? '진행 중' : '대기 중';
  } catch (e) {
    console.warn('status fetch 실패', e);
  }
}

/* ---------------- 좌석 현황 ---------------- */
const seatGrid = document.getElementById('seatGrid');

function seatMark(state) {
  if (state === 'occupied') return '✓';
  if (state === 'alert') return '✕';
  return '○';
}
function seatLabel(state) {
  if (state === 'occupied') return '착석';
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

    // 상단 KPI(착석 인원)도 같은 응답으로 갱신 — 좌석 총원은 서버 응답 길이를 그대로 씀
    const occupied = seats.filter((s) => s.state === 'occupied').length;
    document.getElementById('kpiOccupied').textContent = occupied;
    document.getElementById('kpiSeatTotal').textContent = seats.length;
  } catch (e) {
    console.warn('seats fetch 실패', e);
  }
}

/* ---------------- 로그 테이블 ---------------- */
function badgeFor(status) {
  if (['정상', '완료', '확인됨'].includes(status)) return { cls: 'good', icon: '✓' };
  if (['주의', '확인 대기', '진행중', '진행 중'].includes(status)) return { cls: 'warn', icon: '!' };
  return { cls: 'danger', icon: '✕' };
}

async function refreshLogs(kind, tbodyId, rowFn) {
  try {
    const res = await fetch(`/api/logs/${kind}`);
    const rows = await res.json();
    const tbody = document.getElementById(tbodyId);
    tbody.innerHTML = rows.map(rowFn).join('');
    return rows;
  } catch (e) {
    console.warn(`${kind} logs fetch 실패`, e);
    return [];
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
  const distText = r.distance_m != null ? `${r.distance_m}m` : '-';
  return `<tr>
    <td class="mono-cell">${r.ts}</td><td>${r.seat}</td><td>${r.wtype}</td><td>${r.confidence}</td><td>${distText}</td>
    <td><span class="check-badge ${b.cls}">${b.icon} ${r.handled}</span></td>
  </tr>`;
}

async function refreshAllLogs() {
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

/* ============================================================
   SLAM 맵 — /api/map 실제 occupancy grid + 로봇 위치 + 궤적
   ============================================================ */
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
let W, H;
let lastMapData = null;

function resizeCanvas() {
  const r = canvas.getBoundingClientRect();
  canvas.width = r.width * devicePixelRatio;
  canvas.height = r.height * devicePixelRatio;
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  W = r.width;
  H = r.height;
  if (lastMapData) renderMap(lastMapData);
}
window.addEventListener('resize', resizeCanvas);

function worldToCanvas(x, y, map) {
  const col = (x - map.origin_x) / map.resolution;
  const rowFromBottom = (y - map.origin_y) / map.resolution;
  const row = map.height - 1 - rowFromBottom;
  const scaleX = W / map.width;
  const scaleY = H / map.height;
  return { px: col * scaleX, py: row * scaleY };
}

function renderMap(map) {
  if (!W) resizeCanvas();
  ctx.clearRect(0, 0, W, H);

  if (!map.width || !map.height || !map.data || map.data.length === 0) {
    ctx.fillStyle = 'var(--muted-2)';
    ctx.font = '13px "Space Mono"';
    ctx.fillText('맵 데이터 대기 중... (파이의 SLAM/Nav2 켜져 있는지 확인)', 16, 24);
    return;
  }

  const off = document.createElement('canvas');
  off.width = map.width;
  off.height = map.height;
  const octx = off.getContext('2d');
  const img = octx.createImageData(map.width, map.height);

  for (let row = 0; row < map.height; row++) {
    for (let col = 0; col < map.width; col++) {
      const srcIdx = (map.height - 1 - row) * map.width + col;
      const v = map.data[srcIdx];
      const dstIdx = (row * map.width + col) * 4;
      let r, g, b;
      if (v === -1) {
        r = g = b = 46;
      } else if (v >= 65) {
        r = g = b = 166;
      } else {
        r = 238; g = 240; b = 228;
      }
      img.data[dstIdx] = r;
      img.data[dstIdx + 1] = g;
      img.data[dstIdx + 2] = b;
      img.data[dstIdx + 3] = 255;
    }
  }
  octx.putImageData(img, 0, 0);

  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(off, 0, 0, map.width, map.height, 0, 0, W, H);

  if (map.trail && map.trail.length > 1) {
    ctx.strokeStyle = 'rgba(111,148,104,.7)';
    ctx.lineWidth = 1.6;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    map.trail.forEach(([x, y], i) => {
      const { px, py } = worldToCanvas(x, y, map);
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    });
    ctx.stroke();
    ctx.setLineDash([]);
  }

  if (map.robot && map.robot.valid) {
    const { px, py } = worldToCanvas(map.robot.x, map.robot.y, map);
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-map.robot.yaw);
    ctx.fillStyle = '#d99a2b';
    ctx.beginPath();
    ctx.moveTo(9, 0);
    ctx.lineTo(-6, 5.5);
    ctx.lineTo(-6, -5.5);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    ctx.strokeStyle = 'rgba(217,154,43,.4)';
    ctx.beginPath();
    ctx.arc(px, py, 10, 0, 7);
    ctx.stroke();
  }
}

async function refreshMap() {
  try {
    const res = await fetch('/api/map');
    const map = await res.json();
    lastMapData = map;
    renderMap(map);
  } catch (e) {
    console.warn('map fetch 실패', e);
  }
}

resizeCanvas();

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
let currentAlertId = null;  // 지금 팝업에 뜬 경고가 DB의 몇 번 행인지 (조치 완료 시 사용)

function showPin() {
  document.getElementById('alertPin').classList.add('show');
}
function dismissAlert() {
  document.getElementById('alertPin').classList.remove('show');
}

async function resolveAlert(status) {
  if (currentAlertId != null) {
    await fetch(`/api/warning/${currentAlertId}/resolve`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ status }),
    });
    refreshSeats();
    refreshAllLogs();
  }
  dismissAlert();
  currentAlertId = null;
}

/* ---------------- 데모 트리거 버튼: 서버(DB)에 이벤트 적재 ---------------- */
/* ---------------- 순찰 로그 실시간 감지 -> 이벤트 피드 ---------------- */
let patrolStateMap = new Map(); // patrol_log row id -> 마지막으로 본 status
let patrolFeedReady = false;    // 첫 폴링(기준선 세팅)까지는 이벤트 안 띄움

async function checkPatrolEvents() {
  try {
    const res = await fetch('/api/logs/patrol');
    const rows = await res.json();
    const ordered = [...rows].reverse(); // 서버는 최신순(DESC)으로 주므로, 오래된 순으로 훑어야 순서가 맞음

    ordered.forEach((r) => {
      const prevStatus = patrolStateMap.get(r.id);

      if (patrolFeedReady) {
        if (prevStatus === undefined && r.status === '진행중') {
          pushEvent('🚶 순찰이 시작됩니다', r.ts);
        } else if (prevStatus === '진행중' && r.status === '완료') {
          pushEvent(`✅ 순찰이 종료되었습니다 (소요 ${r.duration})`, r.ts);
        }
      }
      patrolStateMap.set(r.id, r.status);
    });

    patrolFeedReady = true;
  } catch (e) {
    console.warn('patrol event check 실패', e);
  }
}

/* ---------------- 경고 이력 실시간 감지 -> 자동 알림 (YOLO가 서버에서 직접 넣은 것도 포함) ---------------- */
let warningSeenIds = new Set();
let warningFeedReady = false;

async function checkWarningEvents() {
  try {
    const res = await fetch('/api/logs/warning');
    const rows = await res.json();
    const ordered = [...rows].reverse(); // 오래된 순으로

    ordered.forEach((r) => {
      if (!warningSeenIds.has(r.id)) {
        if (warningFeedReady) {
          if (r.wtype === '쓰러짐 의심') {
            currentAlertId = r.id;
            document.getElementById('alertDesc').textContent =
              `${r.seat}에서 쓰러짐이 의심됩니다.`;
            const distText = r.distance_m != null ? `${r.distance_m}m` : '거리 미확인';
            document.getElementById('alertMeta').textContent =
              `CAM01 · 확신도 ${Math.round(r.confidence * 100)}% · 거리 ${distText}`;
            showPin();
            pushEvent(`🚨 <b>${r.seat} 쓰러짐 의심</b> (확신도 ${Math.round(r.confidence * 100)}%, 거리 ${distText})`, r.ts);
          } else {
            pushEvent(`⚠️ ${r.seat} — ${r.wtype} 감지`, r.ts);
          }
        }
        warningSeenIds.add(r.id);
      }
    });

    warningFeedReady = true; // 첫 폴링은 기준선만 세팅
  } catch (e) {
    console.warn('warning event check 실패', e);
  }
}

/* ---------------- 초기 로드 & 폴링 ---------------- */
refreshStatus();
refreshSeats();
refreshAllLogs();
refreshMap();
checkPatrolEvents(); // 기준선만 세팅, 이벤트 안 띄움
checkWarningEvents(); // 기준선만 세팅, 이벤트 안 띄움

setInterval(refreshStatus, 5000);
setInterval(refreshSeats, 15000);
setInterval(refreshMap, 2000);
setInterval(refreshAllLogs, 10000);    // 순찰/경고 로그 주기적 갱신
setInterval(checkPatrolEvents, 2000);  // 순찰 시작/종료 실시간 감지
setInterval(checkWarningEvents, 2000); // 실제 YOLO 감지 경고 실시간 감지