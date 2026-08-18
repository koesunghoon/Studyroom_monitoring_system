

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

/* ---------------- 로봇 상태 / KPI (배터리는 이제 실데이터) ---------------- */
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

/* ---------------- 로그 테이블 ---------------- */
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

/* ---------------- 카메라 감지 박스 (데모 트리거용 오버레이만, 실제 박스는 서버가 프레임에 그려서 보냄) ---------------- */
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

/* ============================================================
   SLAM 맵 — /api/map 에서 실제 occupancy grid + 로봇 위치 + 궤적 받아 렌더링
   ============================================================ */
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
let W, H;
let lastMapData = null; // 리사이즈 시 재렌더링용

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

// world 좌표(m) -> grid 셀(col,row) -> canvas 픽셀 변환
function worldToCanvas(x, y, map) {
  const col = (x - map.origin_x) / map.resolution;
  const rowFromBottom = (y - map.origin_y) / map.resolution;
  const row = map.height - 1 - rowFromBottom; // OccupancyGrid는 y축이 위로 갈수록 증가, canvas는 반대라 뒤집음
  const scaleX = W / map.width;
  const scaleY = H / map.height;
  return { px: col * scaleX, py: row * scaleY, scaleX, scaleY };
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

  // occupancy grid를 오프스크린 캔버스에 셀 단위로 그린 뒤 확대해서 붙임 (성능)
  const off = document.createElement('canvas');
  off.width = map.width;
  off.height = map.height;
  const octx = off.getContext('2d');
  const img = octx.createImageData(map.width, map.height);

  for (let row = 0; row < map.height; row++) {
    for (let col = 0; col < map.width; col++) {
      const srcIdx = (map.height - 1 - row) * map.width + col; // OccupancyGrid row 뒤집기
      const v = map.data[srcIdx];
      const dstIdx = (row * map.width + col) * 4;
      let r, g, b;
      if (v === -1) {
        // 미탐색 영역
        r = g = b = 46;
      } else if (v >= 65) {
        // 벽/장애물
        r = g = b = 166;
      } else {
        // 빈 공간 (탐색 완료)
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

  // 이동 궤적 (실제 로봇 위치 기록)
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

  // 로봇 현재 위치 + 방향
  if (map.robot && map.robot.valid) {
    const { px, py } = worldToCanvas(map.robot.x, map.robot.y, map);
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-map.robot.yaw); // ROS yaw(반시계) -> canvas(시계) 방향 보정
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
function showPin() {
  document.getElementById('alertPin').classList.add('show');
}
function dismissAlert() {
  document.getElementById('alertPin').classList.remove('show');
}

/* ---------------- 데모 트리거 버튼: 서버(DB)에 이벤트 적재 ---------------- */
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
refreshMap();
pushEvent('로봇이 2F 열람실 순찰을 시작했어요 🚶');

setInterval(refreshStatus, 5000);   // 배터리 실데이터라 좀 더 자주
setInterval(refreshSeats, 15000);
setInterval(refreshMap, 2000);      // 맵/로봇위치 갱신