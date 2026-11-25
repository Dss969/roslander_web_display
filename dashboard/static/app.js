const $ = (selector) => document.querySelector(selector);

const ui = {
  connection: $('#connectionPill'), robotName: $('#robotName'), robotStatus: $('#robotStatus'),
  speed: $('#speedValue'), confidence: $('#poseConfidence'), battery: $('#batteryValue'),
  sensors: $('#sensorList'), sensorSummary: $('#sensorSummary'), network: $('#networkLabel'),
  networkAddress: $('#networkAddress'), latency: $('#latencyValue'), mapSource: $('#mapSource'),
  mapScale: $('#mapScale'), coordinates: $('#coordinates'), heading: $('#heading'),
  angular: $('#angularSpeed'), lastUpdate: $('#lastUpdate'), alerts: $('#alertsList'),
  alertCount: $('#alertCount'), cameraFeed: $('#cameraFeed'), cameraPlaceholder: $('#cameraPlaceholder'),
  cameraState: $('#cameraState'), cameraDot: $('#cameraDot'), mapEmpty: $('#mapEmpty'),
  canvas: $('#mapCanvas'), stage: $('#mapStage'), toast: $('#toast'), presets: $('#destinationsList'),
  presetNotice: $('#presetNotice'), navAvailability: $('#navAvailability'), navTask: $('#navigationTask'),
  navStatusLabel: $('#navStatusLabel'), navStatusMessage: $('#navStatusMessage'), navDistance: $('#navDistance'),
  navStop: $('#navStopButton'), navDialog: $('#navConfirmDialog'), navConfirmText: $('#navConfirmText'),
  health: $('#healthBanner'), healthTitle: $('#healthTitle'), healthMessage: $('#healthMessage'),
  healthDetail: $('#healthDetail'), cameraLabel: $('#cameraLabel'),
};

let state = null;
let previousOnline = null;
let lastStateAt = 0;
let cameraActive = false;
let cameraRetryAt = 0;
let cameraMode = null;
let cameraTimer = null;
let selectedPlace = null;
let pollInFlight = false;
let mapImage = new Image();
let mapRevision = -1;
let view = { zoom: 1, panX: 0, panY: 0, rotation: 0, dragging: false, rotating: false, startX: 0, startY: 0, follow: false };

const escapeHtml = (value) => String(value).replace(/[&<>'"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;',"'":'&#39;','"':'&quot;'}[c]));
const degrees = radians => ((radians * 180 / Math.PI) % 360 + 360) % 360;
const frameAge = age => age == null ? '等待数据' : age < 1 ? '刚刚更新' : `${Math.floor(age)} 秒前更新`;

function setClock() {
  $('#clock').textContent = new Intl.DateTimeFormat('zh-CN', {hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false}).format(new Date());
}

function showToast(message) {
  ui.toast.textContent = message;
  ui.toast.classList.add('show');
  clearTimeout(showToast.timeout);
  showToast.timeout = setTimeout(() => ui.toast.classList.remove('show'), 2200);
}

function renderStatus(data) {
  const { robot, network } = data;
  const sim = data.mode === 'simulation', paused = data.simulation?.paused;
  $('#modeBadge').textContent = sim ? (data.read_only ? '仿真监控' : '仿真导航') : '实车控制台';
  $('#velocityDescription').textContent = sim ? 'm/s · 仿真里程计' : 'm/s · 估算速度';
  $('#velocityDescription').title = sim ? '本机仿真轮速与 IMU 融合' : '速度命令积分与 IMU 融合';
  ui.robotName.textContent = robot.name;
  ui.robotStatus.textContent = robot.status_label;
  ui.speed.textContent = robot.velocity_fresh ? robot.velocity.linear.toFixed(2) : '—';
  ui.angular.textContent = robot.velocity_fresh ? `${robot.velocity.angular.toFixed(2)} rad/s` : '—';

  ui.connection.classList.toggle('online', robot.online);
  ui.connection.querySelector('span').textContent = sim ? data.simulation.label : robot.online ? '实车在线' : '实车离线';
  if (previousOnline === false && robot.online) showToast(sim ? '仿真数据已恢复' : '实车连接已恢复');
  if (previousOnline === true && !robot.online) showToast(sim ? '仿真数据已中断' : '实车连接已断开');
  previousOnline = robot.online;

  if (robot.pose) {
    ui.confidence.textContent = robot.pose_held ? '暂停前位置' : robot.localized ? '定位正常' : '位置已过期';
    ui.coordinates.textContent = `X ${robot.pose.x.toFixed(2)}  Y ${robot.pose.y.toFixed(2)}`;
    const deg = degrees(robot.pose.yaw);
    ui.heading.textContent = `${Math.round(deg)}° · 地图坐标`;
    ui.mapEmpty.classList.toggle('hidden', robot.localized);
    ui.mapEmpty.querySelector('strong').textContent = robot.localized ? '定位正常' : '位置待更新';
    ui.mapEmpty.querySelector('small').textContent = '灰色标记为最后位置，正在等待实时定位';
  } else {
    ui.confidence.textContent = '未定位';
    ui.coordinates.textContent = 'X —   Y —';
    ui.heading.textContent = '—';
    ui.mapEmpty.classList.remove('hidden');
    ui.mapEmpty.querySelector('strong').textContent = '等待定位';
    ui.mapEmpty.querySelector('small').textContent = '定位恢复后自动显示实时位置';
  }
  ui.mapEmpty.setAttribute('aria-hidden', String(Boolean(robot.pose && robot.localized)));

  const battery = robot.battery;
  ui.battery.textContent = battery?.fresh && battery.percent != null ? `约 ${battery.percent}%` : battery?.fresh ? '待标定' : '—%';
  $('#batteryDetail').textContent = battery?.fresh && battery.voltage != null ? `${battery.estimated ? '电压估算' : '待确认电池规格'} · ${battery.voltage.toFixed(2)} V` : battery ? '电池数据已过期' : '等待电池数据';
  ui.battery.title = battery?.estimated ? '根据电池规格与电压曲线估算；受负载和温度影响，不是 BMS 实测电量' : '已收到电压；需确认电池串数和类型才能估算电量';
  if (sim) {
    ui.battery.textContent = '不适用'; $('#batteryDetail').textContent = '纯仿真无实车电池'; ui.battery.title = '未订阅实车硬件电池数据';
    $('#networkTitle').textContent = '本机仿真数据';
  }
  ui.network.textContent = !network.checked_at ? '检测中' : network.reachable ? '虚拟网络已连通' : '虚拟网络不可达';
  if (sim) ui.network.textContent = data.simulation.label;
  ui.networkAddress.textContent = network.address;
  ui.latency.textContent = network.latency_ms != null ? `${network.latency_ms} ms` : '—';
  if (sim) { ui.latency.textContent = data.simulation.clock_seconds == null ? '—' : `${data.simulation.clock_seconds.toFixed(1)} s`; ui.latency.title = '仿真时间'; }
  ui.lastUpdate.parentElement.classList.toggle('live', robot.online);
  ui.lastUpdate.textContent = robot.online ? '数据实时更新中' : '等待实时数据';
  if (sim && paused) ui.lastUpdate.textContent = '仿真暂停 · 保留暂停前数据';
}

function renderHealth(data) {
  const critical = data.alerts.find(alert => alert.level === 'critical');
  const warning = data.alerts.find(alert => alert.level === 'warning');
  const issue = critical || warning;
  const display = issue ? presentAlert(issue) : null;
  ui.health.className = `health-banner ${critical ? 'critical' : warning ? 'warning' : 'healthy'}`;
  ui.healthTitle.textContent = display?.title || (data.robot.moving ? '小车正在行驶' : '小车状态正常');
  ui.healthMessage.textContent = display?.message || (data.navigation.safe_to_navigate ? '定位、扫描与导航连接正常' : '实时监控已连接，等待导航服务就绪');
  const hz = data.telemetry?.scan_hz;
  ui.healthDetail.textContent = hz != null ? `激光扫描 ${hz.toFixed(1)} Hz` : '等待扫描数据';
  if (data.mode === 'simulation') {
    if (!display) { ui.healthTitle.textContent = data.simulation.label; ui.healthMessage.textContent = data.simulation.paused ? '网页保持连接，位置与相机保留暂停前数据' : '本机地图、定位与传感器状态持续更新 · 可选择预设点导航'; }
    if (data.simulation.paused) ui.health.className = 'health-banner paused';
    if (data.simulation.clock_seconds != null) ui.healthDetail.textContent = `${data.simulation.clock_seconds.toFixed(1)} s${hz != null ? ` · 激光 ${hz.toFixed(1)} Hz` : ''}`;
  }
}

function presentAlert(alert) {
  if (state?.mode === 'simulation' && alert.title === 'ekf_filter_node: odometry/filtered topic status' && alert.message === 'Frequency too low.') {
    return {...alert, title:'里程计频率诊断提示', message:'EKF 诊断报告发布频率偏低，请结合左侧里程计更新状态查看。', detail:`${alert.title} · ${alert.message}`};
  }
  if (alert.title.includes('ekf_filter_node: odometry/filtered topic status') && alert.message === 'No events recorded.') {
    return {
      ...alert,
      title: '里程计驱动自检告警',
      message: '驱动自检报告未记录到数据，请检查里程计与坐标变换是否恢复。',
      detail: `${alert.title} · ${alert.message}`,
    };
  }
  return alert;
}

function renderSensors(sensors) {
  const ok = sensors.filter(item => item.status === 'ok').length;
  const relevant = sensors.filter(item => !['not_applicable','unavailable'].includes(item.status));
  ui.sensorSummary.textContent = state?.simulation?.paused ? `${relevant.filter(item=>item.status==='paused').length} / ${relevant.length} 暂停` : `${ok} / ${relevant.length} 正常`;
  ui.sensors.innerHTML = sensors.map(item => `
    <div class="sensor-item ${escapeHtml(item.status)}">
      <i class="dot"></i>
      <div><strong>${escapeHtml(item.label)}</strong><small>${escapeHtml(item.status === 'ok' ? frameAge(state?.telemetry?.ages?.[item.id]) : item.detail)}</small></div>
      <span>${({ok:'在线',error:'异常',paused:'暂停',disconnected:'未连接',not_applicable:'不适用',unavailable:'未提供'})[item.status] || '等待'}</span>
    </div>`).join('');
}

function renderAlerts(alerts) {
  ui.alertCount.textContent = `${alerts.length} 条`;
  if (!alerts.length) {
    ui.alerts.innerHTML = '<div class="alert-empty"><span>✓</span><strong>运行状态良好</strong><small>没有需要处理的异常</small></div>';
    return;
  }
  ui.alerts.innerHTML = alerts.map(raw => {
    const alert = presentAlert(raw);
    return `
    <article class="alert-item ${escapeHtml(alert.level)}">
      <span class="alert-icon"><svg><use href="#i-alert"></use></svg></span>
      <div><strong>${escapeHtml(alert.title)}</strong><p>${escapeHtml(alert.message)}</p>${alert.detail ? `<details><summary>查看设备原始诊断</summary><p>${escapeHtml(alert.detail)}</p></details>` : ''}</div>
    </article>`;
  }).join('');
}

function renderCamera(camera) {
  if (state?.mode === 'simulation') {
    const visible = camera.online || camera.held;
    ui.cameraState.textContent = camera.held ? '已暂停' : camera.online ? '实时' : camera.available ? '等待数据' : '未提供';
    ui.cameraDot.classList.toggle('live', camera.online);
    ui.cameraLabel.textContent = `仿真 RGB · ${camera.held ? '暂停前画面' : camera.online ? 'LIVE' : camera.available ? '等待画面' : '未提供'}`;
    ui.cameraPlaceholder.querySelector('strong').textContent = camera.available ? '等待仿真相机' : '当前仿真未提供相机';
    ui.cameraPlaceholder.querySelector('small').textContent = '仅显示本机仿真图像';
    if (!camera.online && cameraTimer) { clearInterval(cameraTimer); cameraTimer = null; }
    if (!visible) { cameraMode=null; ui.cameraFeed.removeAttribute('src'); ui.cameraFeed.classList.remove('visible'); ui.cameraPlaceholder.classList.remove('hidden'); return; }
    if (camera.held) {
      if (cameraMode !== 'held') ui.cameraFeed.src = `${camera.image_url}?t=${Date.now()}`;
      cameraMode='held'; return;
    }
  }
  ui.cameraState.textContent = camera.online ? '实时' : '离线';
  ui.cameraDot.classList.toggle('live', camera.online);
  ui.cameraLabel.textContent = state?.mode === 'simulation' ? '仿真 RGB · LIVE' : camera.online ? 'ASTRA RGB · LIVE' : 'ASTRA RGB · 画面中断';
  if (!camera.online && camera.mode !== 'stream') {
    if (cameraTimer) clearInterval(cameraTimer);
    cameraTimer = null;
    cameraMode = null;
    ui.cameraFeed.removeAttribute('src');
  }
  if (!camera.online) {
    ui.cameraFeed.classList.remove('visible');
    ui.cameraPlaceholder.classList.remove('hidden');
  }
  if (camera.mode === 'snapshot' && cameraMode !== 'snapshot') {
    if (cameraTimer) clearInterval(cameraTimer);
    cameraMode = 'snapshot';
    cameraActive = true;
    const refresh = () => {
      // A slow remote frame must finish before the next src replaces it.
      if (cameraMode === 'snapshot' && ui.cameraFeed.complete && state?.camera.online
          && Date.now() >= cameraRetryAt && Date.now() - lastStateAt < 4000) {
        ui.cameraFeed.src = `${camera.image_url}?t=${Date.now()}`;
      }
    };
    refresh();
    cameraTimer = setInterval(refresh, 350);
  } else if (camera.mode === 'stream' && cameraMode !== 'stream' && Date.now() >= cameraRetryAt) {
    if (cameraTimer) clearInterval(cameraTimer);
    cameraTimer = null;
    cameraMode = 'stream';
    cameraActive = true;
    ui.cameraFeed.src = `${camera.image_url}?t=${Date.now()}`;
  }
}

ui.cameraFeed.addEventListener('load', () => {
  if ((state?.camera.online || state?.camera.held) && Date.now() - lastStateAt < 4000) {
    ui.cameraFeed.classList.add('visible');
    ui.cameraPlaceholder.classList.add('hidden');
  }
});
ui.cameraFeed.addEventListener('error', () => {
  cameraActive = false;
  cameraMode = null;
  cameraRetryAt = Date.now() + 5000;
  ui.cameraFeed.classList.remove('visible');
  ui.cameraPlaceholder.classList.remove('hidden');
});

const activeNavigation = navigation => ['sending', 'executing', 'canceling', 'stop_failed', 'unconfirmed'].includes(navigation.status);

function renderNavigation(navigation) {
  const active = activeNavigation(navigation);
  ui.navAvailability.querySelector('i').classList.toggle('live', navigation.safe_to_navigate);
  ui.navAvailability.querySelector('b').textContent = ['unconfirmed', 'stop_failed'].includes(navigation.status) ? '任务待确认' : navigation.safe_to_navigate ? '导航可用' : navigation.available ? '待设备恢复' : '等待导航';
  ui.navStatusLabel.textContent = navigation.active_place ? `${navigation.status_label} · ${navigation.active_place.name}` : navigation.status_label;
  ui.navStatusMessage.textContent = !active && navigation.blocked_reason ? navigation.blocked_reason : navigation.message;
  ui.navDistance.textContent = navigation.distance_remaining != null ? `${navigation.distance_remaining.toFixed(1)} m` : '—';
  ui.navTask.classList.toggle('active', active);
  ui.navTask.classList.toggle('error', ['failed', 'rejected', 'stop_failed', 'unconfirmed'].includes(navigation.status));
  ui.navStop.disabled = !active || navigation.can_cancel === false;
  ui.navStop.title = navigation.can_cancel === false ? '只能停止本网页发起的当前任务' : '取消当前网页导航任务';
  if (state?.read_only) {
    ui.navStop.disabled = true;
    ui.navStop.title = '仿真监控模式不发送导航或取消命令';
    ui.navAvailability.querySelector('i').classList.toggle('live', navigation.available);
    ui.navAvailability.querySelector('b').textContent = navigation.available ? 'Nav2 已就绪' : '等待 Nav2';
    ui.navStatusMessage.textContent = navigation.message;
  }
  ui.navStop.querySelector('span').textContent = navigation.status === 'canceling' ? '正在停止…' : navigation.status === 'stop_failed' ? '重试停止' : '停止导航';
  const phase = navigation.status === 'succeeded' ? 3 : navigation.status === 'executing' ? 2 : navigation.status === 'sending' ? 1 : 0;
  $('#taskSteps').querySelectorAll('li').forEach((step, index) => {
    step.classList.toggle('done', index + 1 < phase || phase === 3);
    step.classList.toggle('current', index + 1 === phase && phase !== 3);
  });
  const eta = navigation.eta_seconds;
  $('#navEta').textContent = navigation.status === 'unconfirmed' ? '等待任务状态确认' : eta != null && navigation.status === 'executing' ? `预计 ${eta < 60 ? `${Math.ceil(eta)} 秒` : `${Math.ceil(eta / 60)} 分钟`}到达` : '预计到达 —';
  $('#navRecovery').textContent = navigation.recoveries && active ? `正在调整路线 · ${navigation.recoveries} 次` : navigation.status_label;
}

function renderPresets(presets, navigation) {
  ui.presetNotice.textContent = presets.notice || '预设目标发送前必须二次确认';
  if (!presets.places?.length) {
    ui.presets.innerHTML = '<div class="alert-empty"><strong>暂无预设地点</strong></div>';
    return;
  }
  const markup = presets.places.map(place => {
    const available = !navigationRequestPending && presets.navigation_enabled && place.enabled && navigation.safe_to_navigate && !activeNavigation(navigation);
    const reason = activeNavigation(navigation) ? '当前任务未结束，结束确认后可再次选择' : navigation.blocked_reason || '该地点未启用';
    const icon = {home: 'i-home', scan: 'i-scan', place: 'i-box'}[place.icon] || 'i-pin';
    return `<button class="destination-item ${place.verified ? 'verified' : 'unverified'}" data-place="${escapeHtml(place.id)}" type="button" ${available ? '' : 'disabled'} title="${escapeHtml(available ? (state?.mode === 'simulation' ? '点击直接开始导航' : '点击确认后开始导航') : reason)}" aria-label="${escapeHtml(place.name)}${place.verified ? '' : '，待安全验收'}">
      <span class="destination-icon"><svg><use href="#${icon}"></use></svg></span>
      <span class="destination-copy"><strong>${escapeHtml(place.name)}</strong><small>${escapeHtml(place.description)}</small></span>
      <span class="destination-state">${activeNavigation(navigation) ? (navigation.status === 'executing' ? '任务进行中' : '等待确认') : place.verified ? (available ? '可前往' : '等待') : '待安全验收'}</span>
    </button>`;
  }).join('');
  if (ui.presets.innerHTML !== markup) ui.presets.innerHTML = markup;
}

async function postJson(url, body) {
  const response = await fetch(url, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body),
    signal: AbortSignal.timeout(8000),
  });
  const payload = await response.json().catch(() => ({message: `HTTP ${response.status}`}));
  if (!response.ok) throw new Error(payload.message || `HTTP ${response.status}`);
  return payload;
}

async function startNavigation(place) {
  if (navigationRequestPending) return;
  navigationRequestPending = true;
  ui.presets.querySelectorAll('button').forEach(button => { button.disabled = true; });
  try {
    const body = {place_id: place.id};
    if (state?.mode !== 'simulation') body.acknowledge_unverified = !place.verified;
    const payload = await postJson('/api/navigation/start', body);
    showToast(payload.message || '导航任务已发送');
    await pollState();
  } catch (error) {
    showToast(`无法开始导航：${error.message}`);
  } finally {
    navigationRequestPending = false;
    await pollState();
  }
}

async function stopNavigation() {
  if (ui.navStop.disabled) return;
  ui.navStop.disabled = true;
  try {
    const payload = await postJson('/api/navigation/cancel', {});
    showToast(payload.message || '停止请求已发送');
    await pollState();
  } catch (error) {
    showToast(`停止失败：${error.message}`);
    await pollState();
  }
}

function updateMapImage(map) {
  const revision = `${map.generation || ''}:${map.revision}`;
  if (revision === mapRevision) return;
  mapRevision = revision;
  mapImage = new Image();
  mapImage.onload = drawMap;
  mapImage.src = `${map.image_url}?rev=${encodeURIComponent(revision)}`;
}

function renderMapMeta(map) {
  ui.mapSource.classList.toggle('live', map.live);
  ui.mapSource.innerHTML = `<i></i>${map.live ? '实时地图' : state?.mode === 'simulation' ? '仿真配置地图' : '缓存地图'}`;
  ui.mapScale.textContent = `分辨率 ${(map.resolution * 100).toFixed(1).replace(/\.0$/, '')} cm`;
  updateMapImage(map);
}

function mapTransform(canvas, map) {
  const margin = 32;
  const baseScale = Math.min((canvas.clientWidth - margin * 2) / map.width, (canvas.clientHeight - margin * 2) / map.height);
  const scale = baseScale * view.zoom;
  return {
    scale,
    centerX: canvas.clientWidth / 2 + view.panX,
    centerY: canvas.clientHeight / 2 + view.panY,
    rotation: view.rotation,
  };
}

function worldToScreen(x, y, map, transform) {
  const dx = x - map.origin[0], dy = y - map.origin[1], yaw = map.origin[2] || 0;
  const px = (Math.cos(yaw) * dx + Math.sin(yaw) * dy) / map.resolution;
  const py = map.height - (-Math.sin(yaw) * dx + Math.cos(yaw) * dy) / map.resolution;
  const localX = (px - map.width / 2) * transform.scale;
  const localY = (py - map.height / 2) * transform.scale;
  const cos = Math.cos(transform.rotation), sin = Math.sin(transform.rotation);
  return { x: transform.centerX + cos * localX - sin * localY,
           y: transform.centerY + sin * localX + cos * localY };
}

function drawMap() {
  if (!state) return;
  const canvas = ui.canvas;
  const rect = canvas.getBoundingClientRect();
  if (!rect.width || !rect.height) return;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  if (canvas.width !== Math.round(rect.width * dpr) || canvas.height !== Math.round(rect.height * dpr)) {
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);
  if (!mapImage.complete || !mapImage.naturalWidth) return;

  const t = mapTransform(canvas, state.map);
  if (view.follow && state.robot.pose && state.robot.localized) {
    const p = worldToScreen(state.robot.pose.x, state.robot.pose.y, state.map, t);
    const dx = rect.width / 2 - p.x, dy = rect.height / 2 - p.y;
    view.panX += dx; view.panY += dy;
    t.centerX += dx; t.centerY += dy;
  }
  const rotationDegrees = Math.round(view.rotation * 180 / Math.PI);
  $('#rotationValue').textContent = `${rotationDegrees}°`;
  $('#resetRotation').style.transform = `rotate(${rotationDegrees}deg)`;
  canvas.dataset.rotation = String(view.rotation);
  canvas.dataset.zoom = String(view.zoom);
  canvas.dataset.panX = String(view.panX);
  canvas.dataset.panY = String(view.panY);
  ctx.save();
  ctx.translate(t.centerX, t.centerY);
  ctx.rotate(t.rotation);
  ctx.shadowColor = 'rgba(25, 34, 32, .14)';
  ctx.shadowBlur = 22;
  ctx.shadowOffsetY = 8;
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapImage, -state.map.width * t.scale / 2, -state.map.height * t.scale / 2, state.map.width * t.scale, state.map.height * t.scale);
  ctx.restore();

  if (state.path?.length > 1 && activeNavigation(state.navigation) && state.robot.localized) {
    ctx.beginPath();
    state.path.forEach((point, index) => {
      const p = worldToScreen(point.x, point.y, state.map, t);
      index ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y);
    });
    ctx.strokeStyle = 'rgba(20,125,112,.75)';
    ctx.lineWidth = 3;
    ctx.setLineDash([6, 5]);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  const pose = state.robot.pose;
  if (pose) {
    const p = worldToScreen(pose.x, pose.y, state.map, t);
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(Math.PI / 2 - pose.yaw + (state.map.origin[2] || 0) + t.rotation);
    ctx.globalAlpha = state.robot.localized ? 1 : .45;
    ctx.beginPath(); ctx.arc(0, 0, 20, 0, Math.PI * 2); ctx.fillStyle = 'rgba(20,125,112,.14)'; ctx.fill();
    ctx.beginPath(); ctx.moveTo(0, -16); ctx.lineTo(8, 4); ctx.lineTo(0, 1); ctx.lineTo(-8, 4); ctx.closePath();
    ctx.fillStyle = state.robot.localized ? '#147d70' : '#69716e'; ctx.shadowColor = 'rgba(8,72,64,.35)'; ctx.shadowBlur = 10; ctx.fill();
    ctx.beginPath(); ctx.arc(0, 0, 5, 0, Math.PI * 2); ctx.fillStyle = '#f7f4ed'; ctx.shadowBlur = 0; ctx.fill();
    ctx.restore();
  }
}

async function pollState() {
  if (pollInFlight) return;
  pollInFlight = true;
  try {
    const response = await fetch('/api/state', {cache: 'no-store', signal: AbortSignal.timeout(4000)});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    state = await response.json();
    lastStateAt = Date.now();
    renderStatus(state);
    renderHealth(state);
    renderSensors(state.sensors);
    renderAlerts(state.alerts);
    renderCamera(state.camera);
    renderNavigation(state.navigation);
    renderPresets(state.presets, state.navigation);
    renderMapMeta(state.map);
    drawMap();
  } catch (error) {
    ui.connection.classList.remove('online');
    ui.connection.querySelector('span').textContent = '服务未连接';
    ui.robotStatus.textContent = '网页后端不可用';
    ui.health.className = 'health-banner critical';
    ui.healthTitle.textContent = '实时连接已中断';
    ui.healthMessage.textContent = '正在自动重连。历史位置与画面不可作为实时判断依据。';
    ui.healthDetail.textContent = lastStateAt ? `${Math.floor((Date.now() - lastStateAt) / 1000)} 秒未更新` : '等待连接';
    ui.speed.textContent = '—';
    ui.angular.textContent = '—';
    ui.confidence.textContent = '连接中断';
    ui.cameraFeed.classList.remove('visible');
    ui.cameraPlaceholder.classList.remove('hidden');
    ui.cameraState.textContent = '连接中断';
    ui.cameraDot.classList.remove('live');
    ui.cameraLabel.textContent = state?.mode === 'simulation' ? '仿真 RGB · 网页连接中断' : 'ASTRA RGB · 连接中断';
    ui.lastUpdate.textContent = '连接已中断';
    ui.lastUpdate.parentElement.classList.remove('live');
    ui.network.textContent = '网页连接中断'; ui.latency.textContent = '—';
    if (state) {
      renderSensors(state.sensors.map(item => ['not_applicable','unavailable'].includes(item.status) ? item : {...item,status:'disconnected',detail:'网页连接中断，当前状态未知'}));
      ui.sensorSummary.textContent = '连接中断';
    }
    ui.mapSource.classList.remove('live'); ui.mapSource.innerHTML = '<i></i>上次地图';
    ui.mapEmpty.classList.remove('hidden'); ui.mapEmpty.setAttribute('aria-hidden','false');
    ui.mapEmpty.querySelector('strong').textContent = '位置未确认';
    ui.mapEmpty.querySelector('small').textContent = '灰色标记为上次位置，连接恢复后自动更新';
    if (cameraTimer) clearInterval(cameraTimer);
    cameraTimer = null; cameraMode = null;
    ui.navStop.disabled = true;
    ui.navStatusMessage.textContent = '等待网页连接恢复；任务状态暂不可确认';
    if (state) { state.robot.localized = false; state.camera.online = false; drawMap(); }
    ui.presets.querySelectorAll('button').forEach(button => { button.disabled = true; });
    ui.navAvailability.querySelector('b').textContent = '连接中断';
    ui.navAvailability.querySelector('i').classList.remove('live');
  } finally {
    pollInFlight = false;
  }
}

let navigationRequestPending = false;
ui.presets.addEventListener('click', event => {
  const button = event.target.closest('.destination-item[data-place]');
  if (!button || button.disabled || !state) return;
  selectedPlace = state.presets.places.find(place => place.id === button.dataset.place) || null;
  if (!selectedPlace) return;
  if (state.mode === 'simulation' && selectedPlace.verified) {
    const place = selectedPlace;
    selectedPlace = null;
    startNavigation(place);
    return;
  }
  ui.navDialog.querySelector('h2').textContent = selectedPlace.verified ? '确认前往这个地点？' : '确认发送待验收目标？';
  ui.navConfirmText.textContent = selectedPlace.verified ? `小车将前往 ${selectedPlace.name}。` : `${selectedPlace.name} 尚未标记为安全验收完成。请确认目标位置、路线和最终朝向安全后再发送。`;
  ui.navDialog.showModal();
});

ui.navDialog.addEventListener('close', () => {
  if (ui.navDialog.returnValue === 'confirm' && selectedPlace) startNavigation(selectedPlace);
  selectedPlace = null;
});
ui.navStop.addEventListener('click', stopNavigation);

$('#zoomIn').addEventListener('click', () => { view.zoom = Math.min(3, view.zoom * 1.2); drawMap(); });
$('#zoomOut').addEventListener('click', () => { view.zoom = Math.max(.65, view.zoom / 1.2); drawMap(); });
function setFollow(value) { view.follow = value; $('#followRobot').setAttribute('aria-pressed', String(value)); }
$('#resetView').addEventListener('click', () => { setFollow(false); view = {...view, zoom: 1, panX: 0, panY: 0, rotation: 0}; drawMap(); });
$('#followRobot').addEventListener('click', () => { setFollow(!view.follow); drawMap(); });
function rotateMap(delta) {
  view.rotation = Math.atan2(Math.sin(view.rotation + delta), Math.cos(view.rotation + delta));
  drawMap();
}
$('#rotateLeft').addEventListener('click', () => rotateMap(-Math.PI / 12));
$('#rotateRight').addEventListener('click', () => rotateMap(Math.PI / 12));
$('#resetRotation').addEventListener('click', () => { view.rotation = 0; drawMap(); });
const pointers = new Map();
let gesture = null;
function touchGeometry() {
  const [a, b] = [...pointers.values()];
  return {distance: Math.hypot(b.x - a.x, b.y - a.y), angle: Math.atan2(b.y - a.y, b.x - a.x), x: (a.x + b.x) / 2, y: (a.y + b.y) / 2};
}
ui.stage.addEventListener('pointerdown', event => {
  if (event.target.closest('button') || (event.pointerType === 'mouse' && ![0, 1].includes(event.button))) return;
  event.preventDefault();
  setFollow(false);
  pointers.set(event.pointerId, {x: event.clientX, y: event.clientY});
  if (pointers.size === 2) {
    gesture = {...touchGeometry(), zoom: view.zoom, rotation: view.rotation, panX: view.panX, panY: view.panY};
    view.dragging = false;
  } else {
    view.rotating = event.button === 1;
    view.dragging = true;
    view.startX = event.clientX - view.panX;
    view.startY = event.clientY - view.panY;
    view.rotationStartX = event.clientX;
    view.rotationStart = view.rotation;
  }
  ui.stage.classList.add(view.rotating ? 'rotating' : 'dragging');
  ui.stage.setPointerCapture(event.pointerId);
});
ui.stage.addEventListener('pointermove', event => {
  if (!pointers.has(event.pointerId)) return;
  pointers.set(event.pointerId, {x: event.clientX, y: event.clientY});
  if (gesture && pointers.size === 2) {
    const current = touchGeometry();
    view.zoom = Math.max(.4, Math.min(5, gesture.zoom * current.distance / Math.max(1, gesture.distance)));
    view.rotation = gesture.rotation + current.angle - gesture.angle;
    view.panX = gesture.panX + current.x - gesture.x;
    view.panY = gesture.panY + current.y - gesture.y;
  } else if (view.dragging && view.rotating) {
    view.rotation = view.rotationStart + (event.clientX - view.rotationStartX) * .008;
  } else if (view.dragging) {
    view.panX = event.clientX - view.startX;
    view.panY = event.clientY - view.startY;
  }
  drawMap();
});
function endPointer(event) {
  pointers.delete(event.pointerId);
  gesture = null;
  view.dragging = false; view.rotating = false;
  ui.stage.classList.remove('dragging', 'rotating');
  if (ui.stage.hasPointerCapture(event.pointerId)) ui.stage.releasePointerCapture(event.pointerId);
}
ui.stage.addEventListener('pointerup', endPointer);
ui.stage.addEventListener('pointercancel', endPointer);
ui.stage.addEventListener('mousedown', event => { if (event.button === 1) event.preventDefault(); });
ui.stage.addEventListener('auxclick', event => { if (event.button === 1) event.preventDefault(); });
$('#cameraExpand').addEventListener('click', async () => {
  try {
    if (document.fullscreenElement) await document.exitFullscreen();
    else if ($('#cameraFrame').requestFullscreen) await $('#cameraFrame').requestFullscreen();
    else showToast('当前浏览器暂不支持画面全屏');
  } catch { showToast('当前浏览器暂不支持画面全屏'); }
});
ui.stage.addEventListener('wheel', event => { event.preventDefault(); view.zoom = Math.max(.65, Math.min(3, view.zoom * (event.deltaY > 0 ? .9 : 1.1))); drawMap(); }, {passive: false});

new ResizeObserver(drawMap).observe(ui.stage);
setClock();
setInterval(setClock, 1000);
pollState();
setInterval(pollState, 1000);

window.addEventListener('monitor-visible', () => requestAnimationFrame(drawMap));
