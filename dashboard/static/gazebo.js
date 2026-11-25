import RFB from '/novnc/core/rfb.js';

const $ = selector => document.querySelector(selector);
const monitor = $('#monitorView'), gazebo = $('#gazeboView');
const status = $('#gazeboStatus'), screen = $('#gazeboScreen');
let rfb = null, retry = null, active = false, failures = 0, connecting = false, generation = 0;

function scheduleReconnect() {
  clearTimeout(retry);
  if (!active) return;
  const delay = Math.min(15000, 1000 * 2 ** Math.min(failures++, 4));
  status.textContent = `画面断开 · ${delay / 1000} 秒后重连`;
  retry = setTimeout(connect, delay);
}

async function connect() {
  clearTimeout(retry);
  if (rfb || connecting) return;
  connecting = true;
  const version = ++generation;
  status.textContent = '正在连接 Gazebo…';
  try {
    const response = await fetch('/api/gazebo/session', {cache: 'no-store', signal: AbortSignal.timeout(4000)});
    if (response.status === 401) {
      status.textContent = '登录已过期 · 请刷新页面登录';
      return;
    }
    if (!response.ok) throw new Error('图形会话未就绪');
    const credentials = await response.json();
    if (generation !== version) return;
    const url = new URL('/gazebo/websockify', location.href);
    url.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const client = new RFB(screen, url.href, {credentials});
    rfb = client;
    client.viewOnly = false;
    client.scaleViewport = true;
    client.resizeSession = false;
    client.qualityLevel = 6;
    client.compressionLevel = 2;
    client.focusOnClick = true;
    client.addEventListener('connect', () => {
      failures = 0;
      status.textContent = '已连接 · 键鼠可交互';
      if (!active) client.blur();
    });
    client.addEventListener('disconnect', () => {
      if (rfb !== client) return;
      rfb = null;
      scheduleReconnect();
      if (!active) status.textContent = '画面断开 · 切换后重连';
    });
    client.addEventListener('securityfailure', () => { status.textContent = '图形鉴权失败 · 正在重新连接'; });
  } catch (error) {
    if (generation === version) scheduleReconnect();
  } finally {
    if (generation === version) connecting = false;
  }
}

function select(showGazebo) {
  active = showGazebo;
  monitor.hidden = showGazebo;
  gazebo.hidden = !showGazebo;
  $('#monitorTab').setAttribute('aria-pressed', String(!showGazebo));
  $('#gazeboTab').setAttribute('aria-pressed', String(showGazebo));
  if (showGazebo) connect();
  else {
    clearTimeout(retry);
    rfb?.blur();
    window.dispatchEvent(new Event('monitor-visible'));
  }
}

$('#monitorTab').addEventListener('click', () => select(false));
$('#gazeboTab').addEventListener('click', () => select(true));
$('#gazeboReconnect').addEventListener('click', () => {
  generation++;
  connecting = false;
  const old = rfb;
  rfb = null;
  old?.disconnect();
  connect();
});
$('#gazeboFullscreen').addEventListener('click', async () => {
  try {
    if (document.fullscreenElement) await document.exitFullscreen();
    else await gazebo.requestFullscreen();
  } catch { status.textContent = '浏览器未允许全屏，可继续在页面中操作'; }
});
window.addEventListener('online', () => { if (active && !rfb) connect(); });
window.addEventListener('pagehide', () => { clearTimeout(retry); rfb?.disconnect(); });
