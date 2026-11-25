#!/usr/bin/env python3
"""Authenticated loopback entry for the simulation dashboard and Gazebo WS."""
import asyncio
from collections import defaultdict, deque
import hmac
from pathlib import Path
import secrets
import sys
import time
from urllib.parse import urlsplit

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / '.deps/remote/python'))
from aiohttp import ClientError, ClientSession, ClientTimeout, WSMsgType, web

RUNTIME = WS / '.runtime/remote_web'
COOKIE = 'roslander_session'
TTL = 12 * 3600
LOGIN = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>ROSLander · 登录</title>
<style>body{margin:0;min-height:100vh;display:grid;place-items:center;background:#f3f1eb;color:#1f2726;font:16px system-ui}
main{padding:36px;background:#fbfaf6;border:1px solid #dedfd9;border-radius:20px;max-width:360px;margin:20px}
h1{font-size:26px}p{color:#68736e;line-height:1.6}input,button{box-sizing:border-box;width:100%;padding:12px;border-radius:8px;border:1px solid #bbc7c0;font:inherit;margin-top:12px}button{background:#147d70;color:white;cursor:pointer}small{display:block;margin-top:16px;color:#68736e}</style>
<main><h1>ROSLander</h1><p>本机仿真 · 远程状态台</p><form action="/login" method="post">
<label for="password">访问密码</label><input id="password" name="password" type="password" autocomplete="current-password" required maxlength="256">
<button type="submit">进入状态台</button></form><small>使用已设置的固定访问密码；登录会话有效期为 12 小时。</small>ERROR</main></html>'''


def same_origin(request):
    origin = request.headers.get('Origin')
    if not origin:
        return True
    parsed = urlsplit(origin)
    return parsed.scheme == request.scheme and parsed.netloc == request.host


async def serve(app):
    async with ClientSession(timeout=ClientTimeout(total=10)) as client:
        app['client'] = client
        yield


def create_app():
    RUNTIME.mkdir(mode=0o700, parents=True, exist_ok=True)
    RUNTIME.chmod(0o700)
    password_file = RUNTIME / 'password.txt'
    if not password_file.exists():
        raise RuntimeError('未配置固定网页密码，请先在本机创建 .runtime/remote_web/password.txt（权限 600）。')
    password_file.chmod(0o600)
    password = password_file.read_text().strip()
    if not 8 <= len(password) <= 256:
        raise RuntimeError('网页密码应为 8 至 256 字符，请在本机更新凭据文件。')
    sessions = {}
    attempts = defaultdict(deque)

    @web.middleware
    async def protect(request, handler):
        now = time.monotonic()
        # Expired sessions and failed login attempts are bounded in memory.
        for token, expiry in list(sessions.items()):
            if expiry <= now:
                del sessions[token]
        if request.path != '/login' and sessions.get(request.cookies.get(COOKIE), 0) <= now:
            if request.path == '/':
                raise web.HTTPFound('/login')
            return web.json_response({'error': 'authentication_required'}, status=401)
        response = await handler(request)
        response.headers.update({'Cache-Control': 'no-store', 'X-Content-Type-Options': 'nosniff',
                                 'Referrer-Policy': 'same-origin', 'X-Frame-Options': 'SAMEORIGIN'})
        return response

    async def login(request):
        error = ''
        status = 200
        if request.method == 'POST':
            if not same_origin(request):
                raise web.HTTPForbidden(text='Origin mismatch')
            failures = attempts[request.remote]
            now = time.monotonic()
            while failures and failures[0] < now - 60:
                failures.popleft()
            if len(failures) >= 10:
                return web.Response(text='登录尝试过多，请一分钟后重试。', status=429)
            data = await request.post()
            if hmac.compare_digest(str(data.get('password', '')).encode(), password.encode()):
                token = secrets.token_urlsafe(32)
                sessions[token] = now + TTL
                response = web.HTTPSeeOther('/')
                response.set_cookie(COOKIE, token, httponly=True, samesite='Strict',
                                    secure=request.secure,
                                    max_age=TTL, path='/')
                return response
            failures.append(now)
            error, status = '<p role="alert">密码不正确。</p>', 401
        return web.Response(text=LOGIN.replace('ERROR', error), content_type='text/html', status=status)

    async def logout(request):
        if not same_origin(request):
            raise web.HTTPForbidden(text='Origin mismatch')
        sessions.pop(request.cookies.get(COOKIE), None)
        response = web.HTTPSeeOther('/login')
        response.del_cookie(COOKIE)
        return response

    async def credentials(request):
        # Available only after web authentication, never put in URL/storage/logs.
        try:
            password = (WS / '.runtime/gazebo_graphics/vnc-password.txt').read_text().strip()
            return web.json_response({'password': password})
        except OSError:
            return web.json_response({'error': 'Gazebo 图形通道未启动'}, status=503)

    async def websocket(request):
        if not same_origin(request):
            raise web.HTTPForbidden(text='Origin mismatch')
        try:
            upstream = await request.app['client'].ws_connect(
                'http://127.0.0.1:8091/websockify', protocols=('binary',),
                heartbeat=20, max_msg_size=16 * 1024 * 1024)
        except (ClientError, OSError, asyncio.TimeoutError):
            return web.json_response({'error': 'Gazebo 图形通道未连接'}, status=503)
        downstream = web.WebSocketResponse(protocols=('binary',), heartbeat=20, max_msg_size=16 * 1024 * 1024)
        try:
            await downstream.prepare(request)

            async def forward(source, target):
                async for message in source:
                    if sessions.get(request.cookies.get(COOKIE), 0) <= time.monotonic():
                        await downstream.close(code=1008, message=b'Session expired')
                        break
                    if message.type == WSMsgType.BINARY:
                        await target.send_bytes(message.data)
                    elif message.type == WSMsgType.TEXT:
                        await target.send_str(message.data)
                    elif message.type in (WSMsgType.ERROR, WSMsgType.CLOSE, WSMsgType.CLOSED):
                        break
            tasks = [asyncio.create_task(forward(upstream, downstream)),
                     asyncio.create_task(forward(downstream, upstream))]
            try:
                await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            finally:
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
        finally:
            await upstream.close()
            await downstream.close()
        return downstream

    async def proxy(request):
        writable = request.method == 'POST' and request.path in ('/api/navigation/start', '/api/navigation/cancel')
        if request.method not in ('GET', 'HEAD'):
            if not writable:
                return web.json_response({'error': '仅开放仿真预设点导航及当前网页任务取消'}, status=403)
            if not same_origin(request):
                raise web.HTTPForbidden(text='Origin mismatch')
        try:
            body = await request.read() if writable else None
            async with request.app['client'].request(request.method,
                    'http://127.0.0.1:8089' + request.rel_url.raw_path_qs,
                    data=body, headers={'Content-Type': 'application/json'} if writable else None,
                    allow_redirects=False) as response:
                body = await response.read()
                return web.Response(status=response.status, body=body,
                                    headers={'Content-Type': response.headers.get('Content-Type', 'application/octet-stream')})
        except (ClientError, OSError, asyncio.TimeoutError):
            return web.json_response({'error': '仿真数据服务暂不可用'}, status=502)

    app = web.Application(middlewares=[protect], client_max_size=4096)
    app.cleanup_ctx.append(serve)
    app.router.add_get('/login', login)
    app.router.add_post('/login', login)
    app.router.add_post('/logout', logout)
    app.router.add_get('/api/gazebo/session', credentials)
    app.router.add_get('/gazebo/websockify', websocket)
    app.router.add_static('/novnc/', WS / '.deps/remote/noVNC-1.7.0', show_index=False)
    app.router.add_route('*', '/{path:.*}', proxy)
    return app


if __name__ == '__main__':
    web.run_app(create_app(), host='127.0.0.1', port=8088, access_log=None,
                shutdown_timeout=3, print=lambda _: print('统一仿真网页：http://127.0.0.1:8088/', flush=True))
