#!/usr/bin/env python3
"""Live local acceptance: credentials never included in report/output."""
import asyncio
import json
from pathlib import Path
import sys

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / '.deps/remote/python'))
from aiohttp import ClientSession, CookieJar


async def main():
    report = {'scope': '本地仿真网页 HTTP + Gazebo WebSocket', 'checks': {}}
    checks = report['checks']
    base = 'http://127.0.0.1:8088'
    async with ClientSession(cookie_jar=CookieJar(unsafe=True)) as client:
        for path in ('/api/state', '/api/map.png', '/api/camera.jpg', '/novnc/core/rfb.js', '/gazebo/websockify', '/api/gazebo/session', '/app.js'):
            async with client.get(base + path, allow_redirects=False) as response:
                assert response.status == 401, (path, response.status)
        checks['unauthenticated_http_static_and_websocket_denied'] = True
        async with client.post(base + '/login', data={'password': 'incorrect'}, allow_redirects=False) as response:
            assert response.status == 401
        checks['bad_password_denied'] = True
        password = (WS / '.runtime/remote_web/password.txt').read_text().strip()
        async with client.post(base + '/login', data={'password': password}, headers={'Origin': base}, allow_redirects=False) as response:
            assert response.status == 303
            cookie = response.headers.get('Set-Cookie', '')
            assert 'HttpOnly' in cookie and 'SameSite=Strict' in cookie
        checks['login_cookie_flags'] = True
        # ROS subscriptions can discover data shortly after the HTTP socket opens.
        for _ in range(50):
            async with client.get(base + '/api/state') as response:
                ready = await response.json()
            if ready['simulation'].get('clock_seconds') is not None and ready['camera']['online']:
                break
            await asyncio.sleep(.2)
        for path in ('/', '/app.js', '/gazebo.js', '/styles.css', '/novnc/core/rfb.js', '/api/map.png', '/api/camera.jpg'):
            async with client.get(base + path) as response:
                assert response.status == 200, (path, response.status)
                assert response.headers.get('Cache-Control') == 'no-store'
                await response.read()
        checks['protected_assets_map_camera_same_origin'] = True
        async with client.get(base + '/api/state') as response:
            state = await response.json()
            assert state['mode'] == 'simulation'
            report['simulation'] = state['simulation']
        for path, body, expected in (('/api/navigation/start', {'place_id': 'detect', 'x': 99}, 400),
                                     ('/api/navigation/start', {'place_id': 'unknown'}, 404),
                                     ('/api/navigation/cancel', {'goal_id': 'external'}, 400),
                                     ('/api/cmd_vel', {}, 403)):
            async with client.post(base + path, json=body) as response:
                assert response.status == expected, (path, response.status)
        async with client.post(base + '/api/navigation/start', json={'place_id': 'detect'}, headers={'Origin': 'https://untrusted.invalid'}) as response:
            assert response.status == 403
        checks['only_fixed_presets_and_owned_cancel_allowed'] = True
        async with client.get(base + '/api/camera-stream') as response:
            assert response.status == 404
        checks['real_camera_fallback_denied'] = True
        async with client.get(base + '/api/gazebo/session') as response:
            assert response.headers['Cache-Control'] == 'no-store'
            credentials = await response.json()
            assert credentials['password'] == (WS / '.runtime/gazebo_graphics/vnc-password.txt').read_text().strip()
        checks['vnc_credentials_only_after_web_authentication'] = True
        async with client.ws_connect(base + '/gazebo/websockify', protocols=('binary',), origin=base) as websocket:
            message = await websocket.receive(timeout=5)
            assert message.data == b'RFB 003.008\n', repr(message.data)
            await websocket.send_bytes(message.data)
            message = await websocket.receive(timeout=5)
            assert 2 in message.data[1:]  # VNC password security type from real Xvnc.
        checks['binary_websocket_to_real_xvnc'] = True
        try:
            async with client.ws_connect(base + '/gazebo/websockify', origin='https://untrusted.invalid'):
                raise AssertionError('cross-origin websocket allowed')
        except Exception as error:
            assert getattr(error, 'status', None) == 403
        checks['cross_origin_websocket_denied'] = True
        forwarded = {'Host': '127.0.0.1:18088', 'Origin': 'http://127.0.0.1:18088'}
        async with client.post(base + '/login', data={'password': password}, headers=forwarded, allow_redirects=False) as response:
            assert response.status == 303
        async with client.ws_connect(base + '/gazebo/websockify', protocols=('binary',), headers=forwarded) as websocket:
            assert (await websocket.receive(timeout=5)).data == b'RFB 003.008\n'
        checks['ssh_forwarded_host_and_port_login_and_websocket'] = True
        async with client.post(base + '/logout', headers={'Origin': base}, allow_redirects=False) as response:
            assert response.status == 303
        async with client.get(base + '/api/state') as response:
            assert response.status == 401
        checks['logout_revokes_session'] = True
    output = WS / '.runtime/validation'
    output.mkdir(parents=True, exist_ok=True)
    (output / 'http_websocket.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(checks, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    asyncio.run(main())
