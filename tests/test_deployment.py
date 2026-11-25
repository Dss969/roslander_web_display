"""Deployment contracts: fixed login, local-only origins and verified archives."""
import io
import hashlib
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / 'scripts'))
import gazebo_graphics as graphics
import setup_dependencies as setup
import remote_gateway as gateway
from aiohttp.test_utils import TestClient, TestServer


class GatewayTests(unittest.IsolatedAsyncioTestCase):
    async def test_local_fixed_login_logout_and_cross_origin_rejection(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(gateway, 'RUNTIME', Path(temp)):
            (Path(temp) / 'password.txt').write_text('local-test-password\n')
            async with TestClient(TestServer(gateway.create_app())) as client:
                self.assertEqual((await client.get('/api/state')).status, 401)
                response = await client.post('/login', data={'password': 'local-test-password'},
                    headers={'Origin': 'https://external.invalid', 'Host': 'external.invalid',
                             'X-Forwarded-Proto': 'https'}, allow_redirects=False)
                self.assertEqual(response.status, 403)
                response = await client.post('/login', data={'password': 'local-test-password'},
                    headers={'Origin': 'http://127.0.0.1:18088', 'Host': '127.0.0.1:18088'}, allow_redirects=False)
                self.assertEqual(response.status, 303)
                cookie = response.cookies[gateway.COOKIE]
                self.assertTrue(cookie['httponly'])
                self.assertEqual(cookie['samesite'], 'Strict')
                self.assertFalse(cookie['secure'])
                self.assertEqual((await client.post('/api/navigation/start', json={'place_id': 'detect'},
                    headers={'Origin': 'https://external.invalid'})).status, 403)
                self.assertEqual((await client.post('/logout', allow_redirects=False)).status, 303)
                self.assertEqual((await client.get('/api/state')).status, 401)

    async def test_missing_fixed_password_is_not_randomly_recreated(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(gateway, 'RUNTIME', Path(temp)):
            with self.assertRaises(RuntimeError):
                gateway.create_app()
            self.assertFalse((Path(temp) / 'password.txt').exists())


class DependencyTests(unittest.TestCase):
    def test_mirror_fallback_keeps_same_checksum_requirement(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(setup, 'DEPS', Path(temp)):
            payload = b'verified-package'
            item = dict(file='package.deb', url='https://vendor.invalid/package',
                        mirrors=['https://mirror.invalid/package'], sha256=hashlib.sha256(payload).hexdigest())
            with patch.object(setup.urllib.request, 'urlopen', side_effect=[OSError('unreachable'), io.BytesIO(payload)]) as request:
                result = setup.fetch(item)
            self.assertEqual(result.read_bytes(), payload)
            self.assertEqual(request.call_count, 2)

    def test_bad_download_does_not_replace_existing_cache(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(setup, 'DEPS', Path(temp)):
            cache = Path(temp) / 'downloads'
            cache.mkdir()
            file = cache / 'package.deb'
            file.write_bytes(b'existing-cache')
            with patch.object(setup.urllib.request, 'urlopen', return_value=io.BytesIO(b'bad-download')):
                with self.assertRaises(RuntimeError):
                    setup.fetch(dict(file=file.name, url='https://vendor.invalid/package.deb', sha256='0'*64))
            self.assertEqual(file.read_bytes(), b'existing-cache')
            self.assertFalse((cache / 'package.deb.part').exists())

    def test_tar_cannot_write_outside_dependency_directory(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(setup, 'DEPS', Path(temp)/'deps'):
            archive = Path(temp) / 'bad.tar'
            with tarfile.open(archive, 'w') as bundle:
                member = tarfile.TarInfo('../escaped')
                member.size = 3
                bundle.addfile(member, io.BytesIO(b'bad'))
            with self.assertRaises(RuntimeError):
                setup.extract_tar(archive)
            self.assertFalse((Path(temp)/'escaped').exists())


class GraphicsTests(unittest.TestCase):
    def test_render_readiness_uses_current_session_frames(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(graphics, 'RUNTIME', Path(temp)), \
                patch.object(graphics, 'starttime', return_value='alive'):
            old = b'Unable to create glx visual\n'
            (Path(temp)/'gzclient.log').write_bytes(old + b'Connected to gazebo master\nTotal - 16.84 Mpixels/sec- 30.00 fps\n')
            graphics.wait_for_gui_render(123, len(old))
            self.assertEqual(json.loads((Path(temp)/'render_ready.json').read_text())['rendered_fps'], 30.)

    def test_render_readiness_rejects_failed_gl_context(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(graphics, 'RUNTIME', Path(temp)), \
                patch.object(graphics, 'starttime', return_value='alive'):
            (Path(temp)/'gzclient.log').write_text('Connected to gazebo master\nUnable to create glx visual\n')
            with self.assertRaises(RuntimeError):
                graphics.wait_for_gui_render(123, 0)

    def test_render_readiness_rejects_dead_client(self):
        with patch.object(graphics, 'starttime', return_value=None):
            with self.assertRaises(RuntimeError):
                graphics.wait_for_gui_render(123, 0)

    def test_machine_override_and_invalid_frame_rate(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(graphics, 'WS', Path(temp)):
            config = Path(temp) / 'config'
            config.mkdir()
            (config/'graphics.json').write_text(json.dumps(dict(render_device='egl0', require_hardware=True,
                                                              geometry='1280x720', fps=30)))
            local = config/'graphics.local.json'
            local.write_text(json.dumps(dict(render_device='egl1', fps=15)))
            self.assertEqual(graphics.graphics_config()['render_device'], 'egl1')
            self.assertEqual(graphics.graphics_config()['fps'], 15)
            local.write_text(json.dumps(dict(fps=True)))
            with self.assertRaises(RuntimeError):
                graphics.graphics_config()


if __name__ == '__main__':
    unittest.main()
