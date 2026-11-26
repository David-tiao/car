import os
import pathlib
import subprocess
import tempfile
import unittest

DEMO = pathlib.Path(__file__).with_name('csi_camera_demo.sh')

class DemoTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = pathlib.Path(self.tmp.name)
        self.env = dict(os.environ, PATH=str(root) + ':' + os.environ['PATH'],
                        TMPDIR=str(root), CAMS='0 : imx219 [3280x2464]', TRACE=str(root/'trace'))
        hello = root/'rpicam-hello'
        hello.write_text('#!/bin/bash\nif [[ "$1" == --list-cameras ]]; then printf "%s\\n" "$CAMS"; else printf "%s\\n" "$*" > "$TRACE"; exit "${MOCK_RC:-0}"; fi\n')
        still = root/'rpicam-still'
        still.write_text('#!/bin/bash\nwhile (( $# )); do if [[ "$1" == --output ]]; then shift; if [[ "${BAD_JPEG:-0}" == 1 ]]; then echo bad > "$1"; else printf "\\377\\330mock\\377\\331" > "$1"; fi; fi; shift; done\n')
        for p in (hello, still): p.chmod(0o755)
    def call(self, *args):
        return subprocess.run(['bash', str(DEMO), *args], env=self.env, capture_output=True, text=True, timeout=5)
    def test_list(self):
        self.assertIn('[PASS]', self.call('list').stdout)
    def test_missing(self):
        self.env['CAMS'] = 'No cameras available!'
        self.assertEqual(self.call('list').returncode, 1)
    def test_two(self):
        self.env['CAMS'] += '\n1 : ov5647 [2592x1944]'
        self.assertEqual(self.call('preview').returncode, 1)
        self.assertEqual(self.call('preview','1','2').returncode, 0)
        self.assertEqual(pathlib.Path(self.env['TRACE']).read_text().strip(), '--camera 1 --timeout 2000')
    def test_capture(self):
        r = self.call('capture')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('JPEG boundary check', r.stdout)
    def test_bad_jpeg(self):
        self.env['BAD_JPEG'] = '1'
        self.assertEqual(self.call('capture').returncode, 1)
    def test_invalid(self):
        self.assertEqual(self.call('preview','0','0').returncode, 2)
        self.assertEqual(self.call('preview','9').returncode, 1)
    def test_failure_propagates(self):
        self.env['MOCK_RC'] = '7'
        self.assertEqual(self.call('preview','0','1').returncode, 7)

if __name__ == '__main__': unittest.main()
