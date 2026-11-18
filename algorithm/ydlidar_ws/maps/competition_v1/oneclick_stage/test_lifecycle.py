#!/usr/bin/env python3
"""Process-level tests; uses an isolated fake supervisor, never ROS or hardware."""
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
import warnings
warnings.filterwarnings('ignore', message='subprocess .* is still running', category=ResourceWarning)
from unittest.mock import patch

import ai_manager as mgr


FAKE = '''#!/usr/bin/env python3
import json,os,signal,time
from pathlib import Path
p=Path(os.environ['ONECLICK_LOG_DIR'])
token=os.environ['ONECLICK_RUN_ID']
(p/'children.json').write_text('[]',encoding='utf-8')
print('[RUNNING] fake ready',flush=True)
def end(sig,frame):
  (p/'run_result.json').write_text(json.dumps({'result':0,'run_id':token}),encoding='utf-8')
  raise SystemExit(0)
signal.signal(signal.SIGTERM,end)
while True: time.sleep(.1)
'''


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root = self.base/'project'
        self.stage = self.root/'oneclick_stage'
        self.stage.mkdir(parents=True)
        self.script = self.stage/'fake.py'
        self.script.write_text(FAKE, encoding='utf-8')
        self.launcher = self.stage/'start_oneclick.sh'
        self.launcher.write_text('#!/usr/bin/env bash\nexec python3 -u "$(dirname "$0")/fake.py"\n', encoding='utf-8')
        self.launcher.chmod(0o755)
        self.state_home = self.base/'state'
        self.patches = [
            patch.object(mgr,'ROOT',self.root),
            patch.object(mgr,'HERE',self.stage),
            patch.object(mgr,'STATE_HOME',self.state_home),
            patch.object(mgr,'STATE_FILE',self.state_home/'state.json'),
            patch.object(mgr,'MANAGE_LOCK',self.state_home/'lifecycle.lock'),
            patch.object(mgr,'TIMEOUT_START',5),
            patch.object(mgr,'TIMEOUT_STOP',2),
        ]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)
        self.created = []
        self.addCleanup(self.cleanup_processes)

    def cleanup_processes(self):
        try:
            obj=mgr.load_state()
            if obj:
                mgr.cmd_stop()
        except Exception:
            pass

    def test_first_start_status_repeat_stop(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(mgr._start_under_lock(skip_doctor=True),0)
            current = mgr.load_state()
            self.assertEqual(mgr.current_status(current)[0],'RUNNING')
            pid = current['supervisor_pid']
            self.assertEqual(mgr._start_under_lock(skip_doctor=True),0)
            self.assertEqual(mgr.load_state()['supervisor_pid'],pid)
            self.assertEqual(mgr.cmd_stop(),0)
            self.assertEqual(mgr.current_status(mgr.load_state())[0],'STOPPED')
            self.assertTrue(Path(current['log_dir'],'supervisor.log').exists())

    def test_wrong_pid_identity_does_not_signal(self):
        # Unrelated subprocess, intentionally without the instance marker.
        proc=subprocess.Popen(['sleep','20'],start_new_session=True)
        try:
            record={'project':str(self.root),'run_id':'fake', 'state':'RUNNING',
                    'supervisor_pid':proc.pid,'supervisor_start_ticks':mgr.proc_start(proc.pid),
                    'log_dir':str(self.base)}
            mgr.save_state(record)
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(mgr.cmd_stop(),3)
            self.assertIsNone(proc.poll())
        finally:
            proc.terminate()
            proc.wait(timeout=2)

    def test_missing_and_corrupted_state(self):
        self.assertEqual(mgr.current_status(None)[0],'STOPPED')
        mgr.secure_dir(self.state_home)
        mgr.STATE_FILE.write_text('{bad-json')
        with self.assertRaisesRegex(RuntimeError,'状态文件损坏'):
            mgr.load_state()

    def test_orphan_recovery_stops_owned_only(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(mgr._start_under_lock(skip_doctor=True),0)
            obj=mgr.load_state()
            pid=obj['supervisor_pid']
            import signal
            os.kill(pid,signal.SIGKILL)
            for _ in range(40):
                if mgr.is_zombie(pid) or mgr.proc_start(pid) is None:
                    break
                time.sleep(.05)
            self.assertEqual(mgr.cmd_stop(),0)
            self.assertEqual(mgr.current_status(mgr.load_state())[0],'STOPPED')

    def test_offline_diagnosis_uses_no_ros(self):
        with patch.object(mgr.pipeline,'check_inputs',return_value=({'start_zone':'upper_right'},'/dev/none','laser_frame','')):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(mgr.diagnose(live=False),0)
        with patch.object(mgr.pipeline,'check_inputs',side_effect=RuntimeError('missing')):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertNotEqual(mgr.diagnose(live=False),0)

    def test_start_failure_exits_and_records(self):
        self.script.write_text('print("[ERROR] fake failure",flush=True)\nraise SystemExit(6)\n')
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(mgr._start_under_lock(skip_doctor=True),2)
        self.assertEqual(mgr.load_state()['state'],'FAILED')
        self.assertIn('fake failure',Path(mgr.load_state()['log_dir'],'supervisor.log').read_text())

    def test_module_log_is_rotated_and_supervised(self):
        from logging.handlers import RotatingFileHandler
        folder = self.base/'module_logs'
        folder.mkdir()
        sv = mgr.pipeline.Supervisor(folder)
        def small_handler(path, maxBytes, backupCount, encoding):
            return RotatingFileHandler(path, maxBytes=1024, backupCount=2, encoding=encoding)
        with patch.object(mgr.pipeline, 'RotatingFileHandler', side_effect=small_handler):
            with contextlib.redirect_stdout(io.StringIO()):
                try:
                    sv.launch('fake', ['python3','-u','-c',
                                     'import time; [print("logline"*50,flush=True) for _ in range(200)]; time.sleep(3)'])
                    for _ in range(20):
                        if (folder/'fake.log.1').exists():
                            break
                        time.sleep(.05)
                    self.assertTrue((folder/'fake.log.1').is_file())
                finally:
                    sv.stop()


if __name__=='__main__':
    unittest.main(verbosity=2)
