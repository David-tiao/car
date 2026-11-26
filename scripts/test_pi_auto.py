import os, pathlib, shutil, subprocess, tempfile, unittest

HERE = pathlib.Path(__file__).resolve().parent

class AutoTest(unittest.TestCase):
    def test_lifecycle(self):
        with tempfile.TemporaryDirectory() as td:
            root = pathlib.Path(td)
            pc, pi, repo, binp = [root / s for s in ('pc', 'pi', 'repo', 'bin')]
            for d in (pc, pi, repo, binp): d.mkdir()
            env = dict(os.environ, HOME=str(pc), MOCK_PI=str(pi), PATH=str(binp)+':'+os.environ['PATH'])
            def cmd(*args, check=True, **kw):
                return subprocess.run(args, env=env, text=True, capture_output=True, check=check, timeout=40, **kw)
            def git(*args): return cmd('git','-C',str(repo),*args).stdout.strip()
            def mock(name, body):
                p = binp/name; p.write_text('#!/bin/bash\nset -eu\n'+body); p.chmod(0o755)
            mock('ssh', '''while [[ "$1" == -o ]]; do shift 2; done
shift
[[ ! -e "$MOCK_PI/offline" ]] || exit 255
HOME="$MOCK_PI" bash -c "$*"
''')
            mock('scp', '''while [[ "$1" == -o ]]; do shift 2; done
first=$1; second=$2; dest=${3#*:}
cp "$first" "$second" "$MOCK_PI/$dest"
if [[ -e "$MOCK_PI/corrupt" ]]; then echo corrupt >> "$MOCK_PI/$dest/source.tar"; fi
printf x >> "$MOCK_PI/transfers"
''')
            mock('systemctl', '''case "$*" in *is-active*) exit 1;; *) exit 0;; esac''')
            git('init','-b','test'); git('config','user.name','Test'); git('config','user.email','test@example.invalid')
            (repo/'scripts').mkdir(); (repo/'algorithm/pi/tests').mkdir(parents=True)
            shutil.copy(HERE/'pi_auto.sh',repo/'scripts/pi_auto.sh'); shutil.copy(HERE.parent/'sync_pi.sh',repo/'sync_pi.sh')
            sample=repo/'algorithm/pi/tests/demo.py'; sample.write_text('print(1)\n')
            def commit(): git('add','.'); git('commit','-m','test'); return git('rev-parse','HEAD')
            commit(); git('remote','add','origin',str(repo))
            a=pc/'.local/share/car-pi-auto/pi_auto.sh'
            def auto(*args,check=True): return cmd('bash',str(a if a.exists() else HERE/'pi_auto.sh'),*args,check=check)
            result=auto('setup','--repo',str(repo),'--host','pi@fake','--branch','test'); self.assertIn('Timer enabled',result.stdout)
            current=pi/'workspace/12_artifacts/car_source_sync/current'; first=os.readlink(current); self.assertEqual((current/'algorithm/pi/tests/demo.py').read_text(),'print(1)\n')
            auto('run'); self.assertEqual((pi/'transfers').read_text(),'x')
            sample.write_text('print(2)\n'); commit(); auto('run'); second=os.readlink(current); self.assertNotEqual(first,second); self.assertTrue((current.parent/first).is_dir())
            auto('auto-stop'); auto('rollback',first); self.assertEqual(os.readlink(current),first)
            auto('run'); self.assertEqual(os.readlink(current),second)
            (pi/'offline').touch(); self.assertIn('[SKIP]',auto('run').stdout); (pi/'offline').unlink()
            sample.write_text('def invalid(\n'); commit(); self.assertNotEqual(auto('run',check=False).returncode,0); self.assertEqual(os.readlink(current),second)
            sample.write_text('print(3)\n'); commit(); (pi/'corrupt').touch(); self.assertNotEqual(auto('run',check=False).returncode,0); self.assertEqual(os.readlink(current),second); (pi/'corrupt').unlink()
            auto('run'); self.assertEqual((current/'algorithm/pi/tests/demo.py').read_text(),'print(3)\n')
            print('PASS: setup, first sync, repeat reuse, update, old retained, rollback, offline skip, syntax failure, corruption failure, retry')

if __name__=='__main__': unittest.main()
