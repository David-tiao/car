"""Compile actual candidate sources against a deliberately minimal fake HAL; no hardware."""
from pathlib import Path
import sys
TEST_DIR = Path(__file__).resolve().parent
STM = TEST_DIR.parents[1]
REPO = STM.parent
FUSION = REPO / "algorithm/car_ws/maps/competition_v1/fusion_stage"
sys.path.insert(0, str(FUSION))
import pathlib, subprocess, tempfile, unittest
ROOT=pathlib.Path(__file__).resolve().parent
class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.bin={}
        for enabled in (0,1):
            out=pathlib.Path(cls.tmp.name)/f'test{enabled}'
            subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror','-pedantic','-O2',f'-DPI_MOTION_ENABLE={enabled}','-I'+str(ROOT/'stubs'),'-I'+str(STM/'App'),'-I'+str(STM/'Hardware'),str(STM/'App/motion_link.c'),str(STM/'App/pi_motion_adapter.c'),str(STM/'Hardware/pi_uart2_port.c'),str(ROOT/'test_pi_adapter.c'),'-lm','-o',str(out)],check=True)
            cls.bin[enabled]=out
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def run_case(self,name,enabled=1):subprocess.run([str(self.bin[enabled]),name],check=True,timeout=5)
    def test_default_disabled(self):self.run_case('default',0)
    def test_exclusive_owner(self):self.run_case('owner')
    def test_local_grant(self):self.run_case('grant')
    def test_stale_grant(self):self.run_case('fresh')
    def test_turn_binding(self):self.run_case('turn')
    def test_position_once_and_measured_stop(self):self.run_case('done')
    def test_main_loop_stall(self):self.run_case('stall')
    def test_heartbeat_timeout(self):self.run_case('watchdog')
    def test_local_gate_loss(self):self.run_case('local')
    def test_imu_fault(self):self.run_case('imu')
    def test_sample_stale(self):self.run_case('sample')
    def test_rx_overflow(self):self.run_case('overflow')
    def test_uart_rx_fault(self):self.run_case('rxerror')
    def test_uart_tx_fault(self):self.run_case('txerror')
    def test_stop(self):self.run_case('stop')
    def test_bad_frame(self):self.run_case('badframe')
if __name__=='__main__':unittest.main()
