"""PTY round trip against the exact committed STM32 C protocol core.
Only the feedback and independently granted session are synthetic; no HAL.
"""
import ctypes as C
import json
import os
import pathlib
import pty
import subprocess
import tempfile
import unittest
from motion_bridge_core import MotionBridge
from motion_serial import PosixSerial
from motion_link import Frame, ARM, TRANSLATE, HEARTBEAT, STOP, MOVING, DONE, FAULT
from test_motion_bridge import wall

ROOT=pathlib.Path(__file__).resolve().parent
# In the review zip the baseline copies are beside the tests. In a repository
# checkout resolve the original STM32 files, not duplicated production sources.
SOURCE=ROOT if (ROOT/'motion_link.c').is_file() else ROOT.parents[4]/'STM32F4/App'

WRAPPER=r'''
#include "motion_link.h"
static MLCore core;
static MLFeedback fb;
int begin(void) {
    ml_init(&core); fb=(MLFeedback){0,0,0,0,1,1};
    return ml_grant(&core,23,1,0);
}
void sample(uint32_t t,int32_t x,int32_t y,int32_t yaw,uint8_t stationary) {
    fb=(MLFeedback){x,y,yaw,t,1,stationary};ml_tick(&core,&fb,1,t);
}
int exchange(const uint8_t *in,uint8_t *out,uint32_t t) {
    MLFrame f,r;
    return ml_decode(in,&f)&&ml_receive(&core,&f,&fb,1,t,&r)&&ml_encode(&r,out);
}
'''

class CPeerTest(unittest.TestCase):
    def test_pty_c_peer_end_to_end(self):
        with tempfile.TemporaryDirectory() as td:
            wrapper=pathlib.Path(td)/'wrapper.c';wrapper.write_text(WRAPPER)
            so=pathlib.Path(td)/'peer.so'
            subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror','-pedantic','-shared','-fPIC','-I'+str(SOURCE),str(SOURCE/'motion_link.c'),str(wrapper),'-o',str(so)],check=True)
            peer=C.CDLL(str(so));peer.begin.restype=C.c_int
            peer.sample.argtypes=[C.c_uint32,C.c_int32,C.c_int32,C.c_int32,C.c_uint8];peer.sample.restype=None
            peer.exchange.argtypes=[C.POINTER(C.c_uint8),C.POINTER(C.c_uint8),C.c_uint32];peer.exchange.restype=C.c_int
            self.assertEqual(peer.begin(),1)
            master,slave=pty.openpty();os.set_blocking(master,False)
            transport=PosixSerial(os.ttyname(slave));b=MotionBridge(23,transport,operator_enabled=True)
            def permit(t,seq,gid=0):
                p=dict(schema='car_motion_safety_v1',session=23,seq=seq,stamp_ns=wall(t),frame_id='odom',allow=True,
                       local_grant_confirmed=True,tf_verified=True,goal_id=gid,kind='TRANSLATE' if gid else 'IDLE',
                       start=[0,0,0] if gid else None,goal=[100,0,0] if gid else None,
                       center_bounds_mm=[-10,-10,110,10],turn_clear=False)
                self.assertTrue(b.safety(json.dumps(p),t,wall(t)))
            def exchange(t,kind,expected=None,send=True):
                wire=os.read(master,38);self.assertEqual(Frame.decode(wire).kind,kind)
                buf=(C.c_uint8*38)();self.assertEqual(peer.exchange((C.c_uint8*38).from_buffer_copy(wire),buf,round(t*1000)),1)
                response=bytes(buf)
                if expected is not None:self.assertEqual(Frame.decode(response).args[0],expected)
                if send:os.write(master,response)
            try:
                permit(0,1);b.tick(0);exchange(0,ARM);b.tick(.01)
                permit(.02,2,1)
                g=dict(schema='car_motion_goal_v1',session=23,goal_id=1,stamp_ns=wall(.02),frame_id='odom',kind='TRANSLATE',start=[0,0,0],goal=[100,0,0])
                self.assertTrue(b.goal(json.dumps(g),.02,wall(.02)))
                b.tick(.03);exchange(.03,TRANSLATE,MOVING);b.tick(.04)
                for t in (40,90,140):peer.sample(t,100,0,0,1)
                b.tick(.14);exchange(.14,HEARTBEAT,MOVING);b.tick(.15)
                peer.sample(190,100,0,0,1);permit(.20,3,1)
                peer.sample(240,100,0,0,1);b.tick(.25);exchange(.25,HEARTBEAT,DONE);b.tick(.26)
                self.assertEqual(b.status(.26)['completed_goal_id'],1);self.assertEqual(b.status(.26)['pose_mm_mdeg'],[100,0,0])
                b.shutdown();exchange(.26,STOP,FAULT,send=False)
            finally:b.shutdown();os.close(slave);os.close(master)

if __name__=='__main__':unittest.main()
