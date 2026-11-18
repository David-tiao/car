#!/usr/bin/env python3
"""ROS2 nav_msgs/Odometry -> localhost-only HTTP telemetry interface.

Read-only ROS subscriber; never publishes /cmd_vel or motor commands.
The pose in /odom is odometry-frame, NOT necessarily map frame. Align in the UI.
"""
import argparse
import json
import math
import sys
import time
from urllib.request import Request, urlopen
from urllib.error import URLError


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/odom')
    ap.add_argument('--server', default='http://127.0.0.1:8080')
    ap.add_argument('--hz', type=float, default=15)
    args = ap.parse_args()
    if not args.server.startswith(('http://127.0.0.1:', 'http://localhost:', 'http://[::1]:')):
        ap.error('Use localhost only; server rejects remote telemetry posting')
    if not 1 <= args.hz <= 60:
        ap.error('--hz must be 1..60')
    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
    except ImportError as exc:
        sys.exit('ROS2 Python libraries not available. Run on the ROS2 Humble Ubuntu machine, after sourcing setup.bash.\n' + str(exc))

    rclpy.init()
    class Bridge(Node):
        def __init__(self):
            super().__init__('competition_web_odom_bridge')
            self.last_sent=0.0
            self.success=0
            self.warn_at=0
            self.sub=self.create_subscription(Odometry,args.topic,self.handle,10)
            self.get_logger().info(f'Listening {args.topic}; forwarding actual odometry to {args.server}/api/telemetry at <= {args.hz:.0f} Hz')
        def handle(self,msg):
            now=time.monotonic()
            if now-self.last_sent<1.0/args.hz:
                return
            self.last_sent=now
            p=msg.pose.pose.position
            v=msg.twist.twist
            payload={'x':p.x, 'y':p.y, 'yaw':yaw_from_quat(msg.pose.pose.orientation),
                     'vx':v.linear.x, 'vy':v.linear.y, 'wz':v.angular.z,
                     'source':'ros2_odom', 'frame':msg.header.frame_id or 'odom'}
            try:
                body=json.dumps(payload).encode('utf-8')
                request=Request(args.server.rstrip('/')+'/api/telemetry',data=body,
                                headers={'Content-Type':'application/json'},method='POST')
                with urlopen(request,timeout=.45) as res:
                    if res.status!=200: raise RuntimeError(f'HTTP {res.status}')
                self.success+=1
                if self.success%150==0:self.get_logger().info(f'{self.success} odom samples sent')
            except (URLError,OSError,RuntimeError,ValueError) as exc:
                if now-self.warn_at>5:
                    self.warn_at=now
                    self.get_logger().warning('HTTP telemetry send failed: '+str(exc))
    node=Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__': main()
