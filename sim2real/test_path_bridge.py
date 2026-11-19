#!/usr/bin/env python3
import argparse,json
from urllib.request import urlopen
from urllib.error import URLError,HTTPError

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathBridge(Node):
    def __init__(self,server):
        super().__init__('car_test_path_bridge')
        self.url=server.rstrip('/')+'/api/test/reference_path'
        self.pub=self.create_publisher(
            Path,'/car/test/reference_path',10
        )
        self.timer=self.create_timer(.5,self.poll)
        self.get_logger().info(
            'TEST browser route -> ROS2 Path -> mock MCU'
        )

    def poll(self):
        points=[]
        try:
            with urlopen(self.url,timeout=.35) as response:
                obj=json.load(response)
            if obj.get('source_type')=='TEST' and obj.get('online'):
                points=obj['points']
        except (URLError,HTTPError,ValueError,TypeError,KeyError,OSError):
            pass

        msg=Path()
        msg.header.frame_id='odom'
        msg.header.stamp=self.get_clock().now().to_msg()

        for x,y in points:
            p=PoseStamped()
            p.header=msg.header
            p.pose.position.x=float(x)
            p.pose.position.y=float(y)
            p.pose.orientation.w=1.0
            msg.poses.append(p)

        self.pub.publish(msg)

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--server',default='http://127.0.0.1:8080')
    args=p.parse_args()

    if not args.server.startswith('http://127.0.0.1:'):
        p.error('TEST HTTP server must be localhost')

    rclpy.init()
    node=PathBridge(args.server)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()
