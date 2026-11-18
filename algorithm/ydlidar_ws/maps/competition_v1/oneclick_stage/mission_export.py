#!/usr/bin/env python3
"""Write a single frozen rule-ordered mission preview (never send motion commands)."""
import argparse
import json
from pathlib import Path

ZONES = {
    'upper_right': ('C3', 'C2', 'B3', 'B1', 'A2', 'B3', 'B1', 'A2', 'C3'),
    'lower_right': ('C1', 'C2', 'B3', 'B1', 'A2', 'B3', 'B1', 'A2', 'C1'),
}
STAGES = ('出发', '二维码区(示意)', '原料区(示意)', '粗加工区(示意)',
          '暂存区(示意)', '原料区(第二批)', '粗加工区(第二批)',
          '暂存区(第二批码垛)', '返回原启停区')


def validate_route_message(status, expected_nodes):
    if (not isinstance(status, dict) or status.get('status') != 'LOCKED_READY' or
            status.get('source') != 'LIDAR_CONFIRMED' or
            status.get('frozen') is not True or status.get('conflict') is not False):
        return None
    legs = status.get('mission_routes')
    if status.get('waypoints') != list(expected_nodes) or not isinstance(legs, list):
        return None
    if len(legs) != len(expected_nodes) - 1:
        return None
    if not isinstance(status.get('blocked'), list):
        return None
    for i, leg in enumerate(legs):
        if (not isinstance(leg, list) or len(leg) == 0 or
                leg[0] != expected_nodes[i] or leg[-1] != expected_nodes[i + 1] or
                not all(isinstance(n, str) for n in leg)):
            return None
    return legs


def flatten_legs(legs):
    nodes = []
    for leg in legs:
        nodes.extend(leg if not nodes else leg[1:])
    return nodes


def run():
    parser = argparse.ArgumentParser()
    parser.add_argument('--zone', choices=sorted(ZONES), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped
    import importlib.util
    topo_path = Path(__file__).resolve().parent.parent / 'topology_stage6' / 'road_topology.py'
    spec = importlib.util.spec_from_file_location('competition_topology', topo_path)
    topo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(topo)
    nodes = ZONES[args.zone]
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

    class Exporter(Node):
        def __init__(self):
            super().__init__('competition_oneclick_mission_preview')
            self.saved = False
            self.invalidated = False
            self.pub_path = self.create_publisher(RosPath, '/competition/mission_road_path', qos)
            self.sub_status = self.create_subscription(String, '/competition/road_status', self.on_status, qos)

        def on_status(self, msg):
            try:
                status = json.loads(msg.data)
            except (ValueError, TypeError):
                self.get_logger().error('路线状态不是合法 JSON')
                return
            if self.saved:
                if (isinstance(status, dict) and status.get('status') == 'CONFLICT_STOP'
                        and not self.invalidated):
                    payload = json.loads(args.output.read_text(encoding='utf-8'))
                    payload['safety_state'] = 'CONFLICT_STOP'
                    temp = args.output.with_suffix('.tmp')
                    temp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
                    temp.replace(args.output)
                    self.invalidated = True
                    empty = RosPath()
                    empty.header.frame_id = 'map'
                    empty.header.stamp = self.get_clock().now().to_msg()
                    self.pub_path.publish(empty)
                    self.get_logger().error('新障碍物冲突，已清空路径预览并将路线快照标记为不可执行！')
                return
            legs = validate_route_message(status, nodes)
            if legs is None:
                if isinstance(status, dict) and status.get('status') in ('NO_ROUTE', 'CONFLICT_STOP'):
                    self.get_logger().error('无法生成可执行路线：%s' % status['status'])
                return
            all_nodes = flatten_legs(legs)
            path = RosPath()
            path.header.frame_id = 'map'
            path.header.stamp = self.get_clock().now().to_msg()
            for name in all_nodes:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = topo.NODES[name]
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            payload = {
                'purpose': 'ROAD_TOPOLOGY_PREVIEW_ONLY',
                'NOT_FOR_NAV2_FOLLOWPATH': True,
                'safety_state': 'LOCKED_READY',
                'start_zone': args.zone,
                'stages': list(STAGES),
                'waypoints': list(nodes),
                'blocked_roads': status['blocked'],
                'mission_routes': legs,
                'frame_id': 'map',
            }
            # Atomic snapshot; only in the current run's log directory.
            args.output.parent.mkdir(parents=True, exist_ok=True)
            part = args.output.with_suffix('.tmp')
            part.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
            part.replace(args.output)
            self.pub_path.publish(path)
            self.saved = True
            self.get_logger().warning('本场路线已冻结并保存：%s（仅为道路级预览）' % args.output)

    rclpy.init()
    node = Exporter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
