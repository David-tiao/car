#!/usr/bin/env python3
"""只读比赛路线预览：离线 --demo，或 --ros 订阅真实锁路状态。

没有 /cmd_vel、serial.write、STM32 运动权限。默认不写磁盘。
"""
import argparse
import json
import math
import os
from pathlib import Path

from rule_mission_core import PlanError, NODES, build_mission, verify_real_lock


def save_json(path, payload):
    if path is None:
        return
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp.' + str(os.getpid()))
    try:
        tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        os.replace(tmp, path)
    finally:
        if tmp.exists():
            tmp.unlink()


def offline(args):
    stations = json.loads(Path(args.stations_file).read_text(encoding='utf-8')) if args.stations_file else None
    if args.demo:
        plan = build_mission(args.start_zone, args.block, stations, 'DEMO_ONLY')
    else:
        raw = json.loads(Path(args.road_status_file).read_text(encoding='utf-8'))
        blocked = verify_real_lock(raw)
        plan = build_mission(args.start_zone, blocked, stations, 'LIDAR_CONFIRMED')
    print(json.dumps(plan, ensure_ascii=False, indent=2))
    save_json(args.output, plan)
    return 0 if plan['status'] != 'NO_ROUTE' else 3


def live_ros(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

    class RuleMissionPreview(Node):
        def __init__(self):
            super().__init__('competition_rule_mission_preview')
            self.blocked = None
            self.stations = None
            self.stations_frozen = None
            self.stopped = False
            self.last_text = None
            self.pub = self.create_publisher(String, '/competition/rule_mission_status', qos)
            self.path_pub = self.create_publisher(RosPath, '/competition/rule_mission_path', qos)
            self.create_subscription(String, '/competition/road_status', self.on_lock, qos)
            self.create_subscription(String, '/competition/station_poses', self.on_stations, qos)
            self.publish_state({'status': 'WAIT_FOR_LOCK', 'commands_enabled': False}, clear=True)
            self.get_logger().warning('规划预览 ONLY，不输出速度、停车或 STM32 指令')

        def publish_state(self, payload, clear=False):
            text = json.dumps(payload, ensure_ascii=False, sort_keys=True)
            if text != self.last_text:
                self.pub.publish(String(data=text))
                self.last_text = text
                save_json(args.output, payload)
            if clear:
                path = RosPath()
                path.header.frame_id = 'map'
                path.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(path)

        def stop(self, code, detail):
            self.stopped = True
            self.publish_state({'status': code, 'reason': detail,
                                'commands_enabled': False}, clear=True)
            self.get_logger().error('%s: %s', code, detail)

        def on_lock(self, msg):
            if self.stopped:
                return
            try:
                status = json.loads(msg.data)
                if not isinstance(status, dict):
                    raise PlanError('road_status JSON 不是对象')
                # 锁定前只等待；锁定后发生失效或冲突则永久停止本次预览。
                if self.blocked is None and status.get('status') == 'WAIT_FOR_LOCK':
                    return
                if status.get('status') in ('NO_ROUTE', 'CONFLICT_STOP'):
                    self.stop(status['status'], '锁路结果不可规划或存在冲突')
                    return
                blocked = verify_real_lock(status)
                if self.blocked is None:
                    self.blocked = frozenset(blocked)
                    self.get_logger().info('真实道路锁定已接收: %s', sorted(self.blocked))
                elif self.blocked != frozenset(blocked):
                    self.stop('LOCK_CHANGED_STOP', '道路封锁状态在冻结后改变')
                    return
                self.refresh()
            except (ValueError, TypeError, KeyError) as exc:
                if self.blocked is None:
                    self.publish_state({'status': 'WAIT_FOR_LOCK', 'reason': str(exc),
                                        'commands_enabled': False}, clear=True)
                else:
                    self.stop('LOCK_INVALIDATED_STOP', str(exc))

        def on_stations(self, msg):
            if self.stopped:
                return
            try:
                raw = json.loads(msg.data)
                if not isinstance(raw, dict):
                    raise PlanError('站点观测不是 JSON 对象')
                # 只比较可驱动规划结果变化的字段，忽略调试时间戳。
                model = {'source': raw.get('source'), 'frame_id': raw.get('frame_id'),
                         'frozen': raw.get('frozen'), 'stations': raw.get('stations')}
                fingerprint = json.dumps(model, sort_keys=True, ensure_ascii=False)
                if self.stations_frozen is not None:
                    if fingerprint != self.stations_frozen:
                        self.stop('STATION_CHANGED_STOP', '开局工位观测冻结后又被更改')
                    return
                self.stations = raw
                if self.blocked is not None:
                    self.refresh()
            except (ValueError, TypeError) as exc:
                self.stop('STATION_INVALID_STOP', str(exc))

        def refresh(self):
            if self.stopped or self.blocked is None:
                return
            try:
                payload = build_mission(args.start_zone, self.blocked, self.stations,
                                        'LIDAR_CONFIRMED')
            except (ValueError, TypeError) as exc:
                self.stop('PLAN_INVALID_STOP', str(exc))
                return
            self.publish_state(payload, clear=payload['status'] == 'NO_ROUTE')
            if payload['status'] == 'NO_ROUTE':
                self.stopped = True
                return
            if payload['stations_state'] == 'STATIONS_OBSERVED' and self.stations_frozen is None:
                self.stations_frozen = json.dumps({
                    'source': self.stations.get('source'), 'frame_id': self.stations.get('frame_id'),
                    'frozen': self.stations.get('frozen'), 'stations': self.stations.get('stations'),
                }, sort_keys=True, ensure_ascii=False)
            path = RosPath()
            path.header.frame_id = 'map'
            path.header.stamp = self.get_clock().now().to_msg()
            nodes = payload['path_nodes']
            for index, node in enumerate(nodes):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = NODES[node]
                if index + 1 < len(nodes):
                    dx = NODES[nodes[index+1]][0] - NODES[node][0]
                    dy = NODES[nodes[index+1]][1] - NODES[node][1]
                elif index > 0:
                    dx = NODES[node][0] - NODES[nodes[index-1]][0]
                    dy = NODES[node][1] - NODES[nodes[index-1]][1]
                else:
                    dx, dy = 1.0, 0.0
                yaw = math.atan2(dy, dx)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path.poses.append(pose)
            self.path_pub.publish(path)

    rclpy.init()
    node = RuleMissionPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main():
    parser = argparse.ArgumentParser(description='规则顺序 + 已有 Dijkstra 的只读路线验证')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--demo', action='store_true', help='手动封路的纯离线测试，永不视为真实锁路')
    mode.add_argument('--road-status-file', help='由 /competition/road_status 保存的原始 JSON 对象')
    mode.add_argument('--ros', action='store_true', help='订阅真实 /competition/road_status')
    parser.add_argument('--start-zone', choices=('upper_right', 'lower_right'), default='upper_right')
    parser.add_argument('--block', action='append', default=[], help='只能和 --demo 搭配，例如 B3-C3')
    parser.add_argument('--stations-file', help='离线工位数据（必须由实际定位模块提供）')
    parser.add_argument('--output', help='可选 JSON 输出路径；默认不会写文件')
    args = parser.parse_args()
    if (args.block and not args.demo) or (args.stations_file and args.ros):
        parser.error('--block 仅用于 --demo；--stations-file 只用于离线模式')
    try:
        return live_ros(args) if args.ros else offline(args)
    except (PlanError, ValueError, OSError) as exc:
        parser.exit(2, 'ERROR: {}\n'.format(exc))


if __name__ == '__main__':
    raise SystemExit(main())
