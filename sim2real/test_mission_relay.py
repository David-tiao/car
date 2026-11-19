#!/usr/bin/env python3
"""ROS2 TEST topic -> scoped localhost HTTP mission replay. Read-only ROS subscriber."""
import argparse
import json
from urllib.request import Request, urlopen
from urllib.parse import urlsplit


def validate_source_url(url):
    from ipaddress import ip_address
    parsed = urlsplit(url)
    if parsed.scheme != 'http' or not parsed.hostname or not ip_address(parsed.hostname).is_loopback:
        raise ValueError('TEST bridge only supports numeric localhost HTTP, e.g. 127.0.0.1')
    if parsed.username or parsed.password or parsed.path not in ('', '/'):
        raise ValueError('TEST bridge URL must be HTTP origin only')
    return url.rstrip('/')


def main():
    p = argparse.ArgumentParser(description='Read-only ROS2 TEST mission web relay')
    p.add_argument('--server', default='http://127.0.0.1:8080')
    args = p.parse_args()
    url = validate_source_url(args.server) + '/api/test/mission_replay'
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class Relay(Node):
        def __init__(self):
            super().__init__('car_test_mission_web_relay')
            self.sub = self.create_subscription(String, '/car/test/mission/state', self.receive, 10)
            self.get_logger().warning('MOCK-only TEST HTTP relay, never opens UART')

        def receive(self, message):
            try:
                data = json.loads(message.data)
                from test_mission_store import validate
                safe = validate(data)
                request = Request(url, data=json.dumps(safe, ensure_ascii=False,
                                                       allow_nan=False).encode('utf-8'),
                                  headers={'Content-Type': 'application/json'}, method='POST')
                with urlopen(request, timeout=0.4) as resp:
                    if resp.status != 200:
                        raise OSError('HTTP status {}'.format(resp.status))
            except (ValueError, KeyError, OSError, TypeError) as exc:
                self.get_logger().warning('MOCK relay rejected/unavailable: %s', exc)

    rclpy.init()
    node = Relay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
