#!/usr/bin/env python3
"""Honest ROS2/RViz scene + real raw-scan inset + Dijkstra baseline.

The sensor inset is intentionally OUTSIDE the competition field. It is not
transformed into the map and MUST NOT be used as an obstacle location.
Real road locking is published by existing lock_obstacles/road_topology nodes.
"""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path


def color_floats(rgba):
    if len(rgba) != 4 or any(not math.isfinite(float(v)) for v in rgba):
        raise ValueError('invalid RGBA')
    return tuple(float(v) for v in rgba)


def graph_and_scene(zones_file, topology_file):
    zones = json.loads(Path(zones_file).read_text(encoding='utf-8'))
    if zones.get('start_zone') != 'upper_right' or zones.get('field_size') != [2.4, 2.4]:
        raise ValueError('unrecognized V6 field geometry/start zone')
    spec = importlib.util.spec_from_file_location('v6_existing_road_topology', topology_file)
    if spec is None or spec.loader is None:
        raise ValueError('cannot load existing topology')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    nodes, edges = module.NODES, module.EDGES
    if (len(nodes) != 9 or len(edges) != 12 or
            set(nodes) != set(zones['topology_nodes']) or
            set(edges) != set(zones['topology_edges'])):
        raise ValueError('V6 zones and road topology disagree')
    for name, pos in nodes.items():
        if any(abs(float(pos[i])-float(zones['topology_nodes'][name][i])) > 1e-6 for i in range(2)):
            raise ValueError('node coordinates differ: ' + name)
    for name, pair in edges.items():
        if list(pair) != zones['topology_edges'][name]:
            raise ValueError('road endpoints differ: ' + name)
    route = module.shortest('C3', 'A3', set())
    if not route or route[0] != 'C3' or route[-1] != 'A3':
        raise ValueError('Dijkstra baseline failed')
    return zones, nodes, edges, route


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--zones', type=Path, required=True)
    parser.add_argument('--topology', type=Path, required=True)
    parser.add_argument('--ready-file', type=Path, required=True)
    args = parser.parse_args()
    zones, nodes, edges, route = graph_and_scene(args.zones, args.topology)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Point, PoseStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Path as NavPath
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import String

    latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)

    class Overlay(Node):
        def __init__(self):
            super().__init__('competition_v6_static_visual')
            self.scene_pub = self.create_publisher(MarkerArray, '/competition/v6/scene', latched)
            self.path_pub = self.create_publisher(NavPath, '/competition/v6/prelock_path', latched)
            self.status_pub = self.create_publisher(String, '/competition/v6/prelock_status', latched)
            self.latest_scan = None
            self.last_valid = 0
            self.sub = self.create_subscription(LaserScan, '/scan', self.got_scan, qos_profile_sensor_data)
            self.timer = self.create_timer(.5, self.publish)
            self.publish()

        def got_scan(self, msg):
            self.latest_scan = msg

        def marker(self, ident, ns, kind, color, x=0, y=0, z=.1,
                   sx=.04, sy=.04, sz=.04, points=None, title=''):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = ns, ident, kind, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
            m.color.r, m.color.g, m.color.b, m.color.a = color_floats(color)
            m.text = title
            for q in points or []:
                m.points.append(Point(x=float(q[0]), y=float(q[1]), z=float(q[2])))
            return m

        def publish(self):
            ms = MarkerArray()
            add = ms.markers.append
            # Put the scene ABOVE the occupancy grid; it is a visual overlay,
            # never a fake costmap. Default RViz hides the grey occupancy map.
            add(self.marker(0, 'field', Marker.CUBE, (.59,.63,.68,1.),
                            1.2,1.2,.04,2.4,2.4,.045))
            for i, (x0,y0,x1,y1) in enumerate(zones['yellow']):
                add(self.marker(i,'yellow_keepout_visual',Marker.CUBE,(1.,.83,.16,1.),
                                (x0+x1)/2,(y0+y1)/2,.08,x1-x0,y1-y0,.035))
            for i, (x0,y0,x1,y1) in enumerate(zones['blue']):
                add(self.marker(i,'blue_starts',Marker.CUBE,(.08,.29,.95,1.),
                                (x0+x1)/2,(y0+y1)/2,.10,x1-x0,y1-y0,.035))
            for i, (x0,y0,x1,y1) in enumerate(zones['work']):
                add(self.marker(i,'work_zones',Marker.CUBE,(.92,.93,.94,1.),
                                (x0+x1)/2,(y0+y1)/2,.10,x1-x0,y1-y0,.035))
            disk = zones['raw_disk']
            add(self.marker(0,'raw_disk',Marker.CYLINDER,(.87,.94,.98,1.),
                            disk['center'][0],disk['center'][1],.105,
                            2*disk['radius'],2*disk['radius'],.07))
            for i, (label, (a,b)) in enumerate(sorted(edges.items())):
                add(self.marker(i,'roads',Marker.LINE_STRIP,(.06,.8,.22,1.),
                    sx=.027, points=[(*nodes[a],.16),(*nodes[b],.16)]))
            for i, (name,(x,y)) in enumerate(sorted(nodes.items())):
                add(self.marker(i,'road_nodes',Marker.SPHERE,(.13,.32,1.,1.),
                                x,y,.20,.056,.056,.056))
                add(self.marker(i,'road_names',Marker.TEXT_VIEW_FACING,(.98,.98,.98,1.),
                                x,y,.36,sz=.11,title=name))
            # Demonstration route calculated by the ORIGINAL Dijkstra function.
            # Distinct and labeled: not blocked-road planning.
            add(self.marker(0,'unlocked_baseline',Marker.LINE_STRIP,(.05,1.,1.,.84),
                sx=.038,points=[(*nodes[k],.25) for k in route]))
            add(self.marker(0,'baseline_note',Marker.TEXT_VIEW_FACING,(1.,.97,.92,1.),
                            1.15,-.14,.31,sz=.105,title='BASELINE C3 to A3 (not locked)'))
            add(self.marker(0,'start_note',Marker.TEXT_VIEW_FACING,(1.,1.,1.,1.),
                            2.16,2.39,.29,sz=.10,title='START'))
            # Raw sensor inset, placed OUTSIDE [0,2.4]x[0,2.4]. No fake TF.
            cx,cy,scale = 3.65,1.17,.105
            add(self.marker(0,'raw_inset_bounds',Marker.LINE_STRIP,(.29,.48,.67,1.),
                            sx=.016,points=[(cx-.9,cy-.9,.24),(cx+.9,cy-.9,.24),
                                            (cx+.9,cy+.9,.24),(cx-.9,cy+.9,.24),
                                            (cx-.9,cy-.9,.24)]))
            add(self.marker(0,'raw_inset_origin',Marker.SPHERE,(1.,.32,.13,1.),
                            cx,cy,.26,.048,.048,.048))
            dots=[]
            scan=self.latest_scan
            if scan is not None:
                for i,v in enumerate(scan.ranges):
                    if math.isfinite(v) and max(.10,scan.range_min)<=v<=min(8.,scan.range_max):
                        ang=scan.angle_min+i*scan.angle_increment
                        dots.append((v*math.cos(ang)*scale,v*math.sin(ang)*scale,0.))
            add(self.marker(0,'raw_live_points',Marker.POINTS,(1.,.31,.15,1.),
                            x=cx,y=cy,z=.26,sx=.016,sy=.016,points=dots))
            add(self.marker(0,'raw_scan_note',Marker.TEXT_VIEW_FACING,(1.,1.,1.,1.),
                            cx,cy+1.07,.35,sz=.11,title='RAW SCAN / NO MAP TF'))
            add(self.marker(0,'scan_count',Marker.TEXT_VIEW_FACING,(1.,1.,1.,1.),
                            cx,cy-.99,.32,sz=.09,title='valid points: {}'.format(len(dots))))
            self.scene_pub.publish(ms)

            path = NavPath()
            path.header.frame_id = 'map'
            path.header.stamp = self.get_clock().now().to_msg()
            for name in route:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = map(float, nodes[name])
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.path_pub.publish(path)
            self.status_pub.publish(String(data=json.dumps({
                'mode':'UNLOCKED_BASELINE_ONLY', 'start':'C3','goal':'A3',
                'route':route, 'blocked_roads':[], 'raw_scan_points':len(dots),
                'raw_scan_inset':'visualization only, not physical map positions',
                'stm32_motion':False
            },ensure_ascii=False)))

    rclpy.init()
    node = None
    try:
        node = Overlay()
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
        args.ready_file.write_text(json.dumps({'state':'PUBLISHED','pid':os.getpid(),
                                               'baseline':route}),encoding='utf-8')
        print('[VISUAL_READY] scene + raw sensor inset + graph + baseline',flush=True)
        rclpy.spin(node)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
