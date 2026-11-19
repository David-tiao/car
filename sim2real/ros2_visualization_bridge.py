#!/usr/bin/env python3
"""Read-only ROS2 navigation visualization bridge for the V12 self-hosted website.

Runs on the Ubuntu computer hosting car. ROS2 DDS subscriptions may receive
remote Raspberry Pi 5 topics; only localhost can publish website snapshots.
No command publisher and no motor-control functionality are present.
"""
import argparse
import base64
from collections import deque
import json
import math
import sys
import time
from urllib.request import Request, urlopen
from urllib.error import URLError


def yaw_quat(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def point(x, y):
    return [round(float(x),4), round(float(y),4)]


def post(server, route, payload, timeout=.7):
    data=json.dumps(payload, separators=(',', ':'), allow_nan=False).encode('utf-8')
    req=Request(server.rstrip('/')+route, data=data, method='POST', headers={'Content-Type':'application/json'})
    with urlopen(req,timeout=timeout) as response:
        if response.status != 200:
            raise OSError('HTTP '+str(response.status))


def web_route(channel, leaf):
    if channel not in ('test', 'real'):
        return '/api/' + leaf
    return '/api/' + channel + '/' + leaf


def main():
    parser=argparse.ArgumentParser(description='ROS2 LaserScan, OccupancyGrid, Nav2 paths/costmaps, TF -> car website')
    parser.add_argument('--server',default='http://127.0.0.1:8080')
    parser.add_argument('--web-channel', choices=('legacy', 'test', 'real'), default='legacy',
                        help='Isolate outgoing HTTP data by data provenance')
    parser.add_argument('--hz',default=4.0,type=float,help='Dashboard snapshot publishing rate (1..10 Hz)')
    parser.add_argument('--telemetry-hz',default=15.0,type=float,help='Website real pose feedback forwarding rate (1..30 Hz)')
    parser.add_argument('--no-telemetry',action='store_true',help='Disable /odom forwarding if an old odom bridge is already running')
    parser.add_argument('--scan',default='/scan')
    parser.add_argument('--map',default='/map')
    parser.add_argument('--odom',default='/odom')
    parser.add_argument('--global-costmap',default='/global_costmap/costmap')
    parser.add_argument('--local-costmap',default='/local_costmap/costmap')
    parser.add_argument('--plan',default='/plan')
    parser.add_argument('--local-plan',default='/local_plan')
    parser.add_argument('--road-path',default='/competition/road_path',help='Your Stage 6 road topology path')
    parser.add_argument('--road-graph',default='/competition/road_graph',help='Your Stage 6 MarkerArray topology')
    parser.add_argument('--start-markers',default='/competition_start_zones',help='Your blue start area MarkerArray')
    parser.add_argument('--cmd-vel',default='/cmd_vel')
    parser.add_argument('--world-frame',default='map')
    parser.add_argument('--base-frame',default='base_link')
    parser.add_argument('--goal-status',default='/navigate_to_pose/_action/status')
    parser.add_argument('--diagnostics',default='/diagnostics')
    args=parser.parse_args()
    if not args.server.startswith(('http://127.0.0.1:', 'http://localhost:', 'http://[::1]:')):
        parser.error('Website ROS publisher must be on localhost (read-only to LAN viewers)')
    if not 1 <= args.hz <= 10:
        parser.error('Use --hz 1..10 for bandwidth and CPU control')
    if not 1 <= args.telemetry_hz <= 30:
        parser.error('Use --telemetry-hz 1..30')
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.time import Time
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import Twist
        from action_msgs.msg import GoalStatusArray
        from diagnostic_msgs.msg import DiagnosticArray
        from visualization_msgs.msg import MarkerArray
        from tf2_ros import Buffer, TransformListener
    except ImportError as exc:
        sys.exit('ROS2 imports missing. Install ROS2 Humble on Ubuntu, source /opt/ros/humble/setup.bash, and use /usr/bin/python3.\n'+str(exc))

    transient=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    reliable=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    rclpy.init()

    class Viewer(Node):
        def __init__(self):
            super().__init__('competition_web_ros_visualization')
            self.buffer=Buffer(cache_time=None)
            self.listener=TransformListener(self.buffer, self, spin_thread=False)
            self.scan=None
            self.odom=None
            self.paths={}
            self.markers={}
            self.cmd=None
            self.nav={}
            self.diagnostics=[]
            self.pending_grids={}
            self.last_grid_sent={}
            self.last_telemetry=0.0
            self.activity={}
            self.graph_topics=[]
            self.graph_nodes=[]
            self.last_graph=0
            self.last_warn=0
            self.published=0
            self.world_frame=args.world_frame
            subs=[
                ('scan',args.scan,LaserScan,self.on_scan,qos_profile_sensor_data),
                ('map',args.map,OccupancyGrid,lambda m:self.on_grid('map',m),transient),
                ('global_costmap',args.global_costmap,OccupancyGrid,lambda m:self.on_grid('global_costmap',m),reliable),
                ('local_costmap',args.local_costmap,OccupancyGrid,lambda m:self.on_grid('local_costmap',m),reliable),
                ('odom',args.odom,Odometry,self.on_odom,reliable),
                ('global_plan',args.plan,Path,lambda m:self.on_path('global',m),reliable),
                ('local_plan',args.local_plan,Path,lambda m:self.on_path('local',m),reliable),
                ('road_path',args.road_path,Path,lambda m:self.on_path('road',m),reliable),
                ('road_graph',args.road_graph,MarkerArray,lambda m:self.on_markers('road_graph',m),transient),
                ('start_markers',args.start_markers,MarkerArray,lambda m:self.on_markers('start_markers',m),transient),
                ('cmd_vel',args.cmd_vel,Twist,self.on_cmd,reliable),
                ('nav_status',args.goal_status,GoalStatusArray,self.on_nav,reliable),
                ('diagnostics',args.diagnostics,DiagnosticArray,self.on_diagnostics,reliable),
            ]
            self.names={role:name for role,name,_,_,_ in subs}
            self.subscriptions=[self.create_subscription(msgtype,name,lambda m,f=cb,role=role:(self.touch(role),f(m)),qos)
                                for role,name,msgtype,cb,qos in subs if name]
            # /map may be VOLATILE from some SLAM publishers, but TRANSIENT_LOCAL from Nav2 map_server.
            # A second VOLATILE subscriber handles both; latest map wins, duplicates are harmless.
            if args.map:
                self.subscriptions.append(self.create_subscription(OccupancyGrid,args.map,lambda m:(self.touch('map'),self.on_grid('map',m)),reliable))
            self.timer=self.create_timer(1.0/args.hz,self.publish_state)
            self.get_logger().info('ROS2 visualization bridge listening for '+', '.join(f'{r}={n}' for r,n,_,_,_ in subs))
            self.get_logger().info('Waiting for topics. Map and LaserScan must have working TF for map-aligned overlays.')

        def touch(self,role):
            now=time.monotonic()
            rec=self.activity.setdefault(role, {'last':now,'times':deque(maxlen=30)})
            rec['last']=now;rec['times'].append(now)

        def on_scan(self,m):
            self.scan=m

        def on_odom(self,m):
            if args.web_channel == 'real':
                nodes = self.get_node_names_and_namespaces()
                if any('mock' in name.lower() or 'synthetic' in name.lower()
                       for name, _ in nodes):
                    return  # Reject ROS domains containing mock publishers.
                publishers = self.get_publishers_info_by_topic(args.odom)
                if not any(p.node_name in ('car_serial_bridge', 'car_ulog43_bridge')
                           for p in publishers):
                    return  # Unknown odom publisher is not confirmed STM32 telemetry.

            self.odom=m
            if args.no_telemetry:return
            now=time.monotonic()
            if now-self.last_telemetry<1.0/args.telemetry_hz:return
            self.last_telemetry=now
            p=m.pose.pose.position;v=m.twist.twist
            telemetry={'x':float(p.x),'y':float(p.y),'yaw':yaw_quat(m.pose.pose.orientation),
                       'vx':float(v.linear.x),'vy':float(v.linear.y),'wz':float(v.angular.z),
                       'frame':m.header.frame_id or 'odom','source':'ros2_odom'}
            try:post(args.server,web_route(args.web_channel,'telemetry'),telemetry,timeout=.35)
            except (URLError,OSError,ValueError) as e:
                if now-self.last_warn>6:
                    self.last_warn=now
                    self.get_logger().warning(f'Website /api/telemetry unavailable: {e}')

        def on_cmd(self,m):
            self.cmd={'vx':round(float(m.linear.x),4),'vy':round(float(m.linear.y),4),'wz':round(float(m.angular.z),4)}

        def on_nav(self,m):
            states=[int(item.status) for item in m.status_list[-5:]]
            self.nav={'goal_statuses':states,'status':states[-1] if states else None}

        def on_diagnostics(self,m):
            self.diagnostics=[{'name':x.name[:100], 'level':int(x.level), 'message':x.message[:120]} for x in m.status[:16]]

        def on_path(self,key,m):
            poses=m.poses
            skip=max(1, math.ceil(len(poses)/500))
            self.paths[key]={'frame':m.header.frame_id,'points': [point(p.pose.position.x,p.pose.position.y) for p in poses[::skip]][:500],
                             'original_count':len(poses)}

        def on_markers(self,key,m):
            layers=[]
            for v in m.markers[:150]:
                if int(v.action) == 2:  # DELETE
                    continue
                stride=max(1,math.ceil(len(v.points)/180))
                pos=v.pose.position
                layers.append({'type':int(v.type),'frame':v.header.frame_id,'id':int(v.id),
                   'ns':str(v.ns)[:60], 'x':round(float(pos.x),4),'y':round(float(pos.y),4),
                   'yaw':round(yaw_quat(v.pose.orientation),5),
                   'sx':round(float(v.scale.x),4),'sy':round(float(v.scale.y),4),
                   'points':[point(pt.x,pt.y) for pt in v.points[::stride]][:180],
                   'color':[round(float(v.color.r),2),round(float(v.color.g),2),round(float(v.color.b),2),round(float(v.color.a),2)],
                   'text':str(v.text)[:60]})
            self.markers[key]=layers

        def on_grid(self,layer,m):
            w,h=m.info.width,m.info.height
            if w<1 or h<1 or w>1200 or h>1200 or w*h>1500000:
                self.get_logger().warning(f'{layer}: occupancy grid exceeds dashboard limit: {w} x {h}')
                return
            # OccupancyGrid is row-major, bottom-left grid origin; -1 becomes 255 unknown.
            array=bytes(int(v)&0xff for v in m.data)
            origin=m.info.origin
            self.pending_grids[layer]={
                'name':layer,'width':w,'height':h,'resolution':round(float(m.info.resolution),8),
                'origin_x':round(float(origin.position.x),5),'origin_y':round(float(origin.position.y),5),
                'origin_yaw':round(yaw_quat(origin.orientation),6),'frame':m.header.frame_id or 'map',
                'data':base64.b64encode(array).decode('ascii')}
            if layer=='map':
                self.world_frame=m.header.frame_id or args.world_frame

        def lookup(self,parent,child):
            try:
                t=self.buffer.lookup_transform(parent,child,Time())
                p=t.transform.translation
                return {'x':round(float(p.x),5),'y':round(float(p.y),5),
                        'yaw':round(yaw_quat(t.transform.rotation),6),
                        'frame':parent,'child':child}
            except Exception:
                return None

        def scan_points(self,target):
            m=self.scan
            if m is None:return None
            stride=max(1,math.ceil(len(m.ranges)/720))
            raw=[]
            for i in range(0,len(m.ranges),stride):
                r=float(m.ranges[i])
                if math.isfinite(r) and max(.02,m.range_min)<=r<=m.range_max:
                    a=m.angle_min+i*m.angle_increment
                    raw.append(point(r*math.cos(a),r*math.sin(a)))
            transform=self.lookup(target,m.header.frame_id)
            world=[]
            if transform:
                ca=math.cos(transform['yaw']);sa=math.sin(transform['yaw'])
                world=[point(transform['x']+ca*x-sa*y,transform['y']+sa*x+ca*y) for x,y in raw]
            return {'frame':m.header.frame_id,'target_frame':target,'transformed':bool(transform),
                    'points':world,'local_points':raw,'origin':transform,
                    'range_max':round(float(m.range_max),3),'original_count':len(m.ranges)}

        def fetch_graph(self):
            try:
                self.graph_topics=[{'name':n,'types':t[:3]} for n,t in self.get_topic_names_and_types()[:100]]
                self.graph_nodes=[n for n,_ in self.get_node_names_and_namespaces()[:80]]
            except Exception:pass

        def publish_state(self):
            now=time.monotonic()
            if now-self.last_graph>2:
                self.last_graph=now
                self.fetch_graph()
            target=self.world_frame
            pose=self.lookup(target,args.base_frame)
            tf={'map_odom': self.lookup(target,'odom'),
                'odom_base':self.lookup('odom',args.base_frame),
                'base_laser':self.lookup(args.base_frame,self.scan.header.frame_id) if self.scan else None,
                'map_base': pose}
            odom=None
            if self.odom is not None:
                m=self.odom;v=m.twist.twist;p=m.pose.pose.position
                odom={'frame':m.header.frame_id,'child':m.child_frame_id,
                      'x':round(float(p.x),4),'y':round(float(p.y),4),
                      'yaw':round(yaw_quat(m.pose.pose.orientation),5),
                      'vx':round(float(v.linear.x),4),'vy':round(float(v.linear.y),4),'wz':round(float(v.angular.z),4)}
            topics={}
            for role,name in self.names.items():
                rec=self.activity.get(role)
                if rec:
                    ticks=rec['times'];rate=(len(ticks)-1)/(ticks[-1]-ticks[0]) if len(ticks)>1 and ticks[-1]>ticks[0] else 0
                    topics[role]={'topic':name,'received':True,'age_s':round(now-rec['last'],2),'hz':round(rate,1)}
                else:topics[role]={'topic':name,'received':False,'age_s':None,'hz':0}
            packet={'display_frame':target,'robot':pose,'tf':tf,'scan':self.scan_points(target),'odom':odom,
                    'paths':self.paths,'markers':self.markers,'cmd_vel':self.cmd,'topics':topics,
                    'nodes':self.graph_nodes,'nav':self.nav,'diagnostics':self.diagnostics,
                    'bridge':{'name':'ros2_visualization_bridge','graph_topics':self.graph_topics}}
            try:
                post(args.server,web_route(args.web_channel,'ros/state'),packet)
                self.published+=1
                if self.published%100==0:self.get_logger().info(f'Published {self.published} ROS visualization snapshots')
            except (URLError,OSError,ValueError) as e:
                if now-self.last_warn>6:
                    self.last_warn=now
                    self.get_logger().warning(f'Website /api/ros/state unavailable: {e}')
            # Avoid resending high-rate costmaps on every update; send newest only.
            for layer in ('map','global_costmap','local_costmap'):
                obj=self.pending_grids.get(layer)
                if obj is None:continue
                min_delay=0 if layer=='map' else (1.5 if layer=='local_costmap' else 2.0)
                if now-self.last_grid_sent.get(layer,0)<min_delay:continue
                try:
                    post(args.server,web_route(args.web_channel,'ros/grid'),obj, timeout=1.5)
                    self.last_grid_sent[layer]=now
                    if self.pending_grids.get(layer) is obj:
                        self.pending_grids.pop(layer,None)
                except (URLError,OSError,ValueError) as e:
                    if now-self.last_warn>6:
                        self.last_warn=now
                        self.get_logger().warning(f'Website occupancy layer {layer} unavailable: {e}')

    node=Viewer()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':main()
