#!/usr/bin/env python3
"""2027 工创智能搬运：基于现有 Dijkstra 的比赛路线只读规划核心。

绝不驱动 STM32、不发布 /cmd_vel，也不替代既有锁路/地图/任务规划文件。
道路锁定来自 /competition/road_status；工位目标独立来自未来的静态雷达定位。
"""
import math
import sys
from pathlib import Path

# 新算法只放在 car_ws/src；只读复用旧工程 maps/ 中已有模块，不修改原算法。
# 从自身路径定位 car_ws，不依赖启动脚本的当前工作目录。
CAR_WS = Path(__file__).resolve().parents[2]
EXISTING_COMPETITION = CAR_WS / 'maps' / 'competition_v1'
TOPOLOGY = EXISTING_COMPETITION / 'topology_stage6' / 'road_topology.py'
MISSION_EXPORT = EXISTING_COMPETITION / 'oneclick_stage' / 'mission_export.py'
for dependency in (TOPOLOGY, MISSION_EXPORT):
    if not dependency.is_file():
        raise RuntimeError('缺少原工程依赖：{}；请勿复制/重写已有 Dijkstra 代码'.format(dependency))
sys.path.insert(0, str(TOPOLOGY.parent))
sys.path.insert(0, str(MISSION_EXPORT.parent))
from road_topology import NODES, EDGES, canonical, plan_mission  # noqa: E402
from mission_export import ZONES, STAGES  # noqa: E402

# 采用已经存在的 9 站任务表；仅增加工位语义，不更改实际任务顺序。
STATION_FOR_STEP = (None, 'qr_board', 'material', None, None,
                    'material', None, None, None)
ACTION_FOR_STEP = ('START', 'SCAN_QR', 'PICK_BATCH_1', 'ROUGH_PROCESS_BATCH_1',
                   'STORE_BATCH_1', 'PICK_BATCH_2', 'ROUGH_PROCESS_BATCH_2',
                   'STACK_BATCH_2', 'RETURN_START')
REQUIRED_STATIONS = ('qr_board', 'material')
STATION_ANCHOR = {'qr_board': 'C2', 'material': 'B3'}
FIELD_SIZE_M = 2.4
# 防止把非本工位目标匹配到锚点；工程初值，必须由场地实验修正。
MAX_STATION_TO_ANCHOR_M = 0.75
# 工程筛选初值，不代表 X3 已实测达到此置信度。
MIN_STATION_CONFIDENCE = 0.70


class PlanError(ValueError):
    pass


def _finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _safe_blocked(roads):
    if not isinstance(roads, (list, tuple, set, frozenset)):
        raise PlanError('blocked 必须是道路 ID 集合')
    out = set()
    for key in roads:
        if not isinstance(key, str):
            raise PlanError('道路 ID 必须是字符串')
        try:
            normalized = canonical(key)
        except (ValueError, TypeError) as exc:
            raise PlanError('未知封锁道路: {}'.format(key)) from exc
        if normalized in out:
            raise PlanError('重复的封锁道路: {}'.format(normalized))
        out.add(normalized)
    return out


def verify_real_lock(status):
    """严格验证已有 road_topology 发出的真实锁路结果，不能把人工 demo 认作实测。"""
    if not isinstance(status, dict):
        raise PlanError('road_status 必须是 JSON 对象')
    if status.get('status') != 'LOCKED_READY' or status.get('ready') is not True:
        raise PlanError('道路锁定未就绪或无可用路线')
    if status.get('frozen') is not True or status.get('conflict') is not False:
        raise PlanError('道路未冻结或存在障碍物冲突')
    if status.get('source') != 'LIDAR_CONFIRMED':
        raise PlanError('仅接受雷达确认的道路锁定，拒绝 MANUAL_TEST')
    blocked = _safe_blocked(status.get('blocked'))
    if not isinstance(status.get('mission_routes'), list):
        raise PlanError('道路状态缺少已计算的路线')
    # 不依赖路由器当前仅 C3→A3 的预览目标：重新对 9 站顺序逐段规划。
    return blocked


def _station_info(snapshot):
    """验证新定位模块的预留输入；只有通过检查才会随计划一起显示。"""
    if snapshot is None:
        return 'WAIT_STATIONS', {}, ['缺少转盘和二维码板的雷达地图坐标']
    if not isinstance(snapshot, dict) or snapshot.get('frame_id') != 'map':
        raise PlanError('stations 必须使用 map 坐标系')
    if snapshot.get('source') != 'LIDAR_STATIC' or snapshot.get('frozen') is not True:
        raise PlanError('stations 必须是已冻结的开局雷达观测，不接受演示数据')
    items = snapshot.get('stations')
    if not isinstance(items, dict):
        raise PlanError('stations.stations 必须是对象')
    validated = {}
    missing = []
    for name in REQUIRED_STATIONS:
        ob = items.get(name)
        if not isinstance(ob, dict) or ob.get('valid') is not True:
            missing.append(name)
            continue
        x, y, quality = (ob.get('x_m'), ob.get('y_m'), ob.get('confidence'))
        if not all(map(_finite_number, (x, y, quality))):
            raise PlanError('{} 的位置或置信度不是有限数值'.format(name))
        if not (0.0 <= x <= FIELD_SIZE_M and 0.0 <= y <= FIELD_SIZE_M):
            raise PlanError('{} 超出场地范围'.format(name))
        if not (MIN_STATION_CONFIDENCE <= quality <= 1.0):
            missing.append(name + ' (置信度不足)')
            continue
        anchor = STATION_ANCHOR[name]
        if math.dist((x, y), NODES[anchor]) > MAX_STATION_TO_ANCHOR_M:
            raise PlanError('{} 与预设工位锚点 {} 过远，拒绝自动关联'.format(name, anchor))
        out = {'x_m': round(x, 5), 'y_m': round(y, 5),
               'confidence': round(quality, 4), 'frame_id': 'map',
               'approach_anchor': anchor, 'stop_pose_calibrated': False}
        if name == 'qr_board':
            yaw = ob.get('yaw_rad')
            if not _finite_number(yaw) or not -math.pi <= yaw <= math.pi:
                raise PlanError('二维码板缺少有效 yaw_rad（弧度）')
            out['yaw_rad'] = round(yaw, 5)
            out['normal_direction_ambiguous'] = True
        validated[name] = out
    if missing:
        return 'WAIT_STATIONS', validated, ['尚未可靠定位：' + ', '.join(missing)]
    return 'STATIONS_OBSERVED', validated, []


def _edge_name(a, b):
    if a == b:
        raise PlanError('路径出现自环')
    try:
        return canonical('{}-{}'.format(a, b))
    except ValueError as exc:
        raise PlanError('路径中存在非拓扑边: {}-{}'.format(a, b)) from exc


def build_mission(start_zone, blocked_roads, stations=None, provenance='DEMO_ONLY'):
    """复用现有 Dijkstra，为 9 个比赛阶段生成 8 段道路级路径。

    provenance: LIDAR_CONFIRMED 仅由上游 verify_real_lock 通过后传入。
    返回数据仅供预览，没有里程计/转弯/末端对位，不可直接下发 STM32。
    """
    if start_zone not in ZONES:
        raise PlanError('未知启停区: {}'.format(start_zone))
    blocked = _safe_blocked(blocked_roads)
    waypoints = ZONES[start_zone]
    if len(waypoints) != 9 or len(STAGES) != 9 or len(STATION_FOR_STEP) != 9:
        raise PlanError('既有比赛阶段定义不符合九站任务约束')
    if waypoints[0] != waypoints[-1] or tuple(waypoints[1:-1]) != (
            'C2', 'B3', 'B1', 'A2', 'B3', 'B1', 'A2'):
        raise PlanError('既有任务站点顺序与当前初赛方案不符')

    # 禁止把尚未实测的工位数据默认填充为示意坐标。
    station_state, station_positions, warnings = _station_info(stations)
    paths = plan_mission(waypoints, blocked)  # 完全复用原版 Dijkstra
    result = {
        'schema_version': 1,
        'purpose': 'RULE_MISSION_ROAD_PREVIEW_ONLY',
        'NOT_FOR_NAV2_FOLLOWPATH': True,
        'commands_enabled': False,
        'frame_id': 'map',
        'start_zone': start_zone,
        'road_lock_source': provenance,
        'blocked_roads': sorted(blocked),
        'waypoints': list(waypoints),
        'stations_state': station_state,
        'station_positions': station_positions,
        'warnings': warnings + ['工位锚点不等于实测停车点；STM32 控制接口尚未接通',
                                '道路锁定状态校验不能替代整场视野覆盖、车体包络和场地实测验收'],
        'legs': [],
        'path_nodes': [],
    }
    if paths is None:
        result['status'] = 'NO_ROUTE'
        result['warnings'].append('至少一段比赛任务路线不可达；不可发车')
        return result
    if len(paths) != len(waypoints) - 1:
        raise PlanError('Dijkstra 返回的路段数量异常')
    path_nodes = []
    for index, (leg, start, goal) in enumerate(zip(paths, waypoints, waypoints[1:]), 1):
        if not leg or leg[0] != start or leg[-1] != goal:
            raise PlanError('第 {} 段路径首尾与工位顺序不匹配'.format(index))
        edges = [_edge_name(a, b) for a, b in zip(leg, leg[1:])]
        if any(edge in blocked for edge in edges):
            raise PlanError('Dijkstra 路线穿越已封锁道路')
        length = sum(math.dist(NODES[a], NODES[b]) for a, b in zip(leg, leg[1:]))
        role = STATION_FOR_STEP[index]
        result['legs'].append({
            'stage_index': index, 'stage': STAGES[index], 'action': ACTION_FOR_STEP[index],
            'start_node': start, 'goal_node': goal,
            'node_path': list(leg), 'road_ids': edges,
            'distance_m': round(length, 4),
            'target_station': role,
            'target_observation': station_positions.get(role) if role else None,
            'fine_vision_alignment_required': role is not None or goal in ('B1', 'A2'),
        })
        path_nodes.extend(leg if not path_nodes else leg[1:])
    result['path_nodes'] = path_nodes
    result['total_road_distance_m'] = round(sum(x['distance_m'] for x in result['legs']), 4)
    result['status'] = ('DEMO_ROUTE_ONLY' if provenance == 'DEMO_ONLY' else
                        'PREVIEW_WITH_STATIONS' if station_state == 'STATIONS_OBSERVED' else
                        'PREVIEW_WAIT_STATIONS')
    return result
