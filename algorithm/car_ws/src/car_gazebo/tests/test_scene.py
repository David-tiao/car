import importlib.util
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

P = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("scene", P / "scripts/generate_scene.py")
scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene)
MAP_DIR = P.parents[1] / "maps/competition_v1"
ZONES = json.loads((MAP_DIR / "zones.json").read_text())
route_spec = importlib.util.spec_from_file_location("route_gate", P / "scripts/follow_locked_route.py")
route_gate = importlib.util.module_from_spec(route_spec)
route_spec.loader.exec_module(route_gate)


class SceneTest(unittest.TestCase):
    def test_urdf_wheel_contract(self):
        root = ET.fromstring(scene.xml_bytes(scene.robot_description()))
        joints = {j.get("name"): j for j in root.findall("joint")}
        self.assertEqual(set(joints), {"wheel_" + s + "_joint" for s, _, _ in scene.WHEELS})
        for suffix, x, y in scene.WHEELS:
            j = joints["wheel_" + suffix + "_joint"]
            self.assertEqual(j.get("type"), "continuous")
            self.assertEqual(j.find("parent").get("link"), "base_link")
            self.assertEqual(tuple(map(float, j.find("origin").get("xyz").split())),
                             (x, y, scene.R))
        self.assertIsNone(root.find("link[@name='laser_frame']"))

    def test_field_geometry_and_semantics(self):
        root = ET.fromstring(scene.xml_bytes(scene.world_description(ZONES, [(1.2, 1.2)])))
        w = root.find("world")
        self.assertEqual(w.get("name"), "car_preview")
        models = {m.get("name"): m for m in w.findall("model")}
        self.assertEqual(models["field"].findtext("link/collision/geometry/box/size"), "2.4 2.4 0.02")
        for name, model in models.items():
            if name.startswith(("yellow_", "blue_", "work_")):
                self.assertIsNone(model.find(".//collision"))
        self.assertEqual(models["test_obstacle_1"].findtext("link/collision/geometry/cylinder/radius"), "0.025")
        self.assertEqual(models["test_obstacle_1"].findtext("link/collision/geometry/cylinder/length"), "0.1")
        self.assertIsNone(models["car_preview_model"].find(".//collision"))

    def test_invalid_obstacles(self):
        for obstacles in ([(0.6, 0.6)], [(-1, 0)], [(float("nan"), 0.3)],
                          [(1.2, 1.2), (1.21, 1.2)], [(1.2, 2.35)]):
            with self.subTest(obstacles=obstacles), self.assertRaises(ValueError):
                scene.world_description(ZONES, obstacles)

    def test_sim_plugins_and_frames(self):
        world = scene.world_description(ZONES, [(1.2, 1.2)], mode="sim").find("world")
        robot = world.find("model[@name='car_sim_model']")
        self.assertIsNone(robot.find("static"))
        self.assertIsNotNone(robot.find("link/inertial/inertia/izz"))
        envelope = robot.find("link/collision[@name='provisional_envelope']")
        self.assertIsNotNone(envelope)
        length, width, height = map(float, envelope.findtext("geometry/box/size").split())
        self.assertGreaterEqual(length + 1e-9, 2*(scene.LX+scene.R))
        self.assertGreaterEqual(width + 1e-9, 2*scene.LY+scene.WHEEL_WIDTH)
        self.assertGreaterEqual(height + 1e-9, scene.BODY_Z+scene.BODY[2]/2)
        self.assertEqual(robot.findtext("plugin/topic"), "/car_gz/drive_cmd")
        self.assertEqual(robot.findtext("plugin/odom_topic"), "/car_gz/odom_raw")
        sensor = robot.find("link/sensor")
        self.assertEqual(sensor.get("type"), "gpu_lidar")
        self.assertEqual(sensor.findtext("topic"), "/car_gz/scan_raw")
        self.assertLess(scene.BODY_Z+scene.BODY[2]/2, scene.LASER_Z)
        self.assertLess(scene.LASER_Z+0.001, .10)
        urdf = scene.robot_description(sim=True)
        self.assertEqual(urdf.find("joint[@name='laser_joint']/origin").get("xyz"), "0 0 0.092")
        self.assertIsNone(scene.robot_description().find("link[@name='laser_frame']"))

    def test_sim_start_rejects_collision(self):
        for start in ((.6, .6), (0., 1.), (1.2, 2.25), (1.2, 1.2)):
            with self.subTest(start=start), self.assertRaises(ValueError):
                scene.world_description(ZONES, [(1.2, 1.2)], mode="sim", start=start)

    def test_sim_body_velocity_and_watchdog(self):
        self.assertEqual(scene.limited_twist(0., .1, -.2), (0., .1, -.2))
        vx, vy, wz = scene.limited_twist(.3, .4, 2.)
        self.assertAlmostEqual((vx*vx+vy*vy)**.5, .2)
        self.assertEqual(wz, .5)
        self.assertEqual(scene.limited_twist(float("nan"), .1, 0.), (0., 0., 0.))
        cmd = (.1, -.1, .2)
        self.assertEqual(scene.guarded_twist(cmd, .1, .1, .1, .1), cmd)
        for ages in ((.31, 0., 0., 0.), (0., .51, 0., 0.), (0., 0., .51, 0.),
                     (0., 0., 0., .51), (-1., 0., 0., 0.), (0., float("nan"), 0., 0.)):
            self.assertEqual(scene.guarded_twist(cmd, *ages), (0., 0., 0.))
        self.assertEqual(scene.wheel_rates(.04, 0., 0.), (1., 1., 1., 1.))
        self.assertEqual(scene.wheel_rates(0., .04, 0.), (-1., 1., 1., -1.))
        self.assertEqual(scene.wheel_rates(0., 0., 0.), (0., 0., 0., 0.))

    def test_route_gate_checks_existing_topology(self):
        nodes, edges = route_gate.load_topology(MAP_DIR / "topology_stage6/road_topology.py")
        status = {"status": "LOCKED_READY", "ready": True, "frozen": True, "conflict": False,
                  "source": "LIDAR_CONFIRMED", "route": ["C3", "C2", "C1", "B1", "A1"],
                  "blocked": ["B2-B3", "B3-C3"]}
        points = [nodes[n] for n in status["route"]]
        self.assertEqual(route_gate.route_contract(status, points, nodes, edges)[0], tuple(status["route"]))
        for change in ({"status": "CONFLICT_STOP"}, {"ready": False},
                       {"source": "MANUAL_TEST"}, {"conflict": True},
                       {"blocked": ["C2-C3"]}, {"route": ["C3", "A1"]},
                       {"blocked": ["bogus"]}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                route_gate.route_contract(dict(status, **change), points, nodes, edges)
        with self.assertRaises(ValueError):
            route_gate.route_contract(status, [(0, 0)]+points[1:], nodes, edges)
        key = route_gate.route_contract(status, points, nodes, edges)
        lock = {"state": "LOCKED", "locked": True, "map_received": True,
                "expected_obstacles": 2, "obstacles": [{}, {}],
                "blocked_roads": status["blocked"]}
        self.assertTrue(route_gate.matching_lock(lock, key))
        for change in ({"state": "CONFLICT"}, {"locked": False}, {"map_received": False},
                       {"obstacles": [{}]}, {"blocked_roads": ["C2-C3"]}):
            self.assertFalse(route_gate.matching_lock(dict(lock, **change), key))
        dense = route_gate.dense_path(points)
        self.assertEqual(dense[0], points[0])
        self.assertAlmostEqual(dense[-1][0], points[-1][0])
        self.assertAlmostEqual(dense[-1][1], points[-1][1])
        for a,b in zip(dense, dense[1:]):
            self.assertLessEqual(((a[0]-b[0])**2+(a[1]-b[1])**2)**.5, .02500001)

    def test_segment_paths_stop_at_each_locked_node(self):
        nodes, edges = route_gate.load_topology(MAP_DIR / "topology_stage6/road_topology.py")
        names = ("C3", "C2", "B2", "A2", "A1")
        segments = route_gate.segment_paths(names, nodes)
        self.assertEqual(len(segments), 4)
        for leg, a, b in zip(segments, names, names[1:]):
            self.assertEqual(leg[0], nodes[a])
            self.assertEqual(leg[-1], nodes[b])
            for x, y in leg:
                self.assertTrue(abs(x-nodes[a][0]) < 1e-9 or abs(y-nodes[a][1]) < 1e-9)
        with self.assertRaises(ValueError):
            route_gate.segment_paths(("C3",), nodes)

    def test_segment_settle_requires_fresh_measured_stop_and_pose(self):
        ready = route_gate.segment_settled
        target = (1.2, 1.2)
        self.assertTrue(ready((1.2, 1.24), target, (.01, .01), .2, True))
        for pose, speed, age, fresh in (
                ((1.2, 1.28), (.01, .01), .2, True),
                (target, (.026, 0.), .2, True),
                (target, (0., .051), .2, True),
                (target, (0., 0.), .1, True),
                (target, (0., 0.), 3.01, True),
                (target, (0., 0.), .2, False),
                ((float("nan"), 1.2), (0., 0.), .2, True),
                (target, (float("nan"), 0.), .2, True)):
            with self.subTest(pose=pose, speed=speed, age=age, fresh=fresh):
                self.assertFalse(ready(pose, target, speed, age, fresh))

    def test_lock_scan_freshness(self):
        good = {"scan_tf_ready": True, "scan_tf_stamp_ns": 2_000_000_000}
        self.assertTrue(route_gate.fresh_lock_scan(good, 3_000_000_000))
        self.assertFalse(route_gate.fresh_lock_scan(good, 3_500_000_001))
        self.assertFalse(route_gate.fresh_lock_scan(good, 1_800_000_000))
        self.assertFalse(route_gate.fresh_lock_scan(dict(good, scan_tf_ready=False), 2_000_000_000))
        self.assertFalse(route_gate.fresh_lock_scan({}, 2_000_000_000))
        self.assertFalse(route_gate.fresh_lock_scan(dict(good, scan_tf_stamp_ns=True), 2_000_000_000))

    def test_mirror_has_no_synthetic_obstacles(self):
        w = scene.world_description(ZONES, mode="mirror").find("world")
        self.assertEqual(w.get("name"), "car_mirror")
        self.assertIsNotNone(w.find("model[@name='car_mirror_model']"))
        self.assertFalse(any(m.get("name").startswith("test_obstacle") for m in w.findall("model")))
        with self.assertRaises(ValueError):
            scene.world_description(ZONES, [(1.2, 1.2)], "mirror")


if __name__ == "__main__":
    unittest.main()
