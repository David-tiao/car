import importlib.util
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

P = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("scene", P / "scripts/generate_scene.py")
scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene)
ZONES = json.loads((P.parents[1] / "maps/competition_v1/zones.json").read_text())


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

    def test_mirror_has_no_synthetic_obstacles(self):
        w = scene.world_description(ZONES, mode="mirror").find("world")
        self.assertEqual(w.get("name"), "car_mirror")
        self.assertIsNotNone(w.find("model[@name='car_mirror_model']"))
        self.assertFalse(any(m.get("name").startswith("test_obstacle") for m in w.findall("model")))
        with self.assertRaises(ValueError):
            scene.world_description(ZONES, [(1.2, 1.2)], "mirror")


if __name__ == "__main__":
    unittest.main()
