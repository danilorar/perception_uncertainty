"""Regression checks for dropout, control, kinematics and source preservation.

Author: Zhuo Ma
Run: python3 -m unittest discover -s tests -v
These tests use no esmini library or viewer. Native regression results are
documented separately in docs/验证记录.md.
"""
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "simulation"))
from aeb_controller import HostReference, aeb_command, integrate, risk_metrics
from generate_scenarios import generate, sha256
from paths import DATA
from sensor_model import DetectionDropout


class DropoutTests(unittest.TestCase):
    """Check missing-detection semantics and reproducible paired random draws."""

    def test_zero_and_complete_dropout(self):
        ids = [2, 1]
        self.assertEqual(DetectionDropout(0).apply(ids, 1.0, 0), ([1, 2], []))
        self.assertEqual(DetectionDropout(1).apply(ids, 1.0, 0), ([], [1, 2]))
        self.assertEqual(ids, [2, 1])  # Caller-owned input is not mutated.

    def test_pairing_survives_other_objects_and_missing_updates(self):
        off = DetectionDropout(0.1, seed=42)
        on = DetectionDropout(0.1, seed=42)
        for tick in range(200):
            kept_off, _ = off.apply([1, 2, 3], tick * 0.01, 0)
            if tick % 3 == 0:
                continue  # This object was not visible on some ON-run ticks.
            kept_on, _ = on.apply([2], tick * 0.01, 0)
            self.assertEqual(2 in kept_off, 2 in kept_on)

    def test_probability_is_sampled_not_forced_every_tenth_frame(self):
        model = DetectionDropout(0.1, seed=42)
        misses = sum(bool(model.apply([1], i * 0.01, 0)[1]) for i in range(10000))
        self.assertTrue(800 < misses < 1200, misses)

    def test_invalid_configuration(self):
        for probability in (-0.1, 1.1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                DetectionDropout(probability)
        for dt in (0, -1, math.nan):
            with self.assertRaises(ValueError):
                DetectionDropout(dt=dt)


class ControlTests(unittest.TestCase):
    """Check braking boundaries rather than copying the implementation logic."""

    def test_stop_inside_timestep_does_not_reverse(self):
        speed, distance = integrate(5.0, -6.0, 1.0)
        self.assertEqual(speed, 0.0)
        self.assertAlmostEqual(distance, 25.0 / 12.0)
        self.assertEqual(integrate(speed, -6.0, 1.0), (0.0, 0.0))

    def test_detection_gates_trigger_but_does_not_release_latch(self):
        parameters = dict(enabled=True, ttc_threshold=2.0, deceleration=6.0)
        missing = aeb_command(False, True, 12.0, 6.0, 10.0, False, **parameters)
        self.assertEqual(missing[:2], (0.0, False))
        detected = aeb_command(True, True, 12.0, 6.0, 10.0, False, **parameters)
        self.assertEqual(detected, (-6.0, True, 2.0))
        after_dropout = aeb_command(False, False, math.inf, 0.0, 9.0, True, **parameters)
        self.assertEqual(after_dropout[:2], (-6.0, True))

    def test_aeb_off_cannot_trigger(self):
        result = aeb_command(True, True, -0.1, 6.0, 10.0, False,
                             enabled=False, ttc_threshold=2.0, deceleration=6.0)
        self.assertEqual(result[:2], (0.0, False))

    def test_gap_uses_box_edges_and_checks_lateral_overlap(self):
        common = dict(h=0.0, centerOffsetX=0.0, centerOffsetY=0.0, length=5.0, width=1.8)
        ego = SimpleNamespace(x=0.0, y=0.0, speed=10.0, **common)
        target = SimpleNamespace(x=20.0, y=0.0, speed=4.0, **common)
        self.assertEqual(risk_metrics(ego, target), (15.0, 6.0, True))
        target.y = 3.0
        self.assertFalse(risk_metrics(ego, target)[2])


class ScenarioTests(unittest.TestCase):
    """Ensure scenario conversion preserves truth data and control ownership."""

    def test_generation_preserves_sources_and_target_trajectory(self):
        source = DATA / "C_original_1549178.xosc"
        before = sha256(source)
        original = ET.parse(source)
        with tempfile.TemporaryDirectory() as folder:
            path, manifest = generate("1549178", Path(folder))
            converted = ET.parse(path)
            self.assertEqual(sha256(source), before)
            self.assertEqual(manifest["source_sha256"], before)
            groups = converted.findall(".//ManeuverGroup")
            self.assertEqual(len(groups), 1)
            self.assertEqual(groups[0].find("./Actors/EntityRef").get("entityRef"), "object_2")
            old_group = next(g for g in original.findall(".//ManeuverGroup")
                             if g.find('./Actors/EntityRef[@entityRef="object_2"]') is not None)
            old_vertices = [(v.attrib, v.find("./Position/WorldPosition").attrib)
                            for v in old_group.findall(".//Vertex")]
            new_vertices = [(v.attrib, v.find("./Position/WorldPosition").attrib)
                            for v in groups[0].findall(".//Vertex")]
            self.assertEqual(new_vertices, old_vertices)
            self.assertEqual(converted.find("./FileHeader").attrib, original.find("./FileHeader").attrib)
            road = path.parent / converted.find("./RoadNetwork/LogicFile").get("filepath")
            self.assertEqual(road.resolve(), source.with_suffix(".xodr").resolve())
            properties = converted.findall('./Entities/ScenarioObject[@name="object_1"]/ObjectController/Controller/Properties/Property')
            self.assertEqual({p.get('name'): p.get('value') for p in properties}["esminiController"], "ExternalController")
            actions = converted.findall('./Storyboard/Init/Actions/Private[@entityRef="object_1"]/PrivateAction')
            self.assertTrue(all(len(action) == 1 for action in actions))
            with self.assertRaises(FileExistsError):
                generate("1549178", Path(folder))

    def test_time_and_distance_reference_agree_at_original_vertices(self):
        reference = HostReference(DATA / "C_original_1549178.xosc")
        for index in (1, 100, len(reference.times) - 1):
            pose, distance, speed = reference.at_time(reference.times[index])
            self.assertGreater(speed, 0)
            for actual, expected in zip(pose, reference.poses[index]):
                self.assertAlmostEqual(actual, expected)
            for actual, expected in zip(reference.at_distance(distance), pose):
                self.assertAlmostEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
