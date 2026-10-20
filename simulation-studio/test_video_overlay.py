"""Geometry, observation timing and image-format checks for video error boxes."""

import csv
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from PIL import Image

from video import align_visual_models
from video_overlay import ObservationOverlay, PerceivedBox, project_top_point


class VideoOverlayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)
        target = dict(id=1, x=10, y=20, z=.1, h=.2, length=5, width=1.8,
                      height=1.6, centerOffsetX=1.4, centerOffsetY=.3, centerOffsetZ=.7)
        self.frames = {t: [target | {'x': 10 + t, 'h': .2 + t}]
                       for t in (0, .05, .1, .15)}

    def row(self, time, **changes):
        return dict(time_s=time, object_id=1, visible=True, observation_valid=True,
                    observed_center_x_m=30, observed_center_y_m=20,
                    observed_range_m=25) | changes

    def overlay(self, rows):
        with (self.out / 'observations.csv').open('w', newline='', encoding='utf-8') as file:
            writer = csv.DictWriter(file, fieldnames=list(self.row(0)))
            writer.writeheader()
            writer.writerows(rows)
        return ObservationOverlay(self.out, {'uncertainty': 'distance_ou_range'}, self.frames, 0)

    def test_observed_centre_not_truth_or_reference_offset(self):
        box = self.overlay([self.row(0)]).boxes_at(0)[0]
        self.assertEqual((box.x, box.y), (30, 20))
        self.assertEqual((box.length, box.width), (5, 1.8))
        self.assertAlmostEqual(box.z, 1.6)

    def test_vehicle_sized_rotated_rectangle(self):
        box = PerceivedBox(1, 0, 30, 20, 1.6, math.pi / 3, 5, 1.8)
        corners = box.corners()
        self.assertAlmostEqual(math.dist(corners[0], corners[1]), 5)
        self.assertAlmostEqual(math.dist(corners[1], corners[2]), 1.8)
        self.assertAlmostEqual(sum(p[0] for p in corners) / 4, 30)
        self.assertAlmostEqual(sum(p[1] for p in corners) / 4, 20)
        self.assertAlmostEqual(math.atan2(corners[1][1] - corners[0][1],
                                         corners[1][0] - corners[0][0]), math.pi / 3)

    def test_snapshot_holds_pose_and_heading_without_future_leak(self):
        overlay = self.overlay([self.row(.05), self.row(.1, observed_center_x_m=40)])
        self.assertEqual(overlay.boxes_at(.04), [])
        box = overlay.boxes_at(.09)[0]
        self.assertEqual((box.time_s, box.x), (.05, 30))
        self.assertAlmostEqual(box.heading, .25)
        self.assertEqual(overlay.boxes_at(.1)[0].x, 40)

    def test_missing_and_invalid_fresh_frames_clear_old_boxes(self):
        overlay = self.overlay([
            self.row(0), self.row(.05, visible=False, observation_valid=False,
                                  observed_center_x_m='', observed_center_y_m='', observed_range_m=''),
            self.row(.1), self.row(.15, observation_valid=False),
        ])
        self.assertEqual(len(overlay.boxes_at(.04)), 1)
        self.assertEqual(overlay.boxes_at(.07), [])
        self.assertEqual(len(overlay.boxes_at(.12)), 1)
        self.assertEqual(overlay.boxes_at(.18), [])

    def test_multiple_objects_snapshot_replaced_as_a_whole(self):
        for states in self.frames.values():
            states.append(states[0] | {'id': 2, 'length': 4})
        overlay = self.overlay([self.row(0), self.row(0, object_id=2), self.row(.05)])
        self.assertEqual([b.object_id for b in overlay.boxes_at(.04)], [1, 2])
        self.assertEqual([b.object_id for b in overlay.boxes_at(.06)], [1])

    def test_zero_error_still_has_box_in_error_mode(self):
        overlay = self.overlay([self.row(0, observed_center_x_m=10, observed_center_y_m=20)])
        self.assertEqual(len(overlay.boxes_at(0)), 1)

    def test_negative_nonfinite_and_ego_observations_not_drawn(self):
        for changes in [dict(observed_range_m=-1), dict(observed_center_x_m=float('nan')),
                        dict(object_id=0)]:
            with self.subTest(changes=changes):
                self.assertEqual(self.overlay([self.row(0, **changes)]).boxes_at(0), [])

    def test_ideal_modes_do_not_read_observations_or_alter_pixels(self):
        for mode in ('ideal', 'recorded_truth'):
            overlay = ObservationOverlay(self.out, {'uncertainty': mode}, self.frames, 0)
            self.assertFalse(overlay.enabled)
            data = b'unchanged video'
            self.assertIs(overlay.draw(data, 'bgr24', 960, 540, [], 0), data)

    def test_camera_projects_metres_at_correct_scale_and_orientation(self):
        camera = [0, 0, 1000, 1.5 * math.pi]
        centre = project_top_point((0, 0, 0), camera, 960, 540)
        east = project_top_point((5, 0, 0), camera, 960, 540)
        north = project_top_point((0, 1.8, 0), camera, 960, 540)
        scale = 540 / (2000 * math.tan(math.radians(.5)))
        self.assertAlmostEqual(centre[0], 480, places=3)
        self.assertAlmostEqual(centre[0] - east[0], 5 * scale, places=3)
        self.assertAlmostEqual(north[1] - centre[1], 1.8 * scale, places=3)

    def test_rgb_bgr_and_bottom_up_rows_are_preserved(self):
        overlay = self.overlay([self.row(0, observed_center_x_m=0, observed_center_y_m=0)])
        original = Image.new('RGB', (960, 540), (180, 60, 20))
        for pix, raw in [('rgb24', 'RGB'), ('bgr24', 'BGR')]:
            with self.subTest(pixel_format=pix):
                data = original.tobytes('raw', raw, 0, -1)
                rendered = overlay.draw(data, pix, 960, 540, [0, 0, 1000, 1.5 * math.pi], 0)
                image = Image.frombytes('RGB', (960, 540), rendered, 'raw', raw, 0, -1)
                self.assertEqual(image.getpixel((700, 500)), (180, 60, 20))
                # The physical rectangle is near the image centre, never flipped
                # to the wrong row or accidentally filled as an opaque vehicle.
                self.assertEqual(image.getpixel((480, 270)), (180, 60, 20))
                crop = image.crop((380, 205, 580, 335))
                self.assertGreater(list(crop.get_flattened_data()).count((12, 12, 12)), 100)

    def test_replay_meshes_fit_boxes_without_changing_dimensions(self):
        scene = self.out / 'video-scene.xosc'
        scene.write_text('<OpenSCENARIO><Entities><ScenarioObject name="car">'
                         '<Vehicle><BoundingBox><Dimensions length="5" width="1.8" height="1.6"/>'
                         '</BoundingBox><Properties><Property name="model_id" value="2"/>'
                         '<Property name="scaleMode" value="None"/></Properties></Vehicle>'
                         '</ScenarioObject></Entities></OpenSCENARIO>', encoding='utf-8')
        align_visual_models(scene)
        root = ET.parse(scene)
        self.assertEqual(root.find('.//Dimensions').attrib,
                         {'length': '5', 'width': '1.8', 'height': '1.6'})
        self.assertEqual(root.find('.//Property[@name="model_id"]').get('value'), '2')
        self.assertEqual(root.find('.//Property[@name="scaleMode"]').get('value'), 'ModelToBB')


if __name__ == '__main__':
    unittest.main()
