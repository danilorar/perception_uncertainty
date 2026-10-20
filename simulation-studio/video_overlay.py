"""Replay saved perception as vehicle-sized outlines; never resample noise."""

from bisect import bisect_right
import csv
import ctypes
from dataclasses import dataclass
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


NOISE_MODES = {'distance_ou_range', 'distance_gaussian_range', 'gaussian_range'}
LEGEND_TEXT = {
    'zh': ('黑框：感知车辆（与真车同尺寸）', '按观测周期更新；无有效观测时隐藏'),
    'en': ('Black outline: perceived vehicle (actual size)',
           'Updated at sensor ticks; hidden without valid observations'),
}


@dataclass(frozen=True)
class PerceivedBox:
    object_id: int
    time_s: float
    x: float
    y: float
    z: float
    heading: float
    length: float
    width: float

    def corners(self):
        co, si = math.cos(self.heading), math.sin(self.heading)
        return [
            (self.x + co * x - si * y, self.y + si * x + co * y, self.z)
            for x, y in [
                (-self.length / 2, -self.width / 2),
                (self.length / 2, -self.width / 2),
                (self.length / 2, self.width / 2),
                (-self.length / 2, self.width / 2),
            ]
        ]


def project_top_point(point, camera, width, height):
    """Match esmini 3.8.1's fixed top camera, including its tiny pitch tilt.

    Viewer::UpdateCameraFOV uses a 1-degree perspective lens, and
    RubberbandManipulator::CustomCamera stores position/rotation as floats.
    Return top-down pixel coordinates; Engine.image stores bottom-up rows.
    """
    cx, cy, cz, heading = [ctypes.c_float(v).value for v in camera]
    pitch = ctypes.c_float(math.pi / 2 - 1e-5).value
    dx, dy, dz = point[0] - cx, point[1] - cy, point[2] - cz
    co, si = math.cos(heading), math.sin(heading)
    forward_xy = co * dx + si * dy
    depth = math.cos(pitch) * forward_xy - math.sin(pitch) * dz
    if depth <= 0:
        raise ValueError('Video outline is behind the top camera.')
    scale = height / (2 * math.tan(math.radians(.5)) * depth)
    right = si * dx - co * dy
    up = math.sin(pitch) * forward_xy + math.cos(pitch) * dz
    return width / 2 + right * scale, height / 2 - up * scale


class ObservationOverlay:
    def __init__(self, out, config, frames, ego_id, *, language='zh'):
        if language not in LEGEND_TEXT:
            raise ValueError(f'Unsupported video language: {language}')
        self.language = language
        self.enabled = config['uncertainty'] in NOISE_MODES
        self.snapshots = {}
        self.times = []
        self.drawn_frames = 0
        self.drawn_boxes = 0
        self.font = self.small_font = None
        if not self.enabled:
            return
        truth_times = sorted(frames)
        with (Path(out) / 'observations.csv').open(encoding='utf-8', newline='') as file:
            for row in csv.DictReader(file):
                t = float(row['time_s'])
                # A fresh empty/invalid frame clears the entire old snapshot.
                snapshot = self.snapshots.setdefault(t, [])
                ident = int(row['object_id'])
                valid = all(str(row.get(key, '')).lower() in ('true', '1')
                            for key in ('visible', 'observation_valid'))
                if not valid or ident == ego_id:
                    continue
                values = [float(row[key]) for key in
                          ('observed_center_x_m', 'observed_center_y_m', 'observed_range_m')]
                if not all(math.isfinite(v) for v in values) or values[2] < 0:
                    continue
                index = bisect_right(truth_times, t + 1e-9) - 1
                if index < 0 or abs(truth_times[index] - t) > 1e-7:
                    raise ValueError('Saved observation has no matching truth timestamp.')
                state = next(s for s in frames[truth_times[index]] if s['id'] == ident)
                # Observed x/y are already box centres: do NOT add centre offsets
                # again. Heading and dimensions are ideal measured attributes.
                snapshot.append(PerceivedBox(
                    ident, t, values[0], values[1],
                    state['z'] + state['centerOffsetZ'] + state['height'] / 2,
                    state['h'], state['length'], state['width'],
                ))
        self.times = sorted(self.snapshots)

    def boxes_at(self, time_s):
        index = bisect_right(self.times, time_s + 1e-9) - 1
        return self.snapshots[self.times[index]] if index >= 0 else []

    def extent_points(self):
        return [point for boxes in self.snapshots.values()
                for box in boxes for point in box.corners()]

    def draw(self, data, pixel_format, width, height, camera, time_s):
        if not self.enabled:
            return data
        raw_mode = {'rgb24': 'RGB', 'bgr24': 'BGR'}[pixel_format]
        image = Image.frombytes('RGB', (width, height), data, 'raw', raw_mode, 0, -1)
        draw = ImageDraw.Draw(image)
        boxes = self.boxes_at(time_s)
        for box in boxes:
            points = [project_top_point(point, camera, width, height) for point in box.corners()]
            # White edging keeps the black outline readable on the dark road.
            # Both strokes follow the same physical vehicle-sized rectangle.
            draw.line(points + points[:1], fill=(250, 250, 250), width=4, joint='curve')
            draw.line(points + points[:1], fill=(12, 12, 12), width=2, joint='curve')
        self.drawn_frames += bool(boxes)
        self.drawn_boxes += len(boxes)
        if self.font is None:
            # Font availability must never select the legend language.
            font = Path('C:/Windows/Fonts') / ('msyh.ttc' if self.language == 'zh' else 'segoeui.ttf')
            self.font = (ImageFont.truetype(str(font), 16) if font.exists()
                         else ImageFont.load_default(size=16))
            self.small_font = (ImageFont.truetype(str(font), 12) if font.exists()
                               else ImageFont.load_default(size=12))
        label, detail = LEGEND_TEXT[self.language]
        panel_width = math.ceil(max(draw.textlength(label, font=self.font),
                                    draw.textlength(detail, font=self.small_font))) + 54
        draw.rounded_rectangle((12, 12, panel_width + 12, 66), radius=7, fill=(246, 248, 250))
        draw.rectangle((24, 25, 44, 35), outline=(12, 12, 12), width=2)
        draw.text((54, 18), label, font=self.font, fill=(12, 12, 12))
        draw.text((24, 43), detail, font=self.small_font, fill=(75, 81, 91))
        return image.tobytes('raw', raw_mode, 0, -1)

    def metadata(self):
        return {
            'enabled': self.enabled,
            'legend_language': self.language,
            'legend_text': LEGEND_TEXT[self.language] if self.enabled else None,
            'style': 'black outline with white edging',
            'position': 'saved observed_center_x_m / observed_center_y_m',
            'size': 'target bounding-box length and width in metres',
            'heading': 'target heading at the observation timestamp',
            'timing': 'hold the latest complete sensor snapshot; no interpolation or prediction',
            'missing': 'hide on undetected or invalid observations; no truth-position fallback',
            'frames_with_boxes': self.drawn_frames,
            'boxes_drawn': self.drawn_boxes,
        }
