"""Render the actual saved run through esmini; encode an ordinary MP4."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import imageio_ffmpeg
from engine import Engine
from scenario import build_scene
from settings import write_json, sha256
from video_overlay import NOISE_MODES, ObservationOverlay


def align_visual_models(scene):
    """Fit replay-only meshes to the physical boxes used by perception.

    Bundled meshes have their own rear-axle origins and dimensions. Without
    ModelToBB, a zero-error outline can appear displaced from the visible car.
    This edits only video-scene.xosc, never the simulation or source scenario.
    """
    tree = ET.parse(scene)
    for body in tree.findall('./Entities/ScenarioObject/*'):
        if body.tag not in ('Vehicle', 'Pedestrian', 'MiscObject'):
            continue
        props = body.find('Properties')
        if props is None:
            props = ET.SubElement(body, 'Properties')
        scale = props.find("Property[@name='scaleMode']")
        if scale is None:
            scale = ET.SubElement(props, 'Property', name='scaleMode')
        scale.set('value', 'ModelToBB')
    ET.indent(tree)
    tree.write(scene, encoding='utf-8', xml_declaration=True)


def export(out, *, language=None):
    """Export both UI languages; ideal videos need only one text-free file."""
    out = Path(out).resolve()
    config = json.loads((out / 'config.json').read_text(encoding='utf-8'))
    languages = ([language] if language else
                 ['zh', 'en'] if config['uncertainty'] in NOISE_MODES else ['zh'])
    for index, variant in enumerate(languages):
        _export_language(out, variant, .8 + .19 * index / len(languages), .19 / len(languages))


def _export_language(out, language, progress_start, progress_span):
    out = Path(out).resolve()
    os.chdir(out)
    config = json.loads((out / 'config.json').read_text(encoding='utf-8'))
    metadata = json.loads((out / 'reference.json').read_text(encoding='utf-8'))
    base = {s['id']: s for s in metadata['objects']}
    frames = {}
    with (out / 'truth.csv').open(encoding='utf-8') as f:
        for row in csv.DictReader(f):
            t = float(row['time_s'])
            frames.setdefault(t, []).append(
                {
                    **base[int(row['object_id'])],
                    'id': int(row['object_id']),
                    **{
                        k: float(row[field])
                        for k, field in [
                            ('x', 'x_m'),
                            ('y', 'y_m'),
                            ('z', 'z_m'),
                            ('h', 'heading_rad'),
                            ('p', 'pitch_rad'),
                            ('r', 'roll_rad'),
                            ('speed', 'speed_mps'),
                            ('vx', 'vx_mps'),
                            ('vy', 'vy_mps'),
                        ]
                    },
                    **{
                        key: float(row[field])
                        for key, field in [('length', 'length_m'), ('width', 'width_m'),
                                           ('height', 'height_m'),
                                           ('centerOffsetX', 'center_offset_x_m'),
                                           ('centerOffsetY', 'center_offset_y_m'),
                                           ('centerOffsetZ', 'center_offset_z_m')]
                        if row.get(field)
                    },
                }
            )
    times = sorted(frames)
    last = times[-1]
    overlay = ObservationOverlay(out, config, frames, metadata.get('ego_id'), language=language)
    suffix = '.en' if language == 'en' else ''
    video_path = out / f'video{suffix}.mp4'
    # Publish a variant only when encoding succeeds, so preview/download never
    # selects a partially written MP4.
    pending_video = out / f'video{suffix}.mp4.tmp'
    engine = Engine()
    process = None
    # Sampling existing frames preserves the simulation time step and results.
    fps = 25
    sample_times = [
        min(times, key=lambda v: abs(v - i / fps)) for i in range(math.ceil(last * fps) + 1)
    ]
    positions = [s for states in frames.values() for s in states]
    # Keep even large range errors visible in the fixed camera's full-run view.
    positions += [{'x': p[0], 'y': p[1]} for p in overlay.extent_points()]
    xmin, xmax = min(s['x'] for s in positions) - 5, max(s['x'] for s in positions) + 7
    ymin, ymax = min(s['y'] for s in positions) - 7, max(s['y'] for s in positions) + 7
    # esmini 3.8.1 simulates orthographic top view with a 1-degree FOV.
    # See ViewerBase/viewer.cpp ORTHO_FOV, rather than a normal 30-degree lens.
    height = 0.55 * max(ymax - ymin, (xmax - xmin) / (960 / 540), 20) / math.tan(math.radians(0.5))
    camera = [(xmin + xmax) / 2, (ymin + ymax) / 2, height, 1.5 * math.pi]
    # Replay saved truth with external controllers for BOTH vehicles. The live
    # experiment scene now contains a native target trajectory, which must not
    # override the saved target poses during video rendering.
    initial = [base[s['id']] | s for s in frames[times[0]]]
    replay_scene = out / f'video-scene{suffix}.xosc'
    build_scene(replay_scene, metadata, initial, last)
    align_visual_models(replay_scene)
    try:
        engine.init(replay_scene, out / f'video-esmini{suffix}.log', video=True, camera=camera)
        previous = 0
        command = None
        with (out / f'ffmpeg{suffix}.log').open('wb') as log:
            for n, t in enumerate(sample_times):
                # Replay saved motion without rerunning perception or AEB decisions.
                for s in frames[t]:
                    engine.report(s)
                engine.step(t - previous)
                # Read rendered pixels for MP4 encoding with FFmpeg.
                width, height, pix, data = engine.image()
                # Draw outlines from saved full snapshots at the same positions used by control.
                data = overlay.draw(data, pix, width, height, camera, t)
                if process is None:
                    command = [
                        imageio_ffmpeg.get_ffmpeg_exe(),
                        '-y',
                        '-loglevel',
                        'warning',
                        '-f',
                        'rawvideo',
                        '-pix_fmt',
                        pix,
                        '-s',
                        f'{width}x{height}',
                        '-r',
                        str(fps),
                        '-i',
                        'pipe:0',
                        '-vf',
                        'vflip',
                        '-an',
                        '-c:v',
                        'libx264',
                        '-pix_fmt',
                        'yuv420p',
                        '-crf',
                        '21',
                        '-movflags',
                        '+faststart',
                        '-f',
                        'mp4',
                        str(pending_video),
                    ]
                    process = subprocess.Popen(
                        command,
                        stdin=subprocess.PIPE,
                        stdout=subprocess.DEVNULL,
                        stderr=log,
                        creationflags=subprocess.CREATE_NO_WINDOW,
                    )
                process.stdin.write(data)
                previous = t
                if n % 25 == 0:
                    write_json(
                        out / 'progress.json',
                        {'stage': 'video', 'fraction': progress_start + progress_span * n / len(sample_times)},
                    )
            process.stdin.close()
            if process.wait(timeout=60) != 0:
                raise RuntimeError('FFmpeg 视频编码失败')
        pending_video.replace(video_path)
        write_json(
            out / f'video_metadata{suffix}.json',
            {
                'language': language,
                'file': video_path.name,
                'fps': fps,
                'width': width,
                'height': height,
                'frames': len(sample_times),
                'simulation_end_s': last,
                'camera': camera,
                'renderer_sha256': sha256(__file__),
                'overlay_sha256': sha256(Path(__file__).with_name('video_overlay.py')),
                'perception_overlay': overlay.metadata(),
                'visual_model_scaling': 'ModelToBB (replay only)',
                'method': (
                    'esmini offscreen render of saved truth; '
                    'nearest saved simulation sample at 25 fps; '
                    'saved perception outlines for range-error modes'
                ),
                'ffmpeg_command': command,
            },
        )
    finally:
        if process and process.poll() is None:
            process.kill()
            process.wait()
        engine.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('out')
    parser.add_argument('--language', choices=['zh', 'en'], help='Regenerate only one video language')
    args = parser.parse_args()
    export(args.out, language=args.language)
