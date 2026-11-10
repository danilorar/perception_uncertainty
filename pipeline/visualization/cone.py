"""Render saved HiDrive motion with a dropout-gated sensor-cone illustration.

The cone visualizes the saved target detection flag. Its chosen range/FOV are
presentation geometry, not calibrated HiDrive sensor parameters. No controller
is rerun and no simulation CSV is changed.
"""
import ctypes as C
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess

import imageio_ffmpeg
from PIL import Image, ImageDraw, ImageFont

from .native import Engine
from .motion import (align_visual_models, build_scene, center, flag_at,
                     load_motion, project_top_point)


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def camera_for(states, width_m, tilt_deg, aspect):
    ego, target = states[:2]
    ex, ey = center(ego)
    tx, ty = center(target)
    forward = math.cos(ego['h']), math.sin(ego['h'])
    left = -forward[1], forward[0]
    x = (ex + tx)/2 + 3*forward[0] + .8*left[0]
    y = (ey + ty)/2 + 3*forward[1] + .8*left[1]
    height = width_m / (aspect * 2 * math.tan(math.radians(.5)))
    heading = math.pi/2 + ego['h'] - math.radians(tilt_deg)
    return [x, y, height, heading]


def body_polygon(state, camera, width, height):
    x, y = center(state)
    co, si = math.cos(state['h']), math.sin(state['h'])
    return [project_top_point((x + co*u - si*v, y + si*u + co*v, state['z'] + 1.0),
                              camera, width, height)
            for u, v in [(-state['length']/2, -state['width']/2),
                         (state['length']/2, -state['width']/2),
                         (state['length']/2, state['width']/2),
                         (-state['length']/2, state['width']/2)]]


def fonts(scale):
    font = Path('/System/Library/Fonts/Helvetica.ttc')
    def get(size):
        return (ImageFont.truetype(str(font), round(size*scale)) if font.is_file()
                else ImageFont.load_default(size=round(size*scale)))
    return get(19), get(15)


def annotate(data, pix, size, states, camera, detected, time_s, collision, radius, fov):
    width, height = size
    mode = 'BGR' if pix == 'bgr24' else 'RGB'
    frame = Image.frombytes('RGB', size, data, 'raw', mode, 0, -1).convert('RGBA')
    ego = states[0]
    if detected:
        ex, ey = center(ego)
        heading = ego['h']
        # Mount the illustrative cone at the ego's front bumper.
        apex = (ex + ego['length']/2 * math.cos(heading),
                ey + ego['length']/2 * math.sin(heading), ego['z'] + .12)
        angles = [heading + math.radians(-fov/2 + fov*k/80) for k in range(81)]
        arc = [(apex[0] + radius*math.cos(a), apex[1] + radius*math.sin(a), apex[2]) for a in angles]
        points = [project_top_point(p, camera, width, height) for p in [apex, *arc]]
        cone = Image.new('RGBA', size)
        draw = ImageDraw.Draw(cone)
        draw.polygon(points, fill=(0, 187, 210, 58))
        draw.line(points + points[:1], fill=(0, 156, 185, 205), width=max(2, width//600))
        # Keep the native vehicle meshes visible above the ground-plane cone.
        mask = ImageDraw.Draw(cone)
        for state in states:
            mask.polygon(body_polygon(state, camera, width, height), fill=(0, 0, 0, 0))
        frame = Image.alpha_composite(frame, cone)
    scale = width/1280
    font, small = fonts(scale)
    draw = ImageDraw.Draw(frame)
    label = 'Detection available' if detected else 'Detection dropout'
    color = (0, 132, 157) if detected else (186, 52, 48)
    box = (round(18*scale), round(18*scale), round(295*scale), round(82*scale))
    draw.rounded_rectangle(box, radius=round(8*scale), fill=(248, 250, 250, 245))
    draw.ellipse((round(31*scale), round(31*scale), round(43*scale), round(43*scale)), fill=color)
    draw.text((51*scale, 25*scale), label, fill=color, font=font)
    draw.text((31*scale, 53*scale), f't = {time_s:.2f} s', fill=(69, 82, 86), font=small)
    if collision:
        label = 'Collision'
        bounds = draw.textbbox((0, 0), label, font=font)
        tw = bounds[2] - bounds[0]
        draw.rounded_rectangle((width/2-tw/2-20*scale, height-69*scale,
                                width/2+tw/2+20*scale, height-23*scale),
                               radius=8*scale, fill=(174, 47, 42, 245))
        draw.text((width/2-tw/2, height-61*scale), label, fill='white', font=font)
    return frame.convert('RGB')


def render(args):
    source, output = args.run.resolve(), args.output.resolve()
    if output.exists():
        raise ValueError('Video output already exists; choose a new filename.')
    if args.fps < 1 or args.width < 320 or args.width % 16:
        raise ValueError('Use positive fps and a width >= 320 divisible by 16.')
    if args.view_width_m <= 0 or args.cone_range_m <= 0 or not 0 < args.cone_fov_deg < 180:
        raise ValueError('View and cone ranges must be positive; cone FOV must be between 0 and 180 degrees.')
    visual_runtime = args.visual_runtime or os.environ.get('TME180_VISUAL_RUNTIME')
    if visual_runtime is None:
        raise ValueError('Set TME180_VISUAL_RUNTIME or --visual-runtime to a graphics-enabled esmini 3.8.1 installation.')
    output.parent.mkdir(parents=True, exist_ok=True)
    reference, frames, flags = load_motion(source,args.target,args.detection_plugin)
    summary = read_json(source/'summary.json')
    if summary.get('status') not in ('completed','completed_with_warnings'):
        raise ValueError('Cone replay requires a completed simulation.')
    source_names = [name for name in ['config.json','scene.xosc','truth.csv','ego.csv','observations.csv',
                    'metrics.csv','perception_applied.tsv','summary.json'] if (source/name).is_file()]
    fingerprints = {name: sha(source/name) for name in source_names}
    times = sorted(frames)
    end = times[-1]
    size = args.width, round(args.width*9/16)
    camera = camera_for(frames[0.0], args.view_width_m, args.road_angle_deg, 16/9)
    scene = output.parent/'cone-replay.xosc'
    build_scene(scene, reference, frames[0.0], end)
    align_visual_models(scene,visual_runtime,frames[0.0][0]['name'],reference['target'])
    engine = Engine(visual_runtime)
    engine.lib.SE_AddCustomFixedTopCamera.argtypes = [C.c_double]*4
    engine.lib.SE_AddCustomFixedTopCamera.restype = C.c_int
    engine.lib.SE_SetCameraMode.argtypes = [C.c_int]
    engine.lib.SE_SetCameraMode.restype = C.c_int
    requested = [3.04, 3.40, 3.88, end] if args.preview else [k/args.fps for k in range(math.ceil(end*args.fps)+1)]
    sample_times = [min(times, key=lambda v: abs(v-min(t,end))) for t in requested]
    process = None
    timeline = []
    maximum_black_fraction = 0.0
    pending = output.with_suffix('.pending.mp4')
    try:
        # macOS pbuffer capture returns black frames; use a normal GL context.
        engine.init(scene, output.parent/'cone-replay.log', camera=camera,size=size)
        previous = 0.0
        if not args.preview:
            command = [imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error', '-f', 'rawvideo',
                       '-pix_fmt', 'rgb24', '-s', f'{size[0]}x{size[1]}', '-r', str(args.fps),
                       '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                       '-crf', '18', '-movflags', '+faststart', str(pending)]
            log = (output.parent/'cone-ffmpeg.log').open('w')
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                       stderr=log, creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
        for index, t in enumerate(sample_times):
            states = frames[t]
            camera = camera_for(states, args.view_width_m, args.road_angle_deg, 16/9)
            ident = engine.lib.SE_AddCustomFixedTopCamera(*camera)
            if ident < 0 or engine.lib.SE_SetCameraMode(ident):
                raise RuntimeError('Camera update failed')
            for state in states:
                engine.report(state)
            engine.step(t-previous)
            w,h,pix,data = engine.image()
            if (w,h) != size:
                raise RuntimeError('Unexpected capture size')
            raw_mode = 'BGR' if pix == 'bgr24' else 'RGB'
            capture = Image.frombytes('RGB', size, data, 'raw', raw_mode, 0, -1)
            black_fraction = capture.convert('L').histogram()[0] / (w*h)
            maximum_black_fraction = max(maximum_black_fraction, black_fraction)
            if black_fraction > .5:
                raise RuntimeError('Blank native capture; refusing to publish a black-background video')
            detected = flag_at(flags,t)
            collision = bool(summary['collision'] and t >= summary['collision_details']['time_s']-1e-8)
            frame = annotate(data,pix,size,states,camera,detected,t,collision,
                             args.cone_range_m,args.cone_fov_deg)
            if args.preview:
                frame.save(output.parent/f'preview_{t:.2f}.png')
            else:
                process.stdin.write(frame.tobytes())
            timeline.append({'frame':index,'video_time_s':index/args.fps,'simulation_time_s':t,
                             'cone_visible':detected,'camera':camera})
            previous=t
        if process:
            # One second at the actual first-contact state helps presentation playback.
            for _ in range(args.fps):process.stdin.write(frame.tobytes())
            process.stdin.close()
            if process.wait(timeout=60):raise RuntimeError('FFmpeg failed')
            log.close()
            pending.replace(output)
        if any(sha(source/name)!=value for name,value in fingerprints.items()):
            raise RuntimeError('Source simulation data changed')
        record = {'source_run':str(source),'output':str(output),'fps':args.fps,'size':size,
                  'simulation_end_s':end,'end_hold_s':1.0,'source_sha256':fingerprints,
                  'cone':{'range_m':args.cone_range_m,'fov_deg':args.cone_fov_deg,
                          'meaning':'Illustrative coverage; visibility follows the saved detection.missed flag.',
                          'target':reference['target'],'log_column':reference['detection_column'],
                          'calibration':'Display geometry only; not a fitted HiDrive sensor model.'},
                  'visual_runtime':str(Path(visual_runtime).resolve()),
                  'camera':{'view_width_m':args.view_width_m,'road_angle_deg':args.road_angle_deg,
                            'tracking':'midpoint of the two recorded vehicle centres'},
                  'capture_qa':{'maximum_black_pixel_fraction':maximum_black_fraction},
                  'timeline':timeline,'preview_only':args.preview}
        output.with_suffix('.json').write_text(json.dumps(record,indent=2)+'\n')
        print(json.dumps({k:v for k,v in record.items() if k!='timeline'},indent=2))
    finally:
        if process and process.poll() is None:
            process.kill();process.wait()
        engine.close()


