#!/usr/bin/env python3
"""Render native esmini frames and verify poses, speeds and collisions at every step.

Author: Zhuo Ma
Optional visualization utility; requires Pillow. Core simulations do not.
"""
import argparse
import ctypes as C
import json
import math
from pathlib import Path
import shutil
import tempfile

from PIL import Image, ImageDraw, ImageFont
from paths import DATA, ESMINI, RESULTS, LATEST, library_path
from esmini_api import State
from run_scenarios import IDS, load_rows


class NativeImage(C.Structure):
    _fields_ = [('width', C.c_int), ('height', C.c_int), ('pixelSize', C.c_int),
                ('pixelFormat', C.c_int), ('data', C.POINTER(C.c_ubyte))]




def save_gif(destination, frames, durations):
    # Encode on the local disk before copying to the cloud-synced result folder.
    with tempfile.TemporaryDirectory(prefix='data-esmini-gif-') as tmp:
        path = Path(tmp) / destination.name
        frames[0].save(path, save_all=True, append_images=frames[1:],
                       duration=durations, loop=0, optimize=False)
        shutil.copyfile(path, destination)


def bind():
    lib = C.CDLL(str(library_path()))
    for name, restype, argtypes in [
        ('SE_InitWithArgs', C.c_int, [C.c_int, C.POINTER(C.c_char_p)]),
        ('SE_SaveImagesToRAM', C.c_int, [C.c_bool]),
        ('SE_FetchImage', C.c_int, [C.POINTER(NativeImage)]),
        ('SE_StepDT', C.c_int, [C.c_double]),
        ('SE_GetSimulationTime', C.c_double, []),
        ('SE_GetQuitFlag', C.c_int, []),
        ('SE_GetObjectState', C.c_int, [C.c_int, C.POINTER(State)]),
        ('SE_GetObjectNumberOfCollisions', C.c_int, [C.c_int]),
        ('SE_AddCustomFixedCamera', C.c_int, [C.c_double]*5),
        ('SE_SetCameraMode', C.c_int, [C.c_int]),
        ('SE_Close', None, []),
    ]:
        function = getattr(lib, name)
        function.restype, function.argtypes = restype, argtypes
    return lib


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result-dir', type=Path)
    parser.add_argument('--scenario', choices=IDS)
    args = parser.parse_args()
    if args.result_dir is not None:
        result = args.result_dir.resolve()
    else:
        folder_name = LATEST.read_text(encoding='utf-8').strip()
        result = RESULTS / folder_name
    font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 20)
    preview_frames = []
    lib = bind()
    for identifier in (args.scenario,) if args.scenario else IDS:
        folder = result / f'C_original_{identifier}'
        rows = load_rows(folder / 'states.csv')
        xs = [float(r[f'#{i} World_Position_X [m]']) for r in rows for i in (1, 2)]
        ys = [float(r[f'#{i} World_Position_Y [m]']) for r in rows for i in (1, 2)]
        cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        heading = float(rows[0]['#1 World_Heading_Angle [rad]'])
        along = [x*math.cos(heading)+y*math.sin(heading) for x,y in zip(xs,ys)]
        lateral = [-x*math.sin(heading)+y*math.cos(heading) for x,y in zip(xs,ys)]
        height = max((max(along)-min(along)+24) / 0.95,
                     (max(lateral)-min(lateral)+18) / 0.48, 60)
        camera_heading = heading + math.pi/2
        render_dir = folder / 'visualization'
        render_dir.mkdir(exist_ok=True)
        command = ['esmini', '--osc', str(DATA / f'C_original_{identifier}.xosc'),
                       '--path', str(DATA), str(ESMINI / 'resources'),
                       '--window', '60', '60', '1200', '640', '--SingleThreaded',
                       '--ground_plane', 'off', '--collision', '--fixed_timestep', '0.01',
                       '--traj_filter', '0', '--seed', '0', '--disable_controllers',
                       '--info_text', '0', '--disable_shadows', '--disable_stdout', '--generate_without_textures',
                       '--logfile_path', str(render_dir / 'render.log')]
        (render_dir / 'command.json').write_text(json.dumps(command, indent=2) + '\n')
        argv = (C.c_char_p * len(command))(*[a.encode() for a in command])
        if lib.SE_SaveImagesToRAM(True) != 0:
            raise RuntimeError('Cannot enable frame capture')
        frames, timestamps = [], []
        maximum_state_error = 0.0
        try:
            if lib.SE_InitWithArgs(len(command), argv) != 0:
                raise RuntimeError(f'Native render initialization failed: {identifier}')
            camera = lib.SE_AddCustomFixedCamera(cx, cy, height, camera_heading, math.pi/2)
            if camera < 0 or lib.SE_SetCameraMode(camera) != 0:
                raise RuntimeError('Cannot select fixed overhead camera')
            for step in range(1, 1002):
                if lib.SE_StepDT(0.01) != 0:
                    raise RuntimeError('Simulation step failed')
                t = lib.SE_GetSimulationTime()
                row = rows[step]
                if abs(t-float(row['TimeStamp [s]'])) > 1e-7:
                    raise RuntimeError(f'Render clock mismatch: {identifier} at step {step}')
                for i in (1, 2):
                    state = State()
                    if lib.SE_GetObjectState(i-1, C.byref(state)) != 0:
                        raise RuntimeError('Cannot read rendered vehicle state')
                    for attribute, column in [('x','World_Position_X [m]'),('y','World_Position_Y [m]'),
                                              ('z','World_Position_Z [m]'),('h','World_Heading_Angle [rad]'),
                                              ('speed','Current_Speed [m/s]')]:
                        error = abs(getattr(state, attribute)-float(row[f'#{i} {column}']))
                        maximum_state_error = max(maximum_state_error, error)
                        if error > 0.00001:
                            raise RuntimeError(f'Render state mismatch: {identifier}, step {step}, object {i}, {attribute}: {error}')
                    if bool(lib.SE_GetObjectNumberOfCollisions(i-1)) != bool(row[f'#{i} collision_ids'].strip()):
                        raise RuntimeError(f'Render collision mismatch: {identifier} at {t}')
                if step % 10 and step not in (1, 1001):
                    continue
                captured = NativeImage()
                if lib.SE_FetchImage(C.byref(captured)) != 0 or not captured.data:
                    raise RuntimeError(f'No native frame at {t}')
                if captured.pixelSize != 3 or captured.pixelFormat not in (0x1907, 0x80E0):
                    raise RuntimeError('Unexpected native pixel format')
                raw = C.string_at(captured.data, captured.width * captured.height * captured.pixelSize)
                order = 'RGB' if captured.pixelFormat == 0x1907 else 'BGR'
                native = Image.frombytes('RGB', (captured.width, captured.height), raw, 'raw', order).transpose(Image.Transpose.FLIP_TOP_BOTTOM)
                native = native.resize((1200, 640), Image.Resampling.LANCZOS)
                canvas = Image.new('RGB', (native.width, native.height + 45), '#102032')
                canvas.paste(native, (0, 45))
                draw = ImageDraw.Draw(canvas)
                draw.text((16, 11), f'{identifier}  |  t = {t:.2f} s  |  esmini trajectory replay  |  HiDrive / AEB inactive', font=font, fill='white')
                frames.append(canvas)
                timestamps.append(round(t, 8))
                if step == 1:
                    canvas.save(folder / 'native-preview.png')
        finally:
            lib.SE_Close()
        durations = [max(10, round((b-a)*1000)) for a,b in zip(timestamps,timestamps[1:])] + [1200]
        save_gif(folder / 'simulation.gif', frames, durations)
        collision_t = json.loads((folder / 'summary.json').read_text())['first_collision_s']
        index = min(range(len(timestamps)), key=lambda i: abs(timestamps[i]-collision_t))
        frames[index].save(folder / 'native-collision.png')
        preview_frames.append(frames)
        (folder / 'render_summary.json').write_text(json.dumps({
            'renderer': 'esmini native library; SE_FetchImage', 'source': str(DATA / f'C_original_{identifier}.xosc'),
            'simulation_dt_s': 0.01, 'nominal_frame_step_s': 0.1,
            'frames': len(frames), 'state_and_collision_checks_passed': True,
            'checked_time_steps': 1001, 'max_numeric_state_error': maximum_state_error,
            'frame_times_s': timestamps, 'camera': [cx,cy,height,camera_heading,math.pi/2],
        }, indent=2) + '\n')
        print(f'{identifier}: {len(frames)} frames; telemetry identical -> {folder / "simulation.gif"}', flush=True)
    if args.scenario:
        return
    combined = []
    for index in range(min(map(len, preview_frames))):
        tiles = [frames[index].resize((720, 411), Image.Resampling.LANCZOS) for frames in preview_frames]
        canvas = Image.new('RGB', (720, 1233), '#102032')
        for row, tile in enumerate(tiles):
            canvas.paste(tile, (0, row * 411))
        combined.append(canvas)
    save_gif(result / 'simulation-overview.gif', combined, durations)
    combined[50].save(result / 'simulation-overview.png')
    print(f'COMBINED={result / "simulation-overview.gif"}', flush=True)


if __name__ == '__main__':
    main()
