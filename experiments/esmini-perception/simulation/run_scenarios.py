#!/usr/bin/env python3
"""Run the three original scenarios and verify their prescribed trajectory replay.

Author: Zhuo Ma
This is a replay/data check, not an AEB experiment. HiDriveController is disabled.
Each scenario gets CSV, recording, logs and source-file integrity checks.
"""
import argparse
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from paths import DATA, ESMINI, RESULTS, LATEST, SCENARIO_IDS, executable_path

IDS = SCENARIO_IDS


def load_rows(path):
    """Skip the native CSV preamble and read the actual telemetry header."""
    lines = path.read_text().splitlines()
    start = next(i for i, line in enumerate(lines) if line.startswith('Index [-]'))
    return list(csv.DictReader(lines[start:], skipinitialspace=True))


def replay(result, identifier):
    """Open an existing recording; this does not rerun the vehicle simulation."""
    folder = result / f'C_original_{identifier}'
    subprocess.run([
        str(executable_path("replayer")), '--file', str(folder / 'sim.dat'),
        '--path', str(DATA), str(ESMINI / 'resources'),
        '--window', '60', '60', '1200', '700', '--camera_mode', 'top',
        '--collision', 'continue', '--quit_at_end',
        '--logfile_path', str(folder / 'replay.log'),
    ], cwd=folder, check=True)


def run():
    """Run all three cases and record trajectory, timestep and source integrity checks."""
    # Results must never be created in the original-data directory.
    result = RESULTS / datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    result.mkdir(parents=True)
    summaries = []
    for identifier in IDS:
        source = DATA / f'C_original_{identifier}.xosc'
        road = source.with_suffix('.xodr')
        folder = result / source.stem
        folder.mkdir()
        hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (source, road)}
        command = [str(executable_path()), '--headless',
                   '--fixed_timestep', '0.01', '--seed', '0', '--traj_filter', '0',
                   '--disable_controllers', '--osc', str(source),
                   '--path', str(DATA), str(ESMINI / 'resources'), '--collision',
                   '--csv_logger', str(folder / 'states.csv'),
                   '--record', str(folder / 'sim.dat'),
                   '--logfile_path', str(folder / 'run.log')]
        (folder / 'command.json').write_text(json.dumps(command, indent=2) + '\n')
        process = subprocess.run(command, cwd=folder, capture_output=True, text=True, timeout=60)
        (folder / 'console.log').write_text(process.stdout + process.stderr)
        if process.returncode:
            raise RuntimeError(f'{source.name}: exit {process.returncode}; see {folder}')
        log = (folder / 'run.log').read_text()
        rows = load_rows(folder / 'states.csv')
        collision_rows = [r for r in rows if r['#1 collision_ids'].strip()]
        collision = collision_rows[0] if collision_rows else None
        errors = {}
        tree = ET.parse(source)
        for i, polyline in enumerate(tree.findall('.//Polyline'), 1):
            vertices = {round(float(v.get('time')), 2): v.find('.//WorldPosition')
                        for v in polyline}
            errors[f'object_{i}'] = max(
                math.hypot(float(r[f'#{i} World_Position_X [m]']) - float(vertices[t].get('x')),
                           float(r[f'#{i} World_Position_Y [m]']) - float(vertices[t].get('y')))
                for r in rows if (t := round(float(r['TimeStamp [s]']), 2)) in vertices)
        actual_times = [float(r['TimeStamp [s]']) for r in rows]
        steps_valid = all(abs(b - a - 0.01) < 1e-7 for a, b in zip(actual_times, actual_times[1:]))
        unchanged = hashes == {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (source, road)}
        engine_errors = [line for line in log.splitlines() if '[error]' in line]
        known_road_errors = [line for line in engine_errors if 'Last geometry of road 0 does not end at road end' in line]
        road_length = float(ET.parse(road).find('road').get('length'))
        maximum_road_s = max(float(r[f'#{i} Distance_Travelled_Along_Road_Segment [m]'])
                             for r in rows for i in (1, 2))
        checks = {
            'exit_zero': process.returncode == 0,
            'stop_trigger_reached': 'storyBoard runningState -> stopTransition -> completeState' in log,
            'road_loaded': f'Loaded OpenDRIVE: {road}' in log,
            'timestep_0_01_s': steps_valid,
            'trajectory_max_error_below_0_1_mm': max(errors.values()) < 0.0001,
            'source_files_unchanged': unchanged,
            'no_unexpected_engine_errors': engine_errors == known_road_errors,
            'trajectory_stays_before_road_end': maximum_road_s < road_length - 1.0,
        }
        summary = dict(scenario=source.name, esmini_version='3.8.1',
                       mode='Prescribed trajectory replay; all custom controllers disabled',
                       dt_s=0.01, trajectory_filter_radius_m=0, seed=0,
                       exit_code=process.returncode, rows=len(rows),
                       end_time_s=actual_times[-1],
                       first_collision_s=float(collision['TimeStamp [s]']) if collision else None,
                       first_collision_speeds_kmh={f'object_{i}': float(collision[f'#{i} Current_Speed [m/s]']) * 3.6
                                                   for i in (1, 2)} if collision else None,
                       max_trajectory_xy_error_m=errors,
                       road_declared_length_m=road_length, maximum_road_s_m=maximum_road_s,
                       known_source_road_errors=known_road_errors,
                       source_sha256=hashes, checks=checks,
                       warnings=[line for line in log.splitlines() if '[warn]' in line])
        (folder / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
        summaries.append(summary)
        print(f'{source.name}: end={actual_times[-1]:.2f} s; collision={summary["first_collision_s"]} s; checks={all(checks.values())}', flush=True)
        if not all(checks.values()):
            raise RuntimeError(f'Validation failed: {folder / "summary.json"}')
    (result / 'summary.json').write_text(json.dumps(summaries, ensure_ascii=False, indent=2) + '\n')
    report = ['# Data 三场景仿真结果', '',
              '使用 esmini 3.8.1，以 0.01 s 固定步长运行；轨迹点过滤关闭，保留原始采样点。', '',
              '原文件引用的 HiDriveController 不受当前官方 esmini 支持，初次直接运行时引擎回退到默认控制。正式结果明确使用 `--disable_controllers`，仅回放两车预设轨迹；没有接入 AEB 或感知误差模型。', '',
              '原始 XOSC/XODR 未改动，通过搜索路径加载 Data 中的道路文件。', '',
              '| 场景 | 结束时间 / s | 首次包围盒碰撞 / s | 验证 |',
              '|---|---:|---:|---|']
    for s in summaries:
        report.append(f'| {s["scenario"]} | {s["end_time_s"]:.2f} | {s["first_collision_s"]:.2f} | 通过 |')
    report += ['', '每个场景目录包含 `states.csv`（位置、速度、碰撞标志）、`sim.dat`（esmini 回放录像）、`run.log`、`console.log`、`command.json` 和 `summary.json`。', '',
               '验证包括：正常退出、道路加载、结束条件、固定时间步、两车位置与原始轨迹逐点核对，以及原始文件校验值。', '',
               '道路数据问题：1554254 和 1554431 的 XODR 中，最后一段几何终点比声明道路长度长约 1 m，引擎报告 error 后仍正常加载。保留该原始问题；本次车辆均未到达道路末端，且两车全部采样位置与原轨迹吻合。该验证仅覆盖本次轨迹回放，不表示道路文件完全合规。', '',
               '碰撞是预设 5.0 m × 1.8 m 车辆包围盒重叠；esmini 继续执行轨迹，所以碰撞后车辆可能穿过彼此。这不是碰撞动力学、车辆损伤或真实事故冲击速度的验证。', '',
               '通过 Data 中的 `Replay 1549178.command` 等入口观看三维回放。空格播放/暂停，Tab 切换车辆，鼠标缩放/旋转，Esc 关闭。', '',
               '再次双击 `Run all scenarios.command` 会创建新的时间戳结果目录。']
    (result / '仿真结果.md').write_text('\n'.join(report) + '\n')
    LATEST.write_text(str(result) + '\n', encoding='utf-8')
    print(f'RESULT_DIR={result}', flush=True)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--view', choices=(*IDS, 'all'))
    args = parser.parse_args()
    if args.view:
        if LATEST.exists():
            folder_name = LATEST.read_text(encoding='utf-8').strip()
            result = RESULTS / folder_name
        else:
            result = run()

        identifiers = IDS if args.view == 'all' else (args.view,)
        for identifier in identifiers:
            replay(result, identifier)
    else:
        run()
