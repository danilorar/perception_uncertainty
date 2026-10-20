"""Configuration contract shared by the UI, single runs and batch runs."""

from pathlib import Path
import hashlib
import json
import math
import os

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent
RUNTIME = PROJECT / 'esmini-lab/runtime/esmini-full-v3.8.1'
SOURCE = PROJECT / 'esmini-lab/sources/vectorgrp-OSC-NCAP-scenarios-15365d1'
RUNS = ROOT / 'runs'
REFERENCES = ROOT / 'references'
VERSION = '0.12.0'
TEAM_BASELINE = 'feature/esmini-perception@cdb9639e6de21fc922d6f2f0ca9a743ff1956bd7'
# Isolated esmini 3.8.1 build containing the supplied (opaque) HiDrive controller,
# ported by the user to the esmini 3 API (the 2.40 original stays in 'AEB Controler').
HIDRIVE_RUNTIME = PROJECT / 'esmini-lab/runtime/esmini-hidrive-v3.8.1'
HIDRIVE_SOURCE = PROJECT.parent / 'AEB Controler (esmini 3.8.1)'
LEGACY_SCENARIOS = {
    'ccrs': {
        'title': 'CCRs · 静止前车',
        'category': 'car',
        'file': 'CCRs.xosc',
        'variation': 'CCRs_50kph.xosc',
        'speed': 50,
        'road': '直路',
        'objects': '固定目标车辆',
        'description': '自车接近静止前车；固定官方 50 km/h 案例中的目标位置。',
    },
    'cpna': {
        'title': 'CPNA · 行人横穿',
        'category': 'pedestrian',
        'file': 'CPNA.xosc',
        'speed': 30,
        'road': '直路',
        'objects': '固定成年行人轨迹',
        'description': '成年行人从近侧横穿；固定基准运行中的行人轨迹和出发时序。',
    },
    'cccscp': {
        'title': 'CCCscp · 路口交叉来车',
        'category': 'car',
        'file': 'CCCscp.xosc',
        'speed': 20,
        'road': '十字路口',
        'objects': '固定来车及遮挡车辆',
        'description': '自车直行通过路口；其他车辆全部按保存的基准数据回放。',
    },
    'cpfa': {
        'title': 'CPFA · 行人远侧横穿',
        'category': 'pedestrian',
        'file': 'CPNA.xosc',
        'variation': 'CPFA_50_50kph.xosc',
        'speed': 50,
        'road': '直路',
        'objects': '固定成年行人轨迹',
        'description': '成年行人从远侧横穿；使用官方 50 km/h、50% 碰撞位置参数集。',
    },
    'cpla': {
        'title': 'CPLA · 行人沿道路行走',
        'category': 'pedestrian',
        'file': 'CBLA.xosc',
        'variation': 'CPLA_50_50kph.xosc',
        'speed': 50,
        'road': '直路',
        'objects': '固定同向行人轨迹',
        'description': '自车接近沿道路同向行走的成年行人；行人按原版时序起步至 5 km/h。',
    },
    'cbna': {
        'title': 'CBNA · 自行车近侧横穿',
        'category': 'bicycle',
        'file': 'CBNA.xosc',
        'variation': 'CBNA_50_50kph.xosc',
        'speed': 50,
        'road': '十字路口',
        'objects': '固定骑行者及停放车辆',
        'description': '骑行者在路口从近侧横穿自车路径；保留原版停放车辆及 50 km/h 参数集。',
    },
    'cbla': {
        'title': 'CBLA · 自行车沿道路骑行',
        'category': 'bicycle',
        'file': 'CBLA.xosc',
        'variation': 'CBLA_50_50kph.xosc',
        'speed': 50,
        'road': '直路',
        'objects': '固定同向骑行者轨迹',
        'description': '自车接近沿道路同向骑行的自行车；骑行者按原版时序起步至 15 km/h。',
    },
    'cmrs': {
        'title': 'CMRs · 静止摩托车',
        'category': 'motorbike',
        'file': 'CCRs.xosc',
        'variation': 'CMRs_50kph.xosc',
        'speed': 50,
        'road': '直路',
        'objects': '固定静止摩托车',
        'description': '自车接近前方静止摩托车；使用官方摩托车类型、尺寸和位置。',
    },
    'cmrb': {
        'title': 'CMRb · 前方摩托车制动',
        'category': 'motorbike',
        'file': 'CCRs.xosc',
        'variation': 'CMRb_50kph.xosc',
        'speed': 50,
        'road': '直路',
        'objects': '固定摩托车制动轨迹',
        'description': '前方摩托车与自车均以 50 km/h 起步，摩托车按原版时序制动。',
    },
    'ccrm': {
        'title': 'CCRm · 移动前车',
        'category': 'car',
        'file': 'CCRs.xosc',
        'variation': 'CCRm_50kph.xosc',
        'speed': 50,
        'road': '直路',
        'objects': '固定低速前车轨迹',
        'description': '自车以 50 km/h 接近前方以 20 km/h 同向行驶的车辆。',
    },
    'ccrb': {
        'title': 'CCRb · 前车制动',
        'category': 'car',
        'file': 'CCRs.xosc',
        'variation': 'CCRb_50kph.xosc',
        'speed': 50,
        'road': '直路',
        'objects': '固定前车制动轨迹',
        'description': '前车与自车均以 50 km/h 起步，前车按原版时序制动。',
    },
}
SCENARIOS = {
    'gidas_'
    + number: {
        'title': 'GIDAS ' + number + ' · 追尾场景',
        'category': 'recorded',
        'file': 'C_original_' + number + '.xosc',
        'source_kind': 'teacher_trajectory',
        'road': '直路' if number == '1549178' else '原始道路',
        'description': '老师提供的双车追尾场景；object_1 为自车，object_2 为前车。自车由老师的 HiDrive AEB 控制，前车按文件轨迹行驶。',
    }
    for number in ['1549178', '1554254', '1554431']
}
DEFAULT = {
    'scenario': 'gidas_1549178',
    'ego_speed_kph': None,
    'ego_heading_deg': None,
    'uncertainty': 'recorded_truth',
    'noise_zero_range_m': 3.0,
    'noise_reference_range_m': 100.0,
    'noise_reference_mae_pct': 50.0,
    'noise_correlation_time_s': 0.5,
    'seed': 42,
    # Only the teacher's HiDrive AEB remains; kept for provenance in saved runs.
    'controller': 'hidrive',
    'duration_s': None,
    'dt_s': 0.01,
    'export_video': True,
}
# Settings of the removed controllers and sensor model. Old configuration files
# may still contain them; they are dropped instead of rejected.
RETIRED_KEYS = {
    'observation_filter', 'kalman_noise_mode', 'kalman_range_sigma_m', 'kalman_accel_sigma_mps2',
    'kalman_warmup_s', 'range_bias_m', 'range_sigma_m',
    'sensor_range_m', 'sensor_fov_deg', 'acc_time_gap_s', 'ttc_threshold_s', 'brake_decel_mps2',
    'reaction_delay_s', 'observation_period_s', 'aeb_trigger_mode', 'aeb_confirmation_s',
}


def original_duration(case, metadata=None, dt=0.01):
    meta = metadata
    if meta is None:
        try:
            meta = json.loads((REFERENCES / case / 'metadata.json').read_text(encoding='utf-8'))
        except (OSError, ValueError) as exc:
            raise ValueError('无法读取原版场景时长；请重新生成基准数据。') from exc
    if meta.get('timing_policy') == 'team_source_stop_trigger':
        # The supplied source ends on the first integration tick after t=10.
        stop = meta['stop_trigger_s']
        return round((math.floor(round(stop / dt, 8)) + 1) * dt, 8)
    duration = meta.get('duration_s')
    timing_verified = (
        meta.get('timing_policy') == 'original_stop_trigger'
        and meta.get('original_stop_trigger_reached') is True
    ) or (
        meta.get('timing_policy') == 'recorded_trajectory_end'
        and meta.get('source_trajectory_verified') is True
    )
    if (
        meta.get('scenario') != case
        or not timing_verified
        or isinstance(duration, bool)
        or not isinstance(duration, (int, float))
        or not math.isfinite(duration)
        or duration <= 0
    ):
        raise ValueError('基准数据未按原版结束条件生成；请重新生成基准数据。')
    return float(duration)


def validate(raw):
    if not isinstance(raw, dict):
        raise ValueError('配置必须是 JSON 对象。')
    if raw.get('uncertainty') in ('gaussian_range', 'distance_gaussian_range'):
        raise ValueError('独立高斯误差已移除；请在界面载入旧配置并重新保存为 OU 相关误差配置。')
    # Old direct-observation files may contain unused filter fields. Strip
    # those fields, but never silently rerun a filtered experiment as raw.
    if raw.get('observation_filter', 'raw') != 'raw':
        raise ValueError('卡尔曼滤波已移除；请在界面载入旧配置并重新保存为直接观测配置。')
    raw = {key: value for key, value in raw.items() if key not in RETIRED_KEYS}
    extra = set(raw) - set(DEFAULT)
    if extra:
        raise ValueError('不支持的配置项（目标对象数据不可修改）: ' + ', '.join(sorted(extra)))
    if raw.get('controller', 'hidrive') != 'hidrive':
        raise ValueError('其他控制器已移除，只保留老师的 HiDrive AEB（controller 为 hidrive）。')
    c = DEFAULT | raw
    for key, choices in [
        ('scenario', SCENARIOS),
        ('uncertainty', ['recorded_truth', 'ideal', 'distance_ou_range']),
    ]:
        if c[key] not in choices:
            raise ValueError(f'{key}: 无效选项')
    meta = json.loads((REFERENCES / c['scenario'] / 'metadata.json').read_text(encoding='utf-8'))
    initial = meta['initial_ego']
    for key in ['ego_speed_kph', 'ego_heading_deg']:
        if c[key] is None:
            c[key] = initial[key]
    ranges = {
        'ego_speed_kph': (0, 120),
        'ego_heading_deg': (-180, 180),
        'noise_zero_range_m': (0, 499),
        'noise_reference_range_m': (1, 500),
        'noise_reference_mae_pct': (0, 100),
        'noise_correlation_time_s': (0.01, 20),
    }
    for key, (lo, hi) in ranges.items():
        if isinstance(c[key], bool):
            raise ValueError(f'{key}: 必须是数值')
        try:
            c[key] = float(c[key])
        except (ValueError, TypeError):
            raise ValueError(f'{key}: 必须是数值')
        if not math.isfinite(c[key]) or not lo <= c[key] <= hi:
            raise ValueError(f'{key}: 范围为 {lo}–{hi}')
    if (
        c['uncertainty'] == 'distance_ou_range'
        and c['noise_reference_range_m'] <= c['noise_zero_range_m']
    ):
        raise ValueError('误差参考距离必须大于零误差距离。')
    if c['dt_s'] not in [0.01, 0.02, 0.05]:
        raise ValueError('时间步长必须为 0.01、0.02 或 0.05 秒。')
    if (
        isinstance(c['seed'], bool)
        or not isinstance(c['seed'], int)
        or not 0 <= c['seed'] <= 2147483647
    ):
        raise ValueError('随机种子必须为 0–2147483647 的整数。')
    if not isinstance(c['export_video'], bool):
        raise ValueError('export_video 必须是布尔值。')
    if meta.get('source_kind') != 'teacher_trajectory':
        raise ValueError('HiDrive 控制器只用于老师提供的 GIDAS 场景。')
    # The controller drives the ego from the scene's own initial state.
    c.update(initial)
    # Accept old saved configs, but duration is always determined by the scene.
    c['duration_s'] = original_duration(c['scenario'], meta, c['dt_s'])
    return c


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + f'.{os.getpid()}.tmp')
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8'
    )
    temporary.replace(path)


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()
