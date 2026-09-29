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
VERSION = '0.6.1'
LEGACY_SCENARIOS = {
    'ccrs': {'title': 'CCRs · 静止前车', 'category': 'car', 'file': 'CCRs.xosc', 'variation': 'CCRs_50kph.xosc', 'speed': 50, 'road': '直路', 'objects': '固定目标车辆', 'description': '自车接近静止前车；固定官方 50 km/h 案例中的目标位置。'},
    'cpna': {'title': 'CPNA · 行人横穿', 'category': 'pedestrian', 'file': 'CPNA.xosc', 'speed': 30, 'road': '直路', 'objects': '固定成年行人轨迹', 'description': '成年行人从近侧横穿；固定基准运行中的行人轨迹和出发时序。'},
    'cccscp': {'title': 'CCCscp · 路口交叉来车', 'category': 'car', 'file': 'CCCscp.xosc', 'speed': 20, 'road': '十字路口', 'objects': '固定来车及遮挡车辆', 'description': '自车直行通过路口；其他车辆全部按保存的基准数据回放。'},
    'cpfa': {'title': 'CPFA · 行人远侧横穿', 'category': 'pedestrian', 'file': 'CPNA.xosc', 'variation': 'CPFA_50_50kph.xosc', 'speed': 50, 'road': '直路', 'objects': '固定成年行人轨迹', 'description': '成年行人从远侧横穿；使用官方 50 km/h、50% 碰撞位置参数集。'},
    'cpla': {'title': 'CPLA · 行人沿道路行走', 'category': 'pedestrian', 'file': 'CBLA.xosc', 'variation': 'CPLA_50_50kph.xosc', 'speed': 50, 'road': '直路', 'objects': '固定同向行人轨迹', 'description': '自车接近沿道路同向行走的成年行人；行人按原版时序起步至 5 km/h。'},
    'cbna': {'title': 'CBNA · 自行车近侧横穿', 'category': 'bicycle', 'file': 'CBNA.xosc', 'variation': 'CBNA_50_50kph.xosc', 'speed': 50, 'road': '十字路口', 'objects': '固定骑行者及停放车辆', 'description': '骑行者在路口从近侧横穿自车路径；保留原版停放车辆及 50 km/h 参数集。'},
    'cbla': {'title': 'CBLA · 自行车沿道路骑行', 'category': 'bicycle', 'file': 'CBLA.xosc', 'variation': 'CBLA_50_50kph.xosc', 'speed': 50, 'road': '直路', 'objects': '固定同向骑行者轨迹', 'description': '自车接近沿道路同向骑行的自行车；骑行者按原版时序起步至 15 km/h。'},
    'cmrs': {'title': 'CMRs · 静止摩托车', 'category': 'motorbike', 'file': 'CCRs.xosc', 'variation': 'CMRs_50kph.xosc', 'speed': 50, 'road': '直路', 'objects': '固定静止摩托车', 'description': '自车接近前方静止摩托车；使用官方摩托车类型、尺寸和位置。'},
    'cmrb': {'title': 'CMRb · 前方摩托车制动', 'category': 'motorbike', 'file': 'CCRs.xosc', 'variation': 'CMRb_50kph.xosc', 'speed': 50, 'road': '直路', 'objects': '固定摩托车制动轨迹', 'description': '前方摩托车与自车均以 50 km/h 起步，摩托车按原版时序制动。'},
    'ccrm': {'title': 'CCRm · 移动前车', 'category': 'car', 'file': 'CCRs.xosc', 'variation': 'CCRm_50kph.xosc', 'speed': 50, 'road': '直路', 'objects': '固定低速前车轨迹', 'description': '自车以 50 km/h 接近前方以 20 km/h 同向行驶的车辆。'},
    'ccrb': {'title': 'CCRb · 前车制动', 'category': 'car', 'file': 'CCRs.xosc', 'variation': 'CCRb_50kph.xosc', 'speed': 50, 'road': '直路', 'objects': '固定前车制动轨迹', 'description': '前车与自车均以 50 km/h 起步，前车按原版时序制动。'},
}
SCENARIOS = {
    'gidas_' + number: {
        'title': 'GIDAS ' + number + ' · 追尾场景', 'category': 'recorded',
        'file': 'C_original_' + number + '.xosc', 'source_kind': 'teacher_trajectory',
        'road': '直路' if number == '1549178' else '原始道路',
        'description': '老师提供的双车追尾场景；object_1 为自车，object_2 为前车。默认按文件轨迹回放。',
    } for number in ['1549178', '1554254', '1554431']
}
DEFAULT = {
    'scenario': 'gidas_1549178', 'ego_speed_kph': None, 'ego_heading_deg': None,
    'uncertainty': 'recorded_truth', 'range_bias_m': 0.0, 'range_sigma_m': 1.0,
    'noise_zero_range_m': 3.0, 'noise_reference_range_m': 100.0, 'noise_reference_mae_pct': 50.0,
    'sensor_range_m': 150.0, 'sensor_fov_deg': 180.0, 'seed': 1,
    'controller': 'original_trajectory', 'ttc_threshold_s': 1.5, 'brake_decel_mps2': 8.0,
    'reaction_delay_s': 0.1, 'duration_s': None, 'dt_s': 0.01,
    'observation_filter': 'kalman', 'observation_period_s': 0.05,
    'kalman_noise_mode': 'fixed',
    'kalman_range_sigma_m': 10.0, 'kalman_accel_sigma_mps2': 3.0,
    'kalman_warmup_s': 0.15,
    'aeb_trigger_mode': 'confirmed', 'aeb_confirmation_s': 0.15,
    'export_video': True,
}

def original_duration(case, metadata=None):
    meta = metadata
    if meta is None:
        try:
            meta = json.loads((REFERENCES / case / 'metadata.json').read_text(encoding='utf-8'))
        except (OSError, ValueError) as exc:
            raise ValueError('无法读取原版场景时长；请重新生成基准数据。') from exc
    duration = meta.get('duration_s')
    timing_verified = ((meta.get('timing_policy') == 'original_stop_trigger'
                        and meta.get('original_stop_trigger_reached') is True)
                       or (meta.get('timing_policy') == 'recorded_trajectory_end'
                           and meta.get('source_trajectory_verified') is True))
    if (meta.get('scenario') != case or not timing_verified
            or isinstance(duration, bool) or not isinstance(duration, (int, float))
            or not math.isfinite(duration) or duration <= 0):
        raise ValueError('基准数据未按原版结束条件生成；请重新生成基准数据。')
    return float(duration)


def validate(raw):
    if not isinstance(raw, dict):
        raise ValueError('配置必须是 JSON 对象。')
    extra = set(raw) - set(DEFAULT)
    if extra:
        raise ValueError('不支持的配置项（目标对象数据不可修改）: ' + ', '.join(sorted(extra)))
    c = DEFAULT | raw
    # Saved pre-0.5 configurations retain their direct, per-step, single-hit semantics.
    for key, legacy in [('observation_filter','raw'), ('observation_period_s',0.0), ('aeb_trigger_mode','instant'), ('kalman_noise_mode','fixed')]:
        if raw and key not in raw:
            c[key] = legacy
    for key, choices in [('scenario', SCENARIOS), ('uncertainty', ['recorded_truth', 'ideal', 'gaussian_range', 'distance_gaussian_range']), ('controller', ['original_trajectory', 'aeb', 'aeb_follow', 'constant_speed']), ('observation_filter',['raw','kalman']), ('aeb_trigger_mode',['instant','confirmed'])]:
        if c[key] not in choices:
            raise ValueError(f'{key}: 无效选项')
    meta = json.loads((REFERENCES / c['scenario'] / 'metadata.json').read_text(encoding='utf-8'))
    if c['kalman_noise_mode'] not in ['fixed', 'predicted_range']:
        raise ValueError('kalman_noise_mode: 无效选项')
    initial = meta['initial_ego']
    for key in ['ego_speed_kph', 'ego_heading_deg']:
        if c[key] is None:
            c[key] = initial[key]
    ranges = {'ego_speed_kph': (0, 120), 'ego_heading_deg': (-180, 180),
              'range_bias_m': (-30, 30), 'range_sigma_m': (0, 20),
              'noise_zero_range_m': (0, 499), 'noise_reference_range_m': (1, 500),
              'noise_reference_mae_pct': (0, 100),
              'sensor_range_m': (1, 500), 'sensor_fov_deg': (1, 360),
              'ttc_threshold_s': (0.1, 5), 'brake_decel_mps2': (0.1, 12),
              'reaction_delay_s': (0, 2), 'aeb_confirmation_s': (0, 1),
              'kalman_range_sigma_m': (0.1, 200), 'kalman_accel_sigma_mps2': (0.1, 20),
              'kalman_warmup_s': (0, 1)}
    for key, (lo, hi) in ranges.items():
        if isinstance(c[key], bool):
            raise ValueError(f'{key}: 必须是数值')
        try:
            c[key] = float(c[key])
        except (ValueError, TypeError):
            raise ValueError(f'{key}: 必须是数值')
        if not math.isfinite(c[key]) or not lo <= c[key] <= hi:
            raise ValueError(f'{key}: 范围为 {lo}–{hi}')
    if c['uncertainty']=='distance_gaussian_range' and c['noise_reference_range_m'] <= c['noise_zero_range_m']:
        raise ValueError('误差参考距离必须大于零误差距离。')
    if c['dt_s'] not in [0.01, 0.02, 0.05]:
        raise ValueError('时间步长必须为 0.01、0.02 或 0.05 秒。')
    if isinstance(c['observation_period_s'],bool) or c['observation_period_s'] not in [0,0.02,0.05,0.1]:
        raise ValueError('观测周期必须为 0（旧版）、0.02、0.05 或 0.1 秒。')
    if isinstance(c['seed'], bool) or not isinstance(c['seed'], int) or not 0 <= c['seed'] <= 2147483647:
        raise ValueError('随机种子必须为 0–2147483647 的整数。')
    if not isinstance(c['export_video'], bool):
        raise ValueError('export_video 必须是布尔值。')
    if c['controller'] in ['original_trajectory','aeb_follow']:
        c.update(initial)
    # Accept old saved configs, but duration is always determined by the scene.
    c['duration_s'] = original_duration(c['scenario'], meta)
    return c

def write_json(path, value):
    path=Path(path)
    temporary=path.with_suffix(path.suffix+f'.{os.getpid()}.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    temporary.replace(path)

def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()
