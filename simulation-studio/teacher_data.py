"""Import the supplied timed trajectories, without modifying teacher DATA.

Positions/headings are copied exactly. Velocity is a derived 0.20 s secant;
the file's Init speed of zero is superseded by its timed trajectory action.
The replay ends at the last recorded vertex, with no invented tail.
"""
import csv
import math
import xml.etree.ElementTree as ET
import numpy as np
from settings import PROJECT, REFERENCES, SCENARIOS, sha256, write_json


def read_source(case):
    source = PROJECT / 'DATA' / SCENARIOS[case]['file']
    road = source.with_suffix('.xodr')
    tree = ET.parse(source)
    root = tree.getroot()
    definitions = {e.get('name'): ET.tostring(e.find('Vehicle'), encoding='unicode')
                   for e in root.findall('Entities/ScenarioObject')}
    trajectories = {}
    for group in root.findall('.//ManeuverGroup'):
        name = group.find('Actors/EntityRef').get('entityRef')
        follow = group.find('.//FollowTrajectoryAction')
        timing = follow.find('TimeReference/Timing')
        if (timing.get('domainAbsoluteRelative') != 'absolute'
                or float(timing.get('scale')) != 1 or float(timing.get('offset')) != 0
                or follow.find('TrajectoryFollowingMode').get('followingMode') != 'position'):
            raise ValueError('导入器只接受绝对时间、无缩放的位置轨迹。')
        points = []
        for vertex in follow.findall('Trajectory/Shape/Polyline/Vertex'):
            position = vertex.find('Position/WorldPosition')
            points.append([float(vertex.get('time'))] + [float(position.get(k, '0')) for k in ['x','y','z','h','p','r']])
        values = np.array(points)
        if (len(values) < 21 or not np.isfinite(values).all() or values[0,0] != 0
                or not np.allclose(np.diff(values[:,0]), .01, rtol=0, atol=1e-8)):
            raise ValueError('轨迹需要从零开始的有限 0.01 秒采样数据。')
        index = np.arange(len(values))
        lo = np.clip(index-10, 0, len(values)-21)
        hi = lo+20
        velocity = (values[hi,1:3]-values[lo,1:3])/(values[hi,0]-values[lo,0])[:,None]
        trajectories[name] = (values, velocity)
    if set(trajectories) != {'object_1','object_2'} or set(definitions) != set(trajectories):
        raise ValueError('老师场景必须包含 object_1 和 object_2 两条轨迹。')
    if not np.array_equal(trajectories['object_1'][0][:,0], trajectories['object_2'][0][:,0]):
        raise ValueError('两辆车的轨迹时间戳不一致。')
    # Reading the companion road also prevents a usable-looking incomplete pair.
    roads = ET.parse(road).getroot()
    warnings = []
    for item in roads.findall('road'):
        last = item.findall('planView/geometry')[-1]
        if abs(float(last.get('s'))+float(last.get('length'))-float(item.get('length'))) > .01:
            warnings.append('原始道路末段长度与 road length 不一致；保留老师道路并记录 esmini 提示。')
    return source, road, definitions, trajectories, warnings


def prepare_teacher(case):
    source, road, definitions, trajectories, warnings = read_source(case)
    out = REFERENCES / case
    out.mkdir(parents=True, exist_ok=True)
    objects, rows = [], []
    times = trajectories['object_1'][0][:,0]
    for ident, name in enumerate(['object_1','object_2']):
        body = ET.fromstring(definitions[name])
        dimensions = body.find('BoundingBox/Dimensions')
        center = body.find('BoundingBox/Center')
        values, velocity = trajectories[name]
        base = {'id':ident, 'name':name, 'model_id':0 if ident == 0 else 2,
                'objectType':1, 'objectCategory':0,
                **{k:float(dimensions.get(k)) for k in ['length','width','height']},
                **{'centerOffset'+k.upper():float(center.get(k)) for k in ['x','y','z']}}
        for i, point in enumerate(values):
            state = {'time_s':float(point[0]), 'id':ident, 'name':name,
                     **dict(zip(['x','y','z','h','p','r'], map(float, point[1:]))),
                     'speed':float(np.linalg.norm(velocity[i])),
                     'vx':float(velocity[i,0]), 'vy':float(velocity[i,1])}
            rows.append(state)
            if i == 0:
                objects.append(base | {k:v for k,v in state.items() if k != 'time_s'})
    rows.sort(key=lambda r:(r['time_s'],r['id']))
    with (out/'truth.csv').open('w',encoding='utf-8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    ego = objects[0]
    write_json(out/'metadata.json', {
        'scenario':case, 'source_kind':'teacher_trajectory', 'source_file':str(source),
        'source_sha256':sha256(source), 'road_file':str(road), 'road_sha256':sha256(road),
        'truth_sha256':sha256(out/'truth.csv'), 'entity_definitions':definitions,
        'objects':objects, 'ego_id':0, 'ego_name':'object_1', 'dt_s':.01,
        'duration_s':float(times[-1]), 'timing_policy':'recorded_trajectory_end',
        'source_trajectory_verified':True, 'reference_input':str(source),
        'initial_ego':{'ego_speed_kph':ego['speed']*3.6,'ego_heading_deg':math.degrees(ego['h'])},
        'source_warnings':warnings,
        'method':'Exact supplied position and heading vertices; linear position and shortest-angle heading interpolation. Velocity is derived with a 0.20 s secant window, shifted at endpoints. Replay ends at the last recorded vertex (9.98 s), before the source endpoint stop action. HiDriveController is not executed.',
        'initial_speed_method':'Displacement over the first 0.20 s; source contains positions, not measured speeds.',
    })
    return {'scenario':case,'frames':len(times),'duration_s':float(times[-1]),'initial_speed_kph':ego['speed']*3.6,'initial_heading_deg':math.degrees(ego['h'])}


if __name__ == '__main__':
    for case in SCENARIOS:
        print(prepare_teacher(case))
