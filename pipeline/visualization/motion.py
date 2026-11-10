"""Adapt pipeline result files to the existing cone replay geometry."""
from bisect import bisect_right
import csv
import ctypes
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ..common import read_json
from ..scenarios import get as get_scenario


def center(state):
    co, si = math.cos(state['h']), math.sin(state['h'])
    return (state['x'] + co*state['centerOffsetX'] - si*state['centerOffsetY'],
            state['y'] + si*state['centerOffsetX'] + co*state['centerOffsetY'])


def project_top_point(point, camera, width, height):
    """Public esmini fixed-top camera: one-degree FOV and float camera pose."""
    cx, cy, cz, heading = [ctypes.c_float(value).value for value in camera]
    pitch = ctypes.c_float(math.pi/2 - 1e-5).value
    dx, dy, dz = point[0]-cx, point[1]-cy, point[2]-cz
    co, si = math.cos(heading), math.sin(heading)
    forward = co*dx + si*dy
    depth = math.cos(pitch)*forward - math.sin(pitch)*dz
    if depth <= 0:
        raise ValueError('Replay point is behind the camera.')
    scale = height / (2*math.tan(math.radians(.5))*depth)
    return (width/2 + (si*dx-co*dy)*scale,
            height/2 - (math.sin(pitch)*forward+math.cos(pitch)*dz)*scale)


def detection_column(config, plugin_name=None):
    entries = [entry for entry in config['errors'] if entry['type'] == 'detection']
    if plugin_name is not None:
        entries = [entry for entry in entries if (entry.get('name') or entry['type']) == plugin_name]
        if not entries:
            raise ValueError(f'Detection plugin {plugin_name!r} is not in this run.')
    if not entries:
        return None
    if len(entries) != 1:
        raise ValueError('Multiple detection plugins: choose one with --detection-plugin.')
    return (entries[0].get('name') or 'detection') + '.missed'


def read_detection_flags(path, target, column):
    if column is None:
        return {0.0: True}  # Explicit no-detection-error reference, not a truth fallback.
    flags = {}
    with Path(path).open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream, delimiter='\t')
        if column not in (reader.fieldnames or []):
            raise ValueError(f'{column} is missing from the saved perception log.')
        for row in reader:
            if row['object_name'] != target:
                continue
            if row[column] not in ('0', '1'):
                raise ValueError(f'Invalid saved detection flag: {row[column]!r}')
            flags[round(float(row['time_s']), 8)] = row[column] == '0'
    if not flags:
        raise ValueError('No saved detection flags for the chosen target.')
    return flags


def flag_at(flags, time_s):
    times = sorted(flags)
    index = bisect_right(times, time_s+1e-8)-1
    if index < 0:
        raise ValueError('No saved detection flag at this replay time.')
    return flags[times[index]]


def load_motion(out, target=None, plugin_name=None):
    config = read_json(out/'config.json')
    scenario = get_scenario(config['scenario'])
    target_names = [item['name'] for item in scenario.targets]
    if target is None:
        if len(target_names) != 1:
            raise ValueError('Choose the cone target with --target.')
        target = target_names[0]
    if target not in target_names:
        raise ValueError('The chosen target is not in this scenario.')
    frames = {}
    fields = {'x':'x_m', 'y':'y_m', 'z':'z_m', 'h':'heading_rad', 'p':'pitch_rad',
              'speed':'speed_mps', 'vx':'vx_mps', 'vy':'vy_mps', 'length':'length_m',
              'width':'width_m', 'height':'height_m', 'centerOffsetX':'center_offset_x_m',
              'centerOffsetY':'center_offset_y_m', 'centerOffsetZ':'center_offset_z_m'}
    with (out/'truth.csv').open(encoding='utf-8-sig', newline='') as stream:
        for row in csv.DictReader(stream):
            state = {'id':int(row['object_id']), 'name':row['object_name'], 'r':0.0,
                     **{key:float(row[value]) for key,value in fields.items()}}
            frames.setdefault(float(row['time_s']), []).append(state)
    rank = {scenario.ego:0, target:1}
    for states in frames.values():
        states.sort(key=lambda state:(rank.get(state['name'],2),state['id']))
    if not frames or min(frames) != 0:
        raise ValueError('Recorded truth must start at time zero.')
    column = detection_column(config, plugin_name)
    flags = read_detection_flags(out/'perception_applied.tsv', target, column)
    for time_s in frames:
        flag_at(flags,time_s)
    return {'source_scene':out/'scene.xosc', 'target':target, 'detection_column':column}, frames, flags


def build_scene(path, reference, initial, horizon):
    """Replay all saved poses with built-in ExternalController; no AEB rerun."""
    tree = ET.parse(reference['source_scene'])
    root = tree.getroot()
    for state in initial:
        entity = root.find(f'./Entities/ScenarioObject[@name="{state["name"]}"]')
        for old in list(entity.findall('ObjectController')):
            entity.remove(old)
        props = ET.SubElement(ET.SubElement(ET.SubElement(entity,'ObjectController'),'Controller',
                                           name='Replay_'+state['name']),'Properties')
        for key,value in [('esminiController','ExternalController'),('mode','override'),('useGhost','false')]:
            ET.SubElement(props,'Property',name=key,value=value)
    old = root.find('Storyboard')
    root.remove(old)
    storyboard = ET.SubElement(root,'Storyboard')
    actions = ET.SubElement(ET.SubElement(storyboard,'Init'),'Actions')
    for state in initial:
        private = ET.SubElement(actions,'Private',entityRef=state['name'])
        teleport = ET.SubElement(ET.SubElement(private,'PrivateAction'),'TeleportAction')
        position = ET.SubElement(teleport,'Position')
        ET.SubElement(position,'WorldPosition',**{key:str(state[key]) for key in ['x','y','z','h','p','r']})
        speed = ET.SubElement(ET.SubElement(ET.SubElement(private,'PrivateAction'),'LongitudinalAction'),'SpeedAction')
        ET.SubElement(speed,'SpeedActionDynamics',dynamicsShape='step',value='0',dynamicsDimension='time')
        ET.SubElement(ET.SubElement(speed,'SpeedActionTarget'),'AbsoluteTargetSpeed',value=str(state['speed']))
        controller = ET.SubElement(ET.SubElement(private,'PrivateAction'),'ControllerAction')
        ET.SubElement(controller,'ActivateControllerAction',longitudinal='true',lateral='true')
    condition = ET.SubElement(ET.SubElement(ET.SubElement(storyboard,'StopTrigger'),'ConditionGroup'),
                              'Condition',name='Replay end',delay='0',conditionEdge='rising')
    ET.SubElement(ET.SubElement(condition,'ByValueCondition'),'SimulationTimeCondition',
                  value=str(horizon+.01),rule='greaterThan')
    tree.write(path,encoding='utf-8',xml_declaration=True)


def align_visual_models(scene, runtime, ego, target):
    tree = ET.parse(scene)
    for entity in tree.findall('./Entities/ScenarioObject'):
        body = next((item for item in entity if item.tag in ('Vehicle','Pedestrian','MiscObject')),None)
        if body is None:
            continue
        if body.tag not in ('Vehicle','Pedestrian','MiscObject'):
            continue
        props = body.find('Properties')
        if props is None:
            props = ET.SubElement(body,'Properties')
        value = props.find('Property[@name="scaleMode"]')
        if value is None:
            value = ET.SubElement(props,'Property',name='scaleMode')
        value.set('value','ModelToBB')
        if entity.get('name') in (ego,target):
            model='car_white.osgb' if entity.get('name')==ego else 'car_red.osgb'
            for old in list(props.findall('File')):
                props.remove(old)
            ET.SubElement(props,'File',filepath=str(Path(runtime).resolve()/'resources/models'/model))
            ident=props.find('Property[@name="model_id"]')
            if ident is None:
                ident=ET.SubElement(props,'Property',name='model_id')
            ident.set('value','0' if entity.get('name')==ego else '2')
    tree.write(scene,encoding='utf-8',xml_declaration=True)
