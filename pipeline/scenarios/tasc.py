"""Import the teacher's TASC case folders as local scenarios: python -m pipeline import-tasc <folder>

<folder> holds group folders such as "Rear-end car-car", each with Xosc/ and Xodr/. The files are not in Git: the
converted copies go to pipeline/scenarios/local/ (ignored), listed in local/scenarios.json with keys like
tasc_car_1549178. Grids pick them with patterns: "scenarios": ["tasc_car_*"].

The case files lack what a pipeline scenario needs, so each file gets:
- an ObjectController on the ego with the HiDrive properties of the three shared files, and an
  ActivateControllerAction in its Init, where the shared files have it;
- a 10 s stop trigger when the storyboard has none (the cut-in files).
The ego is object_1. Skipped: car-car and cyclist files where object_2 is the rear vehicle at first contact (the road
follows object_1's path, and HiDrive jumps along that road when it brakes the other vehicle: tried 2026-11-11, 5 of 6
cyclist cases jumped about 10 m at brake onset); files whose boxes overlap from the start; car-car copies of cut-in
cases, whose roles and road the teacher set in the cut-in "_s" file. Cut-in files keep object_1, the vehicle whose
lane is entered.
"""
import csv
import hashlib
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from ..common import read_json, write_json

GROUPS = {'Cut-in': 'cutin', 'Rear-end car-car': 'car', 'Rear-end car-cyclist': 'cyclist'}
TITLES = {'cutin': 'TASC cut-in', 'car': 'TASC car-car', 'cyclist': 'TASC car-cyclist', 'bicycle': 'TASC car-bicycle'}
# The cyclist files give object_2 the same car box as the ego. With bicycle=True the cyclist cases are written a second
# time as tasc_bicycle_*, object_2 shaped like this. Assumed size (the files hold none); the model ships with esmini.
# The box stays centred on the trajectory point, as in the files.
BICYCLE = {'category': 'bicycle', 'length': '1.8', 'width': '0.6', 'height': '1.7', 'center_z': '0.85',
           'model': '../models/cyclist.osgb', 'model_id': '9'}  # the replayer draws by model_id (resources/model_ids.txt)
# As in the three shared scenario files (data/C_original_*.xosc).
CONTROLLER = 'HiDriveController'
PROPERTIES = (('timeGap', '1.0'), ('mode', 'additive'), ('ADF_enabled', 'true'), ('Speed_limit', '100'))
STOP_TIME_S = '10.0'


def case_id(path):
    return path.stem.removeprefix('C_original_').removesuffix('_s')


def trajectory(storyboard, name):
    """(t, x, y, h) of the FollowTrajectoryAction that `name` acts in."""
    for group in storyboard.iter('ManeuverGroup'):
        if name in [ref.get('entityRef') for ref in group.findall('./Actors/EntityRef')]:
            polyline = group.find('.//FollowTrajectoryAction//Polyline')
            if polyline is not None:
                points = []
                for vertex in polyline.findall('Vertex'):
                    w = vertex.find('./Position/WorldPosition')
                    points.append((float(vertex.get('time')), float(w.get('x')), float(w.get('y')), float(w.get('h', 0))))
                return points
    return []


def corners(x, y, h, length, width):
    c, s = math.cos(h), math.sin(h)
    return [(x + c * a - s * b, y + s * a + c * b) for a, b in
            ((length / 2, width / 2), (length / 2, -width / 2), (-length / 2, -width / 2), (-length / 2, width / 2))]


def overlap(a, b):
    """Separating-axis test for two boxes given by their corners."""
    for box in (a, b):
        for i in range(4):
            (x1, y1), (x2, y2) = box[i], box[(i + 1) % 4]
            nx, ny = y1 - y2, x2 - x1
            pa = [nx * x + ny * y for x, y in a]
            pb = [nx * x + ny * y for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def first_contact(root):
    """(index, object_2 ahead of object_1 at contact) for the first step the two boxes overlap, or None."""
    dims = {}
    for item in root.findall('./Entities/ScenarioObject'):
        box = item.find('.//BoundingBox/Dimensions')
        dims[item.get('name')] = (float(box.get('length')), float(box.get('width')))
    storyboard = root.find('Storyboard')
    one, two = trajectory(storyboard, 'object_1'), trajectory(storyboard, 'object_2')
    for i, (p, q) in enumerate(zip(one, two)):
        if overlap(corners(*p[1:], *dims['object_1']), corners(*q[1:], *dims['object_2'])):
            heading = math.atan2(math.sin(p[3]) + math.sin(q[3]), math.cos(p[3]) + math.cos(q[3]))
            ahead = (q[1] - p[1]) * math.cos(heading) + (q[2] - p[2]) * math.sin(heading)
            return i, ahead > 0
    return None


def add_controller(root, ego='object_1'):
    item = root.find(f'./Entities/ScenarioObject[@name="{ego}"]')
    if item.find('ObjectController') is None:
        controller = ET.SubElement(ET.SubElement(item, 'ObjectController'), 'Controller', name=CONTROLLER)
        properties = ET.SubElement(controller, 'Properties')
        for name, value in PROPERTIES:
            ET.SubElement(properties, 'Property', name=name, value=value)
    private = root.find(f'./Storyboard/Init/Actions/Private[@entityRef="{ego}"]')
    if private.find('.//ActivateControllerAction') is None:
        holder = next((action for action in private.findall('PrivateAction') if action.find('LongitudinalAction') is not None),
                      None)
        if holder is None:
            holder = ET.SubElement(private, 'PrivateAction')
        ET.SubElement(ET.SubElement(holder, 'ControllerAction'), 'ActivateControllerAction', longitudinal='true', lateral='true')


def add_stop_trigger(root):
    storyboard = root.find('Storyboard')
    stop = storyboard.find('StopTrigger')
    if stop is not None and stop.find('.//SimulationTimeCondition') is not None:
        return False
    if stop is None:
        stop = ET.SubElement(storyboard, 'StopTrigger')
    for child in list(stop):
        stop.remove(child)
    condition = ET.SubElement(ET.SubElement(stop, 'ConditionGroup'), 'Condition', delay='0.0', conditionEdge='rising',
                              name='Simulation Time')
    ET.SubElement(ET.SubElement(condition, 'ByValueCondition'), 'SimulationTimeCondition', rule='greaterThan',
                  value=STOP_TIME_S)
    return True


def make_bicycle(root, name='object_2'):
    """Give `name` the BICYCLE category, box and model."""
    vehicle = root.find(f'./Entities/ScenarioObject[@name="{name}"]/Vehicle')
    vehicle.set('vehicleCategory', BICYCLE['category'])
    vehicle.find('./BoundingBox/Center').set('z', BICYCLE['center_z'])
    box = vehicle.find('./BoundingBox/Dimensions')
    for side in ('length', 'width', 'height'):
        box.set(side, BICYCLE[side])
    model = vehicle.find('./Properties/File')
    if model is not None:
        model.set('filepath', BICYCLE['model'])
    model_id = vehicle.find('./Properties/Property[@name="model_id"]')
    if model_id is not None:
        model_id.set('value', BICYCLE['model_id'])


def convert(xosc, group, target, *, skip_ids=(), bicycle=False):
    """Convert one case file into `target`; returns a report row (status 'imported' or 'skipped').

    bicycle=True writes the case as tasc_bicycle_* with object_2 as a bicycle. The checks below still use the boxes
    of the file, so the same cases pass; 'recorded_contact' then tells whether the recorded paths still touch.
    """
    case = case_id(xosc)
    if bicycle:
        group = 'bicycle'
    key = f'tasc_{group}_{case}'
    row = {'key': key, 'group': group, 'case': case, 'source': xosc.name, 'status': 'skipped', 'reason': '',
           'stop_trigger_added': False, 'recorded_contact': ''}
    if case in skip_ids:
        row['reason'] = 'also in Cut-in, where its roles are set'
        return row
    tree = ET.parse(xosc)
    root = tree.getroot()
    if [item.get('name') for item in root.findall('./Entities/ScenarioObject')] != ['object_1', 'object_2']:
        row['reason'] = 'needs exactly object_1 and object_2'
        return row
    road = Path(root.find('./RoadNetwork/LogicFile').get('filepath')).name
    roads = {path.name.lower(): path for path in (xosc.parent.parent / 'Xodr').glob('*.xodr')}
    if road.lower() not in roads:
        row['reason'] = f'road file {road} not found'
        return row
    contact = first_contact(root)
    if contact is None:
        row['reason'] = 'the boxes never touch'
        return row
    index, two_ahead = contact
    if index == 0:
        row['reason'] = 'the boxes overlap from the start'
        return row
    if group != 'cutin' and not two_ahead:
        row['reason'] = 'object_2 strikes from behind; the road follows object_1'
        return row
    row['recorded_contact'] = True
    if bicycle:
        make_bicycle(root)
        row['recorded_contact'] = first_contact(root) is not None
    add_controller(root)
    row['stop_trigger_added'] = add_stop_trigger(root)
    root.find('./RoadNetwork/LogicFile').set('filepath', f'{key}.xodr')
    target.mkdir(parents=True, exist_ok=True)
    tree.write(target / f'{key}.xosc', encoding='utf-8', xml_declaration=True)
    shutil.copyfile(roads[road.lower()], target / f'{key}.xodr')
    row.update(status='imported', road=roads[road.lower()].name,
               source_sha1=hashlib.sha1(xosc.read_bytes()).hexdigest())
    return row


def import_folder(folder, target, *, bicycle=False):
    """Import every group folder found in `folder` into `target`; returns the report rows.

    bicycle=True also writes each cyclist case as tasc_bicycle_* (see BICYCLE); the tasc_cyclist_* copies stay.
    """
    folder, target = Path(folder), Path(target)
    found = {name: folder / name for name in GROUPS if (folder / name / 'Xosc').is_dir()}
    if not found:
        raise ValueError(f'{folder}: no folder named ' + ', '.join(f'"{name}"' for name in GROUPS) + ' with an Xosc folder.')
    cutin = {case_id(path) for path in (folder / 'Cut-in' / 'Xosc').glob('*.xosc')} if 'Cut-in' in found else set()
    rows = []
    for name, path in found.items():
        group = GROUPS[name]
        for xosc in sorted((path / 'Xosc').glob('*.xosc')):
            rows.append(convert(xosc, group, target, skip_ids=cutin if group == 'car' else ()))
            if bicycle and group == 'cyclist':
                rows.append(convert(xosc, group, target, bicycle=True))
    target.mkdir(parents=True, exist_ok=True)
    registry_path = target / 'scenarios.json'
    registry = read_json(registry_path) if registry_path.is_file() else {}
    for row in rows:
        registry.pop(row['key'], None)
        if row['status'] == 'imported':
            registry[row['key']] = {'title': f'{TITLES[row["group"]]} {row["case"]}', 'file': f'{row["key"]}.xosc',
                                    'group': row['group'], 'case': row['case'], 'source': row['source'],
                                    'source_sha1': row['source_sha1']}
        else:
            for suffix in ('.xosc', '.xodr'):
                (target / f'{row["key"]}{suffix}').unlink(missing_ok=True)
    write_json(registry_path, dict(sorted(registry.items())))
    fields = ['key', 'group', 'case', 'source', 'status', 'reason', 'stop_trigger_added', 'recorded_contact', 'road',
              'source_sha1']
    with (target / 'import_report.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return rows
