"""Fixed target replay and a minimal external-control OpenSCENARIO scene."""
import bisect
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from settings import REFERENCES, sha256, original_duration

class Reference:
    def __init__(self, case):
        self.directory = REFERENCES / case
        self.meta = json.loads((self.directory/'metadata.json').read_text(encoding='utf-8'))
        self.duration = original_duration(case, self.meta)
        if sha256(self.directory/'truth.csv') != self.meta['truth_sha256']:
            raise ValueError('固定对象数据校验失败；拒绝使用已被修改的数据。')
        if self.meta.get('source_kind') == 'teacher_trajectory':
            if (sha256(self.meta['source_file']) != self.meta['source_sha256']
                    or sha256(self.meta['road_file']) != self.meta['road_sha256']):
                raise ValueError('老师源数据已改变，请重新导入场景。')
        self.base = {s['id']:s for s in self.meta['objects']}
        self.frames = {}
        with (self.directory/'truth.csv').open(encoding='utf-8', newline='') as f:
            for row in csv.DictReader(f):
                t = float(row.pop('time_s'))
                ident, name = int(row.pop('id')), row.pop('name')
                self.frames.setdefault(t, []).append(self.base[ident] | {k:float(v) for k,v in row.items()})
        self.times = sorted(self.frames)
        if not self.times or self.times[0] != 0 or abs(self.times[-1] - self.duration) > 1e-7:
            raise ValueError('基准数据范围与原版运行时间不一致。')
        self.ego_id = self.meta.get('ego_id')
        if self.ego_id is None:
            self.ego_id = next(s['id'] for s in self.meta['objects'] if s['name'] == 'Ego')

    def states(self, t):
        i = bisect.bisect_left(self.times, round(t, 8))
        if i >= len(self.times):
            raise ValueError('请求的时间超过固定数据范围')
        if abs(self.times[i]-t) < 1e-7:
            states = [s.copy() for s in self.frames[self.times[i]]]
        else:
            lo, hi = self.times[i-1], self.times[i]
            alpha = (t-lo)/(hi-lo)
            states=[]
            for a,b in zip(self.frames[lo],self.frames[hi]):
                s=a.copy()
                for k in ['x','y','z','p','r','speed'] + (['vx','vy'] if 'vx' in a else []):
                    s[k] = a[k]+alpha*(b[k]-a[k])
                dh=math.atan2(math.sin(b['h']-a['h']), math.cos(b['h']-a['h']))
                s['h']=a['h']+alpha*dh
                states.append(s)
        for s in states:
            if 'vx' not in s:
                s['vx'],s['vy']=s['speed']*math.cos(s['h']),s['speed']*math.sin(s['h'])
        return states

def object_kind(state, metadata):
    definition = metadata.get('entity_definitions', {}).get(state['name'])
    if definition:
        body = ET.fromstring(definition)
        return 'pedestrian' if body.tag == 'Pedestrian' else body.get('vehicleCategory', 'car')
    return 'pedestrian' if state['objectType'] == 2 else 'car'


def object_label(state, metadata):
    return {'pedestrian':'行人', 'bicycle':'自行车', 'motorbike':'摩托车'}.get(object_kind(state, metadata), '车辆')


def build_scene(path, metadata, initial_states, horizon):
    root=ET.Element('OpenSCENARIO')
    header=ET.SubElement(root,'FileHeader',revMajor='1',revMinor='2',date='2026-09-20T00:00:00',author='TME180 Simulation Studio',description='Derived experiment: fixed recorded non-Ego objects + external Ego controller. Not an unmodified NCAP test.')
    if metadata.get('source_kind') == 'teacher_trajectory':
        header.set('description', 'Teacher data: recorded trajectories or an externally controlled ego. Source provenance retained in reference.json; HiDriveController not executed.')
    else:
        ET.SubElement(header,'License',name='Mozilla Public License Version 2.0',resource='https://www.mozilla.org/en-US/MPL/2.0/',spdxId='MPL-2.0')
    ET.SubElement(root,'ParameterDeclarations'); ET.SubElement(root,'CatalogLocations')
    road=ET.SubElement(root,'RoadNetwork'); ET.SubElement(road,'LogicFile',filepath=metadata['road_file'])
    entities=ET.SubElement(root,'Entities')
    storyboard=ET.SubElement(root,'Storyboard')
    actions=ET.SubElement(ET.SubElement(storyboard,'Init'),'Actions')
    for s in initial_states:
        ent=ET.SubElement(entities,'ScenarioObject',name=s['name'])
        pedestrian=s['objectType']==2
        definition=metadata.get('entity_definitions',{}).get(s['name'])
        if definition:
            body=ET.fromstring(definition)
            ent.append(body)
        elif pedestrian:
            body=ET.SubElement(ent,'Pedestrian',name=s['name'],pedestrianCategory='pedestrian',mass='75',model='walkman')
        else:
            body=ET.SubElement(ent,'Vehicle',name=s['name'],vehicleCategory='car')
            ET.SubElement(body,'ParameterDeclarations')
        if not definition:
            box=ET.SubElement(body,'BoundingBox')
            ET.SubElement(box,'Center',**{k:str(s['centerOffset'+k.upper()]) for k in ['x','y','z']})
            ET.SubElement(box,'Dimensions',**{k:str(s[k]) for k in ['width','length','height']})
        if not pedestrian and not definition:
            ET.SubElement(body,'Performance',maxSpeed='100',maxAcceleration='15',maxDeceleration='15')
            axles=ET.SubElement(body,'Axles')
            for tag,x in [('FrontAxle','2.6'),('RearAxle','0')]:
                ET.SubElement(axles,tag,maxSteering='0.5',wheelDiameter='0.6',trackWidth='1.6',positionX=x,positionZ='0.3')
        props=body.find('Properties')
        if props is None:
            props=ET.SubElement(body,'Properties')
        # Use bundled visual models; the teacher's ../models path is not shipped.
        if metadata.get('source_kind') == 'teacher_trajectory':
            for visual_file in list(props.findall('File')):
                props.remove(visual_file)
        model=props.find("Property[@name='model_id']")
        if model is None:
            model=ET.SubElement(props,'Property',name='model_id')
        model.set('value', {'pedestrian':'7','bicycle':'9','motorbike':'10'}.get(object_kind(s,metadata), '0' if s['id']==metadata.get('ego_id') or s['name']=='Ego' else '2'))
        ctrl=ET.SubElement(ET.SubElement(ent,'ObjectController'),'Controller',name='External_'+s['name'])
        props=ET.SubElement(ctrl,'Properties')
        for name,value in [('esminiController','ExternalController'),('mode','override'),('useGhost','false')]:
            ET.SubElement(props,'Property',name=name,value=value)
        private=ET.SubElement(actions,'Private',entityRef=s['name'])
        pa=ET.SubElement(private,'PrivateAction'); pos=ET.SubElement(ET.SubElement(pa,'TeleportAction'),'Position')
        ET.SubElement(pos,'WorldPosition',**{k:str(s[k]) for k in ['x','y','z','h','p','r']})
        pa=ET.SubElement(private,'PrivateAction'); sa=ET.SubElement(ET.SubElement(pa,'LongitudinalAction'),'SpeedAction')
        ET.SubElement(sa,'SpeedActionDynamics',dynamicsShape='step',value='0',dynamicsDimension='time')
        ET.SubElement(ET.SubElement(sa,'SpeedActionTarget'),'AbsoluteTargetSpeed',value=str(s['speed']))
        pa=ET.SubElement(private,'PrivateAction'); ca=ET.SubElement(pa,'ControllerAction')
        ET.SubElement(ca,'ActivateControllerAction',longitudinal='true',lateral='true')
    stop=ET.SubElement(storyboard,'StopTrigger'); group=ET.SubElement(stop,'ConditionGroup')
    condition=ET.SubElement(group,'Condition',name='End',delay='0',conditionEdge='none')
    ET.SubElement(ET.SubElement(condition,'ByValueCondition'),'SimulationTimeCondition',value=str(horizon),rule='greaterOrEqual')
    ET.indent(root); ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
