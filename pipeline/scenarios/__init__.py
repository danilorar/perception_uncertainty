"""Scenarios: OpenSCENARIO files in data/, listed in scenarios.json.

To add one, put the .xosc and its .xodr (same name) in data/ and add a line to scenarios.json. The file needs:
- an ego ("ego" in scenarios.json, default object_1) with an ObjectController and an ActivateControllerAction in Init;
- a storyboard StopTrigger with a SimulationTimeCondition (rule greaterThan).
The IDs gidas_* keep the names of earlier results; the data are TASC data.
"""
from dataclasses import dataclass
from functools import lru_cache
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ..common import read_json

FOLDER = Path(__file__).resolve().parent


@dataclass(frozen=True)
class Scenario:
    key: str
    title: str
    source: Path
    ego: str
    objects: tuple      # ({'id', 'name'}, ...): ego first, then file order
    stop_time_s: float

    @property
    def targets(self):
        return [item for item in self.objects if item['name'] != self.ego]

    def duration(self, dt_s):
        """The scenario ends on the first integration tick after its stop time."""
        return round((math.floor(round(self.stop_time_s / dt_s, 8)) + 1) * dt_s, 8)

    def write_scene(self, path, controller):
        """Copy the scenario with absolute paths and the chosen controller; returns the controller properties."""
        tree = ET.parse(self.source)
        root = tree.getroot()
        root.find('./RoadNetwork/LogicFile').set('filepath', str(self.source.with_suffix('.xodr')))
        for directory in root.findall('./CatalogLocations/*/Directory'):
            directory.set('path', str((self.source.parent / directory.get('path')).resolve()))
        element = root.find(f'./Entities/ScenarioObject[@name="{self.ego}"]/ObjectController/Controller')
        if element is None:
            raise ValueError(f'{self.source.name}: {self.ego} has no ObjectController/Controller.')
        element.set('name', controller.esmini_name)
        properties = element.find('Properties')
        if controller.properties is not None:
            if properties is not None:
                element.remove(properties)
            properties = ET.SubElement(element, 'Properties')
            for name, value in controller.properties.items():
                ET.SubElement(properties, 'Property', name=name, value=str(value))
        tree.write(path, encoding='utf-8', xml_declaration=True)
        return {p.get('name'): p.get('value') for p in properties.findall('Property')} if properties is not None else {}


def load(key, entry):
    source = FOLDER / 'data' / entry['file']
    root = ET.parse(source).getroot()
    names = [item.get('name') for item in root.findall('./Entities/ScenarioObject')]
    ego = entry.get('ego', 'object_1')
    if ego not in names or len(names) < 2:
        raise ValueError(f'{source.name}: needs the ego {ego!r} and at least one other object.')
    stop = root.find('./Storyboard/StopTrigger/ConditionGroup/Condition/ByValueCondition/SimulationTimeCondition')
    if stop is None or stop.get('rule') != 'greaterThan':
        raise ValueError(f'{source.name}: needs a StopTrigger with a greaterThan SimulationTimeCondition.')
    ordered = [ego] + [name for name in names if name != ego]
    return Scenario(key=key, title=entry['title'], source=source, ego=ego,
                    objects=tuple({'id': index, 'name': name} for index, name in enumerate(ordered)),
                    stop_time_s=float(stop.get('value')))


@lru_cache(maxsize=None)
def all_scenarios():
    return read_json(FOLDER / 'scenarios.json')


@lru_cache(maxsize=None)
def get(key):
    entries = all_scenarios()
    if key not in entries:
        raise ValueError(f'Unknown scenario {key!r}. Available: {", ".join(entries)}')
    return load(key, entries[key])
