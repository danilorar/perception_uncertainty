"""Build fixed, hashed object trajectories from the provided Vector scenarios."""
import argparse
import csv
import json
import math
import os
import shutil
import xml.etree.ElementTree as ET
from settings import ROOT, SOURCE, REFERENCES, SCENARIOS, sha256, write_json, RUNTIME
from engine import Engine

def apply_variation(tree, variation):
    """Apply every value from one official single-execution parameter set."""
    distribution = ET.parse(variation)
    values = {}
    for single in distribution.findall('.//DeterministicSingleParameterDistribution'):
        elements = single.findall('DistributionSet/Element')
        if len(elements) != 1:
            raise ValueError('场景参数集必须只包含一次执行。')
        values[single.get('parameterName')] = elements[0].get('value')
    for multi in distribution.findall('.//DeterministicMultiParameterDistribution'):
        sets = multi.findall('ValueSetDistribution/ParameterValueSet')
        if len(sets) != 1:
            raise ValueError('联合参数集必须只包含一次执行。')
        for assignment in sets[0].findall('ParameterAssignment'):
            name, value = assignment.get('parameterRef'), assignment.get('value')
            if name in values and values[name] != value:
                raise ValueError(f'原版参数集存在冲突: {name}')
            values[name] = value
    for name, value in values.items():
        parameter = tree.find(f"./ParameterDeclarations/ParameterDeclaration[@name='{name}']")
        if parameter is None:
            raise ValueError(f'原版参数未在基础场景声明: {name}')
        parameter.set('value', value)
    return values


def entity_definitions(tree, base_directory):
    """Keep original catalog bodies, including bicycle/motorbike categories."""
    parameters = {p.get('name'):p.get('value') for p in tree.findall('./ParameterDeclarations/ParameterDeclaration')}
    def resolve(value):
        return parameters[value[1:]] if value.startswith('$') else value
    catalogs = {}
    for directory in tree.findall('./CatalogLocations/*/Directory'):
        for path in (base_directory/directory.get('path')).glob('*.xosc'):
            catalog = ET.parse(path).find('Catalog')
            if catalog is not None:
                for body in catalog:
                    catalogs[catalog.get('name'),body.get('name')] = (body,path)
    definitions, sources = {}, {}
    for entity in tree.findall('./Entities/ScenarioObject'):
        reference = entity.find('CatalogReference')
        if reference is None or reference.find('ParameterAssignments') is not None:
            raise ValueError('当前基准生成器需要无局部参数覆盖的对象目录引用。')
        key = resolve(reference.get('catalogName')), resolve(reference.get('entryName'))
        body, path = catalogs[key]
        definitions[entity.get('name')] = ET.tostring(body, encoding='unicode')
        sources[str(path.resolve())] = sha256(path)
    return definitions, sources


def prepare(case):
    if SCENARIOS[case].get('source_kind') == 'teacher_trajectory':
        from teacher_data import prepare_teacher
        return prepare_teacher(case)
    out = REFERENCES / case
    out.mkdir(parents=True, exist_ok=True)
    os.chdir(out)
    source_copy = ROOT / 'assets/ncap'
    if not source_copy.exists():
        shutil.copytree(SOURCE, source_copy)
    relative = 'OpenSCENARIO/NCAP/CA-FC_2026/' + SCENARIOS[case]['file']
    original = SOURCE / relative
    template = source_copy / relative
    tree = ET.parse(original)
    variation = None
    applied = {}
    if SCENARIOS[case].get('variation'):
        variation = original.parent/'Variations/SingleExecution'/SCENARIOS[case]['variation']
        declared_source = ET.parse(variation).find('.//ScenarioFile').get('filepath')
        if (variation.parent/declared_source).resolve() != original.resolve():
            raise ValueError('原版参数集与基础场景不匹配。')
        applied = apply_variation(tree, variation)
    definitions, catalog_hashes = entity_definitions(tree, template.parent)
    # Keep the original StopTrigger, including its condition delays. The limit
    # below is a failure guard only; it must never become a replay duration.
    # Variants such as CCRm and CMRb share a base file but need distinct inputs.
    reference_input = template.with_name('_reference_' + case + '.xosc')
    ET.indent(tree)
    tree.write(reference_input, encoding='utf-8', xml_declaration=True)
    engine = Engine()
    try:
        engine.init(reference_input, out/'reference.log')
        states0 = engine.states()
        road = engine.lib.SE_GetODRFilename().decode()
        rows = []
        for i in range(12001):
            if i:
                engine.step(0.01)
            t = round(engine.lib.SE_GetSimulationTime(), 8)
            states = engine.states()
            # esmini can signal completion without advancing another timestep.
            if rows and rows[-1]['time_s'] == t:
                del rows[-len(states):]
            for s in states:
                if not all(math.isfinite(s[k]) for k in ['x','y','z','h','speed']):
                    raise RuntimeError('基准轨迹包含非有限值')
                rows.append({'time_s': t, **{k:s[k] for k in ['id','name','x','y','z','h','p','r','speed']}})
            if engine.lib.SE_GetQuitFlag():
                break
        else:
            raise RuntimeError('原版场景在 120 秒内未触发结束条件；拒绝生成延长基准。')
        duration = round(engine.lib.SE_GetSimulationTime(), 8)
        if duration <= 0 or abs(duration - rows[-1]['time_s']) > 1e-7:
            raise RuntimeError('原版结束时间与基准采样时间不一致。')
        with (out/'truth.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, rows[0].keys())
            writer.writeheader(); writer.writerows(rows)
        write_json(out/'metadata.json', {'scenario':case, 'source_file':str(original),
            'source_sha256':sha256(original), 'truth_sha256':sha256(out/'truth.csv'),
            'variation_file': str(variation) if variation else None,
            'variation_sha256': sha256(variation) if variation else None,
            'applied_parameters':applied, 'entity_definitions':definitions,
            'catalog_files_sha256':catalog_hashes,
            'dt_s':.01, 'duration_s':duration, 'timing_policy':'original_stop_trigger',
            'original_stop_trigger_reached':True, 'objects':states0, 'road_file':road,
            'reference_input_sha256':sha256(reference_input),
            'reference_input':str(reference_input), 'reference_args':engine.args,
            'esmini_version':(RUNTIME/'version.txt').read_text(),
            'method':'Provided scenario actions and unchanged original StopTrigger. Apply all single and joint values from the named official single-execution variation, or use base defaults when no variation is named. Record until esmini reports scenario completion, including the final sample. Replay only this original baseline time interval. Original source files untouched.',
            'derived_scenario':'Experiments use fixed-object replay, not the unmodified NCAP synchronization logic; no protocol compliance claim.'})
    finally:
        engine.close()
    print(json.dumps({'prepared':case,'frames':len(rows)//len(states0),'duration_s':duration,'objects':len(states0)}), flush=True)

if __name__ == '__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('scenario',choices=SCENARIOS)
    prepare(parser.parse_args().scenario)
