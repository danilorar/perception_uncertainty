"""Source-parameter, target-identity and scene-generation regression checks."""
from collections import Counter
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from prepare import apply_variation
from scenario import Reference, build_scene, object_kind
from settings import LEGACY_SCENARIOS as SCENARIOS, SOURCE, sha256

ADDED = {'cpfa':'pedestrian', 'cpla':'pedestrian', 'cbna':'bicycle', 'cbla':'bicycle',
         'cmrs':'motorbike', 'cmrb':'motorbike', 'ccrm':'car', 'ccrb':'car'}


class ScenarioTests(unittest.TestCase):
    def test_archived_ncap_catalog_retained_for_history(self):
        self.assertEqual(set(SCENARIOS), set(ADDED) | {'ccrs','cpna','cccscp'})
        self.assertEqual(Counter(SCENARIOS[c]['category'] for c in ADDED),
                         Counter({'pedestrian':2,'bicycle':2,'motorbike':2,'car':2}))

    def test_all_official_parameters_applied_and_sources_preserved(self):
        paths = set()
        for case in ADDED:
            with self.subTest(case=case):
                ref = Reference(case)
                self.assertEqual(sha256(ref.meta['source_file']), ref.meta['source_sha256'])
                self.assertEqual(sha256(ref.meta['variation_file']), ref.meta['variation_sha256'])
                self.assertEqual(sha256(ref.meta['reference_input']), ref.meta['reference_input_sha256'])
                path = str(Path(ref.meta['reference_input']).resolve()).lower()
                self.assertNotIn(path, paths)
                paths.add(path)
                official = ET.parse(ref.meta['variation_file'])
                expected = {s.get('parameterName'):s.find('DistributionSet/Element').get('value')
                            for s in official.findall('.//DeterministicSingleParameterDistribution')}
                expected.update({s.get('parameterRef'):s.get('value')
                                 for s in official.findall('.//ParameterValueSet/ParameterAssignment')})
                actual = {s.get('name'):s.get('value') for s in
                          ET.parse(path).findall('./ParameterDeclarations/ParameterDeclaration')}
                self.assertEqual(expected, ref.meta['applied_parameters'])
                self.assertEqual(expected, {name:actual[name] for name in expected})
                self.assertEqual(actual['Scenario_ID'].lower(), case)
                self.assertEqual(float(actual['Ego_speed_kph']), SCENARIOS[case]['speed'])

    def test_target_category_shape_and_renderer_model(self):
        model_ids = {'pedestrian':'7','bicycle':'9','motorbike':'10','car':'2'}
        for case, kind in ADDED.items():
            with self.subTest(case=case), tempfile.TemporaryDirectory() as directory:
                ref = Reference(case)
                targets = [s for s in ref.states(0) if s['id'] != ref.ego_id]
                target = next(s for s in targets if object_kind(s,ref.meta) == kind)
                path = Path(directory)/'scene.xosc'
                build_scene(path, ref.meta, ref.states(0), ref.duration)
                entity = ET.parse(path).find(f"./Entities/ScenarioObject[@name='{target['name']}']")
                body = entity.find('Pedestrian' if kind=='pedestrian' else 'Vehicle')
                self.assertIsNotNone(body)
                if kind != 'pedestrian':
                    self.assertEqual(body.get('vehicleCategory'), kind)
                self.assertEqual(body.find("Properties/Property[@name='model_id']").get('value'), model_ids[kind])
                dimensions = body.find('BoundingBox/Dimensions')
                for axis in ['length','width','height']:
                    self.assertAlmostEqual(float(dimensions.get(axis)), target[axis], places=5)
                source_body = ET.fromstring(ref.meta['entity_definitions'][target['name']])
                if kind != 'pedestrian':
                    for axle in ['FrontAxle','RearAxle']:
                        self.assertEqual(body.find('Axles/'+axle).attrib, source_body.find('Axles/'+axle).attrib)

    def test_parameter_distribution_cannot_silently_drop_joint_values(self):
        base = SOURCE/'OpenSCENARIO/NCAP/CA-FC_2026'
        cpla = ET.parse(base/'CBLA.xosc')
        values = apply_variation(cpla,base/'Variations/SingleExecution/CPLA_50_50kph.xosc')
        self.assertEqual(values['Target_catalogEntry'],'NCAP_Adult')
        self.assertEqual(values['Target_catalogName'],'Pedestrians')
        cmrb = ET.parse(base/'CCRs.xosc')
        values = apply_variation(cmrb,base/'Variations/SingleExecution/CMRb_50kph.xosc')
        self.assertEqual(values['Ego_speed_kph'],'50')
        self.assertEqual(values['Target_init_speed_kph'],'50')


if __name__ == '__main__':
    unittest.main(verbosity=2)
