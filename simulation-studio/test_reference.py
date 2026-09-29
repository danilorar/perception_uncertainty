"""Checks for the original-scenario time contract and invalid legacy baselines."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from scenario import Reference
from settings import REFERENCES, SCENARIOS, original_duration, validate, write_json


class ReferenceTests(unittest.TestCase):
    def test_original_stop_triggers_and_recorded_endpoints(self):
        for case in SCENARIOS:
            with self.subTest(case=case):
                ref = Reference(case)
                original = ET.parse(ref.meta['source_file'])
                recorded = ET.parse(ref.meta['reference_input'])
                def conditions(tree):
                    return [(e.tag, e.attrib) for e in tree.find('Storyboard/StopTrigger').iter()]
                self.assertEqual(conditions(original), conditions(recorded))
                self.assertEqual(ref.times[-1], ref.duration)
                self.assertEqual(len(ref.times), round(ref.duration / .01) + 1)
                self.assertTrue(all(abs(b-a-.01)<1e-7 for a,b in zip(ref.times,ref.times[1:])))
                ref.states(ref.duration)
                with self.assertRaises(ValueError):
                    ref.states(ref.duration + .01)

    def test_legacy_duration_cannot_extend_or_shorten_a_run(self):
        for case in SCENARIOS:
            for requested in [None, 1, 10, 20, 120]:
                with self.subTest(case=case, requested=requested):
                    config = validate({'scenario':case, 'duration_s':requested})
                    self.assertEqual(config['duration_s'], original_duration(case))

    def test_unverified_or_invalid_duration_is_rejected(self):
        metadata = json.loads((REFERENCES/'cpna/metadata.json').read_text(encoding='utf-8'))
        for changes in [{'timing_policy':'extended'}, {'original_stop_trigger_reached':False},
                        {'scenario':'ccrs'}, {'duration_s':0}, {'duration_s':True},
                        {'duration_s':float('nan')}]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                original_duration('cpna', metadata | changes)

    def test_extended_csv_cannot_pass_as_short_original_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory)/'cpna'
            target.mkdir()
            metadata = json.loads((REFERENCES/'cpna/metadata.json').read_text(encoding='utf-8'))
            metadata['duration_s'] -= .01
            write_json(target/'metadata.json', metadata)
            (target/'truth.csv').write_bytes((REFERENCES/'cpna/truth.csv').read_bytes())
            with patch('scenario.REFERENCES', Path(directory)), self.assertRaises(ValueError):
                Reference('cpna')

    def test_cpna_has_no_sideways_tail(self):
        ref = Reference('cpna')
        previous = None
        for t in ref.times:
            pedestrian = next(s for s in ref.states(t) if s['name']=='VRU')
            self.assertAlmostEqual(pedestrian['x'],150,places=6)
            if previous is not None:
                self.assertGreaterEqual(pedestrian['y'] + 1e-7, previous)
            previous = pedestrian['y']


if __name__ == '__main__':
    unittest.main(verbosity=2)
