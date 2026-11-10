"""Frozen draws produced by the original compiled C++ generator, not a second Python implementation."""
import hashlib
import json
from pathlib import Path
import unittest

from pipeline.errors.frames import iid_frames, markov_frames


class LegacyGeneratorTests(unittest.TestCase):
    def test_original_cpp_draws_are_preserved(self):
        data=json.loads((Path(__file__).parent/'data/markov_484675b.json').read_text())
        for fixture in data['fixtures']:
            with self.subTest(fixture=fixture):
                if fixture['model']=='iid':
                    flags=iid_frames(data['miss_probability'],fixture['seed'],fixture['target_id'],data['frames'])
                else:
                    flags=markov_frames(data['miss_probability'],fixture['mean_missing_s'],data['period_s'],
                                        fixture['seed'],fixture['target_id'],data['frames'])
                self.assertEqual(hashlib.sha256(bytes(flags)).hexdigest(),fixture['sha256'])


if __name__=='__main__':unittest.main()
