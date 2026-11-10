"""Cone visibility must follow detection, independent of other invalid measurements."""
from pathlib import Path
import tempfile
import unittest

from pipeline.visualization.motion import detection_column, flag_at, read_detection_flags


class DetectionLogTests(unittest.TestCase):
    def test_range_invalidity_is_not_a_detection_dropout(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'perception_applied.tsv'
            path.write_text('time_s\tobject_name\tvalid\tdetection.missed\n'
                            '0\tobject_2\t0\t0\n0.05\tobject_2\t1\t1\n0.1\tobject_2\t1\t0\n')
            flags=read_detection_flags(path,'object_2','detection.missed')
        self.assertTrue(flag_at(flags,0))       # bad range, but detector did not miss
        self.assertFalse(flag_at(flags,.07))   # prediction may be valid while detector misses
        self.assertTrue(flag_at(flags,.1))

    def test_custom_plugin_name_and_ambiguity(self):
        config={'errors':[{'type':'detection','name':'front'}, {'type':'detection','name':'rear'}]}
        with self.assertRaisesRegex(ValueError,'Multiple'):detection_column(config)
        self.assertEqual(detection_column(config,'front'),'front.missed')
        with self.assertRaises(ValueError):detection_column(config,'missing')

    def test_missing_flags_fail_instead_of_inventing_detection(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'log.tsv';path.write_text('time_s\tobject_name\tvalid\n0\tobject_2\t1\n')
            with self.assertRaisesRegex(ValueError,'missing'):read_detection_flags(path,'object_2','detection.missed')
        with self.assertRaises(ValueError):flag_at({.05:True},0)

    def test_no_detection_error_reference_is_explicit(self):
        self.assertIsNone(detection_column({'errors':[{'type':'ou'}]}))
        self.assertTrue(flag_at(read_detection_flags(None,'object_2',None),3))


if __name__=='__main__':unittest.main()
