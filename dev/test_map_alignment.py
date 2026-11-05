"""Numerical regressions for map export, including origin yaw and category borders."""
import ast
import hashlib
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts/calib'))
from align_map import export_map
from map_to_png import read_pgm


class MapAlignmentTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.image = np.array([[0, 254, 205], [254, 0, 254]], dtype=np.uint8)
        self.write_source([1.2, -2.3, 0])

    def tearDown(self):
        self.temporary.cleanup()

    def write_source(self, origin):
        (self.directory / 'source.pgm').write_bytes(b'P5\n3 2\n255\n' + self.image.tobytes())
        (self.directory / 'source.yaml').write_text(
            f'image: source.pgm\nmode: trinary\nresolution: 0.05\norigin: {origin}\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')

    def output(self):
        return read_pgm(str(self.directory / 'aligned.pgm'))

    def test_zero_rotation_and_original_bytes(self):
        original = hashlib.sha256((self.directory / 'source.pgm').read_bytes()).digest()
        export_map(self.directory, 'source', 'aligned', 0)
        np.testing.assert_array_equal(self.output(), self.image)
        self.assertEqual(hashlib.sha256((self.directory / 'source.pgm').read_bytes()).digest(), original)
        self.assertTrue((self.directory / 'aligned.png').read_bytes().startswith(b'\x89PNG\r\n\x1a\n'))

    def test_quarter_turns(self):
        for turns in (-1, 1):
            with self.subTest(turns=turns):
                name = 'turn' + ('positive' if turns > 0 else 'negative')
                export_map(self.directory, 'source', name, turns*math.pi/2)
                np.testing.assert_array_equal(read_pgm(str(self.directory / (name + '.pgm'))), np.rot90(self.image, turns))

    def test_source_yaw_and_nonzero_origin(self):
        self.write_source([1.2, -2.3, 0.3])
        export_map(self.directory, 'source', 'aligned', -0.3)
        np.testing.assert_array_equal(self.output(), self.image)
        metadata = dict(line.split(': ', 1) for line in (self.directory / 'aligned.yaml').read_text().splitlines())
        origin = ast.literal_eval(metadata['origin'])
        expected = [math.cos(.3)*1.2+math.sin(.3)*-2.3, -math.sin(.3)*1.2+math.cos(.3)*-2.3, 0]
        np.testing.assert_allclose(origin, expected, atol=1e-10)

    def test_unknown_border_and_no_interpolation(self):
        export_map(self.directory, 'source', 'aligned', .3)
        values = set(np.unique(self.output()))
        self.assertTrue(values <= {0, 254, 205, 140})
        self.assertIn(140, values)
        self.assertTrue(.25 < (255-140)/255 < .65)

    def test_conflicts_and_invalid_inputs(self):
        export_map(self.directory, 'source', 'aligned', 0)
        before = (self.directory / 'aligned.yaml').read_bytes()
        with self.assertRaises(FileExistsError):
            export_map(self.directory, 'source', 'aligned', .2)
        self.assertEqual((self.directory / 'aligned.yaml').read_bytes(), before)
        for angle in (float('nan'), float('inf'), 2):
            with self.assertRaises(ValueError):
                export_map(self.directory, 'source', 'invalid', angle)
        with self.assertRaises(ValueError):
            export_map(self.directory, '../source', 'invalid', 0)
        self.assertFalse((self.directory / 'invalid.yaml').exists())


if __name__ == '__main__':
    unittest.main()
