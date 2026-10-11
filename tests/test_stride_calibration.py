"""Synthetic model recovery and input rejection; not human accuracy evidence."""
import sys
import unittest
import tempfile
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from calibrate_stride import fit, load

class StrideCalibrationTests(unittest.TestCase):
    def trials(self):
        return [(hz, .3+.2*hz, 100, 100*(.3+.2*hz), split)
                for split, rates in [('train',[1,1,1.5,1.5,2,2]),
                                     ('validation',[1.1,1.6,1.9])]
                for hz in rates]
    def test_recovery(self):
        _, a, b, lo, hi, errors = fit(self.trials())
        self.assertAlmostEqual(a,.3)
        self.assertAlmostEqual(b,.2)
        self.assertEqual((lo,hi),(1,2))
        self.assertLess(errors['adaptive'],1e-10)
        self.assertGreater(errors['fixed'],errors['adaptive'])
    def test_missing_validation(self):
        with self.assertRaises(ValueError): fit(self.trials()[:6])
    def test_insufficient_range(self):
        rows=[(1.5,r[1],r[2],r[3],r[4]) for r in self.trials()]
        with self.assertRaises(ValueError): fit(rows)
    def test_invalid_input(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'trial.csv'
            for steps in ['0','nan','10.5']:
                path.write_text('distance_m,steps,duration_s,split\n20,'+steps+',20,train\n')
                with self.assertRaises(ValueError): load(path)

if __name__=='__main__': unittest.main()
