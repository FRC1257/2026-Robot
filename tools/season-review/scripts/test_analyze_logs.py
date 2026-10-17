import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch
import analyze_logs
from analyze_logs import integrate, value_at, summarize_window

class DurationTests(unittest.TestCase):
    def test_inventory_excludes_viewing_derivatives(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for sub in ['2026','focus','electrical']:
                (root/'logs'/sub).mkdir(parents=True)
                (root/'logs'/sub/'example.wpilog').write_bytes(b'test')
            with patch.object(analyze_logs,'ROOT',root),patch.object(analyze_logs,'analyze',return_value={'enabled_windows':[]}) as parser:
                analyze_logs.main()
                parser.assert_called_once_with(root/'logs/2026/example.wpilog')

    def test_change_only_signal_duration(self):
        self.assertEqual(integrate([(0,False),(10,True),(15,False)],5,20,bool),5)
        self.assertEqual(integrate([(0,True)],5,20,bool),15)

    def test_exact_boundary_and_missing_prestate(self):
        self.assertIsNone(value_at([(10,True)],9))
        self.assertTrue(value_at([(10,True)],10))
        self.assertEqual(integrate([(10,True)],5,20,bool),10)

    def test_reset_jump_excluded_and_missing_signal_preserved(self):
        w=summarize_window({'/RealOutputs/Odometry/Robot':[(0,(0,0,0)),(1,(1,0,0)),(2,(100,0,0))]},0,2)
        self.assertEqual(w['estimated_pose_travel_m'],1)
        self.assertEqual(len(w['pose_jumps']),1)
        self.assertIsNone(w['browned_out_s'])

    def test_battery_min_includes_held_start_value(self):
        w=summarize_window({'/SystemStats/BatteryVoltage':[(0,8.),(12,11.)]},10,15)
        self.assertEqual(w['battery_min_v'],8.)
        self.assertEqual(w['battery_below_9v_s'],2.)

if __name__=='__main__':unittest.main()
