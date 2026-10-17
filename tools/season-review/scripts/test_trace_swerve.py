import unittest
from trace_swerve import intervals, MEASURED, COMMAND, ENABLED
class TraceTests(unittest.TestCase):
    def test_held_values_and_recovery(self):
        d={ENABLED:[(0,True)],MEASURED:[(0,(2,0,0,0,2,0,2,0)),(3,(2,0,2,0,2,0,2,0))]}
        r=intervals(d,5)
        self.assertEqual(r['module1_zero_during_peer_motion_s'],3)
        self.assertEqual(r['module1_nonzero_during_peer_motion_s'],2)
        self.assertEqual(r['zero_intervals_s'],[[0,3]])
    def test_missing_feedback_not_zero(self):
        r=intervals({ENABLED:[(0,True)]},5)
        self.assertEqual(r['peer_motion_enabled_s'],0)
    def test_disabled_and_stationary_not_evidence(self):
        d={ENABLED:[(0,False),(2,True)],MEASURED:[(0,(2,0,0,0,2,0,2,0)),(3,(0,0,0,0,0,0,0,0))]}
        self.assertEqual(intervals(d,5)['module1_zero_during_peer_motion_s'],1)
    def test_signed_reverse_is_motion(self):
        d={ENABLED:[(0,True)],MEASURED:[(0,(-2,0,-2,0,-2,0,-2,0))]}
        self.assertEqual(intervals(d,5)['module1_nonzero_during_peer_motion_s'],5)
if __name__=='__main__':unittest.main()
