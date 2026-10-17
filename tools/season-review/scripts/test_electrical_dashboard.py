import unittest
from electrical_dashboard import states, increased, KEYS, GRAY, GREEN, RED, AMBER, schematic


class ElectricalRulesTest(unittest.TestCase):
    def series(self, **values):
        return {KEYS[k]: [(0, v)] for k,v in values.items()}

    def test_missing_is_unknown(self):
        self.assertTrue(all(c == GRAY for c,_ in states({},0).values()))

    def test_brownout_and_output_loss(self):
        data=self.series(battery=4.04,threshold=6.75,brownout=True,active=False,enabled=True)
        result=states(data,0)
        self.assertEqual(result['BATTERY'][0],RED)
        self.assertEqual(result['RIO'][0],RED)
        self.assertEqual(result['PDH'][0],GRAY)

    def test_disabled_not_reported_as_failure(self):
        self.assertEqual(states(self.series(active=False,enabled=False),0)['RIO'][0],AMBER)

    def test_historical_counter_and_expiry(self):
        data={KEYS['can_OffCount']:[(0,3),(2_000_000,4)]}
        self.assertFalse(increased(data,'can_OffCount',1_000_000))
        self.assertTrue(increased(data,'can_OffCount',2_500_000))
        self.assertFalse(increased(data,'can_OffCount',3_000_000))
        # Counter reset is not a new bus-off event.
        data[KEYS['can_OffCount']].append((4_000_000,0))
        self.assertFalse(increased(data,'can_OffCount',4_000_000))

    def test_sticky_is_historical_warning(self):
        data=self.series(pd_faults=0,pd_sticky=1,pd_voltage=12,threshold=6.75)
        self.assertEqual(states(data,0)['PDH'][0],AMBER)
        data[KEYS['pd_faults']]=[(0,1)]
        self.assertEqual(states(data,0)['PDH'][0],AMBER)
        self.assertIn('verification',states(data,0)['PDH'][1])

    def test_pdh_undervoltage_takes_priority_over_raw_fault_caution(self):
        data=self.series(pd_faults=1,pd_voltage=4,threshold=6.75)
        self.assertEqual(states(data,0)['PDH'][0],RED)

    def test_connected_holds_and_no_future_leak(self):
        data={KEYS['radio']:[(1_000_000,True),(2_000_000,False)]}
        self.assertEqual(states(data,0)['RADIO'][0],GRAY)
        self.assertEqual(states(data,1_500_000)['RADIO'][0],GREEN)
        self.assertEqual(states(data,2_000_000)['RADIO'][0],RED)

    def test_each_component_has_native_color_fields(self):
        f=schematic()
        self.assertEqual(set(f.dynamic),set(states({},0)))
        for paths in f.dynamic.values():
            self.assertEqual(len(paths),5)
            self.assertTrue(all(p in f.initial for p in paths))


if __name__=='__main__':unittest.main()
