import unittest
from fuel_detector import intake_events
class IntakeTest(unittest.TestCase):
 def data(self):
  v=[-6.]*100;c=[3.]*100;s=[80.]*100;c[40]=15.;s[40]=50.
  return v,c,s
 def test_contact(self):self.assertEqual(intake_events(*self.data()),[40])
 def test_reverse_not_pickup(self):
  v,c,s=self.data();self.assertEqual(intake_events([6.]*100,c,s),[])
 def test_startup_not_contact(self):
  v,c,s=self.data();v[35]=0.;self.assertEqual(intake_events(v,c,s),[])
 def test_no_speed_drop(self):
  v,c,s=self.data();self.assertEqual(intake_events(v,c,[80.]*100),[])
 def test_missing_value(self):
  v,c,s=self.data();c[39]=None;self.assertEqual(intake_events(v,c,s),[])
 def test_refractory_period(self):
  v,c,s=self.data();c[45]=15.;s[45]=50.;self.assertEqual(intake_events(v,c,s),[40])
