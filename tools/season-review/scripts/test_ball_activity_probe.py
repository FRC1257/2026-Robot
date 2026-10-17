import unittest
from ball_activity_probe import dips
class DipTest(unittest.TestCase):
 def data(self):
  s=[300.]*100;s[40:44]=[280.,285.,290.,295.]
  return s,[300.]*100,[4.]*100,[12.]*100
 def test_single_recovered_event(self):self.assertEqual(len(dips(*self.data(),10)),1)
 def test_no_feeding(self):
  s,g,k,b=self.data();self.assertEqual(dips(s,g,[0.]*100,b,10),[])
 def test_setpoint_change(self):
  s,g,k,b=self.data();g[39:]=[270.]*61;self.assertEqual(dips(s,g,k,b,10),[])
 def test_voltage_sag(self):
  s,g,k,b=self.data();b[40]=8.;self.assertEqual(dips(s,g,k,b,10),[])
 def test_missing_speed(self):
  s,g,k,b=self.data();s[40]=None;self.assertEqual(dips(s,g,k,b,10),[])
 def test_sustained_drop_not_a_recovered_dip(self):
  s,g,k,b=self.data();s[40:]=[280.]*60;self.assertEqual(dips(s,g,k,b,10),[])
if __name__=='__main__':unittest.main()
