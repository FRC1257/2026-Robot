import unittest
from electrical_dashboard import at
from power_details import updates,motor_inventory,bar_fraction,REVIEW_CHANNELS

class PowerDetailsTests(unittest.TestCase):
    def test_channels_retain_index_and_exact_raw_value(self):
        s={'/PowerDistribution/ChannelCurrent':[(100,list(range(24)))]}
        rows={k:v for k,typ,v in updates(s,100,at,motor_inventory(s))}
        for ch in range(24):self.assertEqual(rows[f'/Electrical/PDH/CH{ch:02}Amps'],ch)
        self.assertEqual(rows['/Electrical/Map/CHBar4/L/color'],'#ffbf47')
        self.assertEqual(rows['/Electrical/Map/CHBar0/L/color'],'#00a2ff')

    def test_missing_current_not_reported_as_zero(self):
        s={};rows={k:v for k,typ,v in updates(s,0,at,motor_inventory(s))}
        self.assertNotIn('/Electrical/PDH/CH00Amps',rows)
        self.assertEqual(rows['/Electrical/MotorMap/currentBar0/L/color'],'#78889b')

    def test_bar_saturates_without_corrupting_value(self):
        self.assertEqual(bar_fraction(200,60),1)
        self.assertEqual(bar_fraction(-6,12),.5)
        s={'/PowerDistribution/ChannelCurrent':[(0,[200]) ]}
        rows={k:v for k,typ,v in updates(s,0,at,motor_inventory(s))}
        self.assertEqual(rows['/Electrical/PDH/CH00Amps'],200)

    def test_legacy_and_new_motor_profiles(self):
        old={'/Flywheel/FlywheelCurrent':[(0,2)]}
        new={'/Flywheel/FlywheelLeaderCurrent':[(0,3)]}
        self.assertEqual(motor_inventory(old)['FLYWHEEL']['current'],'/Flywheel/FlywheelCurrent')
        self.assertEqual(motor_inventory(new)['FLYWHEEL']['current'],'/Flywheel/FlywheelLeaderCurrent')

    def test_mechanism_angle_uses_radians_and_does_not_anticipate_sample(self):
        import math
        s={'/Hood/HoodAngle':[(100,math.pi/2)]}
        before={k:v for k,typ,v in updates(s,99,at,motor_inventory(s))}
        after={k:v for k,typ,v in updates(s,100,at,motor_inventory(s))}
        key='/Electrical/RobotMechanisms/Arm0/L/angle'
        self.assertNotIn(key,before)
        self.assertAlmostEqual(after[key],90)

if __name__=='__main__':unittest.main()
