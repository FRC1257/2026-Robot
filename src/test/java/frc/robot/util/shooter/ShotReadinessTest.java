package frc.robot.util.shooter;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
class ShotReadinessTest {
 @Test void timeoutNeverPermitsFeed(){var p=new ShotReadiness(.1);assertFalse(p.update(10,true,false,false));assertFalse(p.update(100,true,false,false));}
 @Test void dwellRecoveryAndInvalidation(){var p=new ShotReadiness(.1);
  assertFalse(p.update(0,true,true,true));assertFalse(p.update(.09,true,true,true));assertTrue(p.update(.11,true,true,true));
  assertTrue(p.update(.12,true,false,true));assertFalse(p.update(.13,true,false,false));
  assertFalse(p.update(.14,true,true,true));assertTrue(p.update(.25,true,true,true));
  assertFalse(p.update(.26,false,true,true));assertFalse(p.update(.27,true,true,true));
 }
 @Test void resetAndTimeReversalRequireDwell(){var p=new ShotReadiness(.1);p.update(10,true,true,true);assertFalse(p.update(1,true,true,true));assertTrue(p.update(1.2,true,true,true));p.reset();assertFalse(p.update(2,true,true,true));}
}
