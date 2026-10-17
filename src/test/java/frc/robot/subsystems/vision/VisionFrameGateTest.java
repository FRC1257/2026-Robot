package frc.robot.subsystems.vision;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;
class VisionFrameGateTest {
 @Test void acceptsOncePerCamera(){var g=new VisionFrameGate(.5);assertEquals("accepted",g.check(0,10,10.1));assertEquals("duplicate-or-old",g.check(0,10,10.2));assertEquals("accepted",g.check(1,10,10.2));assertEquals("duplicate-or-old",g.check(0,9,10.2));}
 @Test void rejectsStaleFutureAndNanWithoutPoisoning(){var g=new VisionFrameGate(.5);assertEquals("future",g.check(0,100,10));assertEquals("accepted",g.check(0,10,10.1));assertEquals("invalid-time",g.check(0,Double.NaN,10));assertEquals("stale",g.check(0,11,12));assertEquals("duplicate-or-old",g.check(0,11,12));}
 @Test void alignmentWrapsAcrossPi(){assertEquals(2,Math.abs(Rotation2d.fromDegrees(-179).minus(Rotation2d.fromDegrees(179)).getDegrees()),1e-9);}
}
