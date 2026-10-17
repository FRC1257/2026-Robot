package frc.robot.commands;
import static org.junit.jupiter.api.Assertions.*;
import static edu.wpi.first.units.Units.*;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter.Flywheel.*;
import frc.robot.subsystems.Shooter.Hood.*;
import frc.robot.subsystems.Kicker.*;
import frc.robot.subsystems.ActiveFloor.*;
import edu.wpi.first.units.measure.*;
class ShotFeedTest {
 @Test void feedStopsOnLossOfAimAndCancellation() {
  HAL.initialize(500,0);SimHooks.pauseTiming();
  double[] output={0,0};boolean[] aligned={true};
  var fly=new Flywheel(new FlywheelIO(){}) {
   @Override public boolean feedbackValid(){return true;}
   @Override public Trigger isAtGoal(){return new Trigger(()->true);}
   @Override public boolean withinRecoveryBand(){return true;}
  };
  var hood=new Hood(new HoodIO(){}) {
   @Override public boolean feedbackValid(){return true;}
   @Override public Trigger isAtGoal(){return new Trigger(()->true);}
  };
  var kicker=new Kicker(new KickerIO(){
   @Override public void setVelocity(AngularVelocity v){output[0]=v.in(RadiansPerSecond);}
   @Override public void stop(){output[0]=0;}
  }) {@Override public boolean feedbackValid(){return true;}};
  var floor=new ActiveFloor(new ActiveFloorIO(){
   @Override public void setVoltage(Voltage v){output[1]=v.in(Volts);}
   @Override public void stop(){output[1]=0;}
  });
  var command=ShotFeed.create(fly,hood,kicker,floor,()->aligned[0]);
  try {
   command.initialize();command.execute();command.execute();assertEquals(0,output[0]);assertEquals(0,output[1]);
   SimHooks.stepTiming(.12);command.execute();command.execute();assertNotEquals(0,output[0]);assertNotEquals(0,output[1]);
   aligned[0]=false;command.execute();assertEquals(0,output[0]);assertEquals(0,output[1]);
   aligned[0]=true;command.execute();SimHooks.stepTiming(.12);command.execute();command.execute();
   assertNotEquals(0,output[0]);command.end(true);assertEquals(0,output[0]);assertEquals(0,output[1]);
  } finally {SimHooks.resumeTiming();}
 }
}
