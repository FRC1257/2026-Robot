package frc.robot.util.autonomous;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.AfterEach;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.concurrent.atomic.AtomicBoolean;
class AutoStageTest {
  @BeforeAll static void setup() { HAL.initialize(500, 0); SimHooks.pauseTiming(); }
  @AfterEach void cleanup() { edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().clearComposedCommands(); }
  @Test void failedPositionNeverStartsContinuation() {
    var continued=new AtomicBoolean(); var fallback=new AtomicBoolean(); var stopped=new AtomicBoolean();
    var c=AutoStage.moveThen("test",Commands.runEnd(()->{},()->stopped.set(true)),()->false,.1,
        Commands.runOnce(()->continued.set(true)),Commands.runOnce(()->fallback.set(true)));
    c.initialize(); for(int i=0;i<20 && !c.isFinished();i++){c.execute();SimHooks.stepTiming(.02);} c.end(false);
    assertTrue(stopped.get());assertTrue(fallback.get());assertFalse(continued.get());
  }
  @Test void readyStartsContinuation() {
    var continued=new AtomicBoolean();var fallback=new AtomicBoolean();
    var c=AutoStage.moveThen("test",Commands.run(()->{}),()->true,.1,
        Commands.runOnce(()->continued.set(true)),Commands.runOnce(()->fallback.set(true)));
    c.initialize();for(int i=0;i<10&&!c.isFinished();i++)c.execute();c.end(false);
    assertTrue(continued.get());assertFalse(fallback.get());
  }
  @Test void deadlineInterruptsContinuousCompanion() {
    var stopped=new AtomicBoolean();var scored=new AtomicBoolean();
    var c=Commands.waitSeconds(.1).deadlineWith(Commands.runEnd(()->{},()->stopped.set(true)))
        .andThen(Commands.runOnce(()->scored.set(true)));
    c.initialize();for(int i=0;i<20&&!c.isFinished();i++){c.execute();SimHooks.stepTiming(.02);}c.end(false);
    assertTrue(stopped.get());assertTrue(scored.get());
  }
}
