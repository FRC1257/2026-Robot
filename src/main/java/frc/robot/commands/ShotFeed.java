package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.ActiveFloor.ActiveFloor;
import frc.robot.util.shooter.ShotReadiness;
import org.littletonrobotics.junction.Logger;

/** Shared continuously evaluated feed permission for autonomous and teleop. */
public final class ShotFeed {
  private ShotFeed() {}
  public static Command create(Flywheel flywheel, Hood hood, Kicker kicker,
      ActiveFloor floor, BooleanSupplier aligned) {
    // Engineering starting value; validate with counted trials before competition.
    var policy = new ShotReadiness(0.10);
    boolean[] allowed = {false};
    return Commands.run(() -> {
      boolean valid=flywheel.feedbackValid() && hood.feedbackValid() && kicker.feedbackValid();
      boolean aim=hood.isAtGoal().getAsBoolean() && aligned.getAsBoolean();
      allowed[0]=policy.update(Timer.getFPGATimestamp(),valid,
          aim && flywheel.isAtGoal().getAsBoolean(),aim && flywheel.withinRecoveryBand());
      Logger.recordOutput("Shooter/FeedState",policy.state().name());
      Logger.recordOutput("Shooter/FeedAllowed",allowed[0]);
      Logger.recordOutput("Shooter/FeedbackValid",valid);
      Logger.recordOutput("Shooter/AimReady",aim);
    }).alongWith(kicker.runGatedIntake(()->allowed[0]),floor.runGated(()->allowed[0]))
      .beforeStarting(()->{policy.reset();allowed[0]=false;})
      .finallyDo(interrupted->{allowed[0]=false;Logger.recordOutput("Shooter/FeedAllowed",false);
        Logger.recordOutput("Shooter/FeedState","IDLE");});
  }
}
