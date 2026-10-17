package frc.robot.util.autonomous;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

/** A bounded positioning stage; failure owns the continuation and cannot fall through. */
public final class AutoStage {
  private AutoStage() {}
  public static Command moveThen(String name, Command move, BooleanSupplier ready,
      double timeoutSeconds, Command continuation, Command fallback) {
    if (!Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0) {
      throw new IllegalArgumentException("Stage timeout must be finite and positive");
    }
    return move.until(ready).withTimeout(timeoutSeconds)
        .beforeStarting(() -> Logger.recordOutput("Auto/Stage", name))
        .andThen(Commands.either(
            Commands.runOnce(() -> Logger.recordOutput("Auto/StageExitReason", "ready"))
                .andThen(continuation),
            Commands.runOnce(() -> Logger.recordOutput("Auto/StageExitReason", "timeout"))
                .andThen(fallback), ready))
        .finallyDo(interrupted -> {
          if (interrupted) Logger.recordOutput("Auto/InterruptedStage", name);
        });
  }
}
