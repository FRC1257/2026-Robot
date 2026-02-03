package frc.robot.subsystems.HopperIntake;

//import static frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeIntakeSimConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class HopperIntakeSimIO implements HopperIntakeIO {
  private final FlywheelSim sim =
      new FlywheelSim(
        //figure out what's up with these
          LinearSystemId.createFlywheelSystem(
              DCMotor.getNEO(1), HopperIntakeConstants.HopperIntakeSimConstants.kMomentOfInertia, HopperIntakeConstants.HopperIntakeSimConstants.kHopperIntakeGearing),
          DCMotor.getNEO(1));
  private PIDController controller = new PIDController(0, 0, 0);

  private double appliedVoltage = 0.0;

  public HopperIntakeSimIO() {}

  @Override
  public void updateInputs(HopperIntakeIOInputs inputs) {
    sim.update(0.02);
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.tempCelcius = new double[] {60};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = volts;
    sim.setInputVoltage(volts);
  }
}