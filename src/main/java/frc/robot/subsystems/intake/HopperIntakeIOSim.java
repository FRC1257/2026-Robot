package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.HopperIntakeConstants.HopperIntakeSimConstants.*;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class HopperIntakeIOSim implements HopperIntakeIO {

 private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getNEO(1), kMomentOfInertia, kHopperIntakeGearing),
          DCMotor.getNEO(1));// would change if chain and not directly connected?

private double appliedVoltage = 0.0;

  public HopperIntakeIOSim() {}

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