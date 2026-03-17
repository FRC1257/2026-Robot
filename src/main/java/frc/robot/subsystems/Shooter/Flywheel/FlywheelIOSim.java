package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
    private final FlywheelSim sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(2), 0.3, 34/32), DCMotor.getNeoVortex(2));
    private PIDController controller = new PIDController(200, 0, 0);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 60);
    private Voltage appliedVoltage = Volts.of(0.0);
 
    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        sim.update(0.02);
        inputs.flywheelAngularVelocity = RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
        inputs.flywheelVoltage = appliedVoltage;
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        appliedVoltage = 
            Volts.of(MathUtil.clamp(controller.calculate(sim.getAngularVelocityRadPerSec(), velocity.in(RadiansPerSecond))+feedforward.calculate(velocity.in(RadiansPerSecond)), -12, 12));
        sim.setInputVoltage(appliedVoltage.in(Volts));
    }
}
