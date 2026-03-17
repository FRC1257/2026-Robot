package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.Hopper.HopperPivot.HopperPivotConstants;

public class HoodIOSim implements HoodIO {

    private final DCMotor m_armGearbox = DCMotor.getNEO(1);
    private final ArmFeedforward feedforward = new ArmFeedforward(0, 0.0, 4);
    private final PIDController controller = new PIDController(5, 0, 0);

    private Voltage appliedVolts = Volts.of(0);

    private SingleJointedArmSim sim = 
        new SingleJointedArmSim(
            m_armGearbox,
            HopperPivotConstants.HopperPivotSimConstants.kArmReduction,
            SingleJointedArmSim.estimateMOI(
                HopperPivotConstants.HopperPivotSimConstants.kArmLength, 
                HopperPivotConstants.HopperPivotSimConstants.kArmMass),
            HopperPivotConstants.HopperPivotSimConstants.kArmLength,
            HoodConstants.HOOD_MIN_ANGLE.in(Radians),
            HoodConstants.HOOD_MAX_ANGLE.in(Radians),
            false,
            0
        );

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        sim.update(0.02);
        inputs.hoodVolts = appliedVolts;
        inputs.hoodAngle = Radians.of(sim.getAngleRads());
        inputs.hoodVelocity = RadiansPerSecond.of(sim.getVelocityRadPerSec());
    }

    @Override
    public void runAngle(double angle, double velocity) {
        sim.setInputVoltage(controller.calculate(sim.getAngleRads(), angle)+feedforward.calculate(angle, velocity));
        appliedVolts = Volts.of(controller.calculate(sim.getAngleRads(), angle)+feedforward.calculate(angle, velocity));
    }


}
