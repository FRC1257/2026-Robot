package frc.robot.subsystems.Hopper.HopperPivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HopperPivotIOSim implements HopperPivotIO {

    private final DCMotor m_armGearbox = DCMotor.getNEO(2);
    private final ArmFeedforward feedforward = new ArmFeedforward(HopperPivotConstants.HOPPER_PIVOT_KS, HopperPivotConstants.HOPPER_PIVOT_KG, HopperPivotConstants.HOPPER_PIVOT_KV);
    private final PIDController controller = new PIDController(HopperPivotConstants.HOPPER_PIVOT_KP, 0, 0);

    private Voltage appliedVolts = Volts.of(0.0);

    private SingleJointedArmSim sim = 
        new SingleJointedArmSim(
            m_armGearbox,
            HopperPivotConstants.HopperPivotSimConstants.kArmReduction,
            SingleJointedArmSim.estimateMOI(
                HopperPivotConstants.HopperPivotSimConstants.kArmLength, 
                HopperPivotConstants.HopperPivotSimConstants.kArmMass),
            HopperPivotConstants.HopperPivotSimConstants.kArmLength,
            HopperPivotConstants.HopperPivotSimConstants.kArmAngleMin,
            HopperPivotConstants.HopperPivotSimConstants.kArmAngleMax,
            true,
            0.1
        );

    @Override
    public void updateInputs(HopperPivotIOInputs inputs) { 
        sim.update(0.02);
        inputs.leftpivotVoltage = appliedVolts;
        inputs.leftpivotAngle = Radians.of(sim.getAngleRads());
        inputs.leftpivotVelocity = RadiansPerSecond.of(sim.getVelocityRadPerSec());
        
    }

    @Override
    public void runVoltage(Voltage volts) {
        sim.setInputVoltage(volts.in(Volts));
        appliedVolts = volts;
    }

    @Override
    public void runAngle(double angle, double velocity) {
        runVoltage(Volts.of(MathUtil.clamp(controller.calculate(sim.getAngleRads(), angle) + feedforward.calculate(angle, velocity), -12, 12)));
    }

    @Override
    public void stop() {
        sim.setInputVoltage(0);
    }


}