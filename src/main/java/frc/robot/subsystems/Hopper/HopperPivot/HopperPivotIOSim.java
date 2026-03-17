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
    private final ArmFeedforward feedforward = new ArmFeedforward(0, 0.0, 4);
    private final PIDController controller = new PIDController(10, 0, 0);

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
            false,
            0
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
        runVoltage(Volts.of(controller.calculate(sim.getAngleRads(), angle)+feedforward.calculate(angle, velocity)));
    }

    @Override
    public void stop() {
        sim.setInputVoltage(0);
        appliedVolts = Volts.of(0.0);
    }


}