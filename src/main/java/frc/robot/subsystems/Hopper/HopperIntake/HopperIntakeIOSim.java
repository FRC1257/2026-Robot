package frc.robot.subsystems.Hopper.HopperIntake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class HopperIntakeIOSim implements HopperIntakeIO {
    private final DCMotor m_motor = DCMotor.getNEO(1);

    private  final FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            m_motor,
             HopperIntakeConstants.HopperIntakeSimConstants.kMomentOfInertia, 
             HopperIntakeConstants.HopperIntakeSimConstants.kHopperIntakeGearing),
        m_motor
    );

    @Override
    public void updateInputs(HopperIntakeIOInputs inputs) {
        sim.update(0.02);
        inputs.intakeVoltage = Volts.of(sim.getInputVoltage());
        inputs.intakeVelocity = sim.getAngularVelocity();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        sim.setInputVoltage(voltage.in(Volts));
    }
    
}