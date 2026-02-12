package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim motorSim;
    private double appliedVoltage = 0.0;
    private PIDController pidController = new PIDController(0, 0, 0);
    
    public ShooterIOSim() {
        final DCMotor motorGearbox = DCMotor.getNEO(1);

        motorSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), 
                ShooterConstants.flywheelInertia, 
                ShooterConstants.gearRatio
            ), 
            motorGearbox
        );
    }
    
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Update motor simulation with current voltage
        motorSim.setInputVoltage(appliedVoltage);
        motorSim.update(0.02);  // 20ms loop time
        
        // Update inputs from simulation
        inputs.velocityRPM = motorSim.getAngularVelocityRPM();
        inputs.appliedVoltage = appliedVoltage;
        inputs.motorCurrent = motorSim.getCurrentDrawAmps();
    }
    
    @Override
    public void setVoltage(Voltage voltage) {
        appliedVoltage = voltage.in(Volts);  // Convert to double
    }

    @Override
    public double getVoltage() {
        return appliedVoltage;
    }

    @Override
    public void setRPM(AngularVelocity rpm) {
        double targetRPM = rpm.in(RPM);  // Now this works!
        double voltage = pidController.calculate(getRPM(), targetRPM);
        voltage = Math.max(-12.0, Math.min(12.0, voltage));
        appliedVoltage = voltage;  // Set the voltage directly
    }

    @Override
    public double getRPM() {
        return motorSim.getAngularVelocityRPM();
    }
    
    @Override
    public void setPIDGains(double Kp, double Ki, double Kd) {
        pidController.setPID(Kp, Ki, Kd);
    }

    @Override
    public void stop() {
        this.appliedVoltage = 0.0;
    }
}