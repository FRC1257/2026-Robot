package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO{
    private final FlywheelSim motorSim;
    private double appliedVoltage;
    private PIDController pidController = new PIDController(0, 0, 0);
    public ShooterIOSim() {
    final DCMotor motorGearbox = DCMotor.getNEO(1);

    motorSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), ShooterConstants.kMomentOfInertia, ShooterConstants.kIntakeGearing), motorGearbox);
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
    public void setVoltage(double voltage){
        appliedVoltage = voltage;
    }

    @Override
    public double getVoltage(){
        return appliedVoltage;
    }

    @Override
    public void setRPM(double rpm){
        double voltage = pidController.calculate(getRPM(), rpm); 
        voltage = Math.max(-12.0, Math.min(12.0, voltage));
        setVoltage(voltage);
    }

    @Override
    public double getRPM(){
        double velRadPerSec = motorSim.getAngularVelocityRadPerSec();
        return velRadPerSec * 60.0 / (2.0 * Math.PI);
    }
    @Override
    public void setPIDGains(double Kp, double Ki, double Kd) {
        pidController.setPID(Kp, Ki, Kd);
    }

    @Override
    public void stop(){
        this.appliedVoltage = 0.0;
    }
}
