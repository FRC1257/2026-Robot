import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO{
    private final DCMotorSim motorSim;
    private double appliedVoltage;
    private PIDController pidController = new PIDController(0, 0, 0);
    
    public ShooterIOSim() {
        motorSim = new DCMotorSim(
            DCMotor.getNEO(1), 
            ShooterConstants.gearRatio, 
            ShooterConstants.flywheelInertia); //something wrong here
    }
    
    @Override
    public void updateInputs(ShooterIOInputs inputs){
        motorSim.setInputVoltage(appliedVoltage);
        motorSim.update(0.02);
        inputs.velocityRPM = motorSim.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
        inputs.motorCurrent = motorSim.getCurrentDrawAmps();
    }
    
    @Override 
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
        return motorSim.getAngularVelocityRPM();
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