package frc.robot.subsystems.kicker;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;




public class KickerIOSim implements KickerIO{
    private DCMotorSim motorSim;
    private double appliedVoltage;    
   
    public KickerIOSim(){
        
        motorSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.01, 1.0),
                DCMotor.getNEO(1));
    }


    @Override
    public void updateInputs(KickerIOInputs inputs){
        motorSim.setInputVoltage(appliedVoltage);
        motorSim.update(0.02);
       
        inputs.velocityRPM = motorSim.getAngularVelocityRPM();
        inputs.appliedVoltage = appliedVoltage;
        inputs.motorCurrent = motorSim.getCurrentDrawAmps();
        inputs.TemperatureCelsius = 25.0;
    }


    @Override
    public void setVoltage(double voltage){
        this.appliedVoltage = voltage;
    }


    @Override
    public double getVoltage(){
        return appliedVoltage;
    }


    @Override
    public void setRPM(double RPM){
        double voltage = (RPM / KickerConstants.kickRPM) * KickerConstants.kickVoltage;
        setVoltage(voltage);
    }


    @Override
    public double getRPM(){
        return motorSim.getAngularVelocityRPM();
    }


    @Override
    public void stop(){
        this.appliedVoltage = 0.0;
    }
}
