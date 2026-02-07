import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class KickerIOSparkMax implements KickerIO{
    private SparkMax motor;
    private RelativeEncoder encoder; 
    
    public KickerIOSparkMax(){
        motor = new SparkMax(KickerConstants.motorID, MotorType.kBrushless); //error
        encoder = motor.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(KickerConstants.currentLimit);
        config.voltageCompensation(12.0);
        motor.configure(
            config, 
            ResetMode.kResetSafeParameters, //error
            PersistMode.kPersistParameters //error
        );
    }
    
    @Override
    public void updateInputs(KickerIOInputs inputs){
        inputs.velocityRPM = encoder.getVelocity();
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.TemperatureCelsius = motor.getMotorTemperature();
    }
    
    @Override
    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    @Override
    public double getVoltage(){
        return motor.getAppliedOutput();
    }

    @Override
    public void setRPM(double RPM){
        motor.getClosedLoopController().setSetpoint(RPM, SparkMax.ControlType.kVelocity);
    }

    @Override
    public double getRPM(){
        return encoder.getVelocity();
    }

    @Override
    public void stop(){
        motor.stopMotor();
    }



    
}
