package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkFlex;
import static frc.robot.Constants.NEO_VORTEX_CURRENT_LIMIT;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.spark.config.SparkFlexConfig;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
@AutoLog
public class HopperIntakeIOSparkMax implements HopperIntakeIO {
    private SparkFlex motor;
    private AbsoluteEncoder encoder;

 
  private SparkFlexConfig config = new SparkFlexConfig();

public HopperIntakeIOSparkMax() {

    motor = new SparkFlex(HopperIntakeConstants.HOPPER_INTAKE_MOTOR_ID, MotorType.kBrushless);
   
config
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(NEO_VORTEX_CURRENT_LIMIT) 
        .inverted(true);

//encoder = motor.getEncoder(); //cant convert from rel to abs

  motor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  }

 
  @Override
  public void updateInputs(HopperIntakeIOInputs inputs) {
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
    inputs.velocityRadsPerSec = encoder.getVelocity();
  }

  
  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
    Logger.recordOutput("HopperIntake/Desired Voltage", voltage);
  }

  
  @Override
  public void setBrake(boolean brake) {
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
  }


}

