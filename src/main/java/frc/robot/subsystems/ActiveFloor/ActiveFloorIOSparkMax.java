package frc.robot.subsystems.ActiveFloor;

import static frc.robot.Constants.NEO_VORTEX_CURRENT_LIMIT;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.littletonrobotics.junction.Logger;



public class ActiveFloorIOSparkMax implements ActiveFloorIO {

  private SparkFlex motor;
  private RelativeEncoder encoder;

  private SparkFlexConfig config = new SparkFlexConfig();

  public ActiveFloorIOSparkMax() {

    motor = new SparkFlex(ActiveFloorConstants.ACTIVE_FLOOR_MOTOR_ID, MotorType.kBrushless);

    config
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(NEO_VORTEX_CURRENT_LIMIT)
        .inverted(true);

    encoder = motor.getEncoder();

    motor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); 
  }

  /** updates inputs from robot */
  @Override
  public void updateInputs(ActiveFloorIOInputs inputs) {
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
    inputs.velocityRadsPerSec = encoder.getVelocity();
  }

  
  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
    Logger.recordOutput("ActiveFloor/Desired Voltage", voltage);
  }


  @Override
  public void setBrake(boolean brake) {
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
  }
}