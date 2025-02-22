package frc.robot.subsystems.algaeIntake;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

/** Need to import Constants files/classes */
//

public class AlgaeIntakeIOSparkMax implements AlgaeIntakeIO {

  private SparkFlex motor;
  private RelativeEncoder encoder;

  private SparkFlexConfig config = new SparkFlexConfig();

  public AlgaeIntakeIOSparkMax() {
    /** ID needs to be assigned from constants */
    // setPIDConstants(kGroundIntakeP, kGroundIntakeI, kGroundIntakeD);
    motor = new SparkFlex(AlgaeIntakeConstants.ALGAE_INTAKE_MOTOR_ID, MotorType.kBrushless);

    config
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
        .voltageCompensation(12)
        .smartCurrentLimit(NEO_CURRENT_LIMIT)
        .inverted(true);

    encoder = motor.getEncoder();

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** updates inputs from robot */
  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
    inputs.velocityRadsPerSec = encoder.getVelocity();
  }

  /** sets voltage to run motor if necessary */
  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /** sets brake mode to stop */
  @Override
  public void setBrake(boolean brake) {
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
