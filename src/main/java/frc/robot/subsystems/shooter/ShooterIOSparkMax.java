package frc.robot.subsystems.shooter;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSparkMax implements ShooterIO{
    private RelativeEncoder encoder;

    public ShooterIOSparkMax() {
    }   

    @Override
    public void updateInputs(ShooterIOInputs inputs) {}

    @Override
    public double getRPM() {return encoder.getVelocity();}

    @Override
    public void setPIDGains(double Kp, double Ki, double Kd) {}

    @Override
    public void setVoltage(double voltage) {}

    @Override
    public double getVoltage() {}

    @Override
    public void setRPM(double rpm) {}
}
