package frc.robot.subsystems.kicker;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.PIDController;

import java.beans.Encoder;

public class KickerIOSparkMax implements KickerIO {
    private SparkMax kickermotor;

    private PIDController controller = new PIDController(0, 0, 0);
    private RelativeEncoder encoder;
    private DigitalInput limitSwitch;

    public KickerIOSparkMax() {
        kickermotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless); //random id; change it 

        encoder = kickermotor.getEncoder();

        SparkMaxConfig kickerConfig = new SparkMaxConfig(); //TODO add some configuration

        limitSwitch=new DigitalInput(0); //TODO: add this to constants later
    }


    @Override
    public void setPIDGains(double Kp, double Ki, double Kd) {
        controller.setPID(Kp, Ki, Kd);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.velocity = getVelocity();
        inputs.appliedVoltage = kickermotor.getAppliedOutput() * kickermotor.getBusVoltage();
        inputs.current = kickermotor.getOutputCurrent();
        inputs.temperature = kickermotor.getMotorTemperature();
        inputs.limitSwitchPressed=limitSwitchPressed();
    }
    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setVelocity(double rpm) {
        double voltage = controller.calculate(getVelocity(), rpm);
        kickermotor.setVoltage(voltage);
    }
    @Override
    public double getVoltage() {
        return kickermotor.getAppliedOutput() * kickermotor.getBusVoltage(); //added the getBusVoltage from warren temple because why not
    }

    @Override
    public void setVoltage(double voltage) {
        kickermotor.setVoltage(voltage);
        Logger.recordOutput("Kicker/SetVoltage", voltage);
    }

    public boolean limitSwitchPressed() {
        return !limitSwitch.get();
    }


}
