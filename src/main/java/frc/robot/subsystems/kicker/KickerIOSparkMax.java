package frc.robot.subsystems.kicker;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.PIDController;

public class KickerIOSparkMax implements KickerIO {
    private SparkMax kickermotor;

    private PIDController controller = new PIDController(0, 0, 0); 

    public KickerIOSparkMax() {
        kickermotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless); //random id; change it 

        encoder = kickermotor.getEncoder();

        SparkMaxConfig kickerConfig = new SparkMaxConfig();


        @Override 
        public void setPIDGains(double Kp, double Ki, double Kd) {
            controller.setPID(Kp, Ki, Kd);
        }

        @Override 
        public void updateInputs(KickerIOInputs inputs) {
            inputs.velocity = getRPM();
            inputs.appliedVoltage = kickermotor.getAppliedOutput() * kickermotor.getBusVoltage();
            inputs.current = kickermotor.getOutputCurrent();
            inputs.temperature = kickermotor.getMotorTemperature();
        }
        @Override
        public double getRPM() {
            return encoder.getVelocity();
        }
        
        @Override
        public void setRPM(double rpm) {
            double voltage = controller.calculate(getRPM(), rpm);
            kickermotor.setVoltage(voltage);
        }
        @Override
        public double getVoltage() {
            return kickermotor.getAppliedVoltage * kickermotor.getBusVoltage(); //added the getBusVoltage from warren temple because why not
        }

        @Override
        public double setVoltage() {
            kickermotor.setVoltage(voltage);
            Logger.recordOutput("Kicker/SetVoltage", voltage);
        }

    }
}
