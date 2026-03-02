//package frc.robot.subsystems.kicker;
//
//import org.littletonrobotics.junction.Logger;
//
//import com.revrobotics.spark.SparkMax;
//
//import edu.wpi.first.math.controller.PIDController;
//
//public class KickerIOSim implements KickerIO {
//    private final SparkMax kicker_motor; //dunno if name needs to be same so i changed it slightly from sparkmaxio
//    private final PIDController kicker_controller;
//
//    private KickerSim sim = new KickerSim();
//
//    public KickerIOSim() {
//        kicker_controller = new PIDController(0,0,0);
//    }
//
//    @Override
//
//    public void updateInputs(KickerIOInputs inputs) {
//        sim.update(0.02);
//        inputs.velocity = sim.velocity;
//        inputs.appliedVoltage = sim.appliedVoltage;
//        inputs.current = sim.current;
//        inputs.temperature = sim.temperature;
//        inputs.limitSwitchPressed = sim.limitSwitchPressed;
//    }
//
//    @Override
//    public double getVelocity() {
//        sim.getVelocity();
//    }
//
//    @Override
//    public void setVelocity(double rpm) {
//        sim.setVoltage(voltage);
//    }
//    @Override
//    public double getVoltage() {
//        sim.getVoltage();
//    }
//
//    @Override
//    public void setVoltage(double voltage) {
//        sim.setVoltage(voltage);
//    }
//}
