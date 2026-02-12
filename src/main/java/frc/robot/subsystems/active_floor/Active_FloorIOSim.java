package main.java.frc.robot.subsystems.active_floor;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.DCMotorSim;

private final DCMotorSim sim = new DCMotorSim();
DCMotor.getNeoVortex(1);

public class Active_FloorIOSim implements Active_FloorIO {
   
    private double appliedVoltage = 0.0;

    @Override
    public void updateInputs(Active_FloorIOInputs inputs) {
        sim.update(0.02);
        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmps = new double[] { sim.getCurrentDrawAmps() };
        inputs.tempCelsius = new double[] { 60 };
    }

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void floor_on() {
        setInputVoltage(10.0);
    }

    @Override
    public void floor_off(double voltage) {
           setInputVoltage(0.0);
            }

   @Override
    public void updateInputs(Active_Floor inputs) {
        	sim.update(0.02);
    	inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    	inputs.appliedVoltage = appliedVoltage;
    	inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.tempCelcius = new double[] {60};

    }
}
