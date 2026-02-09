package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO{
    // elias I know you can fix all this right
    private final DCMotor motorGearbox = DCMotor.getNEO(1);

    private final FlywheelSim sim =
        new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), kMomentOfInertia, kIntakeGearing),
          motorGearbox);
          
    private PIDController controller = new PIDController(0, 0, 0);
    
    public ShooterIOSim(){

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        sim.update(0.02);
        inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    }

    @Override
    public void setVoltage(double voltage) {
        sim.setInputVoltage(voltage); 
    }

    @Override
    public void setRPM(double rpm) {
        
    }

    @Override
    public void stop() {
        sim.setInputVoltage(0.0);
    }
}

