package frc.robot.subsystems.Climb;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;

public class ClimbIOSim implements ClimbIO {
    private final ElevatorSim climbSim;
    private Voltage appliedVoltage;
    private boolean isBrakeMode = true;
    
    public ClimbIOSim() {
        climbSim = new ElevatorSim(
            DCMotor.getNEO(1),                    
            ClimbConstants.GEAR_RATIO,            
            ClimbConstants.MASS_CARRIED,      
            ClimbConstants.DRUM_RADIUS,    
            ClimbConstants.MIN_HEIGHT,     
            ClimbConstants.MAX_HEIGHT,     
            true,            
            ClimbConstants.START_HEIGHT      
        );
    }
    
    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        
        climbSim.update(0.02);
        climbSim.setInput(appliedVoltage); //error
        double drumCircumference = 2 * Math.PI * ClimbConstants.DRUM_RADIUS;
        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmps = new double[] {climbSim.getCurrentDrawAmps()};
        inputs.tempCelcius = new double[] {25.0}; 
    }
    
    @Override
    public void setVoltage(Voltage voltage) {
        this.appliedVoltage = voltage;
    }
    
    @Override
    public void setBrake(boolean brake){ //error
        this.isBrakeMode = brake;
    }
    
    @Override
    public void stop() {
        this.appliedVoltage = Volts.of(0.0);
    }
}