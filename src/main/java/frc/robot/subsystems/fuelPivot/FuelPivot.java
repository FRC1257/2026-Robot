package frc.robot.subsystems.fuelPivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.fuelPivot.FuelPivotIO.FuelPivotIOInputs;

public class FuelPivot extends SubsystemBase{

    private final FuelPivotIOInputs inputs = new FuelPivotIOInputs();//idk
    private final FuelPivotIO io;
    
    public FuelPivot(FuelPivotIO io) {
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
    }

    public void setBrake(boolean brake) {
        io.setBrake(brake);
    }

    public void stop(){
        io.stop();
    }
    
    public void move(double speed) {

    if (io.getAngle() > FuelPivotConstants.FUEL_PIVOT_MAX_ANGLE && speed > 0) {
      speed = 0;
    } else if (io.getAngle() < FuelPivotConstants.FUEL_PIVOT_MIN_ANGLE && speed < 0) {
      speed = 0;
    }
  }

}
