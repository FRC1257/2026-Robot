package frc.robot.subsystems.fuelPivot;

public class FuelPivot extends subsystemBase{

    private final fuelPivotIO io;


    @overide
    public void periodic(){
        io.updateInputs(inputs);


    }

    public void setBrake(boolean brake) {
        io.setBrake(brake);
    }

    public void stop(){
        io.stop();
    }
    

}
