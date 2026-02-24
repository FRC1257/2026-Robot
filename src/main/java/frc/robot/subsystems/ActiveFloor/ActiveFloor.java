package frc.robot.subsystems.ActiveFloor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ActiveFloor extends SubsystemBase {

    private final ActiveFloorIO io;
    private ActiveFloorIOInputsAutoLogged inputs = new ActiveFloorIOInputsAutoLogged();


    public ActiveFloor(ActiveFloorIO io) {
        this.io = io;
    }
    
}
