package main.java.frc.robot.subsystems.active_floor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Active_Floor extends SubsystemBase {
    private final Active_FloorIO io;
    Active_FloorIOInputsAutoLogged inputs = new Active_FloorIOInputsAutoLogged();
   
    public Active_Floor (Active_FloorIO io) {
        this.io = io;
   }
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Active_Floor", inputs);
    }
    public void runFloor(double voltage) {
        io.floor_on(ActiveFloorConstants.Floor_Voltage_On);
    }
    public void stopFloor() {
        io.floor_off();
    }
    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }
    private void setBrake(boolean brake) {
        io.setBrake(brake);
    }
    private Command runVoltage(DoubleSupplier voltage) {
        return run(() -> setVoltage(voltage.get()));
    }
    private Command intake(double voltage) {
        return runVoltage(()->Active_FloorConstants.Floor_Intake_Voltage);
    }
    private Command outtake(double voltage) {
        return runVoltage(()->Active_FloorConstants.Floor_Outtake_Voltage);
    }
    private Command stopCommand() {
        return runOnce(this::stop);
    }
}