package frc.robot.subsystems.fuelPivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class FuelPivotIOSparkMax implements FuelPivotIO{

    private SparkFlex pivotMotor = new SparkFlex(FuelPivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    
    private ProfiledPIDController pivotController = new ProfiledPIDController(0,0,0, new TrapezoidProfile.Constraints(0,0));
    private ArmFeedforward pivotFeedforward = new ArmFeedforward(0, 0, 0);
    
    
    @Override
    public void updateInputs(FuelPivotIOInputs inputs) {
        //Change inputs here 
    }

    @Override
    public void setVoltage (double motorVolts) {
        Logger.recordOuptput("Pivot Voltage", motorVolts);
        pivotMotor.setVoltage(motorVolts);
    }

    @Override
    public double getAngle() {
        return pivotEncoder.getPosition();
    }

    @Override
    public double getAngleVelocity() {
        return pivotEncoder.getVelocity();
    }

    @Override
    public void setSetpoint(double setpoint) {
        pivotController.setGoal(setpoint);
    }

    @Override
    public void goToSetpoint(){
        //code later
    }

    @Override
    public void setBrake() {
        //code later
    }

    @Override
    public boolean atSetpoint(){
        //code later
    }

    @Override
    public void setPIDGains(double kP, double kI, double kD){
        pivotController.setPID(kP,kI,kD);
    }
    @Override
    public void setFeedForward(double kS, double kG, double kV, double kA){
        pivotFeedforward = new ArmFeedforward(kS,kG,kV,kA);
    }

    @Override
    public double getP() {
        return pivotController.getP();
    }
    
    @Override
    public double getI() {
        return pivotController.getI();
    }

    @Override
    public double getD() {
        return pivotController.getD();
    }

    @Override
    public double getkS() {
        return pivotFeedforward.getKs();
    }

    @Override
    public double getkV() {
        return pivotFeedforward.getKv();
    }

    public double getkG() {
        return pivotFeedforward.getKg();
    }

    @Override
    public double getkA () {
        return pivotFeedforward.getKa();
    }

    


}
