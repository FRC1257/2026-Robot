package frc.robot.subsystems.fuelPivot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class FuelPivotIOSparkMax implements FuelPivotIO{
    
    private SparkMax pivotMotor;
    private SparkMaxConfig config;

    private final ProfiledPIDController pivotController;
    private ArmFeedforward pivotFeedforward = new ArmFeedforward(0, 0, 0, 0);
    private RelativeEncoder pivotEncoder;
    private double setpoint = 0.0;


    public FuelPivotIOSparkMax(){
        pivotMotor = new SparkMax(FuelPivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

        config = new SparkMaxConfig();
        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters );
        
        pivotEncoder = pivotMotor.getEncoder();
        //configure ???

        
        pivotController = new ProfiledPIDController(0,0,0, new TrapezoidProfile.Constraints(0,0));
        ArmFeedforward pivotFeedforward = new ArmFeedforward(0, 0, 0);
    }


    
    @Override
    public void updateInputs(FuelPivotIOInputs inputs) {
        inputs.angleRads = getAngle();
        inputs.angVelocityRadsPerSec = pivotEncoder.getVelocity();
        inputs.appliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    }

    @Override
    public void setVoltage (double motorVolts) {
        Logger.recordOutput("Pivot Voltage", motorVolts);
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
        //Code Later
    }

    @Override
    public void stop() {
        pivotMotor.stopMotor();
    }

    @Override
    public boolean atSetpoint(){
       return Math.abs(getAngle() - setpoint) < FuelPivotConstants.FUEL_PIVOT_PID_TOLERANCE;
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
