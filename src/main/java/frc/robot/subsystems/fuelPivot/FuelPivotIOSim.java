package frc.robot.subsystems.fuelPivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class FuelPivotIOSim implements FuelPivotIO{

    private SparkFlex pivotMotor = new SparkFlex(FuelPivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
    private double pi = FuelPivotConstants.PI;

    private ProfiledPIDController pivotPIDControllerSim = new ProfiledPIDController(0,0,0, new TrapezoidProfile.Constraints(0,0));
    private ArmFeedforward pivotFeedforwardSim = new ArmFeedforward(0, 0, 0);
    
    @Override
    public void updateInputs(FuelPivotIOInputs inputs) {
        sim.update(0.02);//////////////////////////////////
        inputs.appliedVolts =
            pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.angVelocityRadsPerSec = pivotEncoder.getVelocity() * 2 * pi / 60;
        inputs.angleRads = pivotEncoder.getPosition() * 2 * pi;
        inputs.angle = pivotEncoder.getPosition() * 360;
        inputs.velocity = pivotEncoder.getVelocity();
    }

    @Override
    public double getAngle() {
        return FuelPivotIOInputs.angle;//////////////////////////////////////
    }

    @Override
    public double getAngleVelocity() {
        return FuelPivotIOInputs.velocity; /////////////////////////////////////////////
    }
    
    @Override
    public boolean atSetpoint(){
        return false;
    }
    
    @Override
    public void setVoltage (double motorVolts) {}

    @Override
    public void setSetpoint(double setpoint) {}

    @Override
    public void goToSetpoint(){}

    @Override
    public void stop() {}

    @Override
    public void setPIDGains(double kP, double kI, double kD){}

    @Override
    public void setFeedForward(double kS, double kG, double kV, double kA){}

    //stuff

    @Override
    public double getP() {
        return pivotPIDControllerSim.getP();
    }
    
    @Override
    public double getI() {
        return pivotPIDControllerSim.getI();
    }

    @Override
    public double getD() {
        return pivotPIDControllerSim.getD();
    }

    @Override
    public double getkS() {
        return pivotFeedforwardSim.getKs();
    }

    @Override
    public double getkV() {
        return pivotFeedforwardSim.getKv();
    }

    @Override
    public double getkG() {
        return pivotFeedforwardSim.getKg();
    }

    @Override
    public double getkA() {
        return pivotFeedforwardSim.get;/////////////////////////////
    }

}
