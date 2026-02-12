package frc.robot.subsystems.fuelPivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class FuelPivotIOSim implements FuelPivotIO{

    private final DCMotor pivotMotorGearbox = DCMotor.getNEO(1);

    private SparkFlex pivotMotor = new SparkFlex(FuelPivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
    private double pi = FuelPivotConstants.PI;

    private ProfiledPIDController pivotPIDControllerSim = new ProfiledPIDController(0,0,0, new TrapezoidProfile.Constraints(0,0));
    private ArmFeedforward pivotFeedforwardSim = new ArmFeedforward(0, 0, 0);
    

    private SingleJointedArmSim sim = new SingleJointedArmSim(
        // Sim Values, idk what to do here, mostly last years stuff
        pivotMotorGearbox,
          FuelPivotConstants.FuelPivotSimConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(
              FuelPivotConstants.FuelPivotSimConstants.kArmLength,
              FuelPivotConstants.FuelPivotSimConstants.kArmMass),
          FuelPivotConstants.FuelPivotSimConstants.kArmLength,
          FuelPivotConstants.FuelPivotSimConstants.kMinAngleRads,
          FuelPivotConstants.FuelPivotSimConstants.kMaxAngleRads,
          true,
          0.1
    );

    @Override
    public void updateInputs(FuelPivotIOInputs inputs) {
        sim.update(0.02);
        inputs.appliedVolts =
            pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.angVelocityRadsPerSec = pivotEncoder.getVelocity() * 2 * pi / 60;
        inputs.angleRads = pivotEncoder.getPosition() * 2 * pi;
        inputs.angle = pivotEncoder.getPosition() * 360;
        inputs.velocity = pivotEncoder.getVelocity();
    }

    @Override
    public double getAngle() {
        return sim.getAngleRads();
    }

    @Override
    public double getAngleVelocity() {
        return sim.getVelocityRadPerSec();
    }
    
    @Override
    public boolean atSetpoint(){
        return pivotPIDControllerSim.atGoal();
    }
    
    @Override
    public void setVoltage (double motorVolts) {
        pivotMotor.setVoltage(motorVolts);
        //
    }

    @Override
    public void setSetpoint(double setpoint) {
        pivotPIDControllerSim.setGoal(setpoint);
    }

    @Override
    public void goToSetpoint(){
        double pidOutput = pivotPIDControllerSim.calculate(getAngle());
        double feedforwardOutput = pivotFeedforwardSim.calculate(pivotPIDControllerSim.getSetpoint().position, pivotPIDControllerSim.getSetpoint().velocity);
        setVoltage(pidOutput + feedforwardOutput);
    }

    @Override
    public void stop() {
        pivotMotor.setVoltage(0);
    }

    @Override
    public void setPIDGains(double kP, double kI, double kD){
        pivotPIDControllerSim  = new ProfiledPIDController(kP, kI, kD, null);
    }

    @Override
    public void setFeedForward(double kS, double kG, double kV, double kA){
        pivotFeedforwardSim = new ArmFeedforward(kS, kG, kV, kA);
    }


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
        return pivotFeedforwardSim.getKa();
    }

}
