package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkFlexUtil;
import frc.robot.Constants;
import frc.robot.States.ElevatorStates;

public class InnerElevatorSubsystem extends SubsystemBase{
    private final SparkFlex elevatorMotor; 
    private final RelativeEncoder elevatorEncoder;
    private final PIDController elevatorPID;

    private ElevatorStates state;

    private double slowStrafe;

    private double currentPosition;
    private double motorSpeed;
    private boolean inBounds;

    public InnerElevatorSubsystem() {
        elevatorMotor = new SparkFlex(Constants.InnerElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorPID = new PIDController(Constants.InnerElevatorConstants.KP, Constants.InnerElevatorConstants.KI, Constants.InnerElevatorConstants.KD);

        state = ElevatorStates.STARTING_POSITION;
        
        motorSpeed = 0; 
        slowStrafe = 1;
        inBounds = false;
        currentPosition = this.getHeight();

        SparkFlexUtil.setSparkFlexBusUsage(elevatorMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, false);
    }

    
    @Override
    public void periodic() {
        currentPosition = getHeight(); 

        // if(currentPosition >= ElevatorStates.MAX.innerHeight) {
        //     // positive goes up 
        //     motorSpeed = 0.1;
        //     inBounds = false;
        // } else if (currentPosition <= ElevatorStates.MIN.innerHeight) {
        //     motorSpeed = -0.1;
        //     inBounds = false;
        // } else {
            motorSpeed = elevatorPID.calculate(currentPosition) + 0.03;
            if(motorSpeed < -0.1) {
                motorSpeed *= 0.2;
            }
            inBounds = true;
        //}

        elevatorMotor.set(motorSpeed);
        setSmartDashboardValues();
    }

    public void setInnerElevatorState(ElevatorStates state) {
        this.state = state;
        elevatorPID.setSetpoint(state.innerHeight);
    }

    public ElevatorStates getInnerElevatorState() {
        return state;
    }

    public double getHeight() {
        return elevatorEncoder.getPosition() + 20; 
    }

    public boolean atSetPosition() {
        return elevatorPID.atSetpoint();
    }

    public double getSlowStrafe() {
        if(state.equals(ElevatorStates.STARTING_POSITION)) {
            slowStrafe = 1;
        } else {
            slowStrafe = 0.2;
        }

        return slowStrafe;
    }

    private void setSmartDashboardValues() {
        SmartDashboard.putNumber("Inner elevator speed", motorSpeed);
        SmartDashboard.putNumber("Inner elevator current height ", currentPosition);
        SmartDashboard.putNumber("Inner elevator goal position", state.innerHeight);

        SmartDashboard.putString("Inner elevator state", state.toString());

        SmartDashboard.putBoolean("Inner elevator in bounds", inBounds);

    }
}
