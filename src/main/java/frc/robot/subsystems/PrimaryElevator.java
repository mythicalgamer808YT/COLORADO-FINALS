package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.States.ElevatorStates;

public class PrimaryElevator extends SubsystemBase{
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final PIDController elevatorPID;

    private ElevatorStates state;

    private double currentHeight;
    private double motorOutput;
    private boolean inBounds;

    private double addSpeed;

    public PrimaryElevator(){
        leftMotor = new TalonFX(Constants.PrimaryElevatorConstants.LEFT_ELEVATOR_MOTOR_ID);
        rightMotor = new TalonFX(Constants.PrimaryElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID);
        elevatorPID = new PIDController(Constants.PrimaryElevatorConstants.KP, Constants.PrimaryElevatorConstants.KI, Constants.PrimaryElevatorConstants.KD);

        state = ElevatorStates.STARTING_POSITION;

        currentHeight = this.getHeight();
        motorOutput = 0;
        inBounds = false;
    }

    @Override
    public void periodic() {
        currentHeight = this.getHeight();
        
        if(currentHeight >= ElevatorStates.MAX.primaryHeight) {
            inBounds = false;
            motorOutput = -0.1;
        } else if(currentHeight <= ElevatorStates.MIN.primaryHeight ) {
            inBounds = false;
            motorOutput = 0.1;
        } else {
            inBounds = true;
            motorOutput = elevatorPID.calculate(currentHeight);
        }

        setElevatorSpeeds(motorOutput);
        setSmartDashboardValues();
    }

    public void setAddMotor(double add) {
        addSpeed = add;
    }

    public void setPrimaryElevatorState(ElevatorStates state) {
        this.state = state;
        elevatorPID.setSetpoint(state.primaryHeight);
    }

    public ElevatorStates getPrimaryElevatorState() {
        return state;
    }

    public double getHeight() {
        return rightMotor.getPosition().getValueAsDouble() + 0.5;
    }
    
    private void setElevatorSpeeds(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    private void setSmartDashboardValues() {
        SmartDashboard.putNumber("Primary elevator goal position", state.primaryHeight);
        SmartDashboard.putNumber("Primary elevator speed", motorOutput);
        SmartDashboard.putNumber("Primary elevator current height", currentHeight);
        
        SmartDashboard.putString("Primary elevator current state", state.toString());      

        SmartDashboard.putBoolean("Primary elevator in bounds", inBounds);
    }
}
