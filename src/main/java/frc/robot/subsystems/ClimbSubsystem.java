package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.States.ClimbStates;

public class ClimbSubsystem extends SubsystemBase{
    private final SparkFlex climbMotor;
    private final DutyCycleEncoder climbEncoder;

    private ClimbStates state;
    private double currentPosition;
    private double climbSpeed;

    public ClimbSubsystem() {
        climbMotor = new SparkFlex(Constants.ClimbConstants.MOTOR_ID, MotorType.kBrushless);
        climbEncoder = new DutyCycleEncoder(Constants.ClimbConstants.ENCODER_ID);
        state = ClimbStates.STOP;
    }

    @Override
    public void periodic() {
        currentPosition = climbEncoder.get();
        // if(currentPosition > Constants.ClimbConstants.MAX_THRESHOLD) {
        //     climbMotor.set(0.1);
        //     return;
        // } else if (currentPosition < Constants.ClimbConstants.MIN_THRESHOLD) {
        //     climbMotor.set(-0.1);
        //     return;
        // }
        climbMotor.set(climbSpeed);
        setSmartDashboardValues();
    }

    public void setClimbState(ClimbStates state) {
        this.state = state;
        climbSpeed = state.speed;
    }

    private void setSmartDashboardValues() {
        SmartDashboard.putNumber("Climb subsystem current speed", climbSpeed);
        
        SmartDashboard.putString("Climb subsystem state", state.toString());
    }
}
