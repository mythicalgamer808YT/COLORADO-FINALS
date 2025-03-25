package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.States.ClimbStates;

public class ClimbSubsystem extends SubsystemBase{
    private final SparkFlex climbMotor;
    private final DutyCycleEncoder climbEncoder;
    private final PIDController controller; 

    private ClimbStates state;
    private double currentPosition;
    private double climbSpeed;
    private boolean usePid; 

    public ClimbSubsystem() {
        climbMotor = new SparkFlex(Constants.ClimbConstants.MOTOR_ID, MotorType.kBrushless);
        climbEncoder = new DutyCycleEncoder(Constants.ClimbConstants.ENCODER_ID);
        state = ClimbStates.STOP;
        this.controller = new PIDController(5, 0,0);
        controller.setSetpoint(0.32);
        controller.setTolerance(0.01);
        usePid = true;
    }

    @Override
    public void periodic() {
        // if(controller.atSetpoint()) {
        //     usePid = false; 
        //     climbSpeed = state.speed;
        // }


        // currentPosition = climbEncoder.get();
        // if(usePid) {
        //     climbSpeed = controller.calculate(currentPosition);
        // } else {
        //     climbSpeed = state.speed;
        // }
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

    public void setPidUseTrue() {
        this.usePid = true; 
    }


    public void setClimbState(ClimbStates state) {
        this.state = state;
        climbSpeed = state.speed;
    }

    private void setSmartDashboardValues() {
        SmartDashboard.putNumber("Climb subsystem current speed", climbSpeed);
        SmartDashboard.putNumber("sigma boy", climbEncoder.get());
        SmartDashboard.putString("Climb subsystem state", state.toString());
    }
}
