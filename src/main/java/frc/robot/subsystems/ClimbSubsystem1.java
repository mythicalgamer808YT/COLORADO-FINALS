package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem1 extends SubsystemBase{
    private final SparkFlex climbMotor;
    private double climbSpeed;
    public ClimbSubsystem1 () {
        climbMotor = new SparkFlex(ClimbConstants.MOTOR_ID, MotorType.kBrushless);
        climbSpeed = 0;
    }
    @Override
    public void periodic() {
        climbMotor.set(climbSpeed);
    }
    public void setClimbSpeed (double climbSpeed) {
        this.climbSpeed = climbSpeed;
    }
}



