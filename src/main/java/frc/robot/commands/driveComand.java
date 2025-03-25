package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States.ReefPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionVersions.Vision3;

public class driveComand extends Command {
    private final CommandSwerveDrivetrain drivetrain; 
    private final SwerveRequest.RobotCentric robotCentricDrive = 
     new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
     private double startTime; 

    public driveComand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(1)
                 .withVelocityY(0).withRotationalRate(0)).execute();
    }

    @Override
    public void end(boolean isFinished) {
        drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(0)
                 .withVelocityY(0).withRotationalRate(0)).execute();
    }
    
    @Override
    public boolean isFinished() {
        if(Math.abs(Timer.getFPGATimestamp() - startTime) > 3) {
            return true;
        }
        return false; 
    }
}
