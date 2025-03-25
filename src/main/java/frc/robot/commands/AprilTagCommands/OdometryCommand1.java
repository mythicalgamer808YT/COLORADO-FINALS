package frc.robot.commands.AprilTagCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OdometryConsts;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class OdometryCommand1 extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController xController;
    
    private final double goalXDistance;

    private final SwerveRequest.RobotCentric 
    robotCentricDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private Pose2d currentPose;

    public OdometryCommand1(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.xController = new PIDController(1, 0, 0);
        goalXDistance = 0.416;

        this.xController.setTolerance(OdometryConsts.TranslationalThreshold);

        currentPose = new Pose2d();
    }

    @Override
    public void initialize() {
        drivetrain.resetPose(new Pose2d());
    }

    @Override
    public void execute() {
        currentPose = drivetrain.getState().Pose;

        double xSpeed = -xController.calculate(currentPose.getX(), goalXDistance);

        drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(xSpeed)
                 .withVelocityY(0).withRotationalRate(0)).execute();
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint();
    }

    @Override
    public void end(boolean isFinished) {
        drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
                 .withVelocityY(0).withRotationalRate(0)).execute();
    }
}