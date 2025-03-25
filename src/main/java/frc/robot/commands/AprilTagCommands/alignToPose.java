package frc.robot.commands.AprilTagCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States.ReefPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionVersions.Vision3;

public class alignToPose extends Command {
    private final CommandSwerveDrivetrain drivetrain; 
    private final SwerveRequest.RobotCentric robotCentricDrive = 
     new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Pose2d goalPose; 

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;

    private final Vision3 visionSubsystem;
    private final boolean isLeft;
    private final ReefPosition reefPosition;

    public alignToPose(CommandSwerveDrivetrain drivetrain, Vision3 visionSubsystem, boolean isLeft) {
        this.drivetrain = drivetrain;
        this.isLeft = isLeft; 

        if(isLeft) {
            reefPosition = ReefPosition.LEFT;
        } else {
            reefPosition = ReefPosition.RIGHT;
        }

        this.visionSubsystem = visionSubsystem;
        this.goalPose = this.visionSubsystem.getRobotPoseForNearestReefAprilTag(drivetrain.getState().Pose, reefPosition);

        xController = new PIDController(Constants.PhotonConsts.KP_TRANSLATION, Constants.PhotonConsts.KI_TRANSLATION, Constants.PhotonConsts.KD_TRANSLATION);  // Vertical movement
        yController = new PIDController(Constants.PhotonConsts.KP_TRANSLATION, Constants.PhotonConsts.KI_TRANSLATION, Constants.PhotonConsts.KD_TRANSLATION);  // Horitontal movement
        rotController = new PIDController(Constants.PhotonConsts.KP_ROTATION, Constants.PhotonConsts.KI_ROTATION, Constants.PhotonConsts.KD_ROTATION);  // Rotation
    }

    @Override
    public void initialize() {
        xController.setSetpoint(goalPose.getX());
        yController.setSetpoint(goalPose.getY());
        rotController.setSetpoint(goalPose.getRotation().getDegrees());

        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        rotController.setTolerance(0.1);

        rotController.enableContinuousInput(180, -180);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;

        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double rotSpeed = rotController.calculate(currentPose.getRotation().getDegrees());

        drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(xSpeed)
                 .withVelocityY(ySpeed).withRotationalRate(rotSpeed)).execute();
    }

    @Override
    public void end(boolean isFinished) {
        drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
                 .withVelocityY(0).withRotationalRate(0)).execute();
    }
    
    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
    }
}
