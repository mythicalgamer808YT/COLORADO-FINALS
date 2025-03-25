package frc.robot.commands.AprilTagCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OdometryConsts;
import frc.robot.Constants.PhotonConsts;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionVersions.Vision;

import org.photonvision.EstimatedRobotPose;


public class OdometryCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain; 
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rController;
    
    private final Transform2d goalTransform;

    // private final double goalXDistance = 0.416;
    // private final double goalRDistance = 0.416;
    // private final double goalYDistance = 0.416;

    private Vision visionSubsystem;

    private Pose2d currentPose; 

    private final SwerveRequest.RobotCentric 
    robotCentricDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public OdometryCommand(CommandSwerveDrivetrain drivetrain, Vision visionSubsystem, boolean isLeft) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        this.visionSubsystem = visionSubsystem;

        if(isLeft) {
            goalTransform = PhotonConsts.GOAL_LEFT_REEF;
        } else {
            goalTransform = PhotonConsts.GOAL_RIGHT_REEF;
        }

        xController = new PIDController(1, 0.0, 0.0);
        yController = new PIDController(1, 0,0);
        rController = new PIDController(0.3, 0, 0);

        xController.setTolerance(OdometryConsts.TranslationalThreshold);
        yController.setTolerance(OdometryConsts.TranslationalThreshold);
        rController.setTolerance(OdometryConsts.RotationalThreshold);

    }

    @Override
    public void initialize() {
        //drivetrain.resetPose(new Pose2d());
    }

    @Override 
    public void execute() {
        currentPose = drivetrain.getState().Pose;//visionSubsystem.getFieldRelativePose().estimatedPose.toPose2d();
        Pose2d targetPose = visionSubsystem.getClosestPose().transformBy(goalTransform);

        double xSpeed = -xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double rSpeed = rController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(xSpeed)
                 .withVelocityY(ySpeed).withRotationalRate(rSpeed)).execute();
    }

    // @Override
    // public boolean isFinished() {
    //     return xController.atSetpoint() &&
    //         yController.atSetpoint() &&
    //         rController.atSetpoint();
    // }

    // @Override
    // public void end(boolean isFinished) {
    //     drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
    //              .withVelocityY(0).withRotationalRate(0)).execute();
    // }
}