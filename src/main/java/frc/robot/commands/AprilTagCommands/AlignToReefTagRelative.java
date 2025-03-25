package frc.robot.commands.AprilTagCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
  private final PIDController xController, yController, rotController;
  private final SwerveRequest.RobotCentric robotCentricDrive = 
     new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private Timer dontSeeTagTimer, stopTimer;
  private final CommandSwerveDrivetrain drivetrain;

  private boolean isRightScore;
  private double tagID = -1;

  public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivetrain) {
    xController = new PIDController(Constants.PhotonConsts.KP_TRANSLATION, Constants.PhotonConsts.KI_TRANSLATION, Constants.PhotonConsts.KD_TRANSLATION);  // Vertical movement
    yController = new PIDController(Constants.PhotonConsts.KP_TRANSLATION, Constants.PhotonConsts.KI_TRANSLATION, Constants.PhotonConsts.KD_TRANSLATION);  // Horitontal movement
    rotController = new PIDController(Constants.PhotonConsts.KP_ROTATION, Constants.PhotonConsts.KI_ROTATION, Constants.PhotonConsts.KD_ROTATION);  // Rotation
    this.isRightScore = isRightScore;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.PhotonConsts.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.PhotonConsts.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.PhotonConsts.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.PhotonConsts.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Constants.PhotonConsts.Y_SETPOINT_REEF_ALIGNMENT : -Constants.PhotonConsts.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.PhotonConsts.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID(Constants.LimelightName);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV(Constants.LimelightName) && LimelightHelpers.getFiducialID(Constants.LimelightName) == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace(Constants.LimelightName);
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(xSpeed)
        .withVelocityY(ySpeed).withRotationalRate(rotValue)).execute();

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
        .withVelocityY(0).withRotationalRate(0)).execute();    
      }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
      .withVelocityY(0).withRotationalRate(0)).execute();
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.PhotonConsts.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.PhotonConsts.POSE_VALIDATION_TIME);
  }
}