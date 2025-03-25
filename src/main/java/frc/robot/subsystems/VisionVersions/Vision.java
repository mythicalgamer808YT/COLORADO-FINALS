package frc.robot.subsystems.VisionVersions;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.PhotonConsts;
import frc.robot.States.ReefPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain; 

    private final PhotonCamera camera; 
    private final Transform3d cameraToRobot;
    private final PhotonPoseEstimator poseEstimator; 

    private List<PhotonPipelineResult> results;
    private ArrayList<PhotonTrackedTarget> currentTarget;
    private Transform3d tagToCamera; 
    private Transform2d tagToCameraProcessed; 
    //private Rotation2d estimatedYaw; 
    private Pose2d estimatedRobotPoseRelative; 
    private EstimatedRobotPose estimatedRobotPoseField; 

    private Pose2d closestTargetPose;
    private final ArrayList<Pose2d> redReefTags = new ArrayList<>(6);
    private final ArrayList<Pose2d> blueReefTags = new ArrayList<>(6);

    //private boolean targetSeen; 
    
    public Vision(String name, CommandSwerveDrivetrain drive, Transform3d cameraToRobot) {
        this.camera = new PhotonCamera(name);
        this.cameraToRobot = cameraToRobot;
        this.drivetrain = drive;
        poseEstimator = new PhotonPoseEstimator(Constants.PhotonConsts.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobot);
        closestTargetPose = new Pose2d();

         //red 6-11; blue 17-22
        for(int i = 0; i < 6; i++) {
            redReefTags.add(getAprilTagPose2d(i + 6));
            blueReefTags.add(getAprilTagPose2d(i + 17));
        }
    }

    @Override 
    public void periodic() {
        results = camera.getAllUnreadResults();
        currentTarget = new ArrayList<>(0);

        Optional<PhotonPipelineResult> currentResult = Optional.empty();

        for(var change : results) {
            currentResult = Optional.of(change);
        }


        if(currentResult.isEmpty()) {
            setSmartDashboardValues();
            return; 
        }
        
        double dist = Double.MAX_VALUE;

        for(PhotonTrackedTarget target : currentResult.get().getTargets()) {
            currentTarget.add(target);

            tagToCamera = target.getBestCameraToTarget();
            Pose2d tagFieldRelativePose = Constants.PhotonConsts.FIELD_LAYOUT.getTagPose(target.getFiducialId()).get().toPose2d();
            // MUST CHANGE GYRO IN ORDER TO FIT REQURIEMENTS
            // MUST BE 0 WHEN FACING OPPOSING ALLIANCE
            // MUST INCREASE WHEN MOVING COUNTER CLOCKWISE

            // calculate tag to camera transformation 
            // could possible end here for robot relative drive controls 
            tagToCameraProcessed = PhotonUtils.estimateCameraToTarget(tagToCamera.getTranslation().toTranslation2d(),
                    tagFieldRelativePose, 
                    drivetrain.getRotation3d().toRotation2d());  
            
            if(tagToCameraProcessed.getTranslation().getNorm() < dist) {
                closestTargetPose = tagFieldRelativePose;
            }
            // estimatedRobotPoseRelative = PhotonUtils.estimateFieldToRobot(new Transform2d(tagToCamera.getTranslation().toTranslation2d(), tagToCamera.getRotation().toRotation2d()), 
            //     Constants.Photon.FIELD_LAYOUT.getTagPose(target.getFiducialId()).get().toPose2d(),
            //     new Transform2d(cameraToRobot.getTranslation().toTranslation2d(), cameraToRobot.getRotation().toRotation2d()));

            // calculate field relative pose for the robot 
            estimatedRobotPoseRelative = PhotonUtils.estimateFieldToRobot(tagToCameraProcessed, 
                tagFieldRelativePose,
                new Transform2d(cameraToRobot.getTranslation().toTranslation2d(), cameraToRobot.getRotation().toRotation2d()));


            // add vision measurment. 
            if(currentResult.isPresent()) {
                PhotonPipelineResult elResult = currentResult.get();
                Optional<EstimatedRobotPose> estimatedRobotPoseFieldFornow = poseEstimator.update(elResult);
                if(estimatedRobotPoseFieldFornow.isPresent()) {
                    estimatedRobotPoseField = estimatedRobotPoseFieldFornow.get();
                }
                
            }
            drivetrain.addVisionMeasurement(estimatedRobotPoseRelative, Timer.getFPGATimestamp());
            setSmartDashboardValues();
        }
        
    }

    public EstimatedRobotPose getFieldRelativePose() {
        return estimatedRobotPoseField; 
    }
    
    public Pose2d getRobotRelativePose() {
        return estimatedRobotPoseRelative;
    }
    public Pose2d getClosestPose() {
        return closestTargetPose;
    }


    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Red;
    }

    public ArrayList<Pose2d> getReefAprilTags() {
        return isRedAlliance() ? redReefTags : blueReefTags;
    }

    public Pose2d getRobotPoseForNearestReefAprilTag(Pose2d currentRobotPose, ReefPosition reefPosition) {
        Pose2d nearestTagPose = currentRobotPose.nearest(getReefAprilTags());
        return nearestTagPose.plus(reefPosition.tagToRobot);
    }

    // public Pose2d getRotatedPose(int tagID, double xTranslation, double yTranslation) {
    //     switch (tagID) {
    //         case value:
                
    //             break;
        
    //         default:
    //             break;
    //     }
    // }

    public Pose2d getAprilTagPose2d(int tagId) {
        return PhotonConsts.FIELD_LAYOUT.getTagPose(tagId).get().toPose2d();
    }
    
    private void setSmartDashboardValues() {
        // camera to target readings 
        SmartDashboard.putNumber(camera.getName() + "estimated camera to target yaw", tagToCamera.getRotation().toRotation2d().getDegrees());
        SmartDashboard.putNumber(camera.getName() + "estimated camera to target x", tagToCamera.getX());
        SmartDashboard.putNumber(camera.getName() + "estimated camera to target y", tagToCamera.getY());

        // sole photon vision estimated robot position 
        SmartDashboard.putNumber(camera.getName() + "estimated field relative robot yaw", getFieldRelativePose().estimatedPose.getRotation().toRotation2d().getDegrees());
        SmartDashboard.putNumber(camera.getName() + "estimated field relative robot x", getFieldRelativePose().estimatedPose.getX());
        SmartDashboard.putNumber(camera.getName() + "estimated field relative robot y", getFieldRelativePose().estimatedPose.getY());

        SmartDashboard.putNumber(camera.getName() + "pidgeon readings ", drivetrain.getPigeon2().getYaw().getValueAsDouble());

        // robot to target readings 
        SmartDashboard.putNumber(camera.getName() + "estimated robot to target yaw", estimatedRobotPoseRelative.getRotation().getDegrees());
        SmartDashboard.putNumber(camera.getName() + "estimated robot to target x", estimatedRobotPoseRelative.getX());
        SmartDashboard.putNumber(camera.getName() + "estimated robot to target y", estimatedRobotPoseRelative.getY());

    }

}