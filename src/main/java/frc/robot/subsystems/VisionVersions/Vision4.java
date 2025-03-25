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

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Utilities;
import frc.robot.Constants;
import frc.robot.Constants.PhotonConsts;
import frc.robot.States.ReefPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision4 extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain; 

    private final PhotonCamera camera; 
    private final Transform3d cameraToRobot;
    private final PhotonPoseEstimator poseEstimator; 

    private List<PhotonPipelineResult> results;
    private ArrayList<PhotonTrackedTarget> currentTarget;
    private Transform3d tagToCamera; 
    private Transform2d tagToCameraProcessed; 
    private Pose2d estimatedRobotPoseRelative; 
    private EstimatedRobotPose estimatedRobotPoseField; 
    private int targetid12; 
    private Transform3d nearestTransform; 
    private int bestTargetId; 

    private int closestTarget; 
    private double leastDistance; 
    private boolean targetSeen; 

    /*
     * when it's true then the 
     * next time the button is pressed
     * it'll turn off the PhotonAlignCommand
     */
    private boolean override;
    
    private final ArrayList<Pose2d> redReefTags = new ArrayList<>(6);
    private final ArrayList<Pose2d> blueReefTags = new ArrayList<>(6);

    public Vision4(String name, CommandSwerveDrivetrain drive, Transform3d cameraToRobot) {
        this.camera = new PhotonCamera(name);
        this.cameraToRobot = cameraToRobot;
        this.drivetrain = drive;
        poseEstimator = new PhotonPoseEstimator(Constants.PhotonConsts.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobot);
        this.targetSeen = false; 

        this.override = false;

        for(int i = 0; i < 6; i++) {
            redReefTags.add(getAprilTagPose2d(i + 6));
            blueReefTags.add(getAprilTagPose2d(i + 17));
        }
    }

    @Override 
    public void periodic() {
        targetSeen = false; 
        results = camera.getAllUnreadResults();
        Optional<PhotonPipelineResult> currentResult = Optional.empty();
        nearestTransform = null; 

        for(var change : results) {
            currentResult = Optional.of(change);
        }

        if(currentResult == null || currentResult.isEmpty()) {
            SmartDashboard.putBoolean(camera.getName() + " target seen", targetSeen);
            return; 
        }
        
        closestTarget = -1; 
        leastDistance = Double.MAX_VALUE;
        PhotonTrackedTarget target = currentResult.get().getBestTarget();

            targetSeen = true; 
            SmartDashboard.putBoolean(camera.getName() + " target seen", targetSeen);
            targetid12 = target.getFiducialId();
           // currentTarget.add(target);
            // tagToCamera = target.getBestCameraToTarget();
            // if(nearestTransform == null) {
            //     nearestTransform = tagToCamera;
            // } else if (tagToCamera.getTranslation().getNorm() < nearestTransform.getTranslation().getNorm()) {
            //     nearestTransform = tagToCamera; 

            // }

            Pose2d tagFieldRelativePose = Constants.PhotonConsts.FIELD_LAYOUT.getTagPose(target.getFiducialId()).get().toPose2d();

            SmartDashboard.putNumber("Current Seen Tag Field X", tagFieldRelativePose.getX());
            SmartDashboard.putNumber("Current Seen Tag Field Y", tagFieldRelativePose.getY());
            SmartDashboard.putNumber("Current Seen Tag Field Yaw", tagFieldRelativePose.getRotation().getRadians());

            // MUST CHANGE GYRO IN ORDER TO FIT REQURIEMENTS
            // MUST BE 0 WHEN FACING OPPOSING ALLIANCE
            // MUST INCREASE WHEN MOVING COUNTER CLOCKWISE

            // calculate tag to camera transformation 
            // could possible end here for robot relative drive controls 
            tagToCameraProcessed = PhotonUtils.estimateCameraToTarget(tagToCamera.getTranslation().toTranslation2d(),
                    tagFieldRelativePose, 
                    drivetrain.getRotation3d().toRotation2d());  

            
            if(leastDistance > tagToCameraProcessed.getTranslation().getNorm()) {
                closestTarget = target.getFiducialId();
                leastDistance = tagToCameraProcessed.getTranslation().getNorm();
            }
            
            // alternative robot pose estimation could possibly need to be changed later 
            // current version could work out but not too sure 
            // estimatedRobotPoseRelative = PhotonUtils.estimateFieldToRobot(new Transform2d(tagToCamera.getTranslation().toTranslation2d(), tagToCamera.getRotation().toRotation2d()), 
            //     Constants.Photon.FIELD_LAYOUT.getTagPose(target.getFiducialId()).get().toPose2d(),
            //     new Transform2d(cameraToRobot.getTranslation().toTranslation2d(), cameraToRobot.getRotation().toRotation2d()));

            // calculate field relative pose for the robot 
            estimatedRobotPoseRelative = PhotonUtils.estimateFieldToRobot(tagToCameraProcessed, 
                tagFieldRelativePose,
                new Transform2d(cameraToRobot.getTranslation().toTranslation2d(), cameraToRobot.getRotation().toRotation2d()));


            // add vision measurment to drivetrain 
            if(poseEstimator.update(currentResult.get()) == null || poseEstimator.update(currentResult.get()).isPresent()) {
                estimatedRobotPoseField = poseEstimator.update(currentResult.get()).get();
                drivetrain.addVisionMeasurement(estimatedRobotPoseRelative, Timer.getFPGATimestamp());
                setSmartDashboardValues();
            }

            SmartDashboard.putNumber(camera.getName() + "estimated camera to target yaw", tagToCamera.getRotation().toRotation2d().getDegrees());
            SmartDashboard.putNumber(camera.getName() + "estimated camera to target x", tagToCamera.getX());
            SmartDashboard.putNumber(camera.getName() + "estimated camera to target y", tagToCamera.getY());
    
            SmartDashboard.putNumber(camera.getName() + "pidgeon readings ", drivetrain.getPigeon2().getYaw().getValueAsDouble());
            SmartDashboard.putNumber(camera.getName() + " odometry readings yaw", drivetrain.getState().Pose.getRotation().getDegrees());
            SmartDashboard.putNumber(camera.getName() + " odometry readings X", drivetrain.getState().Pose.getX());
            SmartDashboard.putNumber(camera.getName() + " odometry readings Y", drivetrain.getState().Pose.getY());
    
            
            // robot to target readings 
            // SmartDashboard.putNumber(camera.getName() + "estimated robot to target yaw", estimatedRobotPoseRelative.getRotation().getDegrees());
            // SmartDashboard.putNumber(camera.getName() + "estimated robot to target x", estimatedRobotPoseRelative.getX());
            // SmartDashboard.putNumber(camera.getName() + "estimated robot to target y", estimatedRobotPoseRelative.getY());
    

        
    }

    public void setOverride(boolean Override) {
        this.override = Override;
    }

    public double getYaw() {
        // if(targetid12 == 21 || targetid12 == 10) {
        //     return 0;
        // } else if(targetid12 == 20 || targetid12 == 9) {
        //     return 60;
        // } else if(targetid12 == 19 || targetid12 == 8) {
        //     return 60;
        // } else if(targetid12 == 20 || targetid12 == 9) {
        //     return 60;
        // } else if(targetid12 == 20 || targetid12 == 9) {
        //     return 60;
        // } 

        switch (targetid12) {
            // blue alliance 
            case 21:
                return 0; 
            case 20:
                return 60;
            case 19:
                return 120;
            case 18:
                return 180;
            case 17:
                return 240;
            case 22:
                return 300;

            // red alliance 
            case 10:
                return 0; 
            case 11:
                return 60;
            case 6:
                return 120;
            case 7:
                return 180;
            case 8:
                return 240;
            case 9:
                return 300;
        }
        return 0;  
    }

    public Transform3d getBestTagToCamera() {
        return tagToCamera;
    }

    public Pose2d getRobotPoseForNearestReefAprilTag(Pose2d currentRobotPose, ReefPosition reefPosition) {


        Pose2d nearestTagPose = currentRobotPose.nearest(getReefAprilTags());
        Pose2d desiredPose = nearestTagPose.plus(reefPosition.tagToRobot);

        SmartDashboard.putNumber("desired Field Pose X", desiredPose.getX());
        SmartDashboard.putNumber("desired Field Pose Y", desiredPose.getY());
        SmartDashboard.putNumber("desired Field Yaw", desiredPose.getRotation().getRadians());

        return desiredPose;
    }

    public ArrayList<Pose2d> getReefAprilTags() {
        return isRedAlliance() ? redReefTags : blueReefTags;
    }

    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Red;
    }

    // public Pose2d getDesiredRobotToTagFieldPose() {
    //     if(estimatedRobotPoseField != null) {
            
    //     } else {
    //         //return current pose if no 
    //         return drivetrain.getState().Pose;
    //     }
    // }

    public EstimatedRobotPose getFieldRelativePose() {
        return estimatedRobotPoseField; 
    }
    
    public Pose2d getRobotRelativePose() {
        return estimatedRobotPoseRelative;
    }

    public Pose2d getAprilTagPose2d(int tagId) {
        return PhotonConsts.FIELD_LAYOUT.getTagPose(tagId).get().toPose2d();
    }

    public Transform3d getBestTarget() {
        return tagToCamera; 
    }

    public boolean getTargetSeen() {
        return targetSeen; 
    }

    public int getClosestTarget() {
        return closestTarget; 
    }

    public boolean getOverride() {
        return override;
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
        SmartDashboard.putNumber(camera.getName() + "converted pidgeon readings", Utilities.convertGyroReadings(drivetrain.getPigeon2().getYaw().getValueAsDouble()));

        SmartDashboard.putNumber(camera.getName() + " odometry readings yaw", drivetrain.getState().Pose.getRotation().getDegrees());
        SmartDashboard.putNumber(camera.getName() + " odometry readings X", drivetrain.getState().Pose.getX());
        SmartDashboard.putNumber(camera.getName() + " odometry readings Y", drivetrain.getState().Pose.getY());

        
        // robot to target readings 
        SmartDashboard.putNumber(camera.getName() + "estimated robot to target x", tagToCamera.getX());
        SmartDashboard.putNumber(camera.getName() + "estimated robot to target y", tagToCamera.getY());

    }

}