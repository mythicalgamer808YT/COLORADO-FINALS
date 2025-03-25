package frc.robot.subsystems.VisionVersions;
// package frc.robot.subsystems;

// import java.util.ArrayList;
// import java.util.List;
// import java.util.Optional;

// import org.photonvision.PhotonCamera;

// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.PhotonConsts;
// import frc.robot.States.ReefPosition;

// public class PhotonSubsystem extends SubsystemBase {
    
//     private final PhotonPipelineResult NO_RESULT = new PhotonPipelineResult();
    
//     private final PhotonCamera camera;
//     private Optional<PhotonPipelineResult> result = Optional.empty();
//     private final CommandSwerveDrivetrain drivetrain;

//     //i don't think it matters really
//     //private final List<Integer> validIDs;

//     private final ArrayList<Pose2d> redReefTags = new ArrayList<>(6);
//     private final ArrayList<Pose2d> blueReefTags = new ArrayList<>(6);

//     private final Transform3d robotToCamera;
//     //private final PhotonPoseEstimator estimator;

//     private int targetID; 
//     private Optional<Transform3d> bestTransform;

//     private boolean targetSeen;

//     public PhotonSubsystem(String cameraName,
//      CommandSwerveDrivetrain drivetrain,
//      Transform3d robotToCamera) {
//         camera = new PhotonCamera(cameraName);
//         this.robotToCamera = robotToCamera;
        
//         this.drivetrain = drivetrain;

//         // this.estimator = new PhotonPoseEstimator(
//         //     PhotonConsts.aprilTagFieldLayout,
//         //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//         //     robotToCamera
//         // );

//         //red 6-11; blue 17-22
//         for(int i = 0; i < 6; i++) {
//             redReefTags.add(getAprilTagPose2d(i + 6));
//             blueReefTags.add(getAprilTagPose2d(i + 17));
//         }

//         bestTransform = Optional.of(new Transform3d());

//         //validIDs = PhotonConsts.VALID_REEF_IDS;

//     }

//     @Override 
//     public void periodic() {

//         Optional<PhotonPipelineResult> currentResult = Optional.empty();
//         List<PhotonPipelineResult> allUnreadResults = camera.getAllUnreadResults();
//         for(var change : allUnreadResults) {
//             //estimator.update(change);
//             currentResult = Optional.of(change);
//         }

//         targetSeen = currentResult.orElse(NO_RESULT).hasTargets();

//         if(currentResult.isPresent()
//                 || (Timer.getFPGATimestamp() - this.result.orElse(NO_RESULT).getTimestampSeconds()) > PhotonConsts.LAST_RESULT_TIMEOUT) {
//                     this.result = currentResult;
//         }

//         if(hasTargets()) {
//             PhotonTrackedTarget bestTarget = getBestTarget();
//             Transform3d bestTargetTransform = robotToCamera.plus(bestTarget.getBestCameraToTarget());
//             targetID = bestTarget.getFiducialId();
//             try {
//             bestTransform = Optional.of(bestTargetTransform);
//             } catch() {

//             }

//         }

//         drivetrain.addVisionMeasurement(getTagToRobotPose2d(), getLatestResultTimestamp());

//     }

//     public List<PhotonTrackedTarget> getTargets() {
//         return result.orElse(NO_RESULT).getTargets();
//     }

//     public Optional<PhotonTrackedTarget> getTarget(int id) {
//         return getTargets().stream().filter(target -> target.getFiducialId() == id).findFirst();
//     }

//     public double getLatestResultTimestamp() {
//         return result.orElse(NO_RESULT).getTimestampSeconds();
//     }

//     public Pose2d getTagToRobotPose2d() {
//         Transform3d invertedTransform = bestTransform.get().inverse();
//         return PhotonConsts.FIELD_LAYOUT.getTagPose(targetID).get().transformBy(invertedTransform).toPose2d();
        
//     }

//     public int getTagID() {
//         return targetID;
//     }

//     public boolean hasTargets() {
//         return result.orElse(NO_RESULT).hasTargets();
//     }

//     public PhotonTrackedTarget getBestTarget() {
//         return result.orElse(NO_RESULT).getBestTarget();
//     }

//     public boolean isRedAlliance() {
//         var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
//         return alliance == Alliance.Red;
//     }

//     public ArrayList<Pose2d> getReefAprilTags() {
//         return isRedAlliance() ? redReefTags : blueReefTags;
//     }

//     public Pose2d getRobotPoseForNearestReefAprilTag(
//         Pose2d currentRobotPose, ReefPosition reefPosition) {
//             Pose2d nearestTagPose = currentRobotPose.nearest(getReefAprilTags());
            
//             return nearestTagPose.plus(reefPosition.tagToRobot);
//     }

//     public Pose2d getAprilTagPose2d(int tagId) {
//         return PhotonConsts.FIELD_LAYOUT.getTagPose(tagId).get().toPose2d();
//     }

//     private void setSmartDashboardValues() {
//         SmartDashboard.putNumber(camera.getName() + " target ID", targetID);
//         SmartDashboard.putBoolean(camera.getName() + " target seen", targetSeen);
//     }
// }
