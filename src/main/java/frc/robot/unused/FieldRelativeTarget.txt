package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class FieldRelativeTarget {
  // Example constants (adjust these to your application)
  private static final double robotHalfLength = 0.5;      // half the robot’s length (meters)
  private static final double distTagToPeg = 0.3;           // distance offset from tag to peg (meters)
  private static final double desiredDistFromTag = 1.0;       // desired “back” distance from tag (meters)

  // Assuming an enum for the relative direction the robot should position itself
  public enum Direction {
    LEFT,
    RIGHT,
    BACK
    // Add more cases as needed
  }

  /**
   * Calculates a desired field-relative Pose2d for the robot based on a given AprilTag.
   *
   * <p>The method:
   * <ul>
   *   <li>Retrieves the AprilTag’s field pose.
   *   <li>Rotates the tag’s heading by 180° so that the robot faces the tag.
   *   <li>Computes an offset vector that is different if the tag is “top” versus “bottom”
   *       (and similarly for tags on the sides).
   *   <li>The offset is applied relative to the rotated tag pose.
   * </ul>
   *
   * @param tagID the ID of the AprilTag (to fetch its pose)
   * @param dir the desired robot relative direction (LEFT, RIGHT, or BACK)
   * @return a Pose2d that is the target for the robot.
   */
  private Pose2d computeNewPoseFromTag(int tagID, Direction dir) {
    // Retrieve the AprilTag’s pose from the field layout.
    Pose2d tagPose = Constants.PhotonConsts.FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();

    // Rotate the tag’s heading by 180° so that the robot (which will drive to the target)
    // ends up facing the tag.
    tagPose = new Pose2d(
        tagPose.getX(),
        tagPose.getY(),
        tagPose.getRotation().rotateBy(new Rotation2d(Math.toRadians(180)))
    );

    // Determine if the tag is in the "top" or "bottom" half of the field.
    // For this example, we assume a field coordinate system where y > 0 indicates the top.
    boolean isTopTag = tagPose.getY() > 0;

    // Also determine if the tag is on the left side of the field.
    // Here, x < 0 indicates the left side.
    boolean isLeftTag = tagPose.getX() < 0;

    double translationX = 0.0;
    double translationY = 0.0;

    // Compute an offset vector based on the desired robot positioning (dir)
    // with adjustments based on the tag's location.
    switch (dir) {
      case LEFT:
        // For a LEFT relative placement, we start with a fixed offset from the tag.
        translationX = -robotHalfLength;
        // For tags along the top, we might want the y offset to be inverted relative to the bottom.
        translationY = isTopTag ? -distTagToPeg : distTagToPeg;
        break;
      case RIGHT:
        translationX = -robotHalfLength;
        // Reverse the sign compared to LEFT.
        translationY = isTopTag ? distTagToPeg : -distTagToPeg;
        break;
      case BACK:
        // For a BACK offset, move away along the tag’s x-axis.
        translationX = -desiredDistFromTag;
        translationY = 0.0;
        break;
      default:
        // Optionally throw an error or assign default values.
        break;
    }

    // Additionally, if the tag is on one of the field’s sides, you may need further adjustment.
    // For instance, if the tag is on the left side of the field, you might mirror the translationX.
    if (isLeftTag && (dir == Direction.LEFT || dir == Direction.RIGHT)) {
      // This example simply inverts translationX. (Tweak as necessary.)
      translationX = -translationX;
    }

    // Create a translation offset and rotate it by the tag’s (adjusted) rotation so that
    // the offset is correctly applied in field space.
    Translation2d offset = new Translation2d(translationX, translationY)
        .rotateBy(tagPose.getRotation());
    Translation2d targetTranslation = tagPose.getTranslation().plus(offset);

    // The target pose uses the adjusted rotation (which makes the robot face the tag)
    return new Pose2d(targetTranslation, tagPose.getRotation());
  }
}