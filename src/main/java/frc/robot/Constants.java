package frc.robot;

import java.util.concurrent.TransferQueue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {
    public static final double SLEW_ROTATION = 2.5; 

    public static final double SLEW_POSITIVE_MAX = 1;
    public static final double SLEW_NEGATIVE_MAX = 0.8;

    public static final double SLEW_POSITIVE_MIN = 0.6;
    public static final double SLEW_NEGATIVE_MIN = 0.6; 

    public static final String LimelightName = "";


    public static final int BEAMBREAK_ID = 0; 
    
    public static final class PrimaryElevatorConstants {
        public static final int LEFT_ELEVATOR_MOTOR_ID = 14;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 15;
        public static final int ENCODER_ID = 1;

        public static final double KP = 0.27;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
    }

    public static final class InnerElevatorConstants {
        public static final int MOTOR_ID = 22;

        public static final double KP = 0.05; //0.25
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double KS = 0;
        public static final double KG = 0.01;
        public static final double KV = 0;

        public static final double GRAVITY_NEGATION = 0.2;
    }

    public static final class ArmConstants {
        public static final int ARM_MOTOR_ID = 31;
        public static final int INDEX_MOTOR_ID = 25;
        public static final int ENCODER_ID = 0;


        public static final double KP = 0.008;
        public static final double KI = 0.0;   
        public static final double KD = 0.0;

        public static final double KS = 0;
        public static final double KG = 0.02;
        public static final double KV = 0;
    }

    public static final class ClimbConstants {
        public static final int MOTOR_ID = 27;
        public static final int ENCODER_ID = 1;

        //need to get later
        public static final double MAX_THRESHOLD = 270;
        public static final double MIN_THRESHOLD = 0;

    }

    public static final class ControllerConsts {
        public static final double FRIC = 0.09;
        public static final double PRECIECE_ADD_FRIC = 0.7; //how much acceleration will slow when in preciece mode
        public static final double PRECIECE_ADD_TRANSLATIONAL_DEADBAND_RATIO = 0.6; // how much slower the max speed goes when in preciece mode
        public static final double PRECIECE_ADD_ROTATIONAL_DEADBAND_RATIO = 0.5; // how much slower the max rotational speed goes when in preciece mode
        public static final double DEADBAND_RATIO = 0.6;//absolute max speed cutoff when elevator up

        public static final double SLOW_RATIO = 0.57;//how fast it will move when elevator up
        public static final double STRAFE_RATIO = 0.1;
        
    }
    

    public static final class PhotonConsts  {
        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        
        // public static final Transform3d GRAY_PHOTON_CAMERA_TO_ROBOT = new Transform3d();
        // public static final Transform3d BLUE_PHOTON_CAMERA_TO_ROBOT = new Transform3d();

        public static final Transform3d BLUE_PHOTON_CAMERA_TO_ROBOT = new Transform3d(
                    new Translation3d(-0.1956816, 0.2815336, 0),
                    new Rotation3d(0, 0, -0.43633231)
                );

        public static final Transform3d GRAY_PHOTON_CAMERA_TO_ROBOT = new Transform3d(
                    new Translation3d(-0.1956816, -0.2815336, 0),
                    new Rotation3d(0, 0, 0.43633231)
                );

        public static final String GRAY_CAMERA_NAME = "";
        public static final String BLUE_CAMERA_NAME = "";

        public static final double LAST_RESULT_TIMEOUT = 0.4;
                    
        public static final double CENTER_TO_TAG_DELTA_X = 0.94 + 0.3;

        public static final Transform2d GOAL_LEFT_REEF = new Transform2d(0.416, 0, new Rotation2d() );
        public static final Transform2d GOAL_RIGHT_REEF = new Transform2d(-0.416, 0, new Rotation2d());

        public static final double KP_TRANSLATION = 1;
        public static final double KI_TRANSLATION = 0;
        public static final double KD_TRANSLATION = 0;

        public static final double KP_ROTATION = 0.1;
        public static final double KI_ROTATION = 0;
        public static final double KD_ROTATION = 0;

        public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0; 
        public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1; 
        public static final double X_SETPOINT_REEF_ALIGNMENT = 0.416; 
        public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.05;
        public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.17; 
        public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.05; 
        public static final double DONT_SEE_TAG_WAIT_TIME = 1; 
        public static final double POSE_VALIDATION_TIME = 0.2; 

    }

    public static final class OdometryConsts {
        public static final double TranslationalThreshold = 0.3;
        public static final double RotationalThreshold = 0.1;
    }
}
