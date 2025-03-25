package frc.robot;

public class Constants {
    public static final int BEAMBREAK_ID = 0; 
    
    public static final class PrimaryElevatorConstants {
        public static final int LEFT_ELEVATOR_MOTOR_ID = 14;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 15;
        public static final int ENCODER_ID = 1;

        public static final double KP = 0.3;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
    }

    public static final class InnerElevatorConstants {
        public static final int MOTOR_ID = 22;

        public static final double KP = 0.25;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double KS = 0;
        public static final double KG = 0.01;
        public static final double KV = 0;

        public static final double GRAVITY_NEGATION = 0.2;
    }

    public static final class ArmConstants {
        public static final int ARM_MOTOR_ID = 16;
        public static final int INDEX_MOTOR_ID = 25;
        public static final int ENCODER_ID = 0;

        public static final double KP = 0.012;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double KS = 0;
        public static final double KG = 0.003;
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
        public static final double FRIC = 0.07;
    }
}
