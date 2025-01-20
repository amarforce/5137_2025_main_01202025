package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Vision_Constants {
    // Camera transforms (existing constants)
    public static final Transform3d robotToFrontCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final Transform3d robotToLeftCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final Transform3d robotToFrontObjectCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    // Camera names
    public static final class CameraNames {
        public static final String FRONT_CAMERA = "frontCamera";
        public static final String LEFT_CAMERA = "leftCamera";
        public static final String FRONT_OBJECT_CAMERA = "frontObjectCamera";
    }

    // PID constants for vision alignment
    public static final class AlignmentPID {
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.05;
        public static final double TOLERANCE = 1.0;
        public static final double SETPOINT = 0.0;
    }

    // Game object classifications
    public static final class GameObjects {
        public static final String CORAL = "coral";
        public static final String UNKNOWN = "unknown";
    }

    // Vision processing constants
    public static final class Processing {
        public static final double MIN_TARGET_CONFIDENCE = 0.7;
        public static final double MAX_TARGET_DISTANCE = 5.0; // meters
    }
}