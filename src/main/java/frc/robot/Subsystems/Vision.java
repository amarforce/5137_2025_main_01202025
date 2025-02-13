package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision_Constants;
import frc.robot.Constants.Vision_Constants.CameraNames;
import frc.robot.Constants.Vision_Constants.GameObjects;
import frc.robot.Utils.VisionDebugLogger;



/**
 * Subsystem for handling robot vision using PhotonVision.
 * This subsystem manages multiple cameras for AprilTag-based localization and object detection.
 */




public class Vision extends SubsystemBase {
    // Existing AprilTag vision components
    // private final AprilTagFieldLayout fieldLayout;
    private final PhotonCamera frontCamera;
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonCamera leftCamera;
    private final PhotonPoseEstimator leftPoseEstimator;

    // Object detection camera
    private final PhotonCamera frontObjectCamera;
    
    // Debug logger
    private final VisionDebugLogger debugLogger;

    
    /**
     * Constructor for the Vision class.
     * Initializes the vision subsystem, including AprilTag components for localization
     * and a separate camera for object detection. It also initializes a debug logger
     * for vision data.
     */
    public Vision() {
        // Initialize AprilTag components (existing functionality)
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Initialize AprilTag components (existing functionality)
        frontCamera = new PhotonCamera(CameraNames.FRONT_CAMERA);
        frontPoseEstimator = new PhotonPoseEstimator(
            fieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            Vision_Constants.robotToFrontCamera
        );


        leftCamera = new PhotonCamera(CameraNames.LEFT_CAMERA);
        leftPoseEstimator = new PhotonPoseEstimator(
            fieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            Vision_Constants.robotToLeftCamera
        );

        // Initialize object detection camera
        frontObjectCamera = new PhotonCamera(CameraNames.FRONT_OBJECT_CAMERA);
        
        // Initialize debug logger
        debugLogger = new VisionDebugLogger();
    }

    // Existing AprilTag methods
    public Optional<EstimatedRobotPose> getFrontPose() {
        return frontPoseEstimator.update(frontCamera.getLatestResult());
    }

    // Existing AprilTag methods
    public Optional<EstimatedRobotPose> getLeftPose() {
        return leftPoseEstimator.update(leftCamera.getLatestResult());
    }

    // Enhanced object detection methods
    public Optional<PhotonTrackedTarget> getClosestCoral() {
        var result = frontObjectCamera.getLatestResult();
        if (!result.hasTargets()) {
            return Optional.empty();
        }
        
        // Existing object detection logic
        PhotonTrackedTarget target = result.getBestTarget();
        if (target.getPoseAmbiguity() > Vision_Constants.Processing.MIN_TARGET_CONFIDENCE) {
            return Optional.empty();
        }
        
        return Optional.of(target);
    }

    public double getObjectX() {
        var target = getClosestCoral();
        return target.map(t -> t.getYaw()).orElse(0.0);
    }

    // Existing object detection methods
    public String getObjectClass() {
        return getClosestCoral().isPresent() ? GameObjects.CORAL : GameObjects.UNKNOWN;
    }

    // Existing periodic method
    @Override
    public void periodic() {
        updateDashboard();
        logVisionData();
    }

    // Existing private method
    private void updateDashboard() {
        // AprilTag data
        var frontPose = getFrontPose();
        var leftPose = getLeftPose();
        
        SmartDashboard.putBoolean("Vision/FrontCamera/HasPose", frontPose.isPresent());
        SmartDashboard.putBoolean("Vision/LeftCamera/HasPose", leftPose.isPresent());
        
        // Object detection data
        SmartDashboard.putNumber("Vision/Object/X", getObjectX());
        SmartDashboard.putString("Vision/Object/Class", getObjectClass());
        
        // Target information
        var coral = getClosestCoral();
        coral.ifPresent(target -> {
            SmartDashboard.putNumber("Vision/Target/Distance", target.getBestCameraToTarget().getTranslation().getNorm());
            SmartDashboard.putNumber("Vision/Target/Yaw", target.getYaw());
            SmartDashboard.putNumber("Vision/Target/Pitch", target.getPitch());
        });
    }

// Existing private method
    /**
     * Logs vision data for debugging purposes.
     * This includes the X coordinate of the detected object and its class.
     */
    private void logVisionData() {
        debugLogger.logVisionData(getObjectX(), getObjectClass());
    }


    // Start of Selection
    /**
     * Logs vision data for debugging and analysis.
     * This method retrieves the current object's X coordinate and its classification
     * and logs this information using the {@link frc.robot.Utils.DebugLogger}.
     */
    private void logVisionData() {
        debugLogger.logVisionData(getObjectX(), getObjectClass());
    }

    public void resetStats() {
        debugLogger.resetStats();
    }
}