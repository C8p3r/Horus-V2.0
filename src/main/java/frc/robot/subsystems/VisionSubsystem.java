package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Vision subsystem that manages multiple Limelights for field localization
 * Uses MegaTag2 with full 3D localization and advanced quality metrics
 * Publishes vision poses to NetworkTables for AdvantageScope visualization
 */
public class VisionSubsystem extends SubsystemBase {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final String frontLimelightName;
    private final String backLimelightName;
    
    private boolean useVision = true;
    
    // NetworkTables publishers for AdvantageScope
    private final StructPublisher<Pose2d> frontVisionPosePublisher;
    private final StructPublisher<Pose2d> backVisionPosePublisher;
    private final StructPublisher<Pose3d> frontVisionPose3dPublisher;
    private final StructPublisher<Pose3d> backVisionPose3dPublisher;
    private final StructArrayPublisher<Pose2d> allVisionPosesPublisher;
    
    // Publishers for camera-to-tag visualization lines
    private final StructPublisher<Pose3d> frontCameraPose3dPublisher;
    private final StructPublisher<Pose3d> backCameraPose3dPublisher;
    private final StructArrayPublisher<Pose3d> frontTagPosesPublisher;
    private final StructArrayPublisher<Pose3d> backTagPosesPublisher;
    
    // Field2d widget for SmartDashboard/Shuffleboard
    private final Field2d visionField;
    
    /**
     * Creates a new VisionSubsystem
     * @param drivetrain The swerve drivetrain to update with vision measurements
     */
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.frontLimelightName = VisionConstants.LIMELIGHT_FRONT_NAME;
        this.backLimelightName = VisionConstants.LIMELIGHT_BACK_NAME;
        
        // Initialize NetworkTables publishers for AdvantageScope
        var ntTable = edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("Vision");
        
        // 2D pose publishers for field visualization
        frontVisionPosePublisher = ntTable.getStructTopic("FrontCameraPose", Pose2d.struct).publish();
        backVisionPosePublisher = ntTable.getStructTopic("BackCameraPose", Pose2d.struct).publish();
        
        // 3D pose publishers for 3D visualization in AdvantageScope
        frontVisionPose3dPublisher = ntTable.getStructTopic("FrontCameraPose3d", Pose3d.struct).publish();
        backVisionPose3dPublisher = ntTable.getStructTopic("BackCameraPose3d", Pose3d.struct).publish();
        
        // Array publisher for all vision poses (for AdvantageScope multi-pose view)
        allVisionPosesPublisher = ntTable.getStructArrayTopic("AllVisionPoses", Pose2d.struct).publish();
        
        // Publishers for camera-to-tag visualization (camera positions and tag positions)
        frontCameraPose3dPublisher = ntTable.getStructTopic("FrontCameraPosition3d", Pose3d.struct).publish();
        backCameraPose3dPublisher = ntTable.getStructTopic("BackCameraPosition3d", Pose3d.struct).publish();
        frontTagPosesPublisher = ntTable.getStructArrayTopic("FrontVisibleTags", Pose3d.struct).publish();
        backTagPosesPublisher = ntTable.getStructArrayTopic("BackVisibleTags", Pose3d.struct).publish();
        
        // Field2d widget for Shuffleboard/SmartDashboard
        visionField = new Field2d();
        SmartDashboard.putData("Vision Field", visionField);
        
        // Initialize Limelights (LEDs off by default)
        LimelightHelpers.setLEDMode(frontLimelightName, 1);
        LimelightHelpers.setLEDMode(backLimelightName, 1);
        
        // Publish Limelight camera streams to CameraServer for Elastic dashboard
        setupCameraStreams();
    }
    
    /**
     * Sets up HTTP camera streams from Limelights to CameraServer
     * This allows viewing the Limelight streams on the Elastic dashboard
     */
    private void setupCameraStreams() {
        try {
            // Front Limelight stream
            HttpCamera frontCamera = new HttpCamera(
                "Limelight Front", 
                "http://" + frontLimelightName + ".local:5800/stream.mjpg"
            );
            CameraServer.startAutomaticCapture(frontCamera);
            
            // Back Limelight stream
            HttpCamera backCamera = new HttpCamera(
                "Limelight Back", 
                "http://" + backLimelightName + ".local:5800/stream.mjpg"
            );
            CameraServer.startAutomaticCapture(backCamera);
            
            System.out.println("Limelight camera streams published to CameraServer");
        } catch (Exception e) {
            System.err.println("Failed to setup Limelight camera streams: " + e.getMessage());
        }
    }
    
    @Override
    public void periodic() {
        // Always update telemetry and publish poses for visualization
        updateTelemetry();
        publishVisionPoses();
        
        // Only process measurements if vision is enabled
        if (!useVision) {
            return;
        }
        
        // Process front Limelight
        processLimelightMeasurement(frontLimelightName);
        
        // Process back Limelight
        processLimelightMeasurement(backLimelightName);
    }
    
    /**
     * Process vision measurements from a single Limelight using MegaTag2
     */
    private void processLimelightMeasurement(String limelightName) {
        // Check if we have a valid target
        if (!LimelightHelpers.hasTarget(limelightName)) {
            return;
        }
        
        // Get the MegaTag2 pose estimate with full metadata
        LimelightHelpers.PoseEstimate poseEstimate = getLimelightPoseEstimateForAlliance(limelightName);
        
        // Reject if no tags visible
        if (poseEstimate.tagCount == 0) {
            return;
        }
        
        // Reject measurements that are too far away
        if (poseEstimate.avgTagDist > VisionConstants.MAX_VISION_DISTANCE) {
            return;
        }
        
        // Use tag span as a quality metric (larger span = better geometry)
        // For now, we can use a simple threshold or incorporate it into std dev calculation
        
        // Determine standard deviations based on MegaTag2 metadata
        Matrix<N3, N1> stdDevs = getVisionStdDevs(poseEstimate);
        
        // Add vision measurement to drivetrain with timestamp accounting for latency
        drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, stdDevs);
    }
    
    /**
     * Gets the robot pose estimate from Limelight based on current alliance
     */
    private LimelightHelpers.PoseEstimate getLimelightPoseEstimateForAlliance(String limelightName) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
        } else {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        }
    }
    
    /**
     * Calculates vision standard deviations based on MegaTag2 metadata
     * More tags, better geometry (tag span), and closer distance = lower std dev (more trust)
     * 
     * MegaTag2 provides:
     * - tagCount: Number of tags used in pose calculation
     * - tagSpan: Geometric spread of tags (larger = better triangulation)
     * - avgTagDist: Average distance to tags (closer = more reliable)
     * - avgTagArea: Average tag area in image (larger = more pixels = better)
     */
    private Matrix<N3, N1> getVisionStdDevs(LimelightHelpers.PoseEstimate estimate) {
        Matrix<N3, N1> baseStdDevs;
        
        // Choose base standard deviation based on tag count
        if (estimate.tagCount >= 2) {
            baseStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
        } else {
            baseStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
        }
        
        // Distance multiplier: further = less trust
        // Square the distance to penalize far measurements more heavily
        double distanceMultiplier = Math.pow(estimate.avgTagDist, 2) / 4.0;
        
        // Tag span multiplier: larger span = better geometry = more trust
        // A span of 0 means all tags are at the same location (bad)
        // A larger span means better triangulation
        double spanMultiplier = 1.0;
        if (estimate.tagCount >= 2) {
            // Normalize span - typical good span is > 1.0 meters
            // Scale from 0.5 (span >= 2.0m) to 2.0 (span near 0)
            spanMultiplier = Math.max(0.5, 2.0 / Math.max(0.1, estimate.tagSpan));
        }
        
        // Area multiplier: larger tags in view = more reliable
        // avgTagArea is percentage of image (0-100)
        // Larger tags = closer or better view = more reliable
        double areaMultiplier = 1.0;
        if (estimate.avgTagArea > 0) {
            // Scale from 0.7 (large tags >0.8%) to 1.5 (small tags <0.2%)
            areaMultiplier = Math.max(0.7, Math.min(1.5, 1.0 / Math.max(0.5, estimate.avgTagArea)));
        }
        
        // Combine all multipliers
        double overallMultiplier = distanceMultiplier * spanMultiplier * areaMultiplier;
        overallMultiplier = Math.max(1.0, overallMultiplier); // Never trust more than base
        
        return baseStdDevs.times(overallMultiplier);
    }
    
    /**
     * Updates telemetry for debugging - includes MegaTag2 metadata
     */
    private void updateTelemetry() {
        // Get full PoseEstimate data for both cameras
        LimelightHelpers.PoseEstimate frontEstimate = getLimelightPoseEstimateForAlliance(frontLimelightName);
        LimelightHelpers.PoseEstimate backEstimate = getLimelightPoseEstimateForAlliance(backLimelightName);
        
        // Front Limelight
        SmartDashboard.putBoolean("Vision/Front/HasTarget", LimelightHelpers.hasTarget(frontLimelightName));
        SmartDashboard.putNumber("Vision/Front/NumTags", frontEstimate.tagCount);
        SmartDashboard.putNumber("Vision/Front/AvgDistance", frontEstimate.avgTagDist);
        SmartDashboard.putNumber("Vision/Front/TagSpan", frontEstimate.tagSpan);
        SmartDashboard.putNumber("Vision/Front/AvgArea", frontEstimate.avgTagArea);
        SmartDashboard.putNumber("Vision/Front/Latency", frontEstimate.latency);
        
        // Back Limelight
        SmartDashboard.putBoolean("Vision/Back/HasTarget", LimelightHelpers.hasTarget(backLimelightName));
        SmartDashboard.putNumber("Vision/Back/NumTags", backEstimate.tagCount);
        SmartDashboard.putNumber("Vision/Back/AvgDistance", backEstimate.avgTagDist);
        SmartDashboard.putNumber("Vision/Back/TagSpan", backEstimate.tagSpan);
        SmartDashboard.putNumber("Vision/Back/AvgArea", backEstimate.avgTagArea);
        SmartDashboard.putNumber("Vision/Back/Latency", backEstimate.latency);
        
        // Overall
        SmartDashboard.putBoolean("Vision/Enabled", useVision);
        SmartDashboard.putNumber("Vision/TotalTags", frontEstimate.tagCount + backEstimate.tagCount);
    }
    
    /**
     * Publishes vision poses to NetworkTables for AdvantageScope visualization
     * This allows you to see the raw vision estimates on the field
     */
    private void publishVisionPoses() {
        // Get pose estimates from both cameras
        LimelightHelpers.PoseEstimate frontEstimate = getLimelightPoseEstimateForAlliance(frontLimelightName);
        LimelightHelpers.PoseEstimate backEstimate = getLimelightPoseEstimateForAlliance(backLimelightName);
        
        // Publish 2D poses for field visualization
        if (LimelightHelpers.hasTarget(frontLimelightName) && frontEstimate.tagCount > 0) {
            frontVisionPosePublisher.set(frontEstimate.pose);
        }
        if (LimelightHelpers.hasTarget(backLimelightName) && backEstimate.tagCount > 0) {
            backVisionPosePublisher.set(backEstimate.pose);
        }
        
        // Publish 3D poses for 3D visualization in AdvantageScope
        if (LimelightHelpers.hasTarget(frontLimelightName) && frontEstimate.tagCount > 0) {
            frontVisionPose3dPublisher.set(frontEstimate.pose3d);
        }
        if (LimelightHelpers.hasTarget(backLimelightName) && backEstimate.tagCount > 0) {
            backVisionPose3dPublisher.set(backEstimate.pose3d);
        }
        
        // Publish array of all valid vision poses
        List<Pose2d> allPoses = new ArrayList<>();
        if (LimelightHelpers.hasTarget(frontLimelightName) && frontEstimate.tagCount > 0) {
            allPoses.add(frontEstimate.pose);
        }
        if (LimelightHelpers.hasTarget(backLimelightName) && backEstimate.tagCount > 0) {
            allPoses.add(backEstimate.pose);
        }
        allVisionPosesPublisher.set(allPoses.toArray(new Pose2d[0]));
        
        // Update Field2d widget with vision poses
        if (!allPoses.isEmpty()) {
            visionField.setRobotPose(allPoses.get(0)); // Show first pose as "robot"
            // Add additional poses as objects if there are multiple
            if (allPoses.size() > 1) {
                visionField.getObject("Camera2").setPose(allPoses.get(1));
            }
        }
        
        // Publish camera-to-tag visualization data
        publishCameraToTagLines();
    }
    
    /**
     * Publishes camera positions and tag positions for visualization in AdvantageScope
     * This creates lines from each camera to the AprilTags it can see
     */
    private void publishCameraToTagLines() {
        // Get current robot pose from drivetrain
        Pose2d robotPose2d = drivetrain.getState().Pose;
        Pose3d robotPose3d = new Pose3d(robotPose2d);
        
        // Front camera
        if (LimelightHelpers.hasTarget(frontLimelightName)) {
            // Calculate camera position in field space
            Pose3d frontCameraPose = LimelightHelpers.getCameraPose3d(
                frontLimelightName,
                robotPose3d,
                VisionConstants.ROBOT_TO_FRONT_CAMERA
            );
            frontCameraPose3dPublisher.set(frontCameraPose);
            
            // Get visible tag poses in field space
            Pose3d[] frontTagPoses = LimelightHelpers.getVisibleTagPoses3d(frontLimelightName, robotPose3d);
            if (frontTagPoses.length > 0) {
                frontTagPosesPublisher.set(frontTagPoses);
            }
        }
        
        // Back camera
        if (LimelightHelpers.hasTarget(backLimelightName)) {
            // Calculate camera position in field space
            Pose3d backCameraPose = LimelightHelpers.getCameraPose3d(
                backLimelightName,
                robotPose3d,
                VisionConstants.ROBOT_TO_BACK_CAMERA
            );
            backCameraPose3dPublisher.set(backCameraPose);
            
            // Get visible tag poses in field space
            Pose3d[] backTagPoses = LimelightHelpers.getVisibleTagPoses3d(backLimelightName, robotPose3d);
            if (backTagPoses.length > 0) {
                backTagPosesPublisher.set(backTagPoses);
            }
        }
    }
    
    /**
     * Enable or disable vision processing
     */
    public void setVisionEnabled(boolean enabled) {
        this.useVision = enabled;
    }
    
    /**
     * Toggle vision processing on/off
     */
    public void toggleVision() {
        setVisionEnabled(!useVision);
    }
    
    /**
     * Set LED mode for both Limelights
     * 0 = pipeline mode, 1 = off, 2 = blink, 3 = on
     */
    public void setLEDMode(int mode) {
        LimelightHelpers.setLEDMode(frontLimelightName, mode);
        LimelightHelpers.setLEDMode(backLimelightName, mode);
    }
}
