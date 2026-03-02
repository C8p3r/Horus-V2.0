package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

import java.util.ArrayList;
import java.util.List;

/**
 * Vision subsystem that manages multiple Limelights for field localization
 * 
 * POSE ESTIMATION ARCHITECTURE:
 * ============================
 * This system implements a multi-sensor fusion approach for optimal pose estimation:
 * 
 * 1. VISION SOURCES (MegaTag1 ONLY):
 *    - Front Limelight: Uses MegaTag1 with ORB feature tracking for stability
 *    - Back Limelight: Uses MegaTag1 with ORB feature tracking for stability
 *    - MegaTag1 provides more stable estimates by tracking features between frames
 *    - Does NOT use MegaTag2 to avoid potential instability
 * 
 * 2. PIGEON GYRO:
 *    - Provides continuous high-frequency rotation data
 *    - Fused with vision for stable heading estimation
 *    - Configured in drivetrain's pose estimator
 * 
 * 3. WHEEL ODOMETRY:
 *    - Swerve module encoders provide continuous position tracking
 *    - High frequency updates (250 Hz) for smooth motion
 *    - Supplemental to vision and gyro
 * 
 * 4. POSE FUSION (in CommandSwerveDrivetrain):
 *    - SwerveDrivePoseEstimator fuses vision, gyro, and wheel odometry
 *    - Vision measurements provide absolute position corrections
 *    - Gyro provides stable heading
 *    - Wheel odometry fills gaps between vision updates
 * 
 * 5. OUTPUT TO ELASTIC/ADVANTAGESCOPE:
 *    - Fused pose published to NetworkTables at "DriveState/Pose" (50 Hz)
 *    - Individual vision estimates published for debugging
 *    - Field2d widget shows robot position in real-time
 * 
 * QUALITY METRICS:
 * - Tag count: More tags = lower std dev = more trust
 * - Tag span: Wider geometry = better triangulation = more trust  
 * - Distance: Closer tags = more reliable = more trust
 * - Area: Larger tags in image = better resolution = more trust
 */
public class VisionSubsystem extends SubsystemBase {
    
    /**
     * Record containing a vision measurement with its quality metrics
     * Used to determine the most trusted vision measurement
     */
    public record VisionMeasurement(
        Pose2d pose,
        int tagCount,
        double avgTagDist,
        double avgTagArea,
        double tagSpan,
        double timestampSeconds,
        String cameraName,
        double latencySeconds,
        double poseAmbiguity
    ) {
        /**
         * Calculate a comprehensive trust score for this measurement (higher = more trusted)
         * Considers all quality factors: tag count, distance, area, tag span, ambiguity, latency
         */
        public double getTrustScore() {
            double score = 0;
            
            // Tag count factor: more tags = significantly higher score (most important)
            // Multi-tag poses are much more reliable than single-tag
            if (tagCount >= 4) {
                score += 500; // Excellent - 4+ tags
            } else if (tagCount >= 3) {
                score += 350; // Very good - 3 tags
            } else if (tagCount >= 2) {
                score += 200; // Good - 2 tags
            } else {
                score += 50;  // Single tag - less reliable
            }
            
            // Distance factor: closer = higher score (exponential decay)
            // Vision accuracy decreases significantly with distance
            if (avgTagDist > 0) {
                if (avgTagDist < 1.0) {
                    score += 150; // Excellent - very close
                } else if (avgTagDist < 2.0) {
                    score += 100; // Good
                } else if (avgTagDist < 3.0) {
                    score += 50;  // Moderate
                } else if (avgTagDist < 4.0) {
                    score += 25;  // Far
                }
                // Beyond 4m = minimal score addition
            }
            
            // Area factor: larger tags = higher score (larger = closer/better resolution)
            if (avgTagArea > 0.5) {
                score += 100; // Excellent - very large tags
            } else if (avgTagArea > 0.25) {
                score += 75;  // Good
            } else if (avgTagArea > 0.15) {
                score += 50;  // Moderate
            } else if (avgTagArea > 0.08) {
                score += 25;  // Small
            }
            // Below 0.08 = minimal score
            
            // Tag span factor: wider geometry = better triangulation
            if (tagSpan > 1.5) {
                score += 100; // Excellent spread
            } else if (tagSpan > 1.0) {
                score += 75;  // Good spread
            } else if (tagSpan > 0.5) {
                score += 50;  // Moderate spread
            } else if (tagSpan > 0.3) {
                score += 25;  // Small spread
            }
            // Below 0.3 = minimal
            
            // Pose ambiguity factor: lower ambiguity = higher score (only for single tag)
            if (tagCount == 1 && poseAmbiguity >= 0) {
                if (poseAmbiguity < 0.05) {
                    score += 50;  // Very low ambiguity
                } else if (poseAmbiguity < 0.1) {
                    score += 25;  // Low ambiguity
                } else if (poseAmbiguity < 0.2) {
                    score += 10;  // Moderate ambiguity
                }
                // Above 0.2 = minimal (already filtered out in quality check)
            }
            
            // Latency factor: lower latency = higher score
            // Lower latency means the measurement is more recent
            if (latencySeconds >= 0) {
                if (latencySeconds < 0.05) {
                    score += 50;  // Very recent
                } else if (latencySeconds < 0.1) {
                    score += 25;  // Recent
                } else if (latencySeconds < 0.15) {
                    score += 10;  // Somewhat stale
                }
                // Above 150ms = minimal
            }
            
            return score;
        }
        
        /**
         * Calculate standard deviations based on measurement quality
         * Lower std devs = more trust in this measurement
         */
        public Matrix<N3, N1> getStandardDeviations() {
            double xStdDev = 0.1;
            double yStdDev = 0.1;
            double thetaStdDev = 0.2;
            
            // Base std devs based on tag count
            if (tagCount >= 4) {
                xStdDev = 0.005;
                yStdDev = 0.005;
                thetaStdDev = 0.01;
            } else if (tagCount >= 3) {
                xStdDev = 0.008;
                yStdDev = 0.008;
                thetaStdDev = 0.015;
            } else if (tagCount >= 2) {
                xStdDev = 0.01;
                yStdDev = 0.01;
                thetaStdDev = 0.02;
            } else {
                xStdDev = 0.05;
                yStdDev = 0.05;
                thetaStdDev = 0.1;
            }
            
            // Increase std devs for longer distances (less accurate)
            if (avgTagDist > 2.0) {
                double distanceFactor = 1.0 + (avgTagDist - 2.0) * 0.3;
                xStdDev *= distanceFactor;
                yStdDev *= distanceFactor;
                thetaStdDev *= distanceFactor;
            }
            
            // Increase std devs for smaller tags (less accurate)
            if (avgTagArea < 0.15) {
                double areaFactor = 1.0 + (0.15 - avgTagArea) * 3.0;
                xStdDev *= areaFactor;
                yStdDev *= areaFactor;
                thetaStdDev *= areaFactor;
            }
            
            // Increase rotation uncertainty for poor geometry
            if (tagCount == 1 || tagSpan < 0.5) {
                thetaStdDev *= 2.0;
            }
            
            // Increase all std devs for high latency (stale measurement)
            if (latencySeconds > 0.1) {
                double latencyFactor = 1.0 + (latencySeconds - 0.1) * 2.0;
                xStdDev *= latencyFactor;
                yStdDev *= latencyFactor;
                thetaStdDev *= latencyFactor;
            }
            
            return VecBuilder.fill(xStdDev, yStdDev, thetaStdDev);
        }
    }
    
    // Performance tuning constants
    private static final int TELEMETRY_UPDATE_PERIOD = 25; // Update telemetry every 25 cycles (500ms)
    private static final int POSE_PUBLISH_PERIOD = 5; // Publish poses every 5 cycles (100ms) - more frequent for visualization
    
    private final CommandSwerveDrivetrain drivetrain;
    private final String frontLimelightName;
    private final String backLimelightName;
    
    private boolean useVision = true;
    
    // Telemetry throttling with staggered offsets
    private int telemetryCounter = 15; // Staggered offset
    private int posePublishCounter = 0;
    
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
        // Only process measurements if vision is enabled
        if (!useVision) {
            return;
        }
        
        // Process front Limelight
        processLimelightMeasurement(frontLimelightName);
        
        // Process back Limelight
        processLimelightMeasurement(backLimelightName);
        
        // Throttle telemetry updates for performance (200ms)
        telemetryCounter++;
        if (telemetryCounter >= TELEMETRY_UPDATE_PERIOD) {
            telemetryCounter = 0;
            updateTelemetry();
        }
        
        // Throttle pose publishing (40ms) - less critical than measurements
        posePublishCounter++;
        if (posePublishCounter >= POSE_PUBLISH_PERIOD) {
            posePublishCounter = 0;
            publishVisionPoses();
        }
    }
    
    /**
     * Process vision measurements from a single Limelight
     * SIMPLIFIED FILTERING: Only reject if no tags visible
     * All quality filtering and pose jump filtering removed for maximum trust in MegaTag1
     */
    private void processLimelightMeasurement(String limelightName) {
        // Check if we have a valid target
        if (!LimelightHelpers.hasTarget(limelightName)) {
            return;
        }
        
        // Get the pose estimate 
        LimelightHelpers.PoseEstimate poseEstimate = getLimelightPoseEstimateForAlliance(limelightName);
        
        // Only reject if no tags visible
        if (poseEstimate.tagCount == 0) {
            return;
        }
        
        // Calculate dynamic standard deviations based on measurement quality
        Matrix<N3, N1> stdDevs = calculateDynamicStdDevs(poseEstimate);
        
        // Add vision measurement to drivetrain's pose estimator
        drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, stdDevs);
    }
    
    /**
     * Gets the robot pose estimate from Limelight in WPILib blue-origin coordinates
     * 
     * MEGATAG MODE: MegaTag1 ONLY for Maximum Stability
     * - Both Front and Back Limelights: Use MegaTag1 with ORB feature tracking
     * - MegaTag1 provides more stable pose estimates by tracking features between frames
     * - This reduces jitter and provides smoother pose updates
     * - MegaTag2 is NOT used to avoid potential instability
     * 
     * NOTE: ALWAYS use wpiBlue regardless of alliance color!
     * WPILib's coordinate system ALWAYS uses blue-origin (0,0 at blue alliance station)
     * The alliance color does NOT change the field coordinate system.
     * 
     * NOTE: No rotation correction needed - botpose_wpiblue already returns correct orientation
     */
    private LimelightHelpers.PoseEstimate getLimelightPoseEstimateForAlliance(String limelightName) {
        // Use MegaTag1 for BOTH cameras for maximum stability
        // MegaTag1 uses ORB feature tracking which provides smoother estimates
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag1(limelightName);
    }
    
    /**
     * Quality filter to reject bad vision measurements
     * Uses the quality thresholds defined in VisionConstants
     */
    private boolean passesQualityFilter(String limelightName, LimelightHelpers.PoseEstimate estimate) {
        // Check if pose is valid (not null and not at origin)
        if (estimate.pose == null) {
            return false;
        }
        
        // Check tag count
        if (estimate.tagCount < 1) {
            return false;
        }
        
        // Check distance - reject measurements beyond max range
        if (estimate.avgTagDist > VisionConstants.MAX_VISION_DISTANCE) {
            return false;
        }
        
        // Check tag area - reject if tags are too small (too far or poor view)
        if (estimate.avgTagArea < VisionConstants.MIN_TAG_AREA) {
            return false;
        }
        
        // Check ambiguity for single-tag poses (not applicable for multi-tag)
        if (estimate.tagCount == 1) {
            // Get pose ambiguity from Limelight
            double ambiguity = LimelightHelpers.getPoseAmbiguity(limelightName);
            if (ambiguity > VisionConstants.MAX_AMBIGUITY) {
                return false;
            }
        }
        
        // For multi-tag, check tag span (geometry quality)
        if (estimate.tagCount >= 2 && estimate.tagSpan < VisionConstants.MIN_TAG_SPAN_MULTI) {
            return false;
        }
        
        return true;
    }
    
    /**
     * Pose jump filter to reject measurements that are too far from current pose
     * This prevents single bad frames from causing large pose jumps
     */
    private boolean passesPoseJumpFilter(Pose2d visionPose) {
        Pose2d currentPose = drivetrain.getState().Pose;
        
        // Calculate position difference
        double distanceDiff = visionPose.getTranslation().getDistance(currentPose.getTranslation());
        
        // Calculate rotation difference (normalize to -180 to 180)
        double rotationDiff = Math.abs(
            visionPose.getRotation().minus(currentPose.getRotation()).getDegrees()
        );
        if (rotationDiff > 180) {
            rotationDiff = 360 - rotationDiff;
        }
        double rotationDiffRad = Math.toRadians(rotationDiff);
        
        // Reject if position jump is too large
        if (distanceDiff > VisionConstants.MAX_POSE_JUMP_DISTANCE) {
            return false;
        }
        
        // Reject if rotation jump is too large
        if (rotationDiffRad > VisionConstants.MAX_POSE_JUMP_ROTATION) {
            return false;
        }
        
        return true;
    }
    
    /**
     * Calculate dynamic standard deviations based on measurement quality
     * Adapts trust level based on tag count, distance, and area
     */
    private Matrix<N3, N1> calculateDynamicStdDevs(LimelightHelpers.PoseEstimate estimate) {
        // Base std devs for single tag
        double xStdDev = VisionConstants.SINGLE_TAG_STD_DEVS.get(0, 0);
        double yStdDev = VisionConstants.SINGLE_TAG_STD_DEVS.get(1, 0);
        double thetaStdDev = VisionConstants.SINGLE_TAG_STD_DEVS.get(2, 0);
        
        // Multi-tag is more reliable
        if (estimate.tagCount >= 2) {
            xStdDev = VisionConstants.MULTI_TAG_STD_DEVS.get(0, 0);
            yStdDev = VisionConstants.MULTI_TAG_STD_DEVS.get(1, 0);
            thetaStdDev = VisionConstants.MULTI_TAG_STD_DEVS.get(2, 0);
        }
        
        // Increase std devs for longer distances (less accurate)
        if (estimate.avgTagDist > 2.0) {
            double distanceFactor = estimate.avgTagDist / 2.0;
            xStdDev *= distanceFactor;
            yStdDev *= distanceFactor;
            thetaStdDev *= distanceFactor;
        }
        
        // Increase std devs for smaller tags (less accurate)
        if (estimate.avgTagArea < 0.15) {
            double areaFactor = 0.15 / Math.max(estimate.avgTagArea, 0.01);
            xStdDev *= areaFactor;
            yStdDev *= areaFactor;
            thetaStdDev *= areaFactor;
        }
        
        // For single tag with poor geometry, increase rotation uncertainty
        if (estimate.tagCount == 1) {
            thetaStdDev *= 2.0;
        }
        
        return VecBuilder.fill(xStdDev, yStdDev, thetaStdDev);
    }
    
    /**
     * Updates telemetry for debugging - SIMPLIFIED
     */
    private void updateTelemetry() {
        // Get full PoseEstimate data for both cameras
        LimelightHelpers.PoseEstimate frontEstimate = getLimelightPoseEstimateForAlliance(frontLimelightName);
        LimelightHelpers.PoseEstimate backEstimate = getLimelightPoseEstimateForAlliance(backLimelightName);
        
        // Front Limelight
        SmartDashboard.putBoolean("Vision/Front/HasTarget", LimelightHelpers.hasTarget(frontLimelightName));
        SmartDashboard.putNumber("Vision/Front/NumTags", frontEstimate.tagCount);
        
        // Back Limelight
        SmartDashboard.putBoolean("Vision/Back/HasTarget", LimelightHelpers.hasTarget(backLimelightName));
        SmartDashboard.putNumber("Vision/Back/NumTags", backEstimate.tagCount);
        
        // Overall Status
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
        
        // Update Field2d widget with the PRIMARY pose (vision when available, fused when not)
        // This uses the new getPose() method which returns vision pose when available
        Pose2d robotPose = drivetrain.getPose();
        visionField.setRobotPose(robotPose);
        
        // Show raw vision estimates as objects for comparison (smaller to distinguish from main pose)
        if (!allPoses.isEmpty()) {
            visionField.getObject("VisionEstimate1").setPose(allPoses.get(0));
            if (allPoses.size() > 1) {
                visionField.getObject("VisionEstimate2").setPose(allPoses.get(1));
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
    
    /**
     * Get the Field2d widget for adding additional objects
     * @return The vision field widget
     */
    public Field2d getField2d() {
        return visionField;
    }
    
    /**
     * Get the best available vision pose for initialization (when disabled) - SIMPLIFIED
     * @return The best vision pose, or null if no valid pose is available
     */
    /**
     * Get the best vision pose for initialization
     * Simply returns the pose from the camera with the most tags visible
     * NO FILTERING - used for initial pose setup
     * @return The vision pose, or null if no cameras see tags
     */
    public Pose2d getBestVisionPose() {
        // Get estimates from both cameras
        LimelightHelpers.PoseEstimate frontEstimate = getLimelightPoseEstimateForAlliance(frontLimelightName);
        LimelightHelpers.PoseEstimate backEstimate = getLimelightPoseEstimateForAlliance(backLimelightName);
        
        // Check which cameras see tags
        boolean frontHasTags = frontEstimate.tagCount > 0;
        boolean backHasTags = backEstimate.tagCount > 0;
        
        // If neither camera sees tags, return null
        if (!frontHasTags && !backHasTags) {
            return null;
        }
        
        // Return the pose from the camera with more tags (more reliable)
        if (frontHasTags && backHasTags) {
            // Both see tags - use the one with more tags
            if (frontEstimate.tagCount >= backEstimate.tagCount) {
                System.out.println("VISION: Using FRONT camera for initialization | Tags: " + frontEstimate.tagCount + 
                    " | Pose: " + String.format("(%.2f, %.2f, %.1f°)", 
                        frontEstimate.pose.getX(), 
                        frontEstimate.pose.getY(), 
                        frontEstimate.pose.getRotation().getDegrees()));
                return frontEstimate.pose;
            } else {
                System.out.println("VISION: Using BACK camera for initialization | Tags: " + backEstimate.tagCount + 
                    " | Pose: " + String.format("(%.2f, %.2f, %.1f°)", 
                        backEstimate.pose.getX(), 
                        backEstimate.pose.getY(), 
                        backEstimate.pose.getRotation().getDegrees()));
                return backEstimate.pose;
            }
        } else if (frontHasTags) {
            System.out.println("VISION: Using FRONT camera for initialization | Tags: " + frontEstimate.tagCount + 
                " | Pose: " + String.format("(%.2f, %.2f, %.1f°)", 
                    frontEstimate.pose.getX(), 
                    frontEstimate.pose.getY(), 
                    frontEstimate.pose.getRotation().getDegrees()));
            return frontEstimate.pose;
        } else {
            System.out.println("VISION: Using BACK camera for initialization | Tags: " + backEstimate.tagCount + 
                " | Pose: " + String.format("(%.2f, %.2f, %.1f°)", 
                    backEstimate.pose.getX(), 
                    backEstimate.pose.getY(), 
                    backEstimate.pose.getRotation().getDegrees()));
            return backEstimate.pose;
        }
    }
    
    /**
     * Check if a valid vision measurement is available from any camera
     * @return true if at least one camera has a valid target
     */
    public boolean hasValidVisionMeasurement() {
        return getMostTrustedVisionMeasurement() != null;
    }
    
    /**
     * Get the most trusted vision measurement from all available cameras
     * Uses quality metrics (tag count, distance, area, tag span) to determine trust score
     * @return The most trusted vision measurement, or null if no valid measurement available
     */
    public VisionMeasurement getMostTrustedVisionMeasurement() {
        // Get estimates from both cameras
        LimelightHelpers.PoseEstimate frontEstimate = getLimelightPoseEstimateForAlliance(frontLimelightName);
        LimelightHelpers.PoseEstimate backEstimate = getLimelightPoseEstimateForAlliance(backLimelightName);
        
        // Check which cameras have valid targets and pass quality filters
        boolean frontValid = passesQualityFilter(frontLimelightName, frontEstimate) 
            && passesPoseJumpFilter(frontEstimate.pose);
        boolean backValid = passesQualityFilter(backLimelightName, backEstimate) 
            && passesPoseJumpFilter(backEstimate.pose);
        
        // If neither camera has valid target, return null
        if (!frontValid && !backValid) {
            return null;
        }
        
        // Calculate latency (time since capture)
        double frontLatency = frontValid ? 
            (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - frontEstimate.timestampSeconds) : 0;
        double backLatency = backValid ?
            (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - backEstimate.timestampSeconds) : 0;
        
        // Get pose ambiguity for each camera
        double frontAmbiguity = frontValid ? LimelightHelpers.getPoseAmbiguity(frontLimelightName) : 0;
        double backAmbiguity = backValid ? LimelightHelpers.getPoseAmbiguity(backLimelightName) : 0;
        
        // Create VisionMeasurement objects for valid cameras
        VisionMeasurement frontMeasurement = frontValid ? new VisionMeasurement(
            frontEstimate.pose,
            frontEstimate.tagCount,
            frontEstimate.avgTagDist,
            frontEstimate.avgTagArea,
            frontEstimate.tagSpan,
            frontEstimate.timestampSeconds,
            frontLimelightName,
            frontLatency,
            frontAmbiguity
        ) : null;
        
        VisionMeasurement backMeasurement = backValid ? new VisionMeasurement(
            backEstimate.pose,
            backEstimate.tagCount,
            backEstimate.avgTagDist,
            backEstimate.avgTagArea,
            backEstimate.tagSpan,
            backEstimate.timestampSeconds,
            backLimelightName,
            backLatency,
            backAmbiguity
        ) : null;
        
        // If only one camera has valid measurement, return it
        if (frontValid && !backValid) {
            return frontMeasurement;
        }
        if (backValid && !frontValid) {
            return backMeasurement;
        }
        
        // Both cameras have valid measurements - compare trust scores
        double frontScore = frontMeasurement.getTrustScore();
        double backScore = backMeasurement.getTrustScore();
        
        return frontScore >= backScore ? frontMeasurement : backMeasurement;
    }
}
