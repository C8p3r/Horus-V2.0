package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Constants for the Vision subsystem (Limelight)
 */
public final class VisionConstants {
    
    // Network Table Names
    public static final String LIMELIGHT_FRONT_NAME = "limelight-tuhbron";
    public static final String LIMELIGHT_BACK_NAME = "limelight-jhnbron";
    
    // Camera Transforms (robot center to camera)
    // Front Limelight: 0.3m forward, 0m left/right, 0.25m up, tilted 30° down
    public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(
        new Translation3d(0.3, 0.0, 0.25),
        new Rotation3d(0, Math.toRadians(-30), 0)
    );
    
    // Back Limelight: 0.3m back, 0m left/right, 0.25m up, tilted 30° down, rotated 180°
    public static final Transform3d ROBOT_TO_BACK_CAMERA = new Transform3d(
        new Translation3d(-0.3, 0.0, 0.25),
        new Rotation3d(0, Math.toRadians(-30), Math.toRadians(180))
    );
    
    // Standard Deviations for Vision Measurements (MegaTag2)
    // These control how much the pose estimator trusts vision vs wheel odometry
    // Lower values = more trust in vision, Higher values = less trust in vision
    // 
    // TUNED FOR VISION-PRIMARY POSE ESTIMATION:
    // Vision measurements are now heavily trusted to correct wheel odometry drift
    
    // Single tag: Very low uncertainty - trust single tag measurements heavily
    // [x, y, rotation] in meters and radians
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.05, 0.05, 0.1);
    
    // Multiple tags: Extremely low uncertainty - trust multi-tag completely
    // [x, y, rotation] in meters and radians  
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.01, 0.01, 0.02);
    
    // Quality thresholds for filtering bad measurements
    public static final double MAX_VISION_DISTANCE = 4.5; // meters - reject measurements beyond this
    public static final double MIN_TAG_AREA = 0.05; // % - reject if tags too small (too far/poor view)
    public static final double MIN_TAG_SPAN_MULTI = 0.4; // meters - reject multi-tag if geometry is poor
    public static final double MAX_AMBIGUITY = 0.3; // reject if pose ambiguity is too high (0-1 scale)
    public static final int MIN_TAGS_FOR_ROTATION = 2; // need at least 2 tags to trust rotation
    
    // Pose jump filtering - reject measurements that are too far from current pose
    public static final double MAX_POSE_JUMP_DISTANCE = 1.5; // meters - max allowed position jump
    public static final double MAX_POSE_JUMP_ROTATION = Math.toRadians(30); // radians - max allowed rotation jump
}

