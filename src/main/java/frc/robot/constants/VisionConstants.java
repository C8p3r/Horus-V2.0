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
    // Single tag: higher uncertainty [x, y, rotation] in meters and radians
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
    
    // Multiple tags: lower uncertainty [x, y, rotation] in meters and radians
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);
    
    // Maximum Distance
    public static final double MAX_VISION_DISTANCE = 5.0; // meters
}
