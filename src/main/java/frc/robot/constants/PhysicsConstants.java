package frc.robot.constants;

/**
 * Physics and projectile constants for shooter calculations
 */
public final class PhysicsConstants {
    
    // Projectile Properties
    public static final double PROJECTILE_MASS_KG = 0.235; // Note/game piece mass
    public static final double PROJECTILE_DIAMETER_METERS = 0.1524; // 6 inches
    public static final double PROJECTILE_CROSS_SECTION = Math.PI * Math.pow(PROJECTILE_DIAMETER_METERS / 2.0, 2);
    
    // Physics Constants
    public static final double GRAVITY = 9.81; // m/s²
    public static final double AIR_DENSITY = 1.225; // kg/m³ at sea level
    public static final double DRAG_COEFFICIENT = 0.47; // Sphere approximation
    
    // Shooting Performance
    public static final double MIN_SHOT_VELOCITY_MPS = 5.0; // Minimum shot velocity (m/s)
    public static final double MAX_SHOT_VELOCITY_MPS = 25.0; // Maximum shot velocity (m/s)
    
    // Shooter Position Offset from Robot Center (in meters)
    public static final double SHOOTER_OFFSET_X = 0.0; // At robot center
    public static final double SHOOTER_OFFSET_Y = 0.0; // Centered
    public static final double SHOOTER_OFFSET_Z = 0.508; // 20 inches above ground
    
    // ==================== SIMULATION VARIANCE ====================
    // Add realistic variance to simulated fuel launches
    
    public static final double SIM_VELOCITY_VARIANCE_PERCENT = 2.0; // ±2% velocity variance
    public static final double SIM_ANGLE_VARIANCE_DEGREES = 0.5; // ±0.5° launch angle variance
    public static final double SIM_DIRECTION_VARIANCE_DEGREES = 0.3; // ±0.3° horizontal direction variance
    public static final double SIM_POSITION_VARIANCE_METERS = 0.01; // ±1cm spawn position variance
}
