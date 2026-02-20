package frc.robot.constants;

/**
 * Constants for the Intake subsystem
 */
public final class IntakeConstants {
    
    // CAN Configuration
    public static final String CANBUS_NAME = "Drivetrain";
    public static final int ROLLER_MOTOR_ID = 21;
    public static final int DEPLOY_MOTOR_ID = 24;
    
    // Roller Motor (velocity control)
    public static final double ROLLER_MAX_VELOCITY_RPS = 80.0;
    public static final double ROLLER_MAX_ACCELERATION_RPSS = 400.0;
    public static final double ROLLER_MAX_JERK_RPSSS = 4000.0;
    
    // Deploy Motor (position control)
    public static final double DEPLOY_GEAR_RATIO = 50.0; // Motor rotations per mechanism rotation
    public static final double DEPLOY_MAX_VELOCITY_RPS = 2.0; // Mechanism RPS
    public static final double DEPLOY_MAX_ACCELERATION_RPSS = 10.0;
    public static final double DEPLOY_MAX_JERK_RPSSS = 100.0;
    
    // Deploy Positions (in mechanism rotations)
    public static final double STOWED_POSITION_ROTATIONS = 0.0;
    public static final double EXTENDED_POSITION_ROTATIONS = 0.25; // 90 degrees
    
    // Deploy Soft Limits
    public static final double DEPLOY_MIN_SOFT_LIMIT = -0.05;
    public static final double DEPLOY_MAX_SOFT_LIMIT = 0.3;
    public static final double DEPLOY_SOFT_LIMIT_MARGIN = 0.01;
    
    // Deploy Hard Limits
    public static final boolean ENABLE_DEPLOY_FORWARD_HARD_LIMIT = true;
    public static final boolean ENABLE_DEPLOY_REVERSE_HARD_LIMIT = true;
    
    // Motor Configuration
    public static final boolean ROLLER_INVERTED = false;
    public static final boolean DEPLOY_INVERTED = false;
}
