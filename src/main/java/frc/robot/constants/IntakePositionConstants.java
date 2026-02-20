package frc.robot.constants;

/**
 * Constants for the Intake Position subsystem
 */
public final class IntakePositionConstants {
    
    // CAN Configuration
    public static final String CANBUS_NAME = "Drivetrain";
    public static final int MOTOR_ID = 24;
    
    // Motor Configuration
    public static final boolean MOTOR_INVERTED = false;
    
    // Gear Ratio
    public static final double GEAR_RATIO = 50.0; // Motor rotations per mechanism rotation
    
    // Position Control (Motion Magic)
    public static final double MAX_VELOCITY_RPS = 2.0; // Mechanism RPS
    public static final double MAX_ACCELERATION_RPSS = 10.0;
    public static final double MAX_JERK_RPSSS = 100.0;
    
    // Deploy Positions (in mechanism rotations)
    public static final double STOWED_POSITION_ROTATIONS = 0.0;
    public static final double EXTENDED_POSITION_ROTATIONS = 0.25; // 90 degrees
    
    // Soft Limits
    public static final double MIN_SOFT_LIMIT = -0.05;
    public static final double MAX_SOFT_LIMIT = 0.3;
    public static final double SOFT_LIMIT_MARGIN = 0.01;
    
    // Hard Limits
    public static final boolean ENABLE_FORWARD_HARD_LIMIT = true;
    public static final boolean ENABLE_REVERSE_HARD_LIMIT = true;
    
    // Position Tolerance
    public static final double POSITION_TOLERANCE_ROTATIONS = 0.01;
}
