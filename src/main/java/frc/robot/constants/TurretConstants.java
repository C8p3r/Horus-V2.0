package frc.robot.constants;

/**
 * Constants for the Turret subsystem
 */
public final class TurretConstants {
    
    // CAN Configuration
    public static final String CANBUS_NAME = "Drivetrain";
    public static final int MOTOR_ID = 14;
    
    // Turret Position Offset from Robot Center (in meters)
    public static final double OFFSET_X = -0.2159; // 8.5" from back (negative = back)
    public static final double OFFSET_Y = 0.2159;  // 8.5" from left (positive = left)
    public static final double OFFSET_Z = 0.4572;  // 18" above ground
    
    // Initial Configuration
    public static final double INITIAL_ANGLE_DEGREES = 180.0; // Turret starts facing backward
    
    // Mechanical Configuration
    public static final double GEAR_RATIO = 10.0; // 10:1 reduction
    
    // Position Limits (turret can rotate from 50° to 360°)
    public static final double MIN_POSITION_ROTATIONS = 0.1389; // 50 degrees
    public static final double MAX_POSITION_ROTATIONS = 1.0; // 360 degrees
    
    // Soft Limits
    public static final double SOFT_LIMIT_MARGIN_ROTATIONS = 0.005; // ~1.8 degrees
    public static final double MIN_SOFT_LIMIT = MIN_POSITION_ROTATIONS + SOFT_LIMIT_MARGIN_ROTATIONS;
    public static final double MAX_SOFT_LIMIT = MAX_POSITION_ROTATIONS - SOFT_LIMIT_MARGIN_ROTATIONS;
    
    // Hard Limits
    public static final boolean ENABLE_FORWARD_HARD_LIMIT = true;
    public static final boolean ENABLE_REVERSE_HARD_LIMIT = true;
    
    // PID Constants (position control)
    public static final double kP = 28.0;
    public static final double kI = 0.0;
    public static final double kD = 0.5;
    
    // Feedforward Constants
    public static final double kS = 0.1; // Static friction (Volts)
    public static final double kV = 0.12; // Velocity feedforward (Volts per RPS)
    public static final double kA = 0.01; // Acceleration feedforward (Volts per RPS²)
    
    // Motion Constraints
    public static final double MAX_VELOCITY_RPS = 2.0;
    public static final double MAX_ACCELERATION_RPS2 = 8.0;
    
    // Tolerances
    public static final double POSITION_TOLERANCE_ROTATIONS = 0.005; // ~1.8 degrees
    public static final double VELOCITY_TOLERANCE_RPS = 0.1;
    
    // Current Limits
    public static final double SUPPLY_CURRENT_LIMIT = 30.0; // Amps
    public static final double STATOR_CURRENT_LIMIT = 60.0; // Amps
    public static final boolean ENABLE_CURRENT_LIMIT = true;
    
    // Motor Configuration
    public static final boolean MOTOR_INVERTED = false;
    public static final boolean BRAKE_MODE = true;
}
