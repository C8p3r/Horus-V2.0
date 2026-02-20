package frc.robot.constants;

/**
 * Constants for the Hood subsystem
 */
public final class HoodConstants {
    
    // CAN Configuration
    public static final String CANBUS_NAME = "Drivetrain";
    public static final int MOTOR_ID = 16;
    
    // Mechanical Configuration
    public static final double GEAR_RATIO = 42.0/18.0; // 18:42 ratio - 2.333 motor rotations per hood rotation
    
    // Angle Limits (in degrees)
    public static final double MIN_ANGLE_DEGREES = 14.0;
    public static final double MAX_ANGLE_DEGREES = 40.0;
    public static final double ANGLE_TOLERANCE_DEGREES = 0.5;
    
    // Motor Offset (calibration adjustment in degrees)
    public static final double MOTOR_OFFSET_DEGREES = 0.11328125;
    
    // Soft Limits (in rotations after gear ratio)
    public static final double SOFT_LIMIT_MARGIN_DEGREES = 0.5;
    public static final double MIN_SOFT_LIMIT = (MIN_ANGLE_DEGREES + SOFT_LIMIT_MARGIN_DEGREES) / 360.0;
    public static final double MAX_SOFT_LIMIT = (MAX_ANGLE_DEGREES - SOFT_LIMIT_MARGIN_DEGREES) / 360.0;
    
    // Hard Limits
    public static final boolean ENABLE_FORWARD_HARD_LIMIT = true;
    public static final boolean ENABLE_REVERSE_HARD_LIMIT = true;
    
    // PID Constants (position control)
    public static final double kP = 50.0;
    public static final double kI = 0.0;
    public static final double kD = 1.0;
    
    // Feedforward Constants
    public static final double kS = 0.1; // Static friction (Volts)
    public static final double kV = 0.12; // Velocity feedforward (Volts per RPS)
    public static final double kA = 0.01; // Acceleration feedforward (Volts per RPS²)
    
    // Motion Constraints
    public static final double MAX_VELOCITY_RPS = 0.5; // Slow and controlled
    public static final double MAX_ACCELERATION_RPS2 = 2.0;
    
    // Current Limits
    public static final double SUPPLY_CURRENT_LIMIT = 20.0; // Amps
    public static final double STATOR_CURRENT_LIMIT = 40.0; // Amps
    public static final boolean ENABLE_CURRENT_LIMIT = true;
    
    // Motor Configuration
    public static final boolean MOTOR_INVERTED = true;
}
