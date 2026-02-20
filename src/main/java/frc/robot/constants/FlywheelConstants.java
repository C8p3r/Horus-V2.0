package frc.robot.constants;

/**
 * Constants for the Flywheel subsystem
 */
public final class FlywheelConstants {
    
    // CAN Configuration
    public static final String CANBUS_NAME = "Drivetrain";
    public static final int MOTOR_ID = 15;
    
    // Mechanical Configuration
    public static final double GEAR_RATIO = 18.0/15.0; // 15:18 ratio - Motor rotations per wheel rotation
    public static final double WHEEL_DIAMETER_METERS = 0.1016; // 4 inches
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    
    // Velocity Limits (in rotations per second of the wheel)
    public static final double MAX_VELOCITY_RPS = 120.0 * GEAR_RATIO; // ~7200 RPM adjusted for gear ratio
    public static final double VELOCITY_TOLERANCE_RPS = 2.0; // Within 2 RPS of target
    
    // PID Constants (velocity control)
    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    // Feedforward Constants
    public static final double kS = 0.2; // Static friction (Volts)
    public static final double kV = 0.12; // Velocity feedforward (Volts per RPS)
    public static final double kA = 0.01; // Acceleration feedforward (Volts per RPS²)
    
    // Motion Magic Parameters (smooth velocity ramping)
    public static final double MAX_ACCELERATION_RPSS = 300.0; // RPS per second
    public static final double MAX_JERK_RPSSS = 3000.0; // RPS per second per second
    
    // Current Limits
    public static final double SUPPLY_CURRENT_LIMIT = 60.0; // Amps
    public static final double STATOR_CURRENT_LIMIT = 120.0; // Amps
    public static final boolean ENABLE_CURRENT_LIMIT = true;
    
    // Motor Configuration
    public static final boolean MOTOR_INVERTED = false;
    
    // Physics Constants (for modeling)
    public static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.008; // kg*m²
    public static final double FLYWHEEL_MASS_KG = 0.5;
    public static final double ENERGY_TRANSFER_EFFICIENCY = 0.85; // 85%
    public static final double RECOVERY_TIME_SAFETY_FACTOR = 1.2; // 20% safety margin
}
