package frc.robot.constants;

/**
 * Constants for the Intake Winch subsystem
 * The winch retracts the intake after match start using a Kraken X60 motor
 */
public final class IntakeWinchConstants {
    
    // CAN Configuration
    public static final String CANBUS_NAME = "Drivetrain";
    public static final int MOTOR_ID = 24;
    
    // Mechanical Configuration
    public static final double GEAR_RATIO = 80.0; // 80:1 reduction
    
    // Position Limits (in rotations after gear ratio)
    public static final double EXTENDED_POSITION_ROTATIONS = 0.0; // Intake deployed
    public static final double RETRACTED_POSITION_ROTATIONS = 10.0; // Intake retracted (adjust based on mechanism)
    
    // Position Tolerance
    public static final double POSITION_TOLERANCE_ROTATIONS = 0.5;
    
    // Soft Limits
    public static final double SOFT_LIMIT_MARGIN_ROTATIONS = 0.2;
    public static final double MIN_SOFT_LIMIT = EXTENDED_POSITION_ROTATIONS + SOFT_LIMIT_MARGIN_ROTATIONS;
    public static final double MAX_SOFT_LIMIT = RETRACTED_POSITION_ROTATIONS - SOFT_LIMIT_MARGIN_ROTATIONS;
    
    // Hard Limits
    public static final boolean ENABLE_FORWARD_HARD_LIMIT = true;
    public static final boolean ENABLE_REVERSE_HARD_LIMIT = true;
    
    // PID Constants (position control)
    public static final double kP = 30.0;
    public static final double kI = 0.0;
    public static final double kD = 0.5;
    
    // Feedforward Constants
    public static final double kS = 0.2; // Static friction (Volts)
    public static final double kV = 0.12; // Velocity feedforward (Volts per RPS)
    public static final double kA = 0.01; // Acceleration feedforward (Volts per RPS²)
    
    // Motion Constraints
    public static final double MAX_VELOCITY_RPS = 2.0; // Moderate speed
    public static final double MAX_ACCELERATION_RPSS = 8.0;
    public static final double MAX_JERK_RPSSS = 40.0;
    
    // Current Limits
    public static final double SUPPLY_CURRENT_LIMIT = 40.0; // Amps
    public static final double STATOR_CURRENT_LIMIT = 60.0; // Amps
    public static final boolean ENABLE_CURRENT_LIMIT = true;
    
    // Motor Configuration
    public static final boolean MOTOR_INVERTED = false;
}
