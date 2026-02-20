package frc.robot.constants;

/**
 * Constants for the Intake Roller subsystem
 */
public final class IntakeRollerConstants {
    
    // CAN Configuration
    public static final String CANBUS_NAME = "Drivetrain";
    public static final int MOTOR_ID = 21;
    
    // Motor Configuration
    public static final boolean MOTOR_INVERTED = true;
    
    // Velocity Control (Motion Magic)
    public static final double MAX_VELOCITY_RPS = 80.0;
    public static final double MAX_ACCELERATION_RPSS = 400.0;
    public static final double MAX_JERK_RPSSS = 4000.0;
    
    // Velocity Tolerance
    public static final double VELOCITY_TOLERANCE_RPS = 5.0;
    
    // Current Limits (prevent brownouts)
    public static final double SUPPLY_CURRENT_LIMIT = 25.0; // Amps
    public static final double STATOR_CURRENT_LIMIT = 40.0; // Amps
    public static final boolean ENABLE_CURRENT_LIMIT = true;
}
