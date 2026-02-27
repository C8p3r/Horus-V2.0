package frc.robot.constants;

/**
 * Constants for the Climber subsystem
 */
public final class ClimberConstants {
    
    // CAN Configuration
    public static final String CANBUS_NAME = "Drivetrain";
    public static final int CLIMB_MOTOR_ID = 62;
    
    // PWM Configuration
    public static final int RATCHET_SERVO_PWM = 0;
    
    // Motor Configuration
    public static final boolean MOTOR_INVERTED = false;
    
    // Gear Ratio
    public static final double GEAR_RATIO = 100.0; // Motor rotations per mechanism rotation (100:1 reduction)
    
    // Position Control (Motion Magic)
    public static final double MAX_VELOCITY_RPS = 10.0; // Motor RPS
    public static final double MAX_ACCELERATION_RPSS = 40.0;
    public static final double MAX_JERK_RPSSS = 400.0;
    
    // Deploy Positions (in motor rotations)
    public static final double RETRACTED_POSITION_ROTATIONS = 0.0;
    public static final double DEPLOYED_POSITION_ROTATIONS = 300.0; // 300 motor rotations to fully retract
    
    // Servo Positions
    public static final double RATCHET_DISABLED_POSITION = 0.0; // Initial position
    public static final double RATCHET_ENABLED_POSITION = 1.0;  // Locked position
    
    // Soft Limits
    public static final double MIN_SOFT_LIMIT = -10.0;
    public static final double MAX_SOFT_LIMIT = 310.0;
    
    // Position Tolerance
    public static final double POSITION_TOLERANCE_ROTATIONS = 5.0;
    
    // Current Limits (prevent brownouts)
    public static final double SUPPLY_CURRENT_LIMIT = 40.0; // Amps
    public static final double STATOR_CURRENT_LIMIT = 80.0; // Amps
    public static final boolean ENABLE_CURRENT_LIMIT = true;
}
