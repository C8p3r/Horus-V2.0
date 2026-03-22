package frc.robot.constants;

/**
 * Constants for the Intake Roller subsystem
 * NOW USES DUAL KRAKEN X60 MOTORS (upper and lower, opposed configuration)
 */
public final class IntakeRollerConstants {
    
    // CAN Configuration - DUAL X60s
    public static final String CANBUS_NAME = "Drivetrain";
    public static final int UPPER_MOTOR_ID = 21; // Upper roller
    public static final int LOWER_MOTOR_ID = 62; // Lower roller
    
    // Motor Configuration (opposed motors)
    public static final boolean UPPER_MOTOR_INVERTED = true;
    public static final boolean LOWER_MOTOR_INVERTED = false; // Opposed to upper
    
    // Duty Cycle Constants (-1.0 to 1.0, positive = intake)
    public static final double DEFAULT_DUTY_CYCLE = 0.5;   // 50% power when deployed
    public static final double INTAKE_DUTY_CYCLE = 0.75;   // 75% power during active intake
    public static final double EJECT_DUTY_CYCLE = -0.75;    // 75% reverse to eject
    
    // Position-based activation tolerance
    // Roller only runs when intake position is within this tolerance of deployed position
    public static final double DEPLOYED_TOLERANCE_ROTATIONS = 0.05; // ±0.05 rotations
    
    // Velocity Tolerance
    public static final double VELOCITY_TOLERANCE_RPS = 5.0;
    
    // Current Limits (prevent brownouts) - X60 motors
    public static final double SUPPLY_CURRENT_LIMIT = 40.0; // Amps (higher for X60s)
    public static final double STATOR_CURRENT_LIMIT = 60.0; // Amps
    public static final boolean ENABLE_CURRENT_LIMIT = true;
}
