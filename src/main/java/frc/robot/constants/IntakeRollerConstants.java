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
    
    // Duty Cycle Constants (-1.0 to 1.0, positive = intake)
    public static final double DEFAULT_DUTY_CYCLE = 0.5;   // 50% power when deployed (was 20 RPS / 40 = 0.5)
    public static final double INTAKE_DUTY_CYCLE = 0.75;   // 75% power during active intake (was 30 RPS / 40 = 0.75)
    public static final double EJECT_DUTY_CYCLE = -1.0;    // 100% reverse to eject (was -40 RPS / 40 = -1.0)
    
    // Legacy Speed Constants (RPS - Rotations Per Second) - DEPRECATED, use DUTY_CYCLE constants instead
    @Deprecated public static final double DEFAULT_SPEED_RPS = 20.0;
    @Deprecated public static final double INTAKE_SPEED_RPS = 30.0;
    @Deprecated public static final double EJECT_SPEED_RPS = -40.0;
    
    // Position-based activation tolerance
    // Roller only runs when intake position is within this tolerance of deployed position
    public static final double DEPLOYED_TOLERANCE_ROTATIONS = 0.05; // ±0.05 rotations
    
    // Velocity Control (Motion Magic) - Legacy, now using duty cycle
    public static final double MAX_VELOCITY_RPS = 40.0;  // Maximum theoretical RPS at full power
    public static final double MAX_ACCELERATION_RPSS = 400.0;
    public static final double MAX_JERK_RPSSS = 4000.0;
    
    // Velocity Tolerance
    public static final double VELOCITY_TOLERANCE_RPS = 5.0;
    
    // Current Limits (prevent brownouts)
    public static final double SUPPLY_CURRENT_LIMIT = 25.0; // Amps
    public static final double STATOR_CURRENT_LIMIT = 40.0; // Amps
    public static final boolean ENABLE_CURRENT_LIMIT = true;
}
