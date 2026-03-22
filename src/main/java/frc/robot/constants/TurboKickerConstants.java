package frc.robot.constants;

/**
 * Constants for the TurboKicker subsystem
 * Three motors total:
 * - Left and Right X44 motors (CAN 15, 16) feed vertically into kicker
 * - Center kicker motor (CAN 18) kicks notes into shooter
 */
public final class TurboKickerConstants {
    
    // CAN Configuration
    public static final String CANBUS_NAME = "Drivetrain";
    public static final int LEFT_FEED_MOTOR_ID = 15;    // Left X44 vertical feeder
    public static final int RIGHT_FEED_MOTOR_ID = 16;   // Right X44 vertical feeder
    public static final int KICKER_MOTOR_ID = 18;       // Center kicker (original motor)
    
    // Motor Configuration
    public static final boolean LEFT_FEED_INVERTED = false;
    public static final boolean RIGHT_FEED_INVERTED = true;   // Opposed to left
    public static final boolean KICKER_INVERTED = true;       // Same as old fire indexer
    
    // Duty Cycle Control
    // Feed motors (left/right X44s)
    public static final double FEED_MOTORS_DUTY_CYCLE = 1.0;      // Full power when feeding
    public static final double FEED_MOTORS_REVERSE_DUTY_CYCLE = -0.5;  // Reverse for unjamming
    public static final double FEED_MOTORS_HOLD_DUTY_CYCLE = -0.2;     // Light reverse to hold note
    
    // Kicker motor (center motor that kicks into shooter)
    public static final double KICKER_FEED_DUTY_CYCLE = 1.0;      // Full power when shooting
    public static final double KICKER_REVERSE_DUTY_CYCLE = -0.33; // Reverse to clear jams
    public static final double KICKER_HOLD_DUTY_CYCLE = -0.25;    // Light reverse during spinup
    
    public static final double OFF_DUTY_CYCLE = 0.0;       // Motors off
    
    // Current Limits
    public static final double SUPPLY_CURRENT_LIMIT = 40.0; // Amps
    public static final double STATOR_CURRENT_LIMIT = 80.0; // Amps
    public static final boolean ENABLE_CURRENT_LIMIT = true;
}
