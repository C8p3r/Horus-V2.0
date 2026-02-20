package frc.robot.constants;

/**
 * Constants for the Indexer subsystem
 */
public final class IndexerConstants {
    
    // CAN Configuration
    public static final String CANBUS_NAME = "Drivetrain";
    public static final int FLOOR_MOTOR_ID = 20;
    public static final int FIRE_MOTOR_ID = 18;
    
    // Floor Indexer Motor (velocity control - picks up from intake)
    public static final double FLOOR_MAX_VELOCITY_RPS = 60.0;
    public static final double FLOOR_MAX_ACCELERATION_RPSS = 300.0;
    public static final double FLOOR_MAX_JERK_RPSSS = 3000.0;
    public static final double FLOOR_INTAKE_VELOCITY_RPS = 30.0; // Operating speed
    
    // Fire Indexer Motor (velocity control - feeds to shooter)
    public static final double FIRE_MAX_VELOCITY_RPS = 80.0;
    public static final double FIRE_MAX_ACCELERATION_RPSS = 400.0;
    public static final double FIRE_MAX_JERK_RPSSS = 4000.0;
    public static final double FIRE_FEED_VELOCITY_RPS = 60.0; // Operating speed
    
    // Motor Configuration
    public static final boolean FLOOR_INVERTED = false;
    public static final boolean FIRE_INVERTED = false;
}
