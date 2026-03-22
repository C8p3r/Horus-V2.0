package frc.robot.constants;

/**
 * Constants for the Hood - NOW FIXED AT 14 DEGREES (hardware removed)
 * The hood is now a fixed mechanical angle, no motor control
 */
public final class HoodConstants {
    
    // FIXED HOOD ANGLE - Hardware is now fixed at 14 degrees
    public static final double FIXED_HOOD_ANGLE_DEGREES = 14.0;
    
    // Legacy constants kept for compatibility (not used with fixed hood)
    public static final double MIN_ANGLE_DEGREES = 14.0;  
    public static final double MAX_ANGLE_DEGREES = 14.0; 
    public static final double ANGLE_TOLERANCE_DEGREES = 0.25;  
}

