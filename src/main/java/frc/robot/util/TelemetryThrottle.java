package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * Utility class to throttle telemetry updates and prevent network I/O blocking.
 * 
 * The Driver Station Status loop can spike to 200-400ms when SmartDashboard/NetworkTables
 * calls block on network I/O. This class ensures updates happen at a controlled rate
 * (default: 5 Hz / 200ms) instead of every robot loop (50 Hz / 20ms).
 * 
 * Usage:
 * <pre>
 * private final TelemetryThrottle throttle = new TelemetryThrottle();
 * 
 * public void periodic() {
 *     if (throttle.shouldUpdate()) {
 *         SmartDashboard.putNumber("Key", value);
 *     }
 * }
 * </pre>
 */
public class TelemetryThrottle {
    private double lastUpdateTime = 0.0;
    private final double updatePeriod;
    
    /**
     * Creates a throttle with default 200ms (5 Hz) update rate
     */
    public TelemetryThrottle() {
        this(0.2); // 200ms = 5 Hz (down from 50 Hz)
    }
    
    /**
     * Creates a throttle with custom update period
     * 
     * @param updatePeriodSeconds Time between updates in seconds
     */
    public TelemetryThrottle(double updatePeriodSeconds) {
        this.updatePeriod = updatePeriodSeconds;
    }
    
    /**
     * Check if enough time has passed to send another telemetry update
     * 
     * @return true if it's time to update, false otherwise
     */
    public boolean shouldUpdate() {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastUpdateTime >= updatePeriod) {
            lastUpdateTime = currentTime;
            return true;
        }
        return false;
    }
    
    /**
     * Force an immediate update on the next call to shouldUpdate()
     */
    public void forceUpdate() {
        lastUpdateTime = 0.0;
    }
}
