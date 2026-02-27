package frc.robot.util;

import frc.robot.RobotContainer;

/**
 * Centralized telemetry manager
 * Provides a static method to check if telemetry is enabled
 */
public class TelemetryManager {
    
    /**
     * Check if telemetry and logging are enabled
     * @return true if telemetry should be published, false if disabled
     */
    public static boolean isEnabled() {
        return !RobotContainer.DISABLE_ALL_TELEMETRY;
    }
    
    /**
     * Check if telemetry and logging are disabled
     * @return true if telemetry is disabled, false if enabled
     */
    public static boolean isDisabled() {
        return RobotContainer.DISABLE_ALL_TELEMETRY;
    }
}
