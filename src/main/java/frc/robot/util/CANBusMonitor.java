package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Monitors CAN bus health and network connectivity
 * Helps diagnose communication issues
 */
public class CANBusMonitor {
    
    private final TelemetryThrottle telemetryThrottle;
    
    private int canBusOffCount = 0;
    private int canTxErrorCount = 0;
    private int canRxErrorCount = 0;
    
    private long lastUpdateTime = 0;
    private boolean wasConnected = true;
    private int disconnectCount = 0;
    
    public CANBusMonitor() {
        // Throttle telemetry to 5 Hz (200ms) to prevent network I/O blocking
        telemetryThrottle = new TelemetryThrottle(0.2);
    }
    
    /**
     * Update CAN bus telemetry - call this in Robot.periodic()
     * Throttled to 200ms to prevent Status loop blocking
     */
    public void updateTelemetry() {
        // Get CAN bus statistics from RobotController
        double canUtilization = RobotController.getCANStatus().percentBusUtilization;
        int busOffCount = RobotController.getCANStatus().busOffCount;
        int txFullCount = RobotController.getCANStatus().transmitErrorCount;
        int receiveErrorCount = RobotController.getCANStatus().receiveErrorCount;
        
        // Track bus-off events (indicates severe CAN bus problems)
        if (busOffCount > canBusOffCount) {
            canBusOffCount = busOffCount;
            System.err.println("⚠️ CAN BUS OFF EVENT DETECTED! Count: " + canBusOffCount);
        }
        
        // Track error counts
        if (txFullCount > canTxErrorCount) {
            canTxErrorCount = txFullCount;
        }
        if (receiveErrorCount > canRxErrorCount) {
            canRxErrorCount = receiveErrorCount;
        }
        
        // Detect connectivity drops
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime > 100) { // More than 100ms since last update
            if (wasConnected) {
                disconnectCount++;
                wasConnected = false;
                System.err.println("🔴 CONNECTIVITY DROP DETECTED! Count: " + disconnectCount);
            }
        } else {
            wasConnected = true;
        }
        lastUpdateTime = currentTime;
        
        // Throttle telemetry updates to prevent Status loop blocking
        if (!telemetryThrottle.shouldUpdate()) {
            return; // Skip this update cycle
        }
        
        // Publish to SmartDashboard (throttled to 5 Hz)
        SmartDashboard.putNumber("CAN/Utilization %", Math.round(canUtilization * 10.0) / 10.0);
        SmartDashboard.putNumber("CAN/Bus Off Count", canBusOffCount);
        SmartDashboard.putNumber("CAN/TX Error Count", canTxErrorCount);
        SmartDashboard.putNumber("CAN/RX Error Count", canRxErrorCount);
        SmartDashboard.putNumber("CAN/Disconnect Count", disconnectCount);
        
        // Warning indicators
        SmartDashboard.putBoolean("CAN/HIGH UTILIZATION", canUtilization > 80.0);
        SmartDashboard.putBoolean("CAN/BUS OFF ERROR", canBusOffCount > 0);
        SmartDashboard.putBoolean("CAN/TX ERRORS", canTxErrorCount > 0);
        
        // Network health
        SmartDashboard.putBoolean("Network/Connected", wasConnected);
    }
    
    /**
     * Check if CAN bus has critical errors
     */
    public boolean hasCriticalErrors() {
        return canBusOffCount > 0 || canTxErrorCount > 10;
    }
    
    /**
     * Get CAN bus utilization percentage
     */
    public double getCANUtilization() {
        return RobotController.getCANStatus().percentBusUtilization;
    }
}
