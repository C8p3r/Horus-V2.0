package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Monitors robot power consumption and brownout conditions
 * Provides real-time telemetry for diagnosing power issues
 */
public class PowerMonitor {
    
    private final PowerDistribution pdh;
    
    // Brownout detection thresholds
    private static final double BROWNOUT_VOLTAGE = 6.8; // Radio loses connection below ~6.5V
    private static final double WARNING_VOLTAGE = 9.0;  // Warning threshold
    private static final double CRITICAL_CURRENT = 350.0; // Total current warning
    
    // Tracking brownout events
    private int brownoutCount = 0;
    private double lowestVoltage = 12.6;
    private double highestCurrent = 0.0;
    
    public PowerMonitor() {
        // Initialize PDH (REV Power Distribution Hub)
        pdh = new PowerDistribution(1, ModuleType.kRev);
    }
    
    /**
     * Update power telemetry - call this in Robot.periodic()
     */
    public void updateTelemetry() {
        // Get battery voltage from RobotController (most accurate)
        double batteryVoltage = RobotController.getBatteryVoltage();
        
        // Get total current from PDH
        double totalCurrent = pdh.getTotalCurrent();
        
        // Track statistics
        if (batteryVoltage < lowestVoltage) {
            lowestVoltage = batteryVoltage;
        }
        if (totalCurrent > highestCurrent) {
            highestCurrent = totalCurrent;
        }
        
        // Detect brownout conditions
        if (batteryVoltage < BROWNOUT_VOLTAGE) {
            brownoutCount++;
        }
        
        // Publish to SmartDashboard
        SmartDashboard.putNumber("Power/Battery Voltage", Math.round(batteryVoltage * 100.0) / 100.0);
        SmartDashboard.putNumber("Power/Total Current", Math.round(totalCurrent * 10.0) / 10.0);
        SmartDashboard.putNumber("Power/Total Power (W)", Math.round(batteryVoltage * totalCurrent * 10.0) / 10.0);
        
        SmartDashboard.putNumber("Power/Lowest Voltage", Math.round(lowestVoltage * 100.0) / 100.0);
        SmartDashboard.putNumber("Power/Highest Current", Math.round(highestCurrent * 10.0) / 10.0);
        SmartDashboard.putNumber("Power/Brownout Count", brownoutCount);
        
        // Warning indicators
        SmartDashboard.putBoolean("Power/VOLTAGE WARNING", batteryVoltage < WARNING_VOLTAGE);
        SmartDashboard.putBoolean("Power/CURRENT WARNING", totalCurrent > CRITICAL_CURRENT);
        SmartDashboard.putBoolean("Power/BROWNOUT RISK", batteryVoltage < BROWNOUT_VOLTAGE);
        
        // Individual channel currents (for detailed diagnosis)
        SmartDashboard.putNumber("Power/PDH Channel 0", Math.round(pdh.getCurrent(0) * 10.0) / 10.0);
        SmartDashboard.putNumber("Power/PDH Channel 1", Math.round(pdh.getCurrent(1) * 10.0) / 10.0);
        SmartDashboard.putNumber("Power/PDH Channel 2", Math.round(pdh.getCurrent(2) * 10.0) / 10.0);
        SmartDashboard.putNumber("Power/PDH Channel 3", Math.round(pdh.getCurrent(3) * 10.0) / 10.0);
        
        // Temperature monitoring
        SmartDashboard.putNumber("Power/PDH Temperature (C)", Math.round(pdh.getTemperature() * 10.0) / 10.0);
    }
    
    /**
     * Check if robot is in brownout condition
     */
    public boolean isBrownout() {
        return RobotController.getBatteryVoltage() < BROWNOUT_VOLTAGE;
    }
    
    /**
     * Check if robot is approaching brownout
     */
    public boolean isLowVoltage() {
        return RobotController.getBatteryVoltage() < WARNING_VOLTAGE;
    }
    
    /**
     * Get current battery voltage
     */
    public double getBatteryVoltage() {
        return RobotController.getBatteryVoltage();
    }
    
    /**
     * Get total current draw
     */
    public double getTotalCurrent() {
        return pdh.getTotalCurrent();
    }
    
    /**
     * Reset statistics
     */
    public void resetStatistics() {
        brownoutCount = 0;
        lowestVoltage = RobotController.getBatteryVoltage();
        highestCurrent = pdh.getTotalCurrent();
    }
}
