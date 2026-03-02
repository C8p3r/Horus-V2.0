package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Manages shooter calibration data and interpolation.
 * Allows manual tuning via dashboard and interpolates between calibration points.
 */
public class ShooterCalibration {
    
    // Calibration data point
    public static class CalibrationPoint {
        public final double distance;      // Distance to target (meters)
        public final double hoodAngle;     // Hood angle (degrees)
        public final double flywheelRPS;   // Flywheel velocity (RPS)
        
        public CalibrationPoint(double distance, double hoodAngle, double flywheelRPS) {
            this.distance = distance;
            this.hoodAngle = hoodAngle;
            this.flywheelRPS = flywheelRPS;
        }
        
        @Override
        public String toString() {
            return String.format("%.2fm: Hood=%.1f° Flywheel=%.1f RPS", distance, hoodAngle, flywheelRPS);
        }
    }
    
    // Interpolation maps
    private final InterpolatingDoubleTreeMap hoodAngleMap;
    private final InterpolatingDoubleTreeMap flywheelVelocityMap;
    
    // Calibration points list
    private final List<CalibrationPoint> calibrationPoints;
    
    // NetworkTables for Elastic dashboard
    private final NetworkTable calibrationTable;
    private final NetworkTableEntry manualHoodEntry;
    private final NetworkTableEntry manualFlywheelEntry;
    private final NetworkTableEntry manualFloorIndexerEntry;
    private final NetworkTableEntry manualFireIndexerEntry;
    private final NetworkTableEntry currentDistanceEntry;
    private final NetworkTableEntry recordButtonEntry;
    private final NetworkTableEntry calibrationModeEntry;
    
    public ShooterCalibration() {
        hoodAngleMap = new InterpolatingDoubleTreeMap();
        flywheelVelocityMap = new InterpolatingDoubleTreeMap();
        calibrationPoints = new ArrayList<>();
        
        // Setup NetworkTables
        calibrationTable = NetworkTableInstance.getDefault().getTable("ShooterCalibration");
        manualHoodEntry = calibrationTable.getEntry("ManualHood");
        manualFlywheelEntry = calibrationTable.getEntry("ManualFlywheel");
        manualFloorIndexerEntry = calibrationTable.getEntry("ManualFloorIndexer");
        manualFireIndexerEntry = calibrationTable.getEntry("ManualFireIndexer");
        currentDistanceEntry = calibrationTable.getEntry("CurrentDistance");
        recordButtonEntry = calibrationTable.getEntry("RecordPoint");
        calibrationModeEntry = calibrationTable.getEntry("CalibrationMode");
        
        // Set default values
        manualHoodEntry.setDouble(14.0);  // Start at minimum angle
        manualFlywheelEntry.setDouble(0.0);  // Start at zero
        manualFloorIndexerEntry.setDouble(0.0);  // Start at zero
        manualFireIndexerEntry.setDouble(0.0);  // Start at zero
        currentDistanceEntry.setDouble(0.0);
        recordButtonEntry.setBoolean(false);
        calibrationModeEntry.setBoolean(false);  // Start in auto mode
        
        // Load default calibration points
        loadDefaultCalibration();
    }
    
    /**
     * Loads default calibration points as a starting baseline
     */
    private void loadDefaultCalibration() {
        // These are reasonable starting points - tune them empirically!
        // Hood angle: 14° = flattest (close shots), 40° = steepest (far shots)
        addCalibrationPoint(1.0, 14.0, 35.0);   // Close shot - flat angle, lower velocity
        addCalibrationPoint(2.0, 20.0, 45.0);   
        addCalibrationPoint(3.0, 26.0, 55.0);   
        addCalibrationPoint(4.0, 32.0, 65.0);   
        addCalibrationPoint(5.0, 40.0, 75.0);   // Far shot - steep angle, higher velocity
        
        System.out.println("[ShooterCalibration] Loaded default calibration:");
        for (CalibrationPoint point : calibrationPoints) {
            System.out.println("  " + point);
        }
    }
    
    /**
     * Adds a calibration point and rebuilds interpolation maps
     */
    public void addCalibrationPoint(double distance, double hoodAngle, double flywheelRPS) {
        CalibrationPoint point = new CalibrationPoint(distance, hoodAngle, flywheelRPS);
        
        // Remove any existing point at this distance
        calibrationPoints.removeIf(p -> Math.abs(p.distance - distance) < 0.1);
        
        // Add new point
        calibrationPoints.add(point);
        
        // Sort by distance
        calibrationPoints.sort((a, b) -> Double.compare(a.distance, b.distance));
        
        // Rebuild interpolation maps
        rebuildMaps();
        
        System.out.println("[ShooterCalibration] Added calibration point: " + point);
    }
    
    /**
     * Rebuilds the interpolation maps from calibration points
     */
    private void rebuildMaps() {
        hoodAngleMap.clear();
        flywheelVelocityMap.clear();
        
        for (CalibrationPoint point : calibrationPoints) {
            hoodAngleMap.put(point.distance, point.hoodAngle);
            flywheelVelocityMap.put(point.distance, point.flywheelRPS);
        }
    }
    
    /**
     * Gets interpolated hood angle for a given distance
     */
    public double getHoodAngle(double distance) {
        if (calibrationPoints.isEmpty()) {
            return 25.0; // Default fallback
        }
        return hoodAngleMap.get(distance);
    }
    
    /**
     * Gets interpolated flywheel velocity for a given distance
     */
    public double getFlywheelVelocity(double distance) {
        if (calibrationPoints.isEmpty()) {
            return 50.0; // Default fallback
        }
        return flywheelVelocityMap.get(distance);
    }
    
    /**
     * Gets manual values from dashboard
     */
    public double getManualHoodAngle() {
        return manualHoodEntry.getDouble(14.0);
    }
    
    public double getManualFlywheelVelocity() {
        return manualFlywheelEntry.getDouble(0.0);
    }
    
    public double getManualFloorIndexer() {
        return manualFloorIndexerEntry.getDouble(0.0);
    }
    
    public double getManualFireIndexer() {
        return manualFireIndexerEntry.getDouble(0.0);
    }
    
    /**
     * Checks if calibration mode is enabled
     */
    public boolean isCalibrationMode() {
        return calibrationModeEntry.getBoolean(false);
    }
    
    /**
     * Sets calibration mode
     */
    public void setCalibrationMode(boolean enabled) {
        calibrationModeEntry.setBoolean(enabled);
    }
    
    /**
     * Resets manual values to initial state (for robot enable)
     */
    public void resetToInitialValues() {
        manualHoodEntry.setDouble(14.0);  // Flattest angle
        manualFlywheelEntry.setDouble(0.0);  // Zero velocity
        manualFloorIndexerEntry.setDouble(0.0);  // Zero duty cycle
        manualFireIndexerEntry.setDouble(0.0);  // Zero duty cycle
        System.out.println("[ShooterCalibration] Reset to initial values: Hood=14° Flywheel=0 RPS Indexers=0");
    }
    
    /**
     * Sets the current distance being shot (for recording)
     */
    public void setCurrentDistance(double distance) {
        currentDistanceEntry.setDouble(distance);
    }
    
    /**
     * Checks if user wants to record current manual settings as a calibration point
     * @return true if record button was pressed
     */
    public boolean shouldRecordPoint() {
        boolean shouldRecord = recordButtonEntry.getBoolean(false);
        if (shouldRecord) {
            // Reset button
            recordButtonEntry.setBoolean(false);
            return true;
        }
        return false;
    }
    
    /**
     * Records current manual settings as a calibration point at current distance
     */
    public void recordCurrentPoint() {
        double distance = currentDistanceEntry.getDouble(0.0);
        double hood = manualHoodEntry.getDouble(25.0);
        double flywheel = manualFlywheelEntry.getDouble(50.0);
        
        if (distance > 0.1) { // Only record if distance is valid
            addCalibrationPoint(distance, hood, flywheel);
            System.out.println("[ShooterCalibration] RECORDED: Distance=" + distance + "m Hood=" + hood + "° Flywheel=" + flywheel + " RPS");
        } else {
            System.out.println("[ShooterCalibration] Cannot record - invalid distance: " + distance);
        }
    }
    
    /**
     * Updates dashboard with current interpolated values
     */
    public void updateDashboard(double currentDistance) {
        calibrationTable.getEntry("InterpolatedHood").setDouble(getHoodAngle(currentDistance));
        calibrationTable.getEntry("InterpolatedFlywheel").setDouble(getFlywheelVelocity(currentDistance));
        calibrationTable.getEntry("CalibrationPointCount").setDouble(calibrationPoints.size());
        
        // Show current mode
        String mode = isCalibrationMode() ? "MANUAL CALIBRATION" : "AUTO INTERPOLATED";
        calibrationTable.getEntry("CurrentMode").setString(mode);
    }
    
    /**
     * Prints all calibration points to console
     */
    public void printCalibrationPoints() {
        System.out.println("[ShooterCalibration] Current calibration points (" + calibrationPoints.size() + "):");
        for (CalibrationPoint point : calibrationPoints) {
            System.out.println("  " + point);
        }
    }
    
    /**
     * Gets all calibration points as a formatted string (for saving to file later)
     */
    public String getCalibrationDataString() {
        StringBuilder sb = new StringBuilder();
        sb.append("// Shooter Calibration Data\n");
        for (CalibrationPoint point : calibrationPoints) {
            sb.append(String.format("addCalibrationPoint(%.2f, %.1f, %.1f);\n", 
                point.distance, point.hoodAngle, point.flywheelRPS));
        }
        return sb.toString();
    }
}
