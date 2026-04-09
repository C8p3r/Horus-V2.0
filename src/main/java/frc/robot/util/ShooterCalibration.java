package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Manages shooter calibration data using a lookup table.
 * Allows manual tuning via dashboard and uses nearest calibration point for shooting.
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
    
    // Calibration points list (lookup table)
    private final List<CalibrationPoint> calibrationPoints;
    private final List<CalibrationPoint> fixedHoodCalibrationPoints;
    
    // NetworkTables for Elastic dashboard
    private final NetworkTable calibrationTable;
    private final NetworkTableEntry manualHoodEntry;
    private final NetworkTableEntry manualFlywheelEntry;
    private final NetworkTableEntry manualFloorIndexerEntry;
    private final NetworkTableEntry manualFireIndexerEntry;
    private final NetworkTableEntry currentDistanceEntry;
    private final NetworkTableEntry recordButtonEntry;
    private final NetworkTableEntry calibrationModeEntry;
    private final NetworkTableEntry fixedHoodModeEntry;
    private final NetworkTableEntry fixedHoodAngleEntry;
    
    public ShooterCalibration() {
        calibrationPoints = new ArrayList<>();
        fixedHoodCalibrationPoints = new ArrayList<>();
        
        // Setup NetworkTables
        calibrationTable = NetworkTableInstance.getDefault().getTable("ShooterCalibration");
        manualHoodEntry = calibrationTable.getEntry("ManualHood");
        manualFlywheelEntry = calibrationTable.getEntry("ManualFlywheel");
        manualFloorIndexerEntry = calibrationTable.getEntry("ManualFloorIndexer");
        manualFireIndexerEntry = calibrationTable.getEntry("ManualFireIndexer");
        currentDistanceEntry = calibrationTable.getEntry("CurrentDistance");
        recordButtonEntry = calibrationTable.getEntry("RecordPoint");
        calibrationModeEntry = calibrationTable.getEntry("CalibrationMode");
        fixedHoodModeEntry = calibrationTable.getEntry("FixedHoodMode");
        fixedHoodAngleEntry = calibrationTable.getEntry("FixedHoodAngle");
        
        // Set default values
        manualHoodEntry.setDouble(14.0);  // Start at minimum angle
        manualFlywheelEntry.setDouble(0.0);  // Start at zero
        manualFloorIndexerEntry.setDouble(0.0);  // Start at zero
        manualFireIndexerEntry.setDouble(0.0);  // Start at zero
        currentDistanceEntry.setDouble(0.0);
        recordButtonEntry.setBoolean(false);
        calibrationModeEntry.setBoolean(false);  // Start in auto mode
        fixedHoodModeEntry.setBoolean(true);  // Start with hood enabled
        fixedHoodAngleEntry.setDouble(10.0);  // Default fixed angle
        
        // Load default calibration points
        loadDefaultCalibration();
        loadFixedHoodCalibration();
    }
    
    /**
     * Loads default calibration points as a starting baseline
     */
    
    private void loadDefaultCalibration() {
       
        System.out.println("[ShooterCalibration] Loaded default calibration:");
        for (CalibrationPoint point : calibrationPoints) {
            System.out.println("  " + point);
        }
    }
    
    /**
     * Loads fixed hood calibration points (hood stays at fixed angle, only flywheel changes)
     */
    private void loadFixedHoodCalibration() {
       
            /* NEW DATA FOR NE UNH EVENT */
//             // Fixed Hood @ 10 Degrees - High Velocity Curve
// addFixedHoodCalibrationPoint(0.912+0.305316, 10, 46.00); // Measured Point 1
// addFixedHoodCalibrationPoint(1.05+0.305316, 10, 48.37);
// addFixedHoodCalibrationPoint(1.20+2*0.305316, 10, 50.92);
// addFixedHoodCalibrationPoint(1.35+2*0.305316, 10, 53.46);
// addFixedHoodCalibrationPoint(1.50, 10, 56.00); // Measured Point 2
// addFixedHoodCalibrationPoint(1.65, 10, 60.80);
// addFixedHoodCalibrationPoint(1.80+2, 10, 65.60);
// addFixedHoodCalibrationPoint(1.95+2, 10, 70.40);
// addFixedHoodCalibrationPoint(2.10+2, 10, 75.20);
// addFixedHoodCalibrationPoint(2.25+2, 10, 80.00); // Measured Point 3

addFixedHoodCalibrationPoint(2.25+2*0.305316, 10, 45); //lmao
        System.out.println("[ShooterCalibration] Loaded fixed hood calibration (Hood fixed at 10°):");
        for (CalibrationPoint point : fixedHoodCalibrationPoints) {
            System.out.println("  " + point);
        }
    }
    
    /**
     * Adds a fixed hood calibration point
     */
    private void addFixedHoodCalibrationPoint(double distance, double hoodAngle, double flywheelRPS) {
        CalibrationPoint point = new CalibrationPoint(distance, hoodAngle, flywheelRPS);
        fixedHoodCalibrationPoints.add(point);
        fixedHoodCalibrationPoints.sort((a, b) -> Double.compare(a.distance, b.distance));
    }
    
    /**
     * Adds a calibration point to the lookup table
     */
    public void addCalibrationPoint(double distance, double hoodAngle, double flywheelRPS) {
        CalibrationPoint point = new CalibrationPoint(distance, hoodAngle, flywheelRPS);
        
        // Remove any existing point at this distance
        calibrationPoints.removeIf(p -> Math.abs(p.distance - distance) < 0.1);
        
        // Add new point
        calibrationPoints.add(point);
        
        // Sort by distance for easier debugging
        calibrationPoints.sort((a, b) -> Double.compare(a.distance, b.distance));
        
        System.out.println("[ShooterCalibration] Added calibration point: " + point);
    }
    
    /**
     * Checks if fixed hood mode is enabled
     */
    public boolean isFixedHoodMode() {
        return fixedHoodModeEntry.getBoolean(false);
    }
    
    /**
     * Gets the fixed hood angle setting
     */
    public double getFixedHoodAngle() {
        return fixedHoodAngleEntry.getDouble(10.0);
    }
    
    /**
     * Sets fixed hood mode on or off
     */
    public void setFixedHoodMode(boolean enabled) {
        fixedHoodModeEntry.setBoolean(enabled);
    }
    
    /**
     * Sets the fixed hood angle
     */
    public void setFixedHoodAngle(double angle) {
        fixedHoodAngleEntry.setDouble(angle);
    }
    
    /**
     * Finds the nearest calibration point to the given distance
     */
    private CalibrationPoint findNearestPoint(double distance) {
        List<CalibrationPoint> dataSet = isFixedHoodMode() ? fixedHoodCalibrationPoints : calibrationPoints;
        
        if (dataSet.isEmpty()) {
            return null;
        }
        
        CalibrationPoint nearest = dataSet.get(0);
        double minDiff = Math.abs(nearest.distance - distance);
        
        for (CalibrationPoint point : dataSet) {
            double diff = Math.abs(point.distance - distance);
            if (diff < minDiff) {
                minDiff = diff;
                nearest = point;
            }
        }
        
        return nearest;
    }
    
    /**
     * Gets hood angle for a given distance using nearest neighbor lookup
     * If in fixed hood mode, returns the fixed hood angle
     */
    public double getHoodAngle(double distance) {
        if (isFixedHoodMode()) {
            return fixedHoodAngleEntry.getDouble(10.0);
        }
        
        CalibrationPoint nearest = findNearestPoint(distance);
        if (nearest == null) {
            return 25.0; // Default fallback
        }
        return nearest.hoodAngle;
    }
    
    /**
     * Gets flywheel velocity for a given distance using nearest neighbor lookup
     */
    public double getFlywheelVelocity(double distance) {
        CalibrationPoint nearest = findNearestPoint(distance);
        if (nearest == null) {
            return 50.0; // Default fallback
        }
        return nearest.flywheelRPS;
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
        double flywheel = manualFlywheelEntry.getDouble(50.0);
        
        if (distance > 0.1) { // Only record if distance is valid
            if (isFixedHoodMode()) {
                // In fixed hood mode, use the fixed hood angle and save to fixed hood calibration
                double fixedHood = getFixedHoodAngle();
                addFixedHoodCalibrationPoint(distance, fixedHood, flywheel);
                System.out.println("[ShooterCalibration] RECORDED FIXED HOOD: Distance=" + distance + "m Hood=" + fixedHood + "° (LOCKED) Flywheel=" + flywheel + " RPS");
            } else {
                // In variable hood mode, use manual hood and save to regular calibration
                double hood = manualHoodEntry.getDouble(25.0);
                addCalibrationPoint(distance, hood, flywheel);
                System.out.println("[ShooterCalibration] RECORDED: Distance=" + distance + "m Hood=" + hood + "° Flywheel=" + flywheel + " RPS");
            }
        } else {
            System.out.println("[ShooterCalibration] Cannot record - invalid distance: " + distance);
        }
    }
    
    /**
     * Updates dashboard with current lookup values and nearest point info
     */
    public void updateDashboard(double currentDistance) {
        CalibrationPoint nearest = findNearestPoint(currentDistance);
        
        // Update the current distance display on dashboard
        calibrationTable.getEntry("CurrentDistance").setDouble(currentDistance);
        
        calibrationTable.getEntry("LookupHood").setDouble(getHoodAngle(currentDistance));
        calibrationTable.getEntry("LookupFlywheel").setDouble(getFlywheelVelocity(currentDistance));
        
        // Show appropriate calibration point count based on mode
        if (isFixedHoodMode()) {
            calibrationTable.getEntry("CalibrationPointCount").setDouble(fixedHoodCalibrationPoints.size());
            calibrationTable.getEntry("ActiveDataset").setString("FIXED HOOD");
        } else {
            calibrationTable.getEntry("CalibrationPointCount").setDouble(calibrationPoints.size());
            calibrationTable.getEntry("ActiveDataset").setString("VARIABLE HOOD");
        }
        
        if (nearest != null) {
            calibrationTable.getEntry("NearestPointDistance").setDouble(nearest.distance);
            calibrationTable.getEntry("DistanceError").setDouble(Math.abs(currentDistance - nearest.distance));
        }
        
        // Show current mode
        String mode;
        if (isFixedHoodMode()) {
            mode = "FIXED HOOD @ " + getFixedHoodAngle() + "°";
        } else if (isCalibrationMode()) {
            mode = "MANUAL CALIBRATION";
        } else {
            mode = "AUTO LOOKUP TABLE";
        }
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
