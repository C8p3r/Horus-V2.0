package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.TurretConstants;

/**
 * Calculates optimal shooting parameters (turret angle, hood angle, flywheel velocity)
 * based on robot position and target. Uses turret's actual position offset from robot center
 * for accurate distance and angle calculations. Uses lookup table from manually-tuned 
 * calibration points for accurate, real-world shooting.
 * 
 * Calibration workflow:
 * 1. Use Elastic dashboard sliders to manually tune hood angle and flywheel velocity
 * 2. Press "Record Point" button to save current settings at current distance
 * 3. Repeat for multiple distances to build calibration table
 * 4. System uses nearest calibration point for any distance
 */
public class ShootingCalculator {
    
    // Calibration system
    private static final ShooterCalibration calibration = new ShooterCalibration();
    
    /**
     * Represents a complete shooting solution
     */
    public static class ShootingSolution {
        public final double turretAngleDegrees;
        public final double hoodAngleDegrees;
        public final double flywheelVelocityRPS;
        public final double distance;
        public final double entryAngleDegrees;
        public final boolean isValid;
        public final Pose3d[] trajectoryPoints; // For visualization
        
        public ShootingSolution(double turretAngle, double hoodAngle, double flywheelVelocity, 
                              double distance, double entryAngle, boolean isValid) {
            this(turretAngle, hoodAngle, flywheelVelocity, distance, entryAngle, isValid, new Pose3d[0]);
        }
        
        public ShootingSolution(double turretAngle, double hoodAngle, double flywheelVelocity, 
                              double distance, double entryAngle, boolean isValid, Pose3d[] trajectoryPoints) {
            this.turretAngleDegrees = turretAngle;
            this.hoodAngleDegrees = hoodAngle;
            this.flywheelVelocityRPS = flywheelVelocity;
            this.distance = distance;
            this.entryAngleDegrees = entryAngle;
            this.isValid = isValid;
            this.trajectoryPoints = trajectoryPoints;
        }
        
        public Rotation2d getTurretAngle() {
            return Rotation2d.fromDegrees(turretAngleDegrees);
        }
        
        public Rotation2d getHoodAngle() {
            return Rotation2d.fromDegrees(hoodAngleDegrees);
        }
        
        /**
         * Logs this solution to AdvantageKit for visualization
         */
        public void logToAdvantageKit() {
            Logger.recordOutput("SmartShoot/Valid", isValid);
            Logger.recordOutput("SmartShoot/Distance", distance);
            Logger.recordOutput("SmartShoot/TurretAngle", turretAngleDegrees);
            Logger.recordOutput("SmartShoot/HoodAngle", hoodAngleDegrees);
            Logger.recordOutput("SmartShoot/FlywheelVelocity", flywheelVelocityRPS);
            Logger.recordOutput("SmartShoot/EntryAngle", entryAngleDegrees);
            Logger.recordOutput("SmartShoot/Trajectory", trajectoryPoints);
        }
    }
    
    /**
     * Calculates optimal shooting parameters for a given robot pose and target
     * Uses lookup table from calibration (or manual values if in calibration mode)
     * Accounts for turret position offset from robot center for accurate aiming
     * 
     * @param robotPose Current robot pose
     * @param target Target position (3D)
     * @return ShootingSolution with optimal parameters
     */
    public static ShootingSolution calculate(Pose2d robotPose, Translation3d target) {
        // Calculate turret position in field coordinates (accounts for offset from robot center)
        Translation3d turretPos = calculateTurretPosition(robotPose);
        
        // Calculate vector from TURRET to target (not robot center to target)
        Translation2d turretPos2d = turretPos.toTranslation2d();
        Translation2d targetPos2d = target.toTranslation2d();
        Translation2d turretToTarget = targetPos2d.minus(turretPos2d);
        
        // Calculate horizontal distance from turret to target
        double horizontalDistance = turretToTarget.getNorm();
        
        // Calculate turret angle (robot-relative)
        // Field angle to target from turret position
        Rotation2d fieldAngleToTarget = new Rotation2d(turretToTarget.getX(), turretToTarget.getY());
        // Convert to robot-relative angle
        // ADD 180° because launcher now faces backward instead of forward
        double turretAngle = fieldAngleToTarget.minus(robotPose.getRotation()).getDegrees() + 180.0;
        
        // Update calibration system with current distance
        calibration.setCurrentDistance(horizontalDistance);
        calibration.updateDashboard(horizontalDistance);
        
        // Check if user wants to record a calibration point
        if (calibration.shouldRecordPoint()) {
            calibration.recordCurrentPoint();
            calibration.printCalibrationPoints();
        }
        
        // Get values based on mode
        double hoodAngle;
        double flywheelRPS;
        
        if (calibration.isCalibrationMode()) {
            // CALIBRATION MODE: Use manual values from dashboard
            // If in fixed hood mode, lock hood at fixed angle and only tune flywheel
            if (calibration.isFixedHoodMode()) {
                hoodAngle = calibration.getFixedHoodAngle();
                flywheelRPS = calibration.getManualFlywheelVelocity();
                System.out.println(String.format("[ShootCalc] FIXED HOOD CALIBRATION - Distance: %.2fm -> Hood: %.1f° (LOCKED) Flywheel: %.1f RPS",
                    horizontalDistance, hoodAngle, flywheelRPS));
            } else {
                hoodAngle = calibration.getManualHoodAngle();
                flywheelRPS = calibration.getManualFlywheelVelocity();
                System.out.println(String.format("[ShootCalc] MANUAL MODE - Distance: %.2fm -> Hood: %.1f° Flywheel: %.1f RPS",
                    horizontalDistance, hoodAngle, flywheelRPS));
            }
        } else {
            // AUTO MODE: Use interpolated values from calibration
            hoodAngle = calibration.getHoodAngle(horizontalDistance);
            flywheelRPS = calibration.getFlywheelVelocity(horizontalDistance);
            System.out.println(String.format("[ShootCalc] AUTO MODE - Distance: %.2fm -> Hood: %.1f° Flywheel: %.1f RPS",
                horizontalDistance, hoodAngle, flywheelRPS));
        }
        
        // Generate simple trajectory visualization for AdvantageScope
        Pose3d[] trajectoryPoints = generateSimpleTrajectory(turretPos, target, hoodAngle, flywheelRPS);
        
        // Log turret position for visualization
        Pose3d turretPose3d = new Pose3d(turretPos, new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
        Logger.recordOutput("SmartShoot/TurretPosition", turretPose3d);
        
        // Create solution
        ShootingSolution solution = new ShootingSolution(
            turretAngle,
            hoodAngle,
            flywheelRPS,
            horizontalDistance,
            -45.0, // Approximate entry angle
            true,
            trajectoryPoints
        );
        
        solution.logToAdvantageKit();
        return solution;
    }
    
    /**
     * Gets the calibration system instance (for accessing from subsystems)
     */
    public static ShooterCalibration getCalibration() {
        return calibration;
    }
    
    /**
     * Gets manual shooting parameters from dashboard (for manual tuning mode)
     * Accounts for turret position offset from robot center
     * 
     * @param robotPose Current robot pose
     * @param target Target position (3D)
     * @return ShootingSolution with manual parameters
     */
    public static ShootingSolution getManualSolution(Pose2d robotPose, Translation3d target) {
        // Calculate turret position in field coordinates
        Translation3d turretPos = calculateTurretPosition(robotPose);
        
        // Calculate vector from TURRET to target
        Translation2d turretPos2d = turretPos.toTranslation2d();
        Translation2d targetPos2d = target.toTranslation2d();
        Translation2d turretToTarget = targetPos2d.minus(turretPos2d);
        
        // Calculate horizontal distance from turret to target
        double horizontalDistance = turretToTarget.getNorm();
        
        // Calculate turret angle (robot-relative)
        Rotation2d fieldAngleToTarget = new Rotation2d(turretToTarget.getX(), turretToTarget.getY());
        // ADD 180° because launcher now faces backward instead of forward
        double turretAngle = fieldAngleToTarget.minus(robotPose.getRotation()).getDegrees() + 180.0;
        
        // Get manual values from dashboard
        double hoodAngle = calibration.getManualHoodAngle();
        double flywheelRPS = calibration.getManualFlywheelVelocity();
        
        // Update current distance
        calibration.setCurrentDistance(horizontalDistance);
        
        // Log turret position for visualization
        Pose3d turretPose3d = new Pose3d(turretPos, new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
        Logger.recordOutput("SmartShoot/TurretPosition", turretPose3d);
        
        return new ShootingSolution(
            turretAngle,
            hoodAngle,
            flywheelRPS,
            horizontalDistance,
            -45.0,
            true
        );
    }
    
    /**
     * Calculates turret position in field coordinates from robot pose
     * Accounts for turret being offset from robot center
     */
    private static Translation3d calculateTurretPosition(Pose2d robotPose) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotAngle = robotPose.getRotation().getRadians();
        
        // Transform turret offset from robot-relative to field-relative
        // X and Y offsets rotate with the robot
        double fieldX = robotX + TurretConstants.OFFSET_X * Math.cos(robotAngle) 
                               - TurretConstants.OFFSET_Y * Math.sin(robotAngle);
        double fieldY = robotY + TurretConstants.OFFSET_X * Math.sin(robotAngle) 
                               + TurretConstants.OFFSET_Y * Math.cos(robotAngle);
        double fieldZ = TurretConstants.OFFSET_Z;
        
        return new Translation3d(fieldX, fieldY, fieldZ);
    }
    
    /**
     * Generates a simple parabolic trajectory for visualization
     */
    private static Pose3d[] generateSimpleTrajectory(
            Translation3d shooterPos, 
            Translation3d target,
            double hoodAngleDeg, 
            double flywheelRPS) {
        
        int numPoints = 30;
        Pose3d[] trajectory = new Pose3d[numPoints];
        
        // Simple straight line from shooter to target for now
        for (int i = 0; i < numPoints; i++) {
            double t = (double) i / (numPoints - 1);
            double x = shooterPos.getX() + t * (target.getX() - shooterPos.getX());
            double y = shooterPos.getY() + t * (target.getY() - shooterPos.getY());
            double z = shooterPos.getZ() + t * (target.getZ() - shooterPos.getZ());
            
            // Add parabolic arc
            double arc = 4 * t * (1 - t) * 1.0; // Simple parabola, 1m high at peak
            z += arc;
            
            trajectory[i] = new Pose3d(x, y, z, new Rotation3d());
        }
        
        return trajectory;
    }
}
