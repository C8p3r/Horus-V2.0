package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.FlywheelConstants;
import frc.robot.constants.PhysicsConstants;

/**
 * Calculates optimal shooting parameters (turret angle, hood angle, flywheel velocity)
 * based on robot position and target. Uses interpolation from manually-tuned calibration
 * points for accurate, real-world shooting.
 * 
 * Calibration workflow:
 * 1. Use Elastic dashboard sliders to manually tune hood angle and flywheel velocity
 * 2. Press "Record Point" button to save current settings at current distance
 * 3. Repeat for multiple distances to build calibration table
 * 4. System interpolates between calibration points for any distance
 */
public class ShootingCalculator {
    
    // Shooter physical parameters
    private static final double SHOOTER_HEIGHT_METERS = PhysicsConstants.SHOOTER_OFFSET_Z;
    private static final double SHOOTER_OFFSET_X = PhysicsConstants.SHOOTER_OFFSET_X;
    private static final double SHOOTER_OFFSET_Y = PhysicsConstants.SHOOTER_OFFSET_Y;
    
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
     * Uses interpolation from calibration table (or manual values if in calibration mode)
     * 
     * @param robotPose Current robot pose
     * @param target Target position (3D)
     * @return ShootingSolution with optimal parameters
     */
    public static ShootingSolution calculate(Pose2d robotPose, Translation3d target) {
        // Calculate shooter position in field coordinates
        Translation3d shooterPos = calculateShooterPosition(robotPose);
        
        // Calculate turret angle (horizontal angle to target)
        Translation2d robotPos = robotPose.getTranslation();
        Translation2d targetPos = target.toTranslation2d();
        Translation2d robotToTarget = targetPos.minus(robotPos);
        
        // Calculate horizontal distance (ignoring height difference)
        double horizontalDistance = robotToTarget.getNorm();
        
        // Turret angle (robot-relative)
        Rotation2d fieldAngleToTarget = new Rotation2d(robotToTarget.getX(), robotToTarget.getY());
        double turretAngle = fieldAngleToTarget.minus(robotPose.getRotation()).getDegrees();
        
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
            hoodAngle = calibration.getManualHoodAngle();
            flywheelRPS = calibration.getManualFlywheelVelocity();
            System.out.println(String.format("[ShootCalc] MANUAL MODE - Distance: %.2fm -> Hood: %.1f° Flywheel: %.1f RPS",
                horizontalDistance, hoodAngle, flywheelRPS));
        } else {
            // AUTO MODE: Use interpolated values from calibration
            hoodAngle = calibration.getHoodAngle(horizontalDistance);
            flywheelRPS = calibration.getFlywheelVelocity(horizontalDistance);
            System.out.println(String.format("[ShootCalc] AUTO MODE - Distance: %.2fm -> Hood: %.1f° Flywheel: %.1f RPS",
                horizontalDistance, hoodAngle, flywheelRPS));
        }
        
        // Generate simple trajectory visualization for AdvantageScope
        Pose3d[] trajectoryPoints = generateSimpleTrajectory(shooterPos, target, hoodAngle, flywheelRPS);
        
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
     * 
     * @param robotPose Current robot pose
     * @param target Target position (3D)
     * @return ShootingSolution with manual parameters
     */
    public static ShootingSolution getManualSolution(Pose2d robotPose, Translation3d target) {
        // Calculate turret angle
        Translation2d robotPos = robotPose.getTranslation();
        Translation2d targetPos = target.toTranslation2d();
        Translation2d robotToTarget = targetPos.minus(robotPos);
        double horizontalDistance = robotToTarget.getNorm();
        
        Rotation2d fieldAngleToTarget = new Rotation2d(robotToTarget.getX(), robotToTarget.getY());
        double turretAngle = fieldAngleToTarget.minus(robotPose.getRotation()).getDegrees();
        
        // Get manual values from dashboard
        double hoodAngle = calibration.getManualHoodAngle();
        double flywheelRPS = calibration.getManualFlywheelVelocity();
        
        // Update current distance
        calibration.setCurrentDistance(horizontalDistance);
        
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
     * Calculates shooter position in field coordinates from robot pose
     */
    private static Translation3d calculateShooterPosition(Pose2d robotPose) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotAngle = robotPose.getRotation().getRadians();
        
        // Transform shooter offset from robot-relative to field-relative
        double fieldX = robotX + SHOOTER_OFFSET_X * Math.cos(robotAngle) 
                               - SHOOTER_OFFSET_Y * Math.sin(robotAngle);
        double fieldY = robotY + SHOOTER_OFFSET_X * Math.sin(robotAngle) 
                               + SHOOTER_OFFSET_Y * Math.cos(robotAngle);
        double fieldZ = SHOOTER_HEIGHT_METERS;
        
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
