package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShootingCalculator;

import java.util.function.Supplier;

/**
 * Turret subsystem - MOTOR REMOVED
 * Now handles target tracking and calculates chassis rotation assist for aiming
 * Physical turret motor has been removed - uses chassis rotation for aiming
 */
public class TurretSubsystem extends SubsystemBase {
    
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    
    // Target tracking
    private Translation3d currentTarget = null;
    
    public TurretSubsystem(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        // No motor initialization - turret motor has been physically removed
    }
    
    // ==================== BASIC CONTROL ====================
    
    /**
     * Sets the turret to a specific angle - NO-OP (motor removed)
     * @param angle Target angle in degrees
     */
    public void setAngle(double degrees) {
        // No-op - turret motor has been physically removed
    }
    
    /**
     * Gets the current turret angle - always returns 0° (motor removed)
     * @return Always 0.0 degrees (facing forward)
     */
    public double getAngleDegrees() {
        return 0.0;  // Turret always at 0° (robot-relative forward)
    }
    
    /**
     * Checks if turret is at target angle - always returns true (motor removed)
     */
    public boolean atTarget() {
        return true;  // Always "at target" since turret is locked at 0°
    }
    
    /**
     * Zeros the turret encoder - NO-OP (motor removed)
     */
    public void zeroTurret() {
        // No-op - turret motor has been physically removed
    }
    
    /**
     * Sets encoder to specific value - NO-OP (motor removed)
     */
    public void setEncoderPosition(double degrees) {
        // No-op - turret motor has been physically removed
    }
    
    /**
     * Sets encoder position using Rotation2d - NO-OP (motor removed)
     */
    public void setEncoderPosition(Rotation2d angle) {
        // No-op - turret motor has been physically removed
    }
    
    /**
     * Returns the turret to its initial position - NO-OP (motor removed)
     * Called when robot is disabled
     */
    public void returnToInitialPosition() {
        // Disable tracking when returning to initial position
        setTarget(null);
    }
    
    // ==================== TARGET TRACKING ====================
    
    /**
     * Sets the target to track (null to disable tracking)
     */
    public void setTarget(Translation3d target) {
        this.currentTarget = target;
    }
    
    /**
     * Gets the current tracking target
     */
    public Translation3d getTarget() {
        return currentTarget;
    }
    
    /**
     * Checks if tracking is enabled
     */
    public boolean isTracking() {
        return currentTarget != null;
    }
    
    /**
     * Chassis rotation assist - calculates rotation rate needed to aim at target using chassis
     * Turret is disabled and always at 0° (robot-relative forward)
     * 
     * @return Rotation rate in radians per second to aim at target
     */
    public double getChassisRotationAssist() {
        if (!isTracking()) {
            return 0.0; // No target, no assist
        }
        
        Pose2d robotPose = robotPoseSupplier.get();
        Translation2d target2d = currentTarget.toTranslation2d();
        
        // Calculate angle from robot to target
        Translation2d robotToTarget = target2d.minus(robotPose.getTranslation());
        Rotation2d angleToTarget = new Rotation2d(robotToTarget.getX(), robotToTarget.getY());
        
        // Calculate error: how much we need to rotate
        double currentHeading = robotPose.getRotation().getRadians();
        double targetHeading = angleToTarget.getRadians();
        double error = targetHeading - currentHeading;
        
        // Normalize error to [-π, π]
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        
        // Simple proportional control for chassis rotation
        // kP determines how aggressively we rotate to face target
        double kP = 3.5; // Tune this value as needed
        double rotationRate = error * kP;
        
        // Limit rotation rate to prevent excessive spinning
        double maxRotationRate = 6.0; // ~1 rotation per second
        rotationRate = Math.max(-maxRotationRate, Math.min(maxRotationRate, rotationRate));
        
        return rotationRate;
    }
    
    /**
     * Updates turret tracking - turret locked at 0°, chassis aims using getChassisRotationAssist()
     */
    private void updateTracking() {
        // Lock turret at 0° (robot-relative forward)
        setAngle(0.0);
        
        if (!isTracking()) {
            // Clear visualization
            Logger.recordOutput("TrackTarget/TargetPose", new Pose3d());
            Logger.recordOutput("TrackTarget/LineToTarget", new Pose3d[] {});
            Logger.recordOutput("TrackTarget/ChassisAimLine", new Pose3d[] {});
            return;
        }
        
        Pose2d robotPose = robotPoseSupplier.get();
        Translation2d target2d = currentTarget.toTranslation2d();
        
        // ===== VISUALIZATION =====
        Pose3d targetPose3d = new Pose3d(currentTarget, new Rotation3d());
        Logger.recordOutput("TrackTarget/TargetPose", targetPose3d);
        
        Pose3d robotPose3d = new Pose3d(robotPose);
        Logger.recordOutput("TrackTarget/LineToTarget", new Pose3d[] { robotPose3d, targetPose3d });
        
        // Chassis aim line (robot is aiming with chassis, turret at 0°)
        Rotation2d chassisHeading = robotPose.getRotation();
        double aimLineLength = 2.0; // 2 meter visualization line
        Translation2d aimLineEnd = robotPose.getTranslation().plus(
            new Translation2d(aimLineLength, chassisHeading)
        );
        Pose3d aimLineEndPose = new Pose3d(aimLineEnd.getX(), aimLineEnd.getY(), 0.4, new Rotation3d());
        Logger.recordOutput("TrackTarget/ChassisAimLine", new Pose3d[] { robotPose3d, aimLineEndPose });
        
        // Tracking telemetry
        double distance = robotPose.getTranslation().getDistance(target2d);
        Logger.recordOutput("TrackTarget/Distance", distance);
        Logger.recordOutput("TrackTarget/AtTarget", atTarget());
    }
    
    // findBestAngle() method removed - no longer needed since turret motor is removed
    
    @Override
    public void periodic() {
        // No motor signals to refresh - motor has been removed
        
        updateTracking();
        
        // Update smart shooting trajectory visualization continuously
        if (isTracking()) {
            Pose2d robotPose = robotPoseSupplier.get();
            ShootingCalculator.calculate(robotPose, currentTarget);
            // Solution automatically logs to AdvantageKit
        } else {
            // Clear shooting visualization when not tracking
            Logger.recordOutput("SmartShoot/Valid", false);
            Logger.recordOutput("SmartShoot/Trajectory", new Pose3d[0]);
        }
        
        // Turret pose visualization - always at 0° (robot-relative forward)
        Pose2d robotPose = robotPoseSupplier.get();
        double angle = 0.0;  // Turret locked at 0°
        Rotation2d turretFieldHeading = robotPose.getRotation().plus(Rotation2d.fromDegrees(angle));
        Pose3d turretPose = new Pose3d(
            robotPose.getX(), 
            robotPose.getY(), 
            0.5,  // Approximate turret height in meters
            new Rotation3d(0, 0, turretFieldHeading.getRadians())
        );
        Logger.recordOutput("Turret/Pose3d", turretPose);
        
        // Basic telemetry
        Logger.recordOutput("Turret/Angle", angle);
        Logger.recordOutput("Turret/Velocity", 0.0);  // No motor, no velocity
        Logger.recordOutput("Turret/Tracking", isTracking());
        Logger.recordOutput("Turret/AtTarget", atTarget());
    }
}

