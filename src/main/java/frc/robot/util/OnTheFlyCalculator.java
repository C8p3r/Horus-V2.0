package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PhysicsConstants;

/**
 * On-The-Fly shooting calculation utility (simulation-only).
 * 
 * Provides intelligent target selection and shooting solution calculations
 * for moving robot shots. This is a pure calculation utility with no
 * hardware dependencies.
 * 
 * Key features:
 * - Smart target selection (hub vs HPS pass)
 * - Distance-based shooting decisions
 * - Field position awareness
 * - Turret angle calculations
 * - Integration with ShooterSimulator for trajectory
 */
public class OnTheFlyCalculator {
    
    /**
     * Complete shooting solution for on-the-fly shots
     */
    public static class ShootingSolution {
        public final Translation3d targetPosition;
        public final Rotation2d requiredTurretAngle;
        public final Rotation2d requiredHoodAngle;
        public final double requiredFlywheelVelocity;
        public final double distanceToTarget;
        public final double estimatedFlightTime;
        public final boolean isValid;
        public final TargetType targetType;
        
        public ShootingSolution(Translation3d target, Rotation2d turret, Rotation2d hood,
                               double flywheel, double distance, double flightTime, 
                               boolean valid, TargetType type) {
            this.targetPosition = target;
            this.requiredTurretAngle = turret;
            this.requiredHoodAngle = hood;
            this.requiredFlywheelVelocity = flywheel;
            this.distanceToTarget = distance;
            this.estimatedFlightTime = flightTime;
            this.isValid = valid;
            this.targetType = type;
        }
    }
    
    /**
     * Type of target being aimed at
     */
    public enum TargetType {
        HUB,           // Alliance hub (main scoring target)
        HPS_CLOSE,     // Close HPS pass target
        HPS_FAR,       // Far HPS pass target
        CUSTOM         // Custom target position
    }
    
    /**
     * Calculates the complete shooting solution for hitting a target from current position.
     * Uses SMART targeting logic to select appropriate target based on field position.
     * 
     * @param robotPose Current robot pose
     * @param robotVelocity Current robot velocity (field-relative)
     * @param customTarget Optional custom target (null to use smart selection)
     * @return Complete shooting solution
     */
    public static ShootingSolution calculateShot(Pose2d robotPose, 
                                                  ChassisSpeeds robotVelocity,
                                                  Translation3d customTarget) {
        // Determine target using smart logic
        Translation3d target;
        TargetType targetType;
        
        if (customTarget != null) {
            target = customTarget;
            targetType = TargetType.CUSTOM;
        } else {
            // Use SMART targeting logic
            var smartTarget = selectSmartTarget(robotPose);
            target = smartTarget.target;
            targetType = smartTarget.type;
        }
        
        // Calculate shooter position in field coordinates
        Translation3d shooterPos = calculateShooterPosition(robotPose);
        
        // Calculate distance to target
        double distance = shooterPos.getDistance(target);
        
        // Calculate required turret angle (field-relative pointing direction)
        Rotation2d turretAngle = calculateTurretAngle(robotPose, target);
        
        // Use ShooterSimulator to calculate optimal shot
        double initialVelocity = 15.0; // Starting guess (m/s)
        ShooterSimulator.ShootingSolution simSolution = ShooterSimulator.calculateShot(
            shooterPos, target, initialVelocity
        );
        
        // If initial guess didn't work, try higher velocities
        if (!simSolution.isValid) {
            for (double v = 20.0; v <= 30.0; v += 2.0) {
                simSolution = ShooterSimulator.calculateShot(shooterPos, target, v);
                if (simSolution.isValid) break;
            }
        }
        
        // Convert to shooter-relative angles and velocities
        Rotation2d hoodAngle = simSolution.launchAngle;
        double flywheelRPS = velocityToFlywheelRPS(simSolution.launchVelocity);
        
        return new ShootingSolution(
            target,
            turretAngle,
            hoodAngle,
            flywheelRPS,
            distance,
            simSolution.flightTime,
            simSolution.isValid,
            targetType
        );
    }
    
    /**
     * Smart target selection based on robot position and alliance.
     * Automatically chooses between hub and HPS pass targets.
     * 
     * Logic:
     * 1. If in opponent territory → pass to HPS
     * 2. If beyond max hub range → pass to HPS
     * 3. Otherwise → shoot at hub
     */
    private static class SmartTarget {
        Translation3d target;
        TargetType type;
        SmartTarget(Translation3d t, TargetType ty) {
            target = t;
            type = ty;
        }
    }
    
    private static SmartTarget selectSmartTarget(Pose2d robotPose) {
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        
        // Determine hub target
        Translation3d hubTarget = isRed ? FieldConstants.RED_HUB : FieldConstants.BLUE_HUB;
        
        // Calculate shooter position and distance to hub
        Translation3d shooterPos = calculateShooterPosition(robotPose);
        double distanceToHub = shooterPos.getDistance(hubTarget);
        
        // Check if in opponent territory
        double robotX = robotPose.getX();
        boolean inOpponentTerritory = isRed 
            ? robotX < (16.54 - FieldConstants.SMART_SHOOT_MAX_X)
            : robotX > FieldConstants.SMART_SHOOT_MAX_X;
        
        // Decision: pass if too far or in opponent territory
        if (inOpponentTerritory || distanceToHub > FieldConstants.HUB_MAX_SHOOTING_DISTANCE) {
            // Select nearest HPS target
            Translation3d closePass = isRed ? FieldConstants.RED_HPS_CLOSE_PASS : FieldConstants.BLUE_HPS_CLOSE_PASS;
            Translation3d farPass = isRed ? FieldConstants.RED_HPS_FAR_PASS : FieldConstants.BLUE_HPS_FAR_PASS;
            
            double distToClose = shooterPos.getDistance(closePass);
            double distToFar = shooterPos.getDistance(farPass);
            
            if (distToClose < distToFar) {
                return new SmartTarget(closePass, TargetType.HPS_CLOSE);
            } else {
                return new SmartTarget(farPass, TargetType.HPS_FAR);
            }
        }
        
        // Default: shoot at hub
        return new SmartTarget(hubTarget, TargetType.HUB);
    }
    
    /**
     * Calculates shooter position in field coordinates from robot pose.
     */
    private static Translation3d calculateShooterPosition(Pose2d robotPose) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotAngle = robotPose.getRotation().getRadians();
        
        // Transform shooter offset from robot-relative to field-relative
        double fieldX = robotX + PhysicsConstants.SHOOTER_OFFSET_X * Math.cos(robotAngle) 
                               - PhysicsConstants.SHOOTER_OFFSET_Y * Math.sin(robotAngle);
        double fieldY = robotY + PhysicsConstants.SHOOTER_OFFSET_X * Math.sin(robotAngle) 
                               + PhysicsConstants.SHOOTER_OFFSET_Y * Math.cos(robotAngle);
        double fieldZ = PhysicsConstants.SHOOTER_OFFSET_Z;
        
        return new Translation3d(fieldX, fieldY, fieldZ);
    }
    
    /**
     * Calculates required turret angle to point at target.
     * Returns field-relative angle that turret should face.
     * 
     * @param robotPose Current robot pose
     * @param target Target position (3D)
     * @return Required turret angle (field-relative)
     */
    private static Rotation2d calculateTurretAngle(Pose2d robotPose, Translation3d target) {
        Translation3d shooterPos = calculateShooterPosition(robotPose);
        
        // Calculate angle from shooter to target (ignoring Z)
        double dx = target.getX() - shooterPos.getX();
        double dy = target.getY() - shooterPos.getY();
        
        return new Rotation2d(Math.atan2(dy, dx));
    }
    
    /**
     * Finds the closest HPS pass target for current robot position.
     * 
     * @param robotPose Current robot pose
     * @return Closest pass target position
     */
    public static Translation3d getClosestPassTarget(Pose2d robotPose) {
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        
        Translation3d closePass = isRed ? FieldConstants.RED_HPS_CLOSE_PASS : FieldConstants.BLUE_HPS_CLOSE_PASS;
        Translation3d farPass = isRed ? FieldConstants.RED_HPS_FAR_PASS : FieldConstants.BLUE_HPS_FAR_PASS;
        
        Translation3d shooterPos = calculateShooterPosition(robotPose);
        double distToClose = shooterPos.getDistance(closePass);
        double distToFar = shooterPos.getDistance(farPass);
        
        return distToClose < distToFar ? closePass : farPass;
    }
    
    /**
     * Gets the alliance hub target for the current robot alliance.
     * 
     * @return Hub target position
     */
    public static Translation3d getAllianceHub() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return FieldConstants.RED_HUB;
        }
        return FieldConstants.BLUE_HUB;
    }
    
    /**
     * Converts projectile velocity to required flywheel RPS.
     * Uses physics constants for wheel diameter and efficiency.
     * 
     * @param projectileVelocity Desired projectile velocity (m/s)
     * @return Required flywheel velocity (rotations per second)
     */
    private static double velocityToFlywheelRPS(double projectileVelocity) {
        // Assume typical flywheel parameters (these should match FlywheelConstants)
        double wheelDiameter = 0.1016; // 4 inch diameter in meters
        double energyEfficiency = 0.7; // 70% energy transfer
        
        double requiredSurfaceVelocity = projectileVelocity / Math.sqrt(energyEfficiency);
        double wheelCircumference = Math.PI * wheelDiameter;
        return requiredSurfaceVelocity / wheelCircumference;
    }
    
    /**
     * Checks if a shot is feasible from current position to target.
     * 
     * @param robotPose Current robot pose
     * @param target Target position
     * @param maxVelocity Maximum flywheel velocity (m/s)
     * @return true if shot is possible within velocity constraints
     */
    public static boolean isShotFeasible(Pose2d robotPose, Translation3d target, double maxVelocity) {
        Translation3d shooterPos = calculateShooterPosition(robotPose);
        ShooterSimulator.ShootingSolution solution = ShooterSimulator.calculateShot(
            shooterPos, target, maxVelocity
        );
        return solution.isValid;
    }
    
    /**
     * Estimates time to reach shooting position and acquire target.
     * 
     * @param currentPose Current robot pose
     * @param targetPose Desired shooting position
     * @param maxSpeed Maximum robot speed (m/s)
     * @return Estimated time in seconds
     */
    public static double estimateTimeToShot(Pose2d currentPose, Pose2d targetPose, double maxSpeed) {
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double travelTime = distance / maxSpeed;
        double acquisitionTime = 0.5; // Time to stabilize and acquire target
        return travelTime + acquisitionTime;
    }
}
