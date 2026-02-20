package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.PhysicsConstants;

/**
 * Streamlined shooter trajectory simulation utility.
 * This class provides physics-based trajectory calculations for shooter systems
 * without any physical robot hardware interaction.
 * 
 * Key features:
 * - Trajectory calculation with air resistance
 * - Optimal launch angle determination
 * - Shot velocity and time-of-flight calculations
 */
public class ShooterSimulator {
    
    /**
     * Result of a trajectory simulation
     */
    public static class TrajectoryResult {
        public final double flightTime;        // Time to reach target (seconds)
        public final double maxHeight;         // Maximum height reached (meters)
        public final double entryAngle;        // Entry angle at target (degrees, negative = downward)
        public final boolean hitTarget;        // True if trajectory reaches target
        
        public TrajectoryResult(double flightTime, double maxHeight, double entryAngle, boolean hitTarget) {
            this.flightTime = flightTime;
            this.maxHeight = maxHeight;
            this.entryAngle = entryAngle;
            this.hitTarget = hitTarget;
        }
    }
    
    /**
     * Solution for optimal shooting parameters
     */
    public static class ShootingSolution {
        public final double launchVelocity;    // Required launch velocity (m/s)
        public final Rotation2d launchAngle;   // Optimal launch angle
        public final double flightTime;        // Time to reach target (seconds)
        public final double entryAngle;        // Entry angle at target (degrees)
        public final boolean isValid;          // True if solution is physically possible
        
        public ShootingSolution(double velocity, Rotation2d angle, double time, double entry, boolean valid) {
            this.launchVelocity = velocity;
            this.launchAngle = angle;
            this.flightTime = time;
            this.entryAngle = entry;
            this.isValid = valid;
        }
    }
    
    /**
     * Calculates the optimal shooting solution for a given target.
     * Uses iterative trajectory simulation to find the best launch angle.
     * 
     * @param shooterPosition Starting position of projectile (meters)
     * @param targetPosition Target position (meters)
     * @param launchVelocity Initial velocity of projectile (m/s)
     * @return ShootingSolution with optimal parameters
     */
    public static ShootingSolution calculateShot(Translation3d shooterPosition, 
                                                   Translation3d targetPosition,
                                                   double launchVelocity) {
        // Calculate horizontal and vertical distances
        double dx = targetPosition.getX() - shooterPosition.getX();
        double dy = targetPosition.getY() - shooterPosition.getY();
        double horizontalDistance = Math.sqrt(dx * dx + dy * dy);
        double verticalDistance = targetPosition.getZ() - shooterPosition.getZ();
        
        // Check if velocity is sufficient (simple ballistic check)
        double minVelocity = Math.sqrt(PhysicsConstants.GRAVITY * horizontalDistance);
        if (launchVelocity < minVelocity) {
            return new ShootingSolution(launchVelocity, Rotation2d.fromDegrees(45), 0, 0, false);
        }
        
        // Search for optimal angle using binary search
        double bestAngle = 45.0;
        double bestError = Double.MAX_VALUE;
        TrajectoryResult bestResult = null;
        
        // Try angles from 15° to 75°
        for (double angleDeg = 15.0; angleDeg <= 75.0; angleDeg += 0.5) {
            double angleRad = Math.toRadians(angleDeg);
            TrajectoryResult result = simulateTrajectory(
                launchVelocity, angleRad, horizontalDistance, verticalDistance
            );
            
            if (result.hitTarget && result.flightTime < bestError) {
                bestError = result.flightTime;
                bestAngle = angleDeg;
                bestResult = result;
            }
        }
        
        if (bestResult != null && bestResult.hitTarget) {
            return new ShootingSolution(
                launchVelocity,
                Rotation2d.fromDegrees(bestAngle),
                bestResult.flightTime,
                bestResult.entryAngle,
                true
            );
        }
        
        return new ShootingSolution(launchVelocity, Rotation2d.fromDegrees(bestAngle), 0, 0, false);
    }
    
    /**
     * Simulates a projectile trajectory with air resistance.
     * Uses numerical integration to calculate the path.
     * 
     * @param velocity Initial velocity (m/s)
     * @param angle Launch angle (radians)
     * @param targetX Horizontal distance to target (meters)
     * @param targetZ Vertical distance to target (meters)
     * @return TrajectoryResult with simulation data
     */
    public static TrajectoryResult simulateTrajectory(double velocity, double angle, 
                                                       double targetX, double targetZ) {
        final double dt = 0.001; // 1ms time step
        final double maxTime = 10.0; // Maximum simulation time
        final double targetTolerance = 0.1; // 10cm tolerance
        
        // Initial conditions
        double x = 0, z = 0;
        double vx = velocity * Math.cos(angle);
        double vz = velocity * Math.sin(angle);
        
        double maxHeight = 0;
        double time = 0;
        
        // Drag coefficient calculation
        double dragCoeff = 0.5 * PhysicsConstants.AIR_DENSITY * 
                          PhysicsConstants.DRAG_COEFFICIENT * 
                          PhysicsConstants.PROJECTILE_CROSS_SECTION / 
                          PhysicsConstants.PROJECTILE_MASS_KG;
        
        // Simulation loop
        while (time < maxTime && z >= -1.0) { // Stop if projectile goes below ground
            // Current velocity magnitude
            double v = Math.sqrt(vx * vx + vz * vz);
            
            // Drag force
            double dragX = -dragCoeff * v * vx;
            double dragZ = -dragCoeff * v * vz;
            
            // Acceleration (gravity + drag)
            double ax = dragX;
            double az = -PhysicsConstants.GRAVITY + dragZ;
            
            // Update velocity
            vx += ax * dt;
            vz += az * dt;
            
            // Update position
            x += vx * dt;
            z += vz * dt;
            
            // Track max height
            if (z > maxHeight) {
                maxHeight = z;
            }
            
            // Check if we've reached the target
            if (Math.abs(x - targetX) < targetTolerance) {
                double heightError = Math.abs(z - targetZ);
                if (heightError < targetTolerance) {
                    // Calculate entry angle
                    double entryAngleRad = Math.atan2(vz, vx);
                    double entryAngleDeg = Math.toDegrees(entryAngleRad);
                    
                    return new TrajectoryResult(time, maxHeight, entryAngleDeg, true);
                }
            }
            
            // Check if we've passed the target horizontally
            if (x > targetX + targetTolerance) {
                break;
            }
            
            time += dt;
        }
        
        // Did not hit target
        double entryAngleRad = Math.atan2(vz, vx);
        double entryAngleDeg = Math.toDegrees(entryAngleRad);
        return new TrajectoryResult(time, maxHeight, entryAngleDeg, false);
    }
    
    /**
     * Estimates the required flywheel velocity to achieve a target shot velocity.
     * Accounts for energy transfer efficiency.
     * 
     * @param targetShotVelocity Desired projectile velocity (m/s)
     * @param wheelDiameter Flywheel diameter (meters)
     * @param energyEfficiency Energy transfer efficiency (0.0 to 1.0)
     * @return Required flywheel surface velocity (m/s)
     */
    public static double calculateRequiredFlywheelVelocity(double targetShotVelocity,
                                                           double wheelDiameter,
                                                           double energyEfficiency) {
        // Account for energy transfer efficiency
        return targetShotVelocity / Math.sqrt(energyEfficiency);
    }
    
    /**
     * Converts flywheel RPS to projectile launch velocity.
     * 
     * @param flywheelRPS Flywheel rotations per second
     * @param wheelDiameter Flywheel diameter (meters)
     * @param energyEfficiency Energy transfer efficiency (0.0 to 1.0)
     * @return Estimated projectile velocity (m/s)
     */
    public static double flywheelToProjectileVelocity(double flywheelRPS,
                                                       double wheelDiameter,
                                                       double energyEfficiency) {
        double wheelCircumference = Math.PI * wheelDiameter;
        double surfaceVelocity = flywheelRPS * wheelCircumference;
        return surfaceVelocity * Math.sqrt(energyEfficiency);
    }
    
    /**
     * Calculates the time to reach target given initial velocity and angle.
     * Simplified calculation without air resistance for quick estimates.
     * 
     * @param velocity Initial velocity (m/s)
     * @param angle Launch angle (radians)
     * @param horizontalDistance Distance to target (meters)
     * @return Estimated time of flight (seconds)
     */
    public static double estimateFlightTime(double velocity, double angle, double horizontalDistance) {
        double vx = velocity * Math.cos(angle);
        if (vx <= 0) return 0;
        return horizontalDistance / vx;
    }
}
