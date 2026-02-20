package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.PhysicsConstants;
import java.util.function.Supplier;

/**
 * Visualizer for shooter fuel launches using FuelSim advanced physics.
 * Spawns fuel into the FuelSim simulation when shots are fired.
 */
public class ShooterVisualizer {
    private final Supplier<Pose3d> robotPoseSupplier;

    /**
     * Creates a new ShooterVisualizer
     * @param robotPoseSupplier Supplier for robot 3D pose
     */
    public ShooterVisualizer(Supplier<Pose3d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
    }

    /**
     * Call this periodically to check for shots and spawn fuel
     * Call this whenever a shot is fired (from a command or trigger)
     * @param velocity Shot velocity in m/s
     * @param hoodAngleDegrees Hood angle in degrees  
     * @param turretAngleRadians Turret angle in radians (robot-relative)
     */
     
    public void launchFuel(double velocity, double hoodAngleDegrees, double turretAngleRadians) {
        // Get robot pose
        Pose3d robotPose = robotPoseSupplier.get();
        
        // Calculate shooter position offset (forward and up from robot center)
        Translation3d shooterOffset = new Translation3d(
            PhysicsConstants.SHOOTER_OFFSET_X,
            PhysicsConstants.SHOOTER_OFFSET_Y,
            PhysicsConstants.SHOOTER_OFFSET_Z
        );
        
        // Rotate offset by robot rotation
        Translation3d rotatedOffset = shooterOffset.rotateBy(
            new Rotation3d(0, 0, robotPose.getRotation().getZ())
        );
        
        // Calculate shooter world position
        Translation3d shooterPosition = robotPose.getTranslation().plus(rotatedOffset);
        
        // Get shooter velocity and angle (passed as parameters)
        double shotVelocity = velocity; // m/s
        double launchAngleDeg = 90.0 - hoodAngleDegrees; // Convert hood to launch angle
        double launchAngleRad = Math.toRadians(launchAngleDeg);
        
        // Get turret angle (shooter direction)
        double turretAngleRad = turretAngleRadians; // radians, robot-relative
        double worldTurretAngle = robotPose.getRotation().getZ() + turretAngleRad;
        
        // Calculate velocity components
        double vx = shotVelocity * Math.cos(launchAngleRad) * Math.cos(worldTurretAngle);
        double vy = shotVelocity * Math.cos(launchAngleRad) * Math.sin(worldTurretAngle);
        double vz = shotVelocity * Math.sin(launchAngleRad);
        
        Translation3d launchVelocity = new Translation3d(vx, vy, vz);
        
        // Spawn fuel in FuelSim
        FuelSim.getInstance().spawnFuel(shooterPosition, launchVelocity);
    }

    /**
     * Initialize FuelSim integration - call this once during robot initialization
     * @param drivebaseWidth Robot width in meters
     * @param drivebaseLength Robot length in meters  
     * @param bumperHeight Bumper height in meters
     * @param robotPoseSupplier Supplier for robot 2D pose
     * @param fieldSpeedsSupplier Supplier for field-relative chassis speeds
     */
    public static void initializeFuelSim(
            double drivebaseWidth,
            double drivebaseLength,
            double bumperHeight,
            Supplier<edu.wpi.first.math.geometry.Pose2d> robotPoseSupplier,
            Supplier<edu.wpi.first.math.kinematics.ChassisSpeeds> fieldSpeedsSupplier) {
        
        FuelSim sim = FuelSim.getInstance();
        
        // Register robot for collisions
        sim.registerRobot(
            drivebaseWidth,
            drivebaseLength,
            bumperHeight,
            robotPoseSupplier,
            fieldSpeedsSupplier
        );
        
        // Spawn starting fuel
        sim.spawnStartingFuel();
        
        // Start simulation
        sim.start();
    }
}
