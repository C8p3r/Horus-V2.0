package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;

/**
 * PathPlanner subsystem for autonomous path following and on-the-fly path generation.
 * 
 * Features:
 * - Autonomous path following from PathPlanner GUI
 * - On-the-fly path generation to specific poses
 * - Holonomic path following with rotation control
 * - Integration with swerve drivetrain
 * - Real-time telemetry and logging
 * 
 * Usage:
 * <pre>
 * // Follow a pre-made path
 * Command followPath = pathPlannerSubsystem.followPath("MyPath");
 * 
 * // Generate path on the fly to target
 * Command pathToTarget = pathPlannerSubsystem.pathfindToPose(targetPose);
 * 
 * // Load autonomous routine
 * Command auto = pathPlannerSubsystem.getAutonomousCommand("MyAuto");
 * </pre>
 */
public class PathPlannerSubsystem extends SubsystemBase {
    
    private final CommandSwerveDrivetrain drivetrain;
    
    private boolean isPathPlannerConfigured = false;
    private Optional<PathPlannerPath> currentPath = Optional.empty();
    private Optional<String> currentAutoName = Optional.empty();
    
    // ==================== PATHPLANNER TUNING CONSTANTS ====================
    // These constants define the behavior of autonomous path following
    
    // Default path constraints (can be overridden per path in PathPlanner GUI)
    private static final double DEFAULT_MAX_VELOCITY_MPS = 4.0;         // Max velocity during path (m/s)
    private static final double DEFAULT_MAX_ACCEL_MPSPS = 3.0;          // Max acceleration (m/s²)
    private static final double DEFAULT_MAX_ANGULAR_VEL_RPS = Math.PI;  // Max rotational velocity (rad/s)
    private static final double DEFAULT_MAX_ANGULAR_ACCEL_RPSPS = Math.PI; // Max rotational acceleration (rad/s²)
    
    // PPHolonomicDriveController PID tuning
    // These PIDs control how the robot corrects position and rotation errors while following the path
    private static final double TRANSLATION_P = 0.0;    // Position correction gain - very conservative to prevent overshoot
    private static final double TRANSLATION_I = 0.0;    // Integral correction - typically 0 for trajectory following
    private static final double TRANSLATION_D = 0.0;    // Derivative (dampening) - strong dampening to prevent overshoot
    
    private static final double ROTATION_P = 0.0;       // Rotation correction gain - very conservative
    private static final double ROTATION_I = 0.0;       // Integral correction - typically 0 for trajectory following
    private static final double ROTATION_D = 0.0;       // Derivative (dampening) - strong dampening for rotation
    
    /**
     * Creates a new PathPlannerSubsystem
     * 
     * @param drivetrain The swerve drivetrain to control
     */
 public PathPlannerSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Configure PathPlanner AutoBuilder
        configurePathPlanner();
    }
    
    /**
     * Configures PathPlanner's AutoBuilder for this robot
     */
    private void configurePathPlanner() {
        try {
            // Get robot config from file or create default
            RobotConfig config = getRobotConfig();
            
            // Configure AutoBuilder with holonomic drive controller
            AutoBuilder.configure(
                this::getPose,                  // Pose supplier
                this::resetPose,                // Pose reset consumer
                this::getChassisSpeeds,         // ChassisSpeeds supplier
                this::driveRobotRelative,       // ChassisSpeeds consumer (robot-relative)
                new PPHolonomicDriveController(
                    new PIDConstants(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D),  // Translation PID
                    new PIDConstants(ROTATION_P, ROTATION_I, ROTATION_D)             // Rotation PID
                ),
                config,
                this::shouldFlipPath,           // Should flip for red alliance
                drivetrain                      // Subsystem requirement
            );
            
            isPathPlannerConfigured = true;
            Logger.recordOutput("PathPlanner/Configured", true);
            
            // Log configuration for debugging
            System.out.println("[PathPlanner] Successfully configured AutoBuilder");
            System.out.println("[PathPlanner] Translation PID: P=" + TRANSLATION_P + " D=" + TRANSLATION_D);
            System.out.println("[PathPlanner] Rotation PID: P=" + ROTATION_P + " D=" + ROTATION_D);
            System.out.println("[PathPlanner] Default Max Velocity: " + DEFAULT_MAX_VELOCITY_MPS + " m/s");
            System.out.println("[PathPlanner] Default Max Acceleration: " + DEFAULT_MAX_ACCEL_MPSPS + " m/s²");
            
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure PathPlanner: " + e.getMessage(), e.getStackTrace());
            isPathPlannerConfigured = false;
            Logger.recordOutput("PathPlanner/Configured", false);
            System.out.println("[PathPlanner] ERROR: Failed to configure AutoBuilder: " + e.getMessage());
        }
    }
    
    /**
     * Gets the robot configuration for PathPlanner
     * Tries to load from deploy directory
     */
    private RobotConfig getRobotConfig() {
        try {
            // Load from PathPlanner GUI settings file
            // This should be configured in the PathPlanner GUI application
            return RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load PathPlanner robot config! " +
                "Please configure robot settings in PathPlanner GUI and save to deploy folder. " +
                "Error: " + e.getMessage(),
                e.getStackTrace()
            );
            throw new RuntimeException("PathPlanner robot config not found", e);
        }
    }
    
    /**
     * Gets current robot pose from drivetrain odometry
     */
    private Pose2d getPose() {
        return drivetrain.getPose();
    }
    
    /**
     * Resets robot pose in drivetrain odometry
     */
    private void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }
    
    /**
     * Gets current chassis speeds from drivetrain
     */
    private ChassisSpeeds getChassisSpeeds() {
        return drivetrain.getState().Speeds;
    }
    
    /**
     * Drives the robot with robot-relative speeds
     * This is the core output consumer for PathPlanner
     */
    private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        // Apply robot-relative speeds directly to drivetrain with open loop voltage control
        System.out.printf("[PathPlanner] Driving: vx=%.2f, vy=%.2f, omega=%.2f%n", 
            robotRelativeSpeeds.vxMetersPerSecond, 
            robotRelativeSpeeds.vyMetersPerSecond, 
            robotRelativeSpeeds.omegaRadiansPerSecond);
        
        drivetrain.setControl(
            new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(robotRelativeSpeeds)
                .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
        );
    }
    
    /**
     * Determines if path should be flipped for red alliance
     */
    private boolean shouldFlipPath() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
    
    /**
     * Follows a pre-made PathPlanner path by name
     * 
     * @param pathName Name of the path file (without .path extension)
     * @return Command to follow the path
     */
    public Command followPath(String pathName) {
        return followPath(pathName, DEFAULT_MAX_VELOCITY_MPS, DEFAULT_MAX_ACCEL_MPSPS);
    }
    
    /**
     * Follows a pre-made PathPlanner path with custom constraints
     * 
     * @param pathName Name of the path file (without .path extension)
     * @param maxVelocity Maximum velocity in m/s
     * @param maxAcceleration Maximum acceleration in m/s^2
     * @return Command to follow the path
     */
    public Command followPath(String pathName, double maxVelocity, double maxAcceleration) {
        if (!isPathPlannerConfigured) {
            DriverStation.reportError("PathPlanner not configured! Cannot follow path.", false);
            return Commands.none();
        }
        
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            currentPath = Optional.of(path);
            
            // AutoBuilder will handle alliance flipping automatically via shouldFlipPath()
            return AutoBuilder.followPath(path)
                .beforeStarting(() -> {
                    Logger.recordOutput("PathPlanner/ActivePath", pathName);
                    Logger.recordOutput("PathPlanner/IsFollowingPath", true);
                })
                .finallyDo(() -> {
                    Logger.recordOutput("PathPlanner/IsFollowingPath", false);
                    currentPath = Optional.empty();
                });
            
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path: " + pathName, e.getStackTrace());
            return Commands.none();
        }
    }
    
    /**
     * Generates a path on-the-fly to the target pose
     * Uses PathPlanner's pathfinding with the NavGrid
     * 
     * @param targetPose The target pose to path to
     * @return Command to path to the target
     */
    public Command pathfindToPose(Pose2d targetPose) {
        return pathfindToPose(targetPose, new PathConstraints(
            DEFAULT_MAX_VELOCITY_MPS,
            DEFAULT_MAX_ACCEL_MPSPS,
            DEFAULT_MAX_ANGULAR_VEL_RPS,
            DEFAULT_MAX_ANGULAR_ACCEL_RPSPS
        ));
    }
    
    /**
     * Generates a path on-the-fly to the target pose with custom constraints
     * 
     * @param targetPose The target pose to path to
     * @param constraints Path constraints for generation
     * @return Command to path to the target
     */
    public Command pathfindToPose(Pose2d targetPose, PathConstraints constraints) {
        if (!isPathPlannerConfigured) {
            DriverStation.reportError("PathPlanner not configured! Cannot pathfind.", false);
            return Commands.none();
        }
        
        return AutoBuilder.pathfindToPose(targetPose, constraints)
            .beforeStarting(() -> {
                Logger.recordOutput("PathPlanner/TargetPose", targetPose);
                Logger.recordOutput("PathPlanner/IsPathfinding", true);
            })
            .finallyDo(() -> {
                Logger.recordOutput("PathPlanner/IsPathfinding", false);
            });
    }
    
    /**
     * Pathfind to a pose, then follow a path from that position
     * Useful for autonomous routines that start from variable positions
     * 
     * @param pathName Name of the path to follow after pathfinding
     * @return Command sequence
     */
    public Command pathfindThenFollowPath(String pathName) {
        if (!isPathPlannerConfigured) {
            DriverStation.reportError("PathPlanner not configured! Cannot pathfind.", false);
            return Commands.none();
        }
        
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            currentPath = Optional.of(path);
            
            // AutoBuilder will handle alliance flipping automatically via shouldFlipPath()
            return AutoBuilder.pathfindThenFollowPath(
                path,
                new PathConstraints(
                    DEFAULT_MAX_VELOCITY_MPS,
                    DEFAULT_MAX_ACCEL_MPSPS,
                    DEFAULT_MAX_ANGULAR_VEL_RPS,
                    DEFAULT_MAX_ANGULAR_ACCEL_RPSPS
                )
            )
            .beforeStarting(() -> {
                Logger.recordOutput("PathPlanner/ActivePath", pathName);
                Logger.recordOutput("PathPlanner/IsPathfinding", true);
            })
            .finallyDo(() -> {
                Logger.recordOutput("PathPlanner/IsPathfinding", false);
                Logger.recordOutput("PathPlanner/IsFollowingPath", false);
                currentPath = Optional.empty();
            });
                
        } catch (Exception e) {
            DriverStation.reportError("Failed to pathfind then follow: " + pathName, e.getStackTrace());
            return Commands.none();
        }
    }
    
    /**
     * Gets an autonomous command by name from PathPlanner autos
     * 
     * @param autoName Name of the auto (without .auto extension)
     * @return The autonomous command
     */
    public Command getAutonomousCommand(String autoName) {
        if (!isPathPlannerConfigured) {
            DriverStation.reportError("PathPlanner not configured! Cannot load auto.", false);
            System.out.println("[PathPlanner] ERROR: PathPlanner not configured!");
            return Commands.none();
        }
        
        try {
            System.out.println("[PathPlanner] Loading autonomous: " + autoName);
            currentAutoName = Optional.of(autoName);
            
            Command autoCommand = new PathPlannerAuto(autoName);
            System.out.println("[PathPlanner] Successfully created PathPlannerAuto for: " + autoName);
            
            return autoCommand
                .beforeStarting(() -> {
                    System.out.println("[PathPlanner] Starting autonomous: " + autoName);
                    Logger.recordOutput("PathPlanner/ActiveAuto", autoName);
                    Logger.recordOutput("PathPlanner/IsRunningAuto", true);
                })
                .finallyDo(() -> {
                    System.out.println("[PathPlanner] Finished autonomous: " + autoName);
                    Logger.recordOutput("PathPlanner/IsRunningAuto", false);
                    currentAutoName = Optional.empty();
                });
                
        } catch (Exception e) {
            System.out.println("[PathPlanner] ERROR loading auto " + autoName + ": " + e.getMessage());
            e.printStackTrace();
            DriverStation.reportError("Failed to load auto: " + autoName, e.getStackTrace());
            return Commands.none();
        }
    }
    
    /**
     * Stops any active path following
     */
    public void stopPath() {
        drivetrain.setControl(new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake());
        currentPath = Optional.empty();
        Logger.recordOutput("PathPlanner/IsFollowingPath", false);
        Logger.recordOutput("PathPlanner/IsPathfinding", false);
    }
    
    @Override
    public void periodic() {
        // Log current state
        Logger.recordOutput("PathPlanner/CurrentPose", getPose());
        Logger.recordOutput("PathPlanner/ChassisSpeeds", getChassisSpeeds());
        
        // Log active path info if available
        currentPath.ifPresent(path -> {
            Logger.recordOutput("PathPlanner/PathName", path.toString());
        });
        
        currentAutoName.ifPresent(autoName -> {
            Logger.recordOutput("PathPlanner/AutoName", autoName);
        });
    }
}
