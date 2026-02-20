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
    
    // Default path constraints (can be overridden per path)
    private static final double DEFAULT_MAX_VELOCITY_MPS = 4.0;
    private static final double DEFAULT_MAX_ACCEL_MPSPS = 3.0;
    private static final double DEFAULT_MAX_ANGULAR_VEL_RPS = Math.PI;
    private static final double DEFAULT_MAX_ANGULAR_ACCEL_RPSPS = Math.PI;
    
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
                    new PIDConstants(5.0, 0.0, 0.0),  // Translation PID
                    new PIDConstants(5.0, 0.0, 0.0)   // Rotation PID
                ),
                config,
                this::shouldFlipPath,           // Should flip for red alliance
                drivetrain                      // Subsystem requirement
            );
            
            isPathPlannerConfigured = true;
            Logger.recordOutput("PathPlanner/Configured", true);
            
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure PathPlanner: " + e.getMessage(), e.getStackTrace());
            isPathPlannerConfigured = false;
            Logger.recordOutput("PathPlanner/Configured", false);
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
        return drivetrain.getState().Pose;
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
        // Apply robot-relative speeds directly to drivetrain
        drivetrain.setControl(
            new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(robotRelativeSpeeds)
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
            return Commands.none();
        }
        
        try {
            currentAutoName = Optional.of(autoName);
            
            return new PathPlannerAuto(autoName)
                .beforeStarting(() -> {
                    Logger.recordOutput("PathPlanner/ActiveAuto", autoName);
                    Logger.recordOutput("PathPlanner/IsRunningAuto", true);
                })
                .finallyDo(() -> {
                    Logger.recordOutput("PathPlanner/IsRunningAuto", false);
                    currentAutoName = Optional.empty();
                });
                
        } catch (Exception e) {
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
