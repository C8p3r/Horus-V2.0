// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PathPlannerSubsystem;

import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ControllerTelemetry;
import frc.robot.util.FuelSim;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // ==================== CONTROL SCHEME TOGGLE ====================
    // Set to true to disable the intake deploy motor (for testing - saves wear and tear)
    // When disabled, intake roller still works but deploy/retract commands are ignored
    private static final boolean DISABLE_INTAKE_DEPLOY_MOTOR = true;  // Change to true for testing
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    // Vision subsystem with dual Limelights for field localization
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain);
    
    // PathPlanner subsystem for autonomous and on-the-fly pathing
    private final PathPlannerSubsystem pathPlannerSubsystem = new PathPlannerSubsystem(drivetrain);
    
    // Shooter subsystems - streamlined hardware control
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();
    private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    
    // Intake and indexer subsystems for fuel handling
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    
    // Controller telemetry for debugging
    private final ControllerTelemetry controllerTelemetry;

    public RobotContainer() {
        // Initialize controller telemetry
        controllerTelemetry = new ControllerTelemetry(joystick);
        
        // Apply intake deploy motor toggle setting
        if (DISABLE_INTAKE_DEPLOY_MOTOR) {
            intakeSubsystem.disableDeployMotor();
        }
        
        // Set up field-oriented targeting for turret (always active)
        turretSubsystem.setPoseSupplier(() -> drivetrain.getState().Pose);
        
        configureFuelSim();
        configureBindings();
    }

    private void configureFuelSim() {
        // Initialize FuelSim with robot parameters (only in simulation)
        FuelSim.getInstance().registerRobot(
            0.76,  // Robot width in meters (30 inches with bumpers)
            0.76,  // Robot length in meters (30 inches with bumpers)
            0.30,  // Bumper height in meters (~12 inches)
            () -> drivetrain.getState().Pose,  // Robot pose supplier
            () -> drivetrain.getState().Speeds  // Field-relative speeds supplier (note capital S)
        );
        
        // Start FuelSim
        FuelSim.getInstance().start();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
        // Toggle vision processing with right bumper
        joystick.rightBumper().onTrue(Commands.runOnce(() -> visionSubsystem.toggleVision()));
        
        // ==================== SHOOTING CONTROL SCHEMES ====================
        // NOTE: On-the-fly shooting subsystem has been converted to simulation utilities.
        // For hardware control, use the individual FlywheelSubsystem, HoodSubsystem, and TurretSubsystem directly.
        // For trajectory calculations, use OnTheFlyCalculator and ShooterSimulator in util package.
        
        // TODO: Implement new shooting control using streamlined subsystems
        // Example:
        // joystick.rightTrigger().whileTrue(Commands.run(() -> {
        //     // Use OnTheFlyCalculator to get shooting solution
        //     // Set flywheel, hood, and turret to calculated values
        // }));
        
        // CLIMB PATHS - D-pad for pathfinding to climb positions
        // Right D-pad (90°) - Pathfind then follow "Far Climb"
        joystick.povLeft().onTrue(pathPlannerSubsystem.pathfindThenFollowPath("Far Climb"));
        
        // Left D-pad (270°) - Pathfind then follow "Close Climb"
        joystick.povRight().onTrue(pathPlannerSubsystem.pathfindThenFollowPath("Close Climb"));
        
        // ==================== OPERATOR MANUAL ADJUSTMENT CONTROLS ====================
        // Operator controller (port 1) for manual hood and turret adjustment
        // Left stick Y-axis - Hood angle control (up = increase angle, down = decrease angle)
        // Right stick X-axis - Turret angle control (right = clockwise, left = counterclockwise)
        
        // Hood manual adjustment with left stick Y-axis (continuous while held)
        // Rate: 50 degrees per second with squared input curve for smooth, precise control
        hoodSubsystem.setDefaultCommand(
            Commands.run(() -> {
                double leftY = -operatorController.getLeftY(); // Negative because Y is inverted
                
                // Apply deadband
                if (Math.abs(leftY) < 0.08) {
                    leftY = 0.0;
                }
                
                // Square the input while preserving sign for smoother control
                double sign = Math.signum(leftY);
                double squared = leftY * leftY * sign;
                
                // Apply rate (50 deg/sec * squared input * 20ms period)
                double currentAngle = hoodSubsystem.getAngle().getDegrees();
                hoodSubsystem.setAngle(edu.wpi.first.math.geometry.Rotation2d.fromDegrees(
                    currentAngle + squared * 50.0 * 0.02
                ));
            }, hoodSubsystem)
        );
        
        // Turret manual adjustment with right stick X-axis (continuous while held)
        // Sets field-oriented heading target that is maintained by stabilization
        // Rate: 180 degrees per second with squared input curve for smooth, responsive control
        turretSubsystem.setDefaultCommand(
            Commands.run(() -> {
                double rightX = operatorController.getRightX();
                
                // Apply deadband
                if (Math.abs(rightX) < 0.08) {
                    rightX = 0.0;
                }
                
                // Square the input while preserving sign for smoother control
                double sign = Math.signum(rightX);
                double squared = rightX * rightX * sign;
                
                // Apply rate (180 deg/sec * squared input * 20ms period)
                // Adjusts field-oriented target - turret will maintain this heading
                turretSubsystem.adjustFieldOrientedTarget(squared * 180.0 * 0.02);
            }, turretSubsystem)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Gets the autonomous command.
     * Uses PathPlanner autonomous if available, otherwise falls back to simple drive forward.
     * 
     * To use PathPlanner autos:
     * 1. Create autonomous routines in PathPlanner GUI
     * 2. Save them to src/main/deploy/pathplanner/autos/
     * 3. Update this method to load your desired auto by name
     * 
     * @return The autonomous command
     */
    public Command getAutonomousCommand() {
        // OPTION 1: Use PathPlanner autonomous
        // Uncomment and specify your auto name:
        // return pathPlannerSubsystem.getAutonomousCommand("YourAutoName");
        
        // OPTION 2: Use a specific PathPlanner path
        // return pathPlannerSubsystem.followPath("YourPathName");
        
        // OPTION 3: Fallback - simple drive forward
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
    
    /**
     * Gets the PathPlanner subsystem for use in other commands
     * 
     * @return The PathPlanner subsystem
     */
    public PathPlannerSubsystem getPathPlannerSubsystem() {
        return pathPlannerSubsystem;
    }
    
    /**
     * Called periodically to update telemetry systems.
     * Should be called from Robot.robotPeriodic()
     */
    public void periodic() {
        controllerTelemetry.periodic();
    }
}
