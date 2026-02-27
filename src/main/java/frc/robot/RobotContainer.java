package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.constants.IntakePositionConstants;
import frc.robot.constants.IntakeRollerConstants;
import frc.robot.util.ControllerTelemetry;
import frc.robot.util.FuelSim;
import frc.robot.util.TelemetryThrottle;

public class RobotContainer {
    
    // ==================== CONSTANTS ====================
    private static final boolean DISABLE_INTAKE_DEPLOY_MOTOR = false;
    
    /**
     * Master toggle to disable ALL telemetry and logging
     * Set to true to disable: SmartDashboard updates, AdvantageKit logging, controller telemetry,
     * power monitoring, CAN bus monitoring, drivetrain telemetry, and subsystem periodic telemetry
     */
    public static final boolean DISABLE_ALL_TELEMETRY = false;
    
    /**
     * Toggle to control whether intake retracts during shooting sequence
     * Set to true to retract intake while shooting (default behavior)
     * Set to false to keep intake deployed during shooting
     */
    public static final boolean RETRACT_INTAKE_WHILE_SHOOTING = false;
    
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    
    // Climbing mode: Reduced speed for fine adjustments when climber is deployed
    private static final double CLIMBING_SPEED_REDUCTION = 0.25; // 25% speed when climbing
    
    // ==================== SWERVE REQUESTS ====================
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
      
    // ==================== TELEMETRY ====================
    private final Telemetry logger = new Telemetry(maxSpeed);
    
    // ==================== CONTROLLERS ====================
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    // ==================== SUBSYSTEMS ====================
    // Drivetrain
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    // Vision & Autonomous
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain);
    private final PathPlannerSubsystem pathPlannerSubsystem = new PathPlannerSubsystem(drivetrain);
    
    // Shooter
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();
    private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    
    // Intake & Indexer
    private final IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();
    private final IntakePositionSubsystem intakePositionSubsystem = new IntakePositionSubsystem();
    private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    
    // Climber
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    
    // LEDs
    private final CANdleSubsystem candleSubsystem = new CANdleSubsystem();
    
    // Utilities
    private final ControllerTelemetry controllerTelemetry;
    private final TelemetryThrottle robotContainerTelemetryThrottle = new TelemetryThrottle(0.2);
    
    // ==================== AUTONOMOUS CHOOSER ====================
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        controllerTelemetry = new ControllerTelemetry(joystick);
        
        if (DISABLE_INTAKE_DEPLOY_MOTOR) {
            intakePositionSubsystem.disableDeployMotor();
        }
        
        configureFuelSim();
        configureAutoChooser();
        configureTurretCalibrationButtons();
        configureBindings();
        configureDefaultCommands();
    }
    
    // ==================== CONFIGURATION ====================
    
    /**
     * Configure default commands for subsystems
     */
    private void configureDefaultCommands() {
        // Intake roller runs at default speed only when deployed (within tolerance of deployed position)
        intakeRollerSubsystem.setDefaultCommand(
            Commands.run(() -> {
                double currentPosition = intakePositionSubsystem.getPosition();
                double targetPosition = IntakePositionConstants.EXTENDED_POSITION_ROTATIONS;
                double tolerance = IntakeRollerConstants.DEPLOYED_TOLERANCE_ROTATIONS;
                
                // Check if position is within ±tolerance of deployed position
                boolean isNearDeployed = Math.abs(currentPosition - targetPosition) <= tolerance;
                
                if (isNearDeployed) {
                    intakeRollerSubsystem.setDutyCycle(IntakeRollerConstants.DEFAULT_DUTY_CYCLE);
                } else {
                    intakeRollerSubsystem.stop();
                }
            }, intakeRollerSubsystem)
        );
    }
    
    /**
     * Publishes turret calibration buttons to SmartDashboard
     */
    private void configureTurretCalibrationButtons() {
        if (DISABLE_ALL_TELEMETRY) {
            return; // Skip SmartDashboard updates when telemetry is disabled
        }
        
        // Button to zero turret at current position
        SmartDashboard.putData("Turret: Zero Here", 
            Commands.runOnce(() -> turretSubsystem.zeroTurret(), turretSubsystem)
                .ignoringDisable(true)
                .withName("Zero Turret")
        );
        
        // Button to set turret position to 180 degrees (useful if turret is backwards)
        SmartDashboard.putData("Turret: Set to 180°", 
            Commands.runOnce(() -> {
                // Set encoder position to 180 degrees (0.5 rotations)
                turretSubsystem.setEncoderPosition(Rotation2d.fromDegrees(180.0));
            }, turretSubsystem)
                .ignoringDisable(true)
                .withName("Set Turret to 180°")
        );
    }
    
    private void configureFuelSim() {
        FuelSim.getInstance().registerRobot(
            0.76,  // Robot width (meters)
            0.76,  // Robot length (meters)
            0.30,  // Bumper height (meters)
            () -> drivetrain.getState().Pose,
            () -> drivetrain.getState().Speeds
        );
        FuelSim.getInstance().start();
    }

    private void configureBindings() {
        configureDrivetrainControls();
        configureShooterControls();
        configureIntakeControls();
        configureClimberControls();
        configureVisionControls();
        configurePathPlannerControls();
    }
    
    // ==================== DRIVETRAIN CONTROLS ====================
    
    private void configureDrivetrainControls() {
        // Default command: field-centric drive with reduced speed when climber deployed
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // Apply speed reduction when climber is deployed for fine adjustments
                double speedMultiplier = climberSubsystem.isDeployed() ? CLIMBING_SPEED_REDUCTION : 1.0;
                
                return drive.withVelocityX(-joystick.getLeftY() * maxSpeed * speedMultiplier)
                    .withVelocityY(-joystick.getLeftX() * maxSpeed * speedMultiplier)
                    .withRotationalRate(-joystick.getRightX() * maxAngularRate * speedMultiplier);
            })
        );

        // Idle while disabled
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // A button: X-brake
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        
        // B button: Point wheels at joystick direction
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // SysId routines (back/start + X/Y)
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        if (!DISABLE_ALL_TELEMETRY) {
            drivetrain.registerTelemetry(logger::telemeterize);
        }
    }
    
    // ==================== VISION CONTROLS ====================
    
    private void configureVisionControls() {
        // Left bumper (operator): Reset field-centric heading
        operatorController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
        // Right bumper (operator): Toggle vision processing
        operatorController.rightBumper().onTrue(Commands.runOnce(() -> visionSubsystem.toggleVision()));
    }
    
    // ==================== PATH PLANNER CONTROLS ====================
    
    private void configurePathPlannerControls() {
        // D-pad left: Pathfind to "Far Climb"
        joystick.povLeft().onTrue(pathPlannerSubsystem.pathfindThenFollowPath("Far Climb"));
        
        // D-pad right: Pathfind to "Close Climb"
        joystick.povRight().onTrue(pathPlannerSubsystem.pathfindThenFollowPath("Close Climb"));
    }
    
    // ==================== SHOOTER CONTROLS ====================
    
    private void configureShooterControls() {
        // Hood default command: Manual adjustment OR auto-pack to 0° when idle
        hoodSubsystem.setDefaultCommand(
            Commands.run(() -> {
                // Check if manually adjusting with joystick
                double input = -operatorController.getLeftY();
                if (Math.abs(input) >= 0.08) {
                    // Manual control - adjust hood
                    double squared = input * input * Math.signum(input);
                    double currentAngle = hoodSubsystem.getAngle().getDegrees();
                    hoodSubsystem.setAngle(Rotation2d.fromDegrees(
                        currentAngle + squared * 50.0 * 0.02
                    ));
                } else {
                    // No manual input - pack to 0° (this only runs when default command is active)
                    hoodSubsystem.setAngle(Rotation2d.fromDegrees(0));
                }
            }, hoodSubsystem)
        );
        
        // Turret manual adjustment (operator right stick X)
        turretSubsystem.setDefaultCommand(
            Commands.run(() -> {
                double input = operatorController.getRightX();
                if (Math.abs(input) < 0.08) return;
                
                double squared = input * input * Math.signum(input);
                Rotation2d currentAngle = turretSubsystem.getCurrentAngle();
                Rotation2d adjustment = Rotation2d.fromDegrees(squared * 180.0 * 0.02);
                turretSubsystem.setTargetAngle(currentAngle.plus(adjustment));
            }, turretSubsystem)
        );
        
        // Operator A button: Zero turret to current position
        operatorController.a().onTrue(
            Commands.runOnce(() -> turretSubsystem.zeroTurret(), turretSubsystem)
        );
        
        // Right trigger: Shoot at 60 RPS with turret/hood angles (while held)
        joystick.rightTrigger().whileTrue(
            ShootCommand.shootContinuous(
                flywheelSubsystem, 
                indexerSubsystem, 
                turretSubsystem,
                hoodSubsystem,
                intakeRollerSubsystem,
                intakePositionSubsystem,
                50.0,
                Rotation2d.fromDegrees(355),   // Turret angle: forward
                Rotation2d.fromDegrees(45)   // Hood angle: 45 degrees
            )
        );
        
        // Left trigger: Shoot at 40 RPS with turret/hood angles (while held)
        joystick.leftTrigger().whileTrue(
            ShootCommand.shootContinuous(
                flywheelSubsystem, 
                indexerSubsystem,
                turretSubsystem,
                hoodSubsystem,
                intakeRollerSubsystem,
                intakePositionSubsystem,
                50.0,
                Rotation2d.fromDegrees(180), // Turret angle: forward
                Rotation2d.fromDegrees(30)   // Hood angle: 30 degrees
            )
        );
    }
    
    // ==================== INTAKE CONTROLS ====================
    
    private void configureIntakeControls() {
        // Right bumper (driver): Intake (while held)
        joystick.rightBumper().whileTrue(
            IntakeCommand.intake(intakeRollerSubsystem, intakePositionSubsystem, indexerSubsystem)
        );
        
        // Left bumper (driver): Eject (while held)
        joystick.leftBumper().whileTrue(
            IntakeCommand.eject(intakeRollerSubsystem, indexerSubsystem)
        );
    }
    
    // ==================== CLIMBER CONTROLS ====================
    
    private void configureClimberControls() {
        // Y button (operator): Deploy climber (unspool winch)
        operatorController.y().onTrue(
            ClimberCommand.deployClimber(climberSubsystem, intakePositionSubsystem, intakeRollerSubsystem, 
                indexerSubsystem, flywheelSubsystem, turretSubsystem, hoodSubsystem)
        );
        
        // X button (operator): Store climber (spool winch - actively climb)
        operatorController.x().onTrue(
            ClimberCommand.storeClimber(climberSubsystem, intakePositionSubsystem, intakeRollerSubsystem,
                indexerSubsystem, flywheelSubsystem, turretSubsystem, hoodSubsystem)
        );
    }
    
    // ==================== AUTONOMOUS ====================
    
    /**
     * Configures the autonomous chooser with PathPlanner auto routines
     */
    private void configureAutoChooser() {
        // Default option: Do nothing
        autoChooser.setDefaultOption("None", Commands.none());
        
        // PathPlanner auto routines (from deploy/pathplanner/autos/*.auto files)
        autoChooser.addOption("Example Auto", pathPlannerSubsystem.getAutonomousCommand("Example Auto"));
    
        // Simple test auto
        autoChooser.addOption("Drive Forward", 
            Commands.sequence(
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                ).withTimeout(3.0),
                drivetrain.applyRequest(() -> idle)
            )
        );
        
        // Put chooser on SmartDashboard
        if (!DISABLE_ALL_TELEMETRY) {
            SmartDashboard.putData("Auto Chooser", autoChooser);
        }
    }
    
    public Command getAutonomousCommand() {
        // Return the selected autonomous command from the chooser
        return autoChooser.getSelected();
    }
    
    // ==================== GETTERS ====================
    
    public PathPlannerSubsystem getPathPlannerSubsystem() {
        return pathPlannerSubsystem;
    }
    
    // ==================== PERIODIC ====================
    
    // Flywheel ready state tracking for rumble
    private boolean lastFlywheelReadyState = false;
    
    // Velocity dip detection for orange flash
    private double lastFlywheelVelocity = 0.0;
    private static final double VELOCITY_DIP_THRESHOLD = 5; // RPS drop to trigger flash
    private int orangeFlashCounter = 0;
    private static final int ORANGE_FLASH_DURATION = 10; // cycles (~200ms)
    
    public void periodic() {
        if (!DISABLE_ALL_TELEMETRY) {
            controllerTelemetry.periodic();
            
            // Throttle SmartDashboard updates to prevent loop overruns
            if (robotContainerTelemetryThrottle.shouldUpdate()) {
                // Display climbing mode status (throttled to 5 Hz)
                SmartDashboard.putBoolean("Climbing Mode (Reduced Speed)", climberSubsystem.isDeployed());
            }
        }
        updateVelocityDipDetection();
        updateLEDFeedback();
        updateControllerRumble();
    }
    
    /**
     * Detects velocity dips in flywheel (note being shot)
     */
    private void updateVelocityDipDetection() {
        double currentVelocity = flywheelSubsystem.getVelocityRPS();
        
        // Only check for dips when flywheel is supposed to be spinning
        if (flywheelSubsystem.getTargetVelocityRPS() > 0.1) {
            double velocityDrop = lastFlywheelVelocity - currentVelocity;
            
            // Detect significant velocity drop (note shot)
            if (velocityDrop > VELOCITY_DIP_THRESHOLD && orangeFlashCounter == 0) {
                orangeFlashCounter = ORANGE_FLASH_DURATION; // Start flash
            }
        }
        
        // Countdown flash timer
        if (orangeFlashCounter > 0) {
            orangeFlashCounter--;
        }
        
        lastFlywheelVelocity = currentVelocity;
    }
    
    /**
     * Updates controller rumble based on robot state
     */
    private void updateControllerRumble() {
        // Check if flywheel just reached target speed
        boolean flywheelReady = flywheelSubsystem.atTargetVelocity() 
                                && flywheelSubsystem.getTargetVelocityRPS() > 0.1;
        
        // Trigger rumble when flywheel becomes ready (rising edge)
        if (flywheelReady && !lastFlywheelReadyState) {
            // Short rumble burst to indicate ready to shoot
            joystick.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 1.0);
            operatorController.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 1.0);
            
            // Schedule rumble to stop after 0.3 seconds
            new Thread(() -> {
                try {
                    Thread.sleep(300);
                    joystick.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0);
                    operatorController.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }
        
        lastFlywheelReadyState = flywheelReady;
    }
    
    // ==================== LED FEEDBACK ====================
    
    /**
     * Updates LED colors based on robot state
     * Provides visual feedback for shooting, intake, and other states
     */
    private void updateLEDFeedback() {
        // CRITICAL OVERRIDE: Flywheel overheating - rapid red blink
        if (flywheelSubsystem.isOverheating()) {
            candleSubsystem.setRapidRedBlink();
            return;
        }
        
        // HIGHEST PRIORITY: Climber deployed - slow rainbow (waiting to climb)
        if (climberSubsystem.isFullyDeployed() && climberSubsystem.isDeployed()) {
            candleSubsystem.setSlowRainbow();
            return;
        }
        
        // HIGHEST PRIORITY: Actively climbing - rapid rainbow
        if (climberSubsystem.isDeployed() && !climberSubsystem.isRetracted()) {
            candleSubsystem.setRapidRainbow();
            return;
        }
        
        // HIGH PRIORITY: Velocity dip detected - orange flash (note scored!)
        if (orangeFlashCounter > 0) {
            candleSubsystem.setRGB(255, 165, 0); // Bright orange flash
            return;
        }
        
        // Priority 1: Shooting state (highest priority)
        if (flywheelSubsystem.getTargetVelocityRPS() > 0.1) {
            if (flywheelSubsystem.atTargetVelocity()) {
                // Flywheel ready - bright pulsing green (fast pulse = ready to shoot!)
                candleSubsystem.setRGBWithModulation(0, 255, 0, 3.0, 0.6, 1.0);
            } else {
                // Flywheel spinning up - breathing yellow (slower = wait)
                candleSubsystem.setRGBWithModulation(255, 255, 0, 1.5, 0.4, 0.9);
            }
            return;
        }
        
        // Priority 2: Intake running (check if velocity is non-zero)
        if (Math.abs(intakeRollerSubsystem.getVelocity()) > 1.0) {
            // Intake active - pulsing blue
            candleSubsystem.setRGBWithModulation(0, 0, 255, 2.5, 0.5, 1.0);
            return;
        }
        
        // Priority 3: Aiming (check if turret/hood are at target positions)
        if (turretSubsystem.atTarget() && hoodSubsystem.atTargetAngle()) {
            // Aimed and ready - bright pulsing purple
            candleSubsystem.setRGBWithModulation(128, 0, 255, 2.0, 0.7, 1.0);
            return;
        }
        
        // Default: Robot idle - slow breathing dim white
        candleSubsystem.setRGBWithModulation(80, 80, 80, 0.5, 0.1, 0.3);
    }
}
