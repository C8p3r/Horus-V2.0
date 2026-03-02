package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
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

import frc.robot.util.FuelSim;
import frc.robot.util.ShootingCalculator;
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
    
    /**
     * Shooter calibration mode toggle
     * Set to true for MANUAL CALIBRATION MODE (use dashboard sliders to tune)
     * Set to false for AUTO MODE (use interpolated values from calibration table)
     * 
     * MANUAL MODE: Hood and flywheel follow ManualHood/ManualFlywheel sliders
     * AUTO MODE: Hood and flywheel calculated from distance using calibration points
     * 
     * Note: This can also be toggled via dashboard (ShooterCalibration/CalibrationMode)
     */
    public static final boolean SHOOTER_CALIBRATION_MODE = true;
    
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
    
    // Shooter (turret needs robot pose and chassis speeds for predictive tracking)
    private final TurretSubsystem turretSubsystem = new TurretSubsystem(
        () -> drivetrain.getState().Pose,
        () -> drivetrain.getState().Speeds
    );
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
    
    //UTIL
    private final TelemetryThrottle robotContainerTelemetryThrottle = new TelemetryThrottle(0.2);
    
    // ==================== AUTONOMOUS CHOOSER ====================
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
     
        // Connect vision subsystem to drivetrain for vision-primary pose estimation
        drivetrain.setVisionSubsystem(visionSubsystem);
    
        if (DISABLE_INTAKE_DEPLOY_MOTOR) {
            intakePositionSubsystem.disableDeployMotor();
        }
        
        // Set initial calibration mode from constant
        ShootingCalculator.getCalibration().setCalibrationMode(SHOOTER_CALIBRATION_MODE);
        
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
        // Default command does nothing - intake subsystems maintain their last commanded state
        // The TalonFX motors will hold their last setpoint until commanded otherwise
        intakeRollerSubsystem.setDefaultCommand(
            Commands.run(() -> {
                // Empty run command - just requires the subsystem so other commands can interrupt
                // Motors maintain their last duty cycle or stop command
            }, intakeRollerSubsystem)
        );
        
        indexerSubsystem.setDefaultCommand(
            Commands.run(() -> {
                // Empty run command - maintains last commanded state
            }, indexerSubsystem)
        );
        
        // Turret default command: Continuously track the current target
        // The target is set via button presses (see configureShooterControls)
        turretSubsystem.setDefaultCommand(
            Commands.run(() -> {
                // Tracking logic is handled automatically in TurretSubsystem.periodic()
                // This command just ensures the subsystem is always required
            }, turretSubsystem).withName("Turret Tracking")
        );
        
        // Hood default command: Follow calibration system values (manual or auto)
        hoodSubsystem.setDefaultCommand(
            Commands.run(() -> {
                var calibration = ShootingCalculator.getCalibration();
                
                // In calibration mode, ALWAYS use manual values regardless of target
                if (calibration.isCalibrationMode()) {
                    double hoodAngle = calibration.getManualHoodAngle();
                    hoodSubsystem.setAngle(Rotation2d.fromDegrees(hoodAngle));
                    System.out.println("[HOOD] Calibration Mode: " + hoodAngle + "°");
                } else {
                    // Auto mode - require target
                    var target = turretSubsystem.getTarget();
                    if (target != null) {
                        var robotPose = drivetrain.getState().Pose;
                        double distance = robotPose.getTranslation().getDistance(target.toTranslation2d());
                        calibration.setCurrentDistance(distance);
                        calibration.updateDashboard(distance);
                        
                        double hoodAngle = calibration.getHoodAngle(distance);
                        hoodSubsystem.setAngle(Rotation2d.fromDegrees(hoodAngle));
                    } else {
                        // No target - pack to 0°
                        hoodSubsystem.setAngle(Rotation2d.fromDegrees(0));
                    }
                }
            }, hoodSubsystem).withName("Hood Auto/Manual Tracking")
        );
        
        // Flywheel default command: Follow calibration system values (manual or auto)
        flywheelSubsystem.setDefaultCommand(
            Commands.run(() -> {
                var calibration = ShootingCalculator.getCalibration();
                
                // In calibration mode, ALWAYS use manual values regardless of target
                if (calibration.isCalibrationMode()) {
                    double flywheelRPS = calibration.getManualFlywheelVelocity();
                    flywheelSubsystem.setVelocity(flywheelRPS);
                    System.out.println("[FLYWHEEL] Calibration Mode: " + flywheelRPS + " RPS");
                } else {
                    // Auto mode - require target
                    var target = turretSubsystem.getTarget();
                    if (target != null) {
                        var robotPose = drivetrain.getState().Pose;
                        double distance = robotPose.getTranslation().getDistance(target.toTranslation2d());
                        
                        double flywheelRPS = calibration.getFlywheelVelocity(distance);
                        flywheelSubsystem.setVelocity(flywheelRPS);
                    } else {
                        // No target - stop flywheel
                        flywheelSubsystem.stop();
                    }
                }
            }, flywheelSubsystem).withName("Flywheel Auto/Manual Tracking")
        );
        
        // Indexer default command: Follow calibration system values in manual mode
        indexerSubsystem.setDefaultCommand(
            Commands.run(() -> {
                var calibration = ShootingCalculator.getCalibration();
                
                // In calibration mode, use manual slider values
                if (calibration.isCalibrationMode()) {
                    double floorDutyCycle = calibration.getManualFloorIndexer();
                    double fireDutyCycle = calibration.getManualFireIndexer();
                    indexerSubsystem.setFloorIndexerDutyCycle(floorDutyCycle);
                    indexerSubsystem.setFireIndexerDutyCycle(fireDutyCycle);
                    System.out.println("[INDEXER] Calibration Mode: Floor=" + floorDutyCycle + " Fire=" + fireDutyCycle);
                } else {
                    // Auto mode - stop indexers (actual shooting commands will control them)
                    indexerSubsystem.stopAll();
                }
            }, indexerSubsystem).withName("Indexer Manual Control")
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
        // Also includes chassis rotation assist when turret is tracking and target is in deadzone
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // Apply speed reduction when climber is deployed for fine adjustments
                double speedMultiplier = climberSubsystem.isDeployed() ? CLIMBING_SPEED_REDUCTION : 1.0;
                
                // Get chassis rotation assist from turret (non-zero when target in deadzone)
                double chassisRotationAssist = turretSubsystem.getChassisRotationAssist();
                
                // Combine driver input with chassis rotation assist
                double driverRotation = -joystick.getRightX() * maxAngularRate * speedMultiplier;
                double totalRotation = driverRotation + chassisRotationAssist;
                
                return drive.withVelocityX(-joystick.getLeftY() * maxSpeed * speedMultiplier)
                    .withVelocityY(-joystick.getLeftX() * maxSpeed * speedMultiplier)
                    .withRotationalRate(totalRotation);
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
        // ===== AUTOMATIC TARGET TRACKING =====
        // Driver left trigger: Track BLUE alliance hub
        joystick.leftTrigger().onTrue(
            Commands.runOnce(() -> turretSubsystem.setTarget(frc.robot.constants.FieldConstants.BLUE_HUB))
        );
        
        // Driver right trigger: Track RED alliance hub
        joystick.rightTrigger().onTrue(
            Commands.runOnce(() -> turretSubsystem.setTarget(frc.robot.constants.FieldConstants.RED_HUB))
        );
        
        // Driver Y button: Disable tracking (turret holds position)
        joystick.y().onTrue(
            Commands.runOnce(() -> turretSubsystem.setTarget(null))
        );
        
        // ===== SMART SHOOTING =====
        // Driver A button: Smart shoot at tracked target (auto-calculate parameters)
        joystick.a().onTrue(
            createSmartShootCommand()
        );
        
        // Driver B button: Manual shoot with fixed parameters (for testing)
        joystick.b().onTrue(
            ShootCommand.shoot(
                flywheelSubsystem, indexerSubsystem, turretSubsystem, hoodSubsystem,
                intakeRollerSubsystem, intakePositionSubsystem,
                60.0, // Fixed flywheel velocity
                Rotation2d.fromDegrees(0.0), // Fixed turret angle
                Rotation2d.fromDegrees(25.0) // Fixed hood angle
            )
        );
        
        // Operator A button: Zero turret to current position (for calibration)
        operatorController.a().onTrue(
            Commands.runOnce(() -> turretSubsystem.zeroTurret(), turretSubsystem)
        );
    }
    
    /**
     * Creates a smart shooting command that automatically calculates optimal parameters
     * based on distance to target. Optimizes for minimum flywheel velocity.
     */
    private Command createSmartShootCommand() {
        return Commands.defer(() -> {
            // Get current target
            var target = turretSubsystem.getTarget();
            if (target == null) {
                System.out.println("No target set for smart shoot!");
                return Commands.none();
            }
            
            // Calculate optimal shooting parameters
            var robotPose = drivetrain.getState().Pose;
            var solution = ShootingCalculator.calculate(robotPose, target);
            
            if (!solution.isValid) {
                System.out.println("No valid shooting solution found!");
                return Commands.none();
            }
            
            // Log the solution
            System.out.println(String.format(
                "Smart Shoot: Distance=%.2fm, Turret=%.1f°, Hood=%.1f°, Flywheel=%.1f RPS, Entry=%.1f°",
                solution.distance, solution.turretAngleDegrees, solution.hoodAngleDegrees,
                solution.flywheelVelocityRPS, solution.entryAngleDegrees
            ));
            
            // Return shoot command with calculated parameters
            return ShootCommand.shoot(
                flywheelSubsystem, indexerSubsystem, turretSubsystem, hoodSubsystem,
                intakeRollerSubsystem, intakePositionSubsystem,
                solution.flywheelVelocityRPS,
                solution.getTurretAngle(),
                solution.getHoodAngle()
            );
        }, java.util.Set.of(turretSubsystem, drivetrain));
    }
    
    // ==================== INTAKE CONTROLS ====================
    
    private void configureIntakeControls() {
        // Right bumper (driver): Toggle intake position and power
        // Press once: deploy and turn on
        // Press again: retract and turn off
        joystick.rightBumper().onTrue(
            IntakeCommand.toggleIntake(intakeRollerSubsystem, intakePositionSubsystem, indexerSubsystem)
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
    
    // ==================== SUBSYSTEM ACCESSORS ====================
    
    /**
     * Get the vision subsystem for pose initialization
     * @return The vision subsystem
     */
    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }
    
    /**
     * Get the turret subsystem
     * @return The turret subsystem
     */
    public TurretSubsystem getTurretSubsystem() {
        return turretSubsystem;
    }
    
    /**
     * Get the drivetrain subsystem for pose initialization
     * @return The drivetrain subsystem
     */
    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
}
