package frc.robot;

/*
 * =====================================================================
 * BUTTON BINDINGS OVERVIEW
 * =====================================================================
 * 
 * DRIVER CONTROLLER (Xbox Controller 0):
 *   Left Stick       - Translate robot (field-centric)
 *   Right Stick      - Rotate robot + chassis rotation assist
 *   Left Trigger     - Track BLUE hub + Smart shoot
 *   Right Trigger    - Track RED hub + Smart shoot
 *   A Button         - Smart shoot (uses current target)
 *   Left Bumper      - Eject (hold)
 *   Right Bumper     - Toggle intake (deploy/retract + on/off)
 *   POV Left         - Pathfind to "Far Climb"
 *   POV Right        - Pathfind to "Close Climb"
 *   POV Up           - Reset field-centric heading
 * 
 * OPERATOR CONTROLLER (Xbox Controller 1):
 *   A Button         - Zero turret at current position
 *   B Button         - Manual shoot (fixed test parameters)
 *   X Button         - Store climber (spool winch - climb)
 *   Y Button         - Deploy climber (unspool winch)
 *   Right Bumper     - Toggle vision processing
 *   POV Down         - Clear target tracking
 *   Back + X/Y       - SysId routines (drivetrain characterization)
 *   Start + X/Y      - SysId routines (drivetrain characterization)
 * 
 * SPEED MODIFIERS:
 *   - Normal: 100% speed
 *   - Climbing: 25% speed (when climber deployed)
 * 
 * =====================================================================
 */

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import frc.robot.util.FuelSim;
import frc.robot.util.ShootingCalculator;

public class RobotContainer {
    
    // ==================== CONFIGURATION CONSTANTS ====================
    
    /** Master toggle to disable ALL telemetry and logging */
    public static final boolean DISABLE_ALL_TELEMETRY = false;
    
    /** Shooter calibration mode: true = manual sliders, false = auto interpolation */
    public static final boolean SHOOTER_CALIBRATION_MODE = false;
    
    // ==================== DRIVETRAIN CONSTANTS ====================
    
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    
    // ==================== CONTROLLERS ====================
    
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    // ==================== SUBSYSTEMS ====================
    
    // Drivetrain & Vision
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain);
    private final PathPlannerSubsystem pathPlannerSubsystem = new PathPlannerSubsystem(drivetrain);
    
    // Shooter System
    private final TurretSubsystem turretSubsystem = new TurretSubsystem(
        () -> drivetrain.getPose(),
        () -> drivetrain.getState().Speeds
    );
    private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
    
    // Intake System
    public final IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();
    public final TurboKickerSubsystem turboKickerSubsystem = new TurboKickerSubsystem();
    private final IntakeWinchSubsystem intakeWinchSubsystem = new IntakeWinchSubsystem();
    
    // LEDs
    private final CANdleSubsystem candleSubsystem = new CANdleSubsystem();
    
    // ==================== SWERVE REQUESTS ====================
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    
    // ==================== UTILITIES ====================
    
    private final Telemetry logger = new Telemetry(maxSpeed);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // ==================== CONSTRUCTOR ====================
    
    public RobotContainer() {
        // Initialize subsystem connections
        drivetrain.setVisionSubsystem(visionSubsystem);
        
        // Apply configuration flags
        ShootingCalculator.getCalibration().setCalibrationMode(SHOOTER_CALIBRATION_MODE);
        
        // Register named commands for autonomous use
        registerNamedCommands();
        
        // Configure robot
        configureFuelSim();
        configureAutoChooser();
        configureTurretCalibrationButtons();
        configureDefaultCommands();
        configureBindings();
    }
    
    // ==================== FUEL SIMULATION ====================
    
    private void configureFuelSim() {
        FuelSim.getInstance().registerRobot(0.76, 0.76, 0.30,
            () -> drivetrain.getPose(), () -> drivetrain.getState().Speeds);
        FuelSim.getInstance().start();
    }
    
    // ==================== AUTONOMOUS COMMANDS ====================
    // These are public methods that can be called from autonomous routines
    
    private void registerNamedCommands() {
        // Intake commands
        NamedCommands.registerCommand("IntakeOn", 
            IntakeCommand.intake(intakeRollerSubsystem, turboKickerSubsystem).withTimeout(8.0));
        
        NamedCommands.registerCommand("IntakeOff", 
            IntakeCommand.eject(intakeRollerSubsystem, turboKickerSubsystem).withTimeout(2.0));
        
        NamedCommands.registerCommand("IntakeIdle", 
            IntakeCommand.idle(intakeRollerSubsystem, turboKickerSubsystem).withTimeout(0.5));
        
        NamedCommands.registerCommand("IntakeStop",
            Commands.runOnce(() -> {
                CommandScheduler.getInstance().cancel(intakeRollerSubsystem.getCurrentCommand());
                intakeRollerSubsystem.stop();
                turboKickerSubsystem.stop();
            }));
        
        // Shooter commands
        NamedCommands.registerCommand("ShootPreload",
            ShootCommand.shoot(
                flywheelSubsystem, turboKickerSubsystem, turretSubsystem,
                intakeRollerSubsystem, intakeWinchSubsystem, drivetrain, candleSubsystem,
                45.0, Rotation2d.fromDegrees(0.0)
            ).withTimeout(5.0));
        NamedCommands.registerCommand("SmartShoot",
            createSmartShootCommand());
        
        // Turret commands
        NamedCommands.registerCommand("ZeroTurret",
            Commands.runOnce(() -> turretSubsystem.zeroTurret(), turretSubsystem));
        NamedCommands.registerCommand("ClearTarget",
            Commands.runOnce(() -> turretSubsystem.setTarget(null), turretSubsystem));
        NamedCommands.registerCommand("SetBlueTarget",
            Commands.runOnce(() -> turretSubsystem.setTarget(frc.robot.constants.FieldConstants.BLUE_HUB), turretSubsystem));
        NamedCommands.registerCommand("SetRedTarget",
            Commands.runOnce(() -> turretSubsystem.setTarget(frc.robot.constants.FieldConstants.RED_HUB), turretSubsystem));
        
        // LED commands
        NamedCommands.registerCommand("LEDsIdle",
            Commands.runOnce(() -> candleSubsystem.setIdle(), candleSubsystem));
        NamedCommands.registerCommand("LEDsIntaking",
            Commands.runOnce(() -> candleSubsystem.setIntaking(), candleSubsystem));
        NamedCommands.registerCommand("LEDsOuttaking",
            Commands.runOnce(() -> candleSubsystem.setOuttaking(), candleSubsystem));
        NamedCommands.registerCommand("LEDsLocking",
            Commands.runOnce(() -> candleSubsystem.setShootingLocking(), candleSubsystem));
        NamedCommands.registerCommand("LEDsReady",
            Commands.runOnce(() -> candleSubsystem.setShootingReady(), candleSubsystem));
        
        // Climber commands
        NamedCommands.registerCommand("ClimberDeploy",
            Commands.runOnce(() -> intakeWinchSubsystem.setTargetPosition(0.0), intakeWinchSubsystem));
        NamedCommands.registerCommand("ClimberStore",
            Commands.runOnce(() -> intakeWinchSubsystem.setTargetPosition(8.0), intakeWinchSubsystem));
    }
    
    public Command getIntakeCommand() {
        return IntakeCommand.intake(intakeRollerSubsystem, turboKickerSubsystem);
    }
    
    public Command getEjectCommand() {
        return IntakeCommand.eject(intakeRollerSubsystem, turboKickerSubsystem);
    }
    
    public Command getIdleIntakeCommand() {
        return IntakeCommand.idle(intakeRollerSubsystem, turboKickerSubsystem);
    }
    
    public Command getShootPreloadCommand() {
        return ShootCommand.shoot(
            flywheelSubsystem, turboKickerSubsystem, turretSubsystem,
            intakeRollerSubsystem, intakeWinchSubsystem, drivetrain, candleSubsystem,
            60.0, Rotation2d.fromDegrees(0.0)
        );
    }
    
    public Command getSmartShootCommand() {
        return createSmartShootCommand();
    }
    
    public Command getZeroTurretCommand() {
        return Commands.runOnce(() -> turretSubsystem.zeroTurret(), turretSubsystem);
    }
    
    public Command getClearTargetCommand() {
        return Commands.runOnce(() -> turretSubsystem.setTarget(null), turretSubsystem);
    }
    
    public Command getIdleLEDsCommand() {
        return Commands.runOnce(() -> candleSubsystem.setIdle(), candleSubsystem);
    }
    
    public Command getIntakingLEDsCommand() {
        return Commands.runOnce(() -> candleSubsystem.setIntaking(), candleSubsystem);
    }
    
    public Command getEjectingLEDsCommand() {
        return Commands.runOnce(() -> candleSubsystem.setOuttaking(), candleSubsystem);
    }
    
    // ==================== DASHBOARD BUTTONS ====================
    
    private void configureTurretCalibrationButtons() {
        if (DISABLE_ALL_TELEMETRY) return;
        
        SmartDashboard.putData("Turret: Zero Here", 
            Commands.runOnce(() -> turretSubsystem.zeroTurret(), turretSubsystem)
                .ignoringDisable(true).withName("Zero Turret"));
        SmartDashboard.putData("Turret: Set to 180°", 
            Commands.runOnce(() -> turretSubsystem.setEncoderPosition(Rotation2d.fromDegrees(180.0)), turretSubsystem)
                .ignoringDisable(true).withName("Set Turret to 180°"));
    }
    // ==================== DEFAULT COMMANDS ====================
    
    public void configureDefaultCommands() {
        configureDrivetrainDefaultCommand();
        configureIntakeDefaultCommands();
        configureShooterDefaultCommands();
        configureCANdleDefaultCommand();
    }
    
    private void configureDrivetrainDefaultCommand() {
        // Field-centric drive with chassis rotation assist
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double chassisRotationAssist = turretSubsystem.getChassisRotationAssist();
                double driverRotation = -joystick.getRightX() * maxAngularRate;
                double totalRotation = driverRotation + chassisRotationAssist;
                
                return drive.withVelocityX(joystick.getLeftY() * maxSpeed)
                    .withVelocityY(joystick.getLeftX() * maxSpeed)
                    .withRotationalRate(totalRotation);
            })
        );
    }
    
    private void configureIntakeDefaultCommands() {
        // Intake runs at low idle speed when no buttons are pressed
        intakeRollerSubsystem.setDefaultCommand(
            IntakeCommand.idle(intakeRollerSubsystem, turboKickerSubsystem)
        );
    }
    
    private void configureShooterDefaultCommands() {
        // Turret continuously tracks target (tracking logic in TurretSubsystem.periodic())
        turretSubsystem.setDefaultCommand(
            Commands.run(() -> {}, turretSubsystem)
                .withName("Turret Tracking")
        );
        
        // Flywheel follows calibration values (manual sliders or auto interpolation)
        flywheelSubsystem.setDefaultCommand(
            Commands.run(() -> {
                updateFlywheelFromCalibration();
            }, flywheelSubsystem).withName("Flywheel Tracking")
        );
        
        // TurboKicker follows manual calibration sliders in calibration mode
        turboKickerSubsystem.setDefaultCommand(
            Commands.run(() -> {
                updateTurboKickerFromCalibration();
            }, turboKickerSubsystem).withName("TurboKicker Manual Control")
        );
    }
    
    private void configureCANdleDefaultCommand() {
        // CANdle continuously updates based on robot state
        candleSubsystem.setDefaultCommand(
            Commands.run(() -> {
                updateLEDFeedback();
            }, candleSubsystem).withName("LED Feedback")
        );
    }
    
    private void updateFlywheelFromCalibration() {
        var calibration = ShootingCalculator.getCalibration();
        
        if (calibration.isCalibrationMode()) {
            // Manual mode: Use dashboard slider
            flywheelSubsystem.setVelocity(calibration.getManualFlywheelVelocity());
        } else {
            // Auto mode: Interpolate based on distance to target
            var target = turretSubsystem.getTarget();
            if (target != null) {
                var robotPose = drivetrain.getPose();
                double distance = robotPose.getTranslation().getDistance(target.toTranslation2d());
                flywheelSubsystem.setVelocity(calibration.getFlywheelVelocity(distance));
            } else {
                flywheelSubsystem.stop();
            }
        }
    }
    
    private void updateTurboKickerFromCalibration() {
        var calibration = ShootingCalculator.getCalibration();
        
        if (calibration.isCalibrationMode()) {
            // Manual mode: TurboKicker uses preset speeds, no manual control
            turboKickerSubsystem.stop();
        } else {
            // Auto mode: Stop (shooting commands will control TurboKicker)
            turboKickerSubsystem.stop();
        }
    }
    
    // ==================== BUTTON BINDINGS ====================
    
    private void configureBindings() {
        configureDrivetrainControls();
        configureVisionControls();
        configureShooterControls();
        configureIntakeControls();
        configurePathPlannerControls();
    }
    private void configureDrivetrainControls() {
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

        if (!DISABLE_ALL_TELEMETRY) {
            drivetrain.registerTelemetry(logger::telemeterize);
        }
    }
    
    private void configureVisionControls() {
        // Reset field-centric heading (moved to driver POV Up)
        joystick.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
        // SysId routines (moved to operator controller)
        operatorController.back().and(operatorController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        operatorController.back().and(operatorController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        operatorController.start().and(operatorController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        operatorController.start().and(operatorController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
        operatorController.rightBumper().onTrue(Commands.runOnce(() -> visionSubsystem.toggleVision()));
    }
    
    private void configurePathPlannerControls() {
        joystick.povLeft().onTrue(pathPlannerSubsystem.pathfindThenFollowPath("Far Climb"));
        joystick.povRight().onTrue(pathPlannerSubsystem.pathfindThenFollowPath("Close Climb"));
    }
    
    private void configureShooterControls() {
        // Left trigger = Blue hub targeting + shoot
        // onTrue: Starts the shoot sequence when trigger pressed
        // onFalse: Cancels shooting and clears target when trigger released
        joystick.leftTrigger()
            .onTrue(
                Commands.sequence(
                    Commands.runOnce(() -> turretSubsystem.setTarget(frc.robot.constants.FieldConstants.BLUE_HUB)),
                    createSmartShootCommand()
                )
            )
            .onFalse(
                Commands.runOnce(() -> {
                    System.out.println("[RobotContainer] Trigger released - cancelling shoot and clearing target");
                    turretSubsystem.setTarget(null);
                    // CommandScheduler will handle cancelling the current shoot command
                }).andThen(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()))
            );
        
        // Right trigger = Red hub targeting + shoot
        // onTrue: Starts the shoot sequence when trigger pressed
        // onFalse: Cancels shooting and clears target when trigger released
        joystick.rightTrigger()
            .onTrue(
                Commands.sequence(
                    Commands.runOnce(() -> turretSubsystem.setTarget(frc.robot.constants.FieldConstants.RED_HUB)),
                    createSmartShootCommand()
                )
            )
            .onFalse(
                Commands.runOnce(() -> {
                    System.out.println("[RobotContainer] Trigger released - cancelling shoot and clearing target");
                    turretSubsystem.setTarget(null);
                    // CommandScheduler will handle cancelling the current shoot command
                }).andThen(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()))
            );
        
        // Driver A button also does smart shoot (uses already-set target)
        joystick.a().onTrue(createSmartShootCommand());
        
        // Operator controls
        operatorController.povDown().onTrue(Commands.runOnce(() -> turretSubsystem.setTarget(null))); // Clear target
        operatorController.b().onTrue(ShootCommand.shoot( // Manual shoot with X-stance and LED feedback
            flywheelSubsystem, turboKickerSubsystem, turretSubsystem,
            intakeRollerSubsystem, intakeWinchSubsystem, drivetrain, candleSubsystem,
            60.0, Rotation2d.fromDegrees(0.0)
        ));
        
        // Calibration
        operatorController.a().onTrue(Commands.runOnce(() -> turretSubsystem.zeroTurret(), turretSubsystem));
    }
    
    /**
     * Immediate shooter stop - forcefully stops all systems NOW
     */
    private Command createImmediateShooterStopCommand() {
        return Commands.runOnce(() -> {
            // FORCE STOP ALL SYSTEMS IMMEDIATELY - no sequence, no delays
            turretSubsystem.setTarget(null);
            turboKickerSubsystem.stop();
            flywheelSubsystem.stop();
            candleSubsystem.setIdle();
        });
    }
    
    /**
     * Create shutdown sequence: reversed burst on kicker motors then complete stop
     */
    private Command createShooterShutdownCommand() {
        return Commands.sequence(
            // Reversed burst for 0.2 seconds
            Commands.run(() -> {
                turboKickerSubsystem.setFeedMotorsDutyCycle(-0.5);
                turboKickerSubsystem.setKickerMotorDutyCycle(-0.5);
                turboKickerSubsystem.setBeltFloorDutyCycle(-0.5);
            }, turboKickerSubsystem).withTimeout(0.2),
            // Then stop everything
            Commands.runOnce(() -> {
                // Stop all turboKicker motors
                turboKickerSubsystem.stop();
                // Stop flywheel
                flywheelSubsystem.stop();
                // Clear target and stop turret tracking
                turretSubsystem.setTarget(null);
                // Reset LEDs to idle state
                candleSubsystem.setIdle();
            })
        );
    }
    private Command createSmartShootCommand() {
        return Commands.defer(() -> {
            var target = turretSubsystem.getTarget();
            if (target == null) return Commands.none();
            
            var solution = ShootingCalculator.calculate(drivetrain.getPose(), target);
            if (!solution.isValid) return Commands.none();
            
            // Return the shoot command - trigger control handles cancellation via .onFalse()
            return ShootCommand.shoot(
                flywheelSubsystem, turboKickerSubsystem, turretSubsystem,
                intakeRollerSubsystem, intakeWinchSubsystem, drivetrain, candleSubsystem,
                solution.flywheelVelocityRPS, solution.getTurretAngle()
            );
        }, java.util.Set.of(flywheelSubsystem, turboKickerSubsystem, turretSubsystem, intakeRollerSubsystem, intakeWinchSubsystem));
    }
    
    private void configureIntakeControls() {
        // Right bumper held = full speed intake, left bumper held = reverse/eject
        joystick.rightBumper().whileTrue(IntakeCommand.intake(
            intakeRollerSubsystem, turboKickerSubsystem));
        joystick.leftBumper().whileTrue(IntakeCommand.eject(
            intakeRollerSubsystem, turboKickerSubsystem));
    }
    
    // ==================== AUTONOMOUS ====================
    
    private void configureAutoChooser() {
        // Set default to None
        autoChooser.setDefaultOption("None", Commands.none());
        autoChooser.addOption("Auto-A", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-A"), java.util.Set.of()));
        autoChooser.addOption("Auto-B", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-B"), java.util.Set.of()));
        autoChooser.addOption("Auto-C", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-C"), java.util.Set.of()));
        autoChooser.addOption("Auto-D", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-D"), java.util.Set.of()));
        autoChooser.addOption("Auto-E", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-E"), java.util.Set.of()));
        autoChooser.addOption("Auto-F", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-F"), java.util.Set.of()));
        autoChooser.addOption("Auto-G", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-G"), java.util.Set.of()));
        autoChooser.addOption("Auto-H", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-H"), java.util.Set.of()));
        autoChooser.addOption("Auto-I", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-I"), java.util.Set.of()));
        autoChooser.addOption("Auto-J", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-J"), java.util.Set.of()));
        autoChooser.addOption("Auto-K", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-K"), java.util.Set.of()));
        autoChooser.addOption("Auto-L", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-L"), java.util.Set.of()));
        autoChooser.addOption("Auto-M", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-M"), java.util.Set.of()));
        autoChooser.addOption("Auto-N", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-N"), java.util.Set.of()));
        autoChooser.addOption("Auto-O", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-O"), java.util.Set.of()));
        autoChooser.addOption("Auto-P", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-P"), java.util.Set.of()));
        autoChooser.addOption("Auto-Q", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-Q"), java.util.Set.of()));
        autoChooser.addOption("Auto-R", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-R"), java.util.Set.of()));
        autoChooser.addOption("Auto-S", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-S"), java.util.Set.of()));
        autoChooser.addOption("Auto-T", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-T"), java.util.Set.of()));
        autoChooser.addOption("Auto-U", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-U"), java.util.Set.of()));
        autoChooser.addOption("Auto-V", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-V"), java.util.Set.of()));
        autoChooser.addOption("Auto-W", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-W"), java.util.Set.of()));
        autoChooser.addOption("Auto-X", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-X"), java.util.Set.of()));
        autoChooser.addOption("Auto-Y", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-Y"), java.util.Set.of()));
        autoChooser.addOption("Auto-Z", Commands.defer(() -> pathPlannerSubsystem.getAutonomousCommand("Auto-Z"), java.util.Set.of()));
        
        if (!DISABLE_ALL_TELEMETRY) {
            SmartDashboard.putData("Auto Chooser", autoChooser);
        }
    }
    
    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        System.out.println("[RobotContainer] Selected autonomous command: " + (selected != null ? selected.getName() : "null"));
        return selected;
    }
    
    // ==================== PERIODIC UPDATES ====================
    
    private boolean lastFlywheelReadyState = false;
    private double lastFlywheelVelocity = 0.0;
    private int orangeFlashCounter = 0;
    private static final double VELOCITY_DIP_THRESHOLD = 5.0;
    private static final int ORANGE_FLASH_DURATION = 10;
    
    public void periodic() {
        updateVelocityDipDetection();
        updateControllerRumble();
    }
    
    private void updateVelocityDipDetection() {
        double currentVelocity = flywheelSubsystem.getVelocityRPS();
        
        if (flywheelSubsystem.getTargetVelocityRPS() > 0.1) {
            double velocityDrop = lastFlywheelVelocity - currentVelocity;
            if (velocityDrop > VELOCITY_DIP_THRESHOLD && orangeFlashCounter == 0) {
                orangeFlashCounter = ORANGE_FLASH_DURATION;
            }
        }
        
        if (orangeFlashCounter > 0) orangeFlashCounter--;
        lastFlywheelVelocity = currentVelocity;
    }
    
    private void updateControllerRumble() {
        boolean flywheelReady = flywheelSubsystem.atTargetVelocity() 
                                && flywheelSubsystem.getTargetVelocityRPS() > 0.1;
        
        if (flywheelReady && !lastFlywheelReadyState) {
            joystick.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 1.0);
            operatorController.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 1.0);
            
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
    
    private void updateLEDFeedback() {
        // Don't override if a command (like ShootCommand) has locked the LEDs
        if (candleSubsystem.areLEDsLocked()) {
            return;
        }
        
        // Two-zone LED system:
        // Zone 1 (CANdle 0-7): Debugging feedback
        // Zone 2 (Lightbar 8-35): Visual feedback
        
        // Determine robot state priority: Shooting > Intaking > Outtaking > Idle
        // Use button states for immediate LED response
        boolean isIntakingButton = joystick.rightBumper().getAsBoolean();
        boolean isOuttakingButton = joystick.leftBumper().getAsBoolean();
        boolean isShooting = flywheelSubsystem.getTargetVelocityRPS() > 0.1;
        boolean flywheelAtSpeed = flywheelSubsystem.atTargetVelocity();
        
        if (isShooting) {
            // Shooting state
            if (flywheelAtSpeed) {
                // At speed: Blue strobe
                candleSubsystem.setShootingReady();
                System.out.println("LED: SHOOTING_READY (velocity=" + flywheelSubsystem.getVelocityRPS() + 
                                   ", target=" + flywheelSubsystem.getTargetVelocityRPS() + ")");
            } else {
                // Spinning up: Rapid blue Larson with progress
                double progress = flywheelSubsystem.getVelocityRPS() / flywheelSubsystem.getTargetVelocityRPS();
                candleSubsystem.setShootingSpinup(progress);
            }
        } else if (isIntakingButton) {
            // Intaking state (right bumper held)
            candleSubsystem.setIntaking();
        } else if (isOuttakingButton) {
            // Outtaking state (left bumper held)
            candleSubsystem.setOuttaking();
        } else {
            // Idle state
            candleSubsystem.setIdle();
        }
    }
    
    // ==================== SUBSYSTEM ACCESSORS ====================
    
    public VisionSubsystem getVisionSubsystem() { return visionSubsystem; }
    public TurretSubsystem getTurretSubsystem() { return turretSubsystem; }
    public CommandSwerveDrivetrain getDrivetrain() { return drivetrain; }
    public PathPlannerSubsystem getPathPlannerSubsystem() { return pathPlannerSubsystem; }
}
