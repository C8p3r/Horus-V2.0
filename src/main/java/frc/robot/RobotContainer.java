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
    
    // ==================== CONFIGURATION CONSTANTS ====================
    
    /** Master toggle to disable ALL telemetry and logging */
    public static final boolean DISABLE_ALL_TELEMETRY = false;
    
    /** Shooter calibration mode: true = manual sliders, false = auto interpolation */
    public static final boolean SHOOTER_CALIBRATION_MODE = true;
    
    /** Toggle to control whether intake retracts during shooting sequence */
    public static final boolean RETRACT_INTAKE_WHILE_SHOOTING = false;
    
    /** Disable intake deployment motor for testing */
    private static final boolean DISABLE_INTAKE_DEPLOY_MOTOR = false;
    
    /** Climbing mode speed reduction (25% speed when climber deployed) */
    private static final double CLIMBING_SPEED_REDUCTION = 0.25;
    
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
        () -> drivetrain.getState().Pose,
        () -> drivetrain.getState().Speeds
    );
    private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    
    // Intake System
    private final IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();
    private final IntakePositionSubsystem intakePositionSubsystem = new IntakePositionSubsystem();
    private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    
    // Climber
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    
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
    private final TelemetryThrottle robotContainerTelemetryThrottle = new TelemetryThrottle(0.2);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // ==================== CONSTRUCTOR ====================
    
    public RobotContainer() {
        // Initialize subsystem connections
        drivetrain.setVisionSubsystem(visionSubsystem);
        
        // Apply configuration flags
        if (DISABLE_INTAKE_DEPLOY_MOTOR) {
            intakePositionSubsystem.disableDeployMotor();
        }
        ShootingCalculator.getCalibration().setCalibrationMode(SHOOTER_CALIBRATION_MODE);
        
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
            () -> drivetrain.getState().Pose, () -> drivetrain.getState().Speeds);
        FuelSim.getInstance().start();
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
    
    private void configureDefaultCommands() {
        configureDrivetrainDefaultCommand();
        configureIntakeDefaultCommands();
        configureShooterDefaultCommands();
    }
    
    private void configureDrivetrainDefaultCommand() {
        // Field-centric drive with speed reduction when climbing and chassis rotation assist
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double speedMultiplier = climberSubsystem.isDeployed() ? CLIMBING_SPEED_REDUCTION : 1.0;
                double chassisRotationAssist = turretSubsystem.getChassisRotationAssist();
                double driverRotation = -joystick.getRightX() * maxAngularRate * speedMultiplier;
                double totalRotation = driverRotation + chassisRotationAssist;
                
                return drive.withVelocityX(-joystick.getLeftY() * maxSpeed * speedMultiplier)
                    .withVelocityY(-joystick.getLeftX() * maxSpeed * speedMultiplier)
                    .withRotationalRate(totalRotation);
            })
        );
    }
    
    private void configureIntakeDefaultCommands() {
        // Intake roller maintains last commanded state
        intakeRollerSubsystem.setDefaultCommand(
            Commands.run(() -> {}, intakeRollerSubsystem)
                .withName("Intake Roller Idle")
        );
    }
    
    private void configureShooterDefaultCommands() {
        // Turret continuously tracks target (tracking logic in TurretSubsystem.periodic())
        turretSubsystem.setDefaultCommand(
            Commands.run(() -> {}, turretSubsystem)
                .withName("Turret Tracking")
        );
        
        // Hood follows calibration values (manual sliders or auto interpolation)
        hoodSubsystem.setDefaultCommand(
            Commands.run(() -> {
                updateHoodFromCalibration();
            }, hoodSubsystem).withName("Hood Tracking")
        );
        
        // Flywheel follows calibration values (manual sliders or auto interpolation)
        flywheelSubsystem.setDefaultCommand(
            Commands.run(() -> {
                updateFlywheelFromCalibration();
            }, flywheelSubsystem).withName("Flywheel Tracking")
        );
        
        // Indexer follows manual calibration sliders in calibration mode
        indexerSubsystem.setDefaultCommand(
            Commands.run(() -> {
                updateIndexerFromCalibration();
            }, indexerSubsystem).withName("Indexer Manual Control")
        );
    }
    
    private void updateHoodFromCalibration() {
        var calibration = ShootingCalculator.getCalibration();
        
        if (calibration.isCalibrationMode()) {
            // Manual mode: Use dashboard slider
            double hoodAngle = calibration.getManualHoodAngle();
            hoodSubsystem.setAngle(Rotation2d.fromDegrees(hoodAngle));
        } else {
            // Auto mode: Interpolate based on distance to target
            var target = turretSubsystem.getTarget();
            if (target != null) {
                var robotPose = drivetrain.getState().Pose;
                double distance = robotPose.getTranslation().getDistance(target.toTranslation2d());
                calibration.setCurrentDistance(distance);
                calibration.updateDashboard(distance);
                hoodSubsystem.setAngle(Rotation2d.fromDegrees(calibration.getHoodAngle(distance)));
            } else {
                hoodSubsystem.setAngle(Rotation2d.fromDegrees(0)); // Pack hood
            }
        }
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
                var robotPose = drivetrain.getState().Pose;
                double distance = robotPose.getTranslation().getDistance(target.toTranslation2d());
                flywheelSubsystem.setVelocity(calibration.getFlywheelVelocity(distance));
            } else {
                flywheelSubsystem.stop();
            }
        }
    }
    
    private void updateIndexerFromCalibration() {
        var calibration = ShootingCalculator.getCalibration();
        
        if (calibration.isCalibrationMode()) {
            // Manual mode: Use dashboard sliders
            indexerSubsystem.setFloorIndexerDutyCycle(calibration.getManualFloorIndexer());
            indexerSubsystem.setFireIndexerDutyCycle(calibration.getManualFireIndexer());
        } else {
            // Auto mode: Stop (shooting commands will control indexers)
            indexerSubsystem.stopAll();
        }
    }
    
    // ==================== BUTTON BINDINGS ====================
    
    private void configureBindings() {
        configureDrivetrainControls();
        configureVisionControls();
        configureShooterControls();
        configureIntakeControls();
        configureClimberControls();
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
        // Target tracking and smart shooting with triggers
        joystick.leftTrigger().onTrue(Commands.sequence(
            Commands.runOnce(() -> turretSubsystem.setTarget(frc.robot.constants.FieldConstants.BLUE_HUB)),
            createSmartShootCommand()
        ));
        joystick.rightTrigger().onTrue(Commands.sequence(
            Commands.runOnce(() -> turretSubsystem.setTarget(frc.robot.constants.FieldConstants.RED_HUB)),
            createSmartShootCommand()
        ));
        
        // Driver A button also does smart shoot (uses already-set target)
        joystick.a().onTrue(createSmartShootCommand());
        
        // Operator controls
        operatorController.povDown().onTrue(Commands.runOnce(() -> turretSubsystem.setTarget(null))); // Clear target
        operatorController.b().onTrue(ShootCommand.shoot( // Manual shoot
            flywheelSubsystem, indexerSubsystem, turretSubsystem, hoodSubsystem,
            intakeRollerSubsystem, intakePositionSubsystem,
            60.0, Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(25.0)
        ));
        
        // Calibration
        operatorController.a().onTrue(Commands.runOnce(() -> turretSubsystem.zeroTurret(), turretSubsystem));
    }
    private Command createSmartShootCommand() {
        return Commands.defer(() -> {
            var target = turretSubsystem.getTarget();
            if (target == null) return Commands.none();
            
            var solution = ShootingCalculator.calculate(drivetrain.getState().Pose, target);
            if (!solution.isValid) return Commands.none();
            
            return ShootCommand.shoot(
                flywheelSubsystem, indexerSubsystem, turretSubsystem, hoodSubsystem,
                intakeRollerSubsystem, intakePositionSubsystem,
                solution.flywheelVelocityRPS, solution.getTurretAngle(), solution.getHoodAngle()
            );
        }, java.util.Set.of(turretSubsystem, drivetrain));
    }
    
    private void configureIntakeControls() {
        joystick.rightBumper().onTrue(IntakeCommand.toggleIntake(
            intakeRollerSubsystem, intakePositionSubsystem, indexerSubsystem));
        joystick.leftBumper().whileTrue(IntakeCommand.eject(
            intakeRollerSubsystem, indexerSubsystem));
    }
    
    private void configureClimberControls() {
        operatorController.y().onTrue(ClimberCommand.deployClimber(
            climberSubsystem, intakePositionSubsystem, intakeRollerSubsystem, 
            indexerSubsystem, flywheelSubsystem, turretSubsystem, hoodSubsystem));
        operatorController.x().onTrue(ClimberCommand.storeClimber(
            climberSubsystem, intakePositionSubsystem, intakeRollerSubsystem,
            indexerSubsystem, flywheelSubsystem, turretSubsystem, hoodSubsystem));
    }
    
    // ==================== AUTONOMOUS ====================
    
    private void configureAutoChooser() {
        autoChooser.setDefaultOption("None", Commands.none());
        autoChooser.addOption("Example Auto", pathPlannerSubsystem.getAutonomousCommand("Example Auto"));
        autoChooser.addOption("Drive Forward", Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() -> drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
                .withTimeout(3.0),
            drivetrain.applyRequest(() -> idle)
        ));
        
        if (!DISABLE_ALL_TELEMETRY) {
            SmartDashboard.putData("Auto Chooser", autoChooser);
        }
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
    // ==================== PERIODIC UPDATES ====================
    
    private boolean lastFlywheelReadyState = false;
    private double lastFlywheelVelocity = 0.0;
    private int orangeFlashCounter = 0;
    private static final double VELOCITY_DIP_THRESHOLD = 5.0;
    private static final int ORANGE_FLASH_DURATION = 10;
    
    public void periodic() {
        if (!DISABLE_ALL_TELEMETRY && robotContainerTelemetryThrottle.shouldUpdate()) {
            SmartDashboard.putBoolean("Climbing Mode", climberSubsystem.isDeployed());
        }
        updateVelocityDipDetection();
        updateLEDFeedback();
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
        // Priority order: Overheating > Climbing > Note scored > Shooting > Intake > Aiming > Idle
        if (flywheelSubsystem.isOverheating()) {
            candleSubsystem.setRapidRedBlink();
        } else if (climberSubsystem.isFullyDeployed() && climberSubsystem.isDeployed()) {
            candleSubsystem.setSlowRainbow();
        } else if (climberSubsystem.isDeployed() && !climberSubsystem.isRetracted()) {
            candleSubsystem.setRapidRainbow();
        } else if (orangeFlashCounter > 0) {
            candleSubsystem.setRGB(255, 165, 0);
        } else if (flywheelSubsystem.getTargetVelocityRPS() > 0.1) {
            if (flywheelSubsystem.atTargetVelocity()) {
                candleSubsystem.setRGBWithModulation(0, 255, 0, 3.0, 0.6, 1.0);
            } else {
                candleSubsystem.setRGBWithModulation(255, 255, 0, 1.5, 0.4, 0.9);
            }
        } else if (Math.abs(intakeRollerSubsystem.getVelocity()) > 1.0) {
            candleSubsystem.setRGBWithModulation(0, 0, 255, 2.5, 0.5, 1.0);
        } else if (turretSubsystem.atTarget() && hoodSubsystem.atTargetAngle()) {
            candleSubsystem.setRGBWithModulation(128, 0, 255, 2.0, 0.7, 1.0);
        } else {
            candleSubsystem.setRGBWithModulation(80, 80, 80, 0.5, 0.1, 0.3);
        }
    }
    
    // ==================== SUBSYSTEM ACCESSORS ====================
    
    public VisionSubsystem getVisionSubsystem() { return visionSubsystem; }
    public TurretSubsystem getTurretSubsystem() { return turretSubsystem; }
    public CommandSwerveDrivetrain getDrivetrain() { return drivetrain; }
    public PathPlannerSubsystem getPathPlannerSubsystem() { return pathPlannerSubsystem; }
}
