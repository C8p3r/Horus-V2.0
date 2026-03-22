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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

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
    public static final boolean SHOOTER_CALIBRATION_MODE = true;
    
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
    
    // Intake System
    private final IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();
    private final TurboKickerSubsystem turboKickerSubsystem = new TurboKickerSubsystem();
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
        // Field-centric drive with chassis rotation assist
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double chassisRotationAssist = turretSubsystem.getChassisRotationAssist();
                double driverRotation = -joystick.getRightX() * maxAngularRate;
                double totalRotation = driverRotation + chassisRotationAssist;
                
                return drive.withVelocityX(-joystick.getLeftY() * maxSpeed)
                    .withVelocityY(-joystick.getLeftX() * maxSpeed)
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
        // Target tracking and smart shooting with triggers
        joystick.leftTrigger()
            .onTrue(Commands.sequence(
                Commands.runOnce(() -> turretSubsystem.setTarget(frc.robot.constants.FieldConstants.BLUE_HUB)),
                createSmartShootCommand()
            ))
            .onFalse(Commands.runOnce(() -> turretSubsystem.setTarget(null))); // Clear target on release
        
        joystick.rightTrigger()
            .onTrue(Commands.sequence(
                Commands.runOnce(() -> turretSubsystem.setTarget(frc.robot.constants.FieldConstants.RED_HUB)),
                createSmartShootCommand()
            ))
            .onFalse(Commands.runOnce(() -> turretSubsystem.setTarget(null))); // Clear target on release
        
        // Driver A button also does smart shoot (uses already-set target)
        joystick.a().onTrue(createSmartShootCommand());
        
        // Operator controls
        operatorController.povDown().onTrue(Commands.runOnce(() -> turretSubsystem.setTarget(null))); // Clear target
        operatorController.b().onTrue(ShootCommand.shoot( // Manual shoot
            flywheelSubsystem, turboKickerSubsystem, turretSubsystem,
            intakeRollerSubsystem,
            60.0, Rotation2d.fromDegrees(0.0)
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
                flywheelSubsystem, turboKickerSubsystem, turretSubsystem,
                intakeRollerSubsystem,
                solution.flywheelVelocityRPS, solution.getTurretAngle()
            );
        }, java.util.Set.of(flywheelSubsystem, turboKickerSubsystem, turretSubsystem, intakeRollerSubsystem));
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
                // At speed: Blue fire
                candleSubsystem.setShootingReady();
            } else {
                // Spinning up: Normal fire with progress
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
