package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;

import org.littletonrobotics.junction.Logger;

/**
 * ==========================================================================================================================================================================================================================================================================================
 * ===                            TURRET SUBSYSTEM - USAGE GUIDE                                  ===
 * ==========================================================================================================================================================================================================================================================================================
 * ===                                                                                            ===
 * ===  OVERVIEW:                                                                                 ===
 * ===  This subsystem controls a 360-degree rotating turret driven by a Kraken X60 motor        ===
 * ===  with a 10:1 reduction. The turret uses the motor's integrated encoder for position       ===
 * ===  tracking and initializes to a configured starting angle on robot boot.                   ===
 * ===                                                                                            ===
 * ===  HARDWARE SETUP:                                                                           ===
 * ===  - Kraken X60 motor with 10:1 gear reduction                                              ===
 * ===  - Motor integrated encoder for position feedback                                         ===
 * ===  - 360-degree rotation range (0° to 360° clockwise from starting position)                ===
 * ===  - Soft limits prevent cable damage                                                       ===
 * ===                                                                                            ===
 * ===  INITIALIZATION:                                                                           ===
 * ===  1. Position the turret at its desired "zero" position (usually facing forward)           ===
 * ===  2. Set INITIAL_ANGLE_DEGREES in Constants.java to match the physical position            ===
 * ===  3. On robot boot, the motor encoder is set to this initial angle                         ===
 * ===  4. The system automatically sets up soft limits for cable management                     ===
 * ===                                                                                            ===
 * ==========================================================================================================================================================================================================================================================================================
 * ===                              BASIC USAGE EXAMPLES                                          ===
 * ==========================================================================================================================================================================================================================================================================================
 * ===                                                                                            ===
 * ===  1. POINT TO ABSOLUTE ANGLE (Robot-Relative):                                             ===
 * ===     // Point turret 90 degrees clockwise from starting position                           ===
 * ===     turret.setTargetAngle(Rotation2d.fromDegrees(90));                                    ===
 * ===                                                                                            ===
 * ===     // Point turret back to starting position                                             ===
 * ===     turret.setTargetAngle(Rotation2d.fromDegrees(0));                                     ===
 * ===                                                                                            ===
 * ===  2. POINT TO FIELD POSITION (Field-Relative):                                             ===
 * ===     // Point at a specific position on the field (e.g., speaker)                          ===
 * ===     Translation2d speakerPos = new Translation2d(0.0, 5.5); // Field coordinates          ===
 * ===     Pose2d robotPose = drivetrain.getPose();                                              ===
 * ===     turret.pointAtFieldPosition(speakerPos, robotPose);                                   ===
 * ===                                                                                            ===
 * ===  3. CHECK IF ON TARGET:                                                                   ===
 * ===     if (turret.atTarget()) {                                                              ===
 * ===         // Turret is pointed at target, safe to shoot                                     ===
 * ===         shooter.fire();                                                                    ===
 * ===     }                                                                                      ===
 * ===                                                                                            ===
 * ===  4. MANUAL CONTROL:                                                                        ===
 * ===     // Use with joystick for manual aiming (-1.0 to 1.0)                                  ===
 * ===     turret.setManualControl(joystick.getX() * 0.5); // 50% speed                          ===
 * ===                                                                                            ===
 * ===  5. STOP TURRET:                                                                           ===
 * ===     turret.stop();                                                                         ===
 * ===                                                                                            ===
 * ==========================================================================================================================================================================================================================================================================================
 * ===                              COMMAND EXAMPLES                                              ===
 * ==========================================================================================================================================================================================================================================================================================
 * ===                                                                                            ===
 * ===  1. CONTINUOUS TRACKING COMMAND:                                                           ===
 * ===     public Command trackSpeaker(Supplier<Pose2d> robotPoseSupplier) {                     ===
 * ===         return run(() -> {                                                                 ===
 * ===             Translation2d speakerPos = FieldConstants.getSpeakerPosition();               ===
 * ===             pointAtFieldPosition(speakerPos, robotPoseSupplier.get());                    ===
 * ===         }).withName("TrackSpeaker");                                                       ===
 * ===     }                                                                                      ===
 * ===                                                                                            ===
 * ===  2. POINT AND WAIT COMMAND:                                                                ===
 * ===     public Command pointToAngleAndWait(Rotation2d angle) {                                ===
 * ===         return runOnce(() -> setTargetAngle(angle))                                       ===
 * ===             .andThen(Commands.waitUntil(this::atTarget))                                  ===
 * ===             .withName("PointToAngle");                                                     ===
 * ===     }                                                                                      ===
 * ===                                                                                            ===
 * ===  3. MANUAL JOYSTICK CONTROL:                                                               ===
 * ===     public Command manualControl(DoubleSupplier speedSupplier) {                          ===
 * ===         return run(() -> setManualControl(speedSupplier.getAsDouble()))                   ===
 * ===             .withName("ManualTurret");                                                     ===
 * ===     }                                                                                      ===
 * ===                                                                                            ===
 * ===  4. RETURN TO HOME:                                                                        ===
 * ===     public Command returnToHome() {                                                        ===
 * ===         return pointToAngleAndWait(Rotation2d.fromDegrees(0));                            ===
 * ===     }                                                                                      ===
 * ===                                                                                            ===
 * ==========================================================================================================================================================================================================================================================================================
 * ===                          INTEGRATION WITH SHOOTING SYSTEM                                  ===
 * ==========================================================================================================================================================================================================================================================================================
 * ===                                                                                            ===
 * ===  TYPICAL SHOOTING SEQUENCE:                                                                ===
 * ===                                                                                            ===
 * ===  Command shootSequence = Commands.parallel(                                               ===
 * ===      turret.trackSpeaker(() -> drivetrain.getPose()),  // Aim at speaker                  ===
 * ===      shooter.spinUp()                                   // Spin up flywheel               ===
 * ===  ).until(() -> turret.atTarget() && shooter.atSpeed()) // Wait until ready                ===
 * ===   .andThen(shooter.feed())                              // Feed note                      ===
 * ===   .withTimeout(2.0);                                    // Safety timeout                 ===
 * ===                                                                                            ===
 * ===  ON-THE-FLY SHOOTING (Moving while shooting):                                              ===
 * ===  - Use pointAtFieldPosition() continuously in periodic                                     ===
 * ===  - Account for robot velocity in trajectory calculations                                  ===
 * ===  - Check atTarget() to ensure turret has settled before shooting                          ===
 * ===                                                                                            ===
 * ==========================================================================================================================================================================================================================================================================================
 * ===                              SAFETY FEATURES                                               ===
 * ==========================================================================================================================================================================================================================================================================================
 * ===                                                                                            ===
 * ===  === Soft limits prevent over-rotation (0== to 360== range)                                   ===
 * ===  === Current limits protect motor and gearbox                                               ===
 * ===  === Automatic wrapping finds shortest path to target                                       ===
 * ===  === Position feedback from absolute encoder (no homing required)                           ===
 * ===  === Motion Magic ensures smooth, controlled movements                                      ===
 * ===                                                                                            ===
 * ==========================================================================================================================================================================================================================================================================================
 * ===                              TUNING GUIDE                                                  ===
 * ==========================================================================================================================================================================================================================================================================================
 * ===                                                                                            ===
 * ===  1. POSITION PID (TurretConstants):                                                        ===
 * ===     - Start with kP = 24.0, kI = 0.0, kD = 0.5                                            ===
 * ===     - Increase kP if response is too slow                                                 ===
 * ===     - Increase kD if there's oscillation                                                  ===
 * ===     - Add kI only if there's steady-state error                                           ===
 * ===                                                                                            ===
 * ===  2. MOTION CONSTRAINTS:                                                                    ===
 * ===     - MAX_VELOCITY_RPS: Maximum rotation speed (default 2.0 = 720==/s)                     ===
 * ===     - MAX_ACCELERATION_RPS2: How quickly it reaches max speed                             ===
 * ===                                                                                            ===
 * ===  3. FEEDFORWARD (kS, kV, kA):                                                              ===
 * ===     - kS: Voltage to overcome static friction (tune if turret sticks)                     ===
 * ===     - kV: Voltage per unit velocity (use Phoenix Tuner SysId)                             ===
 * ===     - kA: Voltage per unit acceleration                                                   ===
 * ===                                                                                            ===
 * ===  4. ENCODER OFFSET:                                                                        ===
 * ===     - Position turret at desired "zero" position                                          ===
 * ===     - Read current encoder value from Phoenix Tuner                                       ===
 * ===     - Set ENCODER_OFFSET in TurretConstants to that value                                 ===
 * ===                                                                                            ===
 * ==========================================================================================================================================================================================================================================================================================
 * ===                              TROUBLESHOOTING                                               ===
 * ==========================================================================================================================================================================================================================================================================================
 * ===                                                                                            ===
 * ===  ISSUE: Turret oscillates around target                                                    ===
 * ===  === Increase kD, decrease kP                                                                ===
 * ===                                                                                            ===
 * ===  ISSUE: Turret moves too slowly                                                            ===
 * ===  === Increase MAX_VELOCITY_RPS and/or kP                                                     ===
 * ===                                                                                            ===
 * ===  ISSUE: Turret doesn't reach target                                                        ===
 * ===  === Check soft limits, verify encoder direction, increase kP                               ===
 * ===                                                                                            ===
 * ===  ISSUE: Turret position drifts                                                             ===
 * ===  === Verify CANcoder is configured as RemoteCANcoder feedback                               ===
 * ===  === Check that encoder is mechanically secure                                              ===
 * ===                                                                                            ===
 * ===  ISSUE: "Turret at limit" warnings                                                         ===
 * ===  === Adjust SOFT_LIMIT_MARGIN or check if cables need more slack                            ===
 * ===                                                                                            ===
 * ==========================================================================================================================================================================================================================================================================================
 */
public class TurretSubsystem extends SubsystemBase {
    
    private final TalonFX turretMotor;
    
    // Control requests
    private final MotionMagicVoltage positionRequest;
    private final NeutralOut neutralRequest;
    
    // NetworkTables publishers
    private final DoublePublisher currentAnglePublisher;
    private final DoublePublisher targetAnglePublisher;
    private final DoublePublisher velocityPublisher;
    
    // State tracking
    private Rotation2d targetAngle;
    private boolean manualControlActive = false;
    
    // Field-oriented targeting - used by both manual and auto modes
    private Translation2d manualTargetPoint = null; // Point 100m away for manual mode
    private java.util.function.Supplier<Pose2d> poseSupplier = null;
    
    /**
     * Creates a new TurretSubsystem
     */
    public TurretSubsystem() {
        turretMotor = new TalonFX(TurretConstants.MOTOR_ID, TurretConstants.CANBUS_NAME);
        
        // Configure motor
        configureMotor();
        
        // Initialize control requests
        positionRequest = new MotionMagicVoltage(0).withSlot(0);
        neutralRequest = new NeutralOut();
        
        // Initialize NetworkTables publishers
        var ntTable = NetworkTableInstance.getDefault().getTable("Turret");
        currentAnglePublisher = ntTable.getDoubleTopic("CurrentAngle").publish();
        targetAnglePublisher = ntTable.getDoubleTopic("TargetAngle").publish();
        velocityPublisher = ntTable.getDoubleTopic("Velocity").publish();
        
        // Initialize target to current position
        targetAngle = getCurrentAngle();
        
        System.out.println("Turret initialized at " + targetAngle.getDegrees() + " degrees");
    }
    
    /**
     * Sets the pose supplier for field-oriented targeting
     * @param poseSupplier Supplier that returns the current robot pose
     */
    public void setPoseSupplier(java.util.function.Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        if (poseSupplier != null) {
            // Initialize manual target point 100m away in current turret direction
            initializeManualTargetPoint();
        }
    }
    
    /**
     * Initializes the manual target point 100m away from current robot position
     * in the direction the turret is currently facing
     */
    private void initializeManualTargetPoint() {
        if (poseSupplier == null) return;
        
        Pose2d robotPose = poseSupplier.get();
        Rotation2d turretFieldAngle = getCurrentAngle().plus(robotPose.getRotation());
        
        // Calculate point 100m away in turret's field-oriented direction
        double distance = 100.0; // meters
        manualTargetPoint = new Translation2d(
            robotPose.getX() + distance * turretFieldAngle.getCos(),
            robotPose.getY() + distance * turretFieldAngle.getSin()
        );
    }
    
    /**
     * Gets the angle to aim at the manual target point from current robot position
     * @return Rotation2d angle in robot-relative frame, or current angle if no target
     */
    private Rotation2d getAngleToManualTarget() {
        if (poseSupplier == null || manualTargetPoint == null) {
            return getCurrentAngle();
        }
        
        Pose2d robotPose = poseSupplier.get();
        Translation2d turretPosition = robotPose.getTranslation().plus(
            new Translation2d(TurretConstants.OFFSET_X, TurretConstants.OFFSET_Y)
                .rotateBy(robotPose.getRotation())
        );
        
        // Calculate angle from turret position to manual target point
        Translation2d toTarget = manualTargetPoint.minus(turretPosition);
        Rotation2d fieldAngle = new Rotation2d(toTarget.getX(), toTarget.getY());
        
        // Convert to robot-relative angle
        return fieldAngle.minus(robotPose.getRotation());
    }
    
    /**
     * Adjusts the manual target point by rotating it around the robot
     * Used for manual control - rotates the target point in field space
     * @param deltaDegrees Amount to adjust target direction (positive = counterclockwise)
     */
    public void adjustFieldOrientedTarget(double deltaDegrees) {
        if (poseSupplier == null) {
            // If no pose supplier, just nudge normally
            nudgeCounterClockwise(deltaDegrees);
            return;
        }
        
        // Initialize manual target if needed
        if (manualTargetPoint == null) {
            initializeManualTargetPoint();
        }
        
        Pose2d robotPose = poseSupplier.get();
        Translation2d turretPosition = robotPose.getTranslation().plus(
            new Translation2d(TurretConstants.OFFSET_X, TurretConstants.OFFSET_Y)
                .rotateBy(robotPose.getRotation())
        );
        
        // Get current direction to manual target
        Translation2d toTarget = manualTargetPoint.minus(turretPosition);
        
        // Rotate the target point around the turret position
        Translation2d rotatedOffset = toTarget.rotateBy(Rotation2d.fromDegrees(deltaDegrees));
        manualTargetPoint = turretPosition.plus(rotatedOffset);
        
        // Update turret to aim at new manual target point
        setTargetAngle(getAngleToManualTarget());
    }
    
    /**
     * Configures the Kraken X60 motor for position control
     */
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.Inverted = TurretConstants.MOTOR_INVERTED 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = TurretConstants.BRAKE_MODE 
            ? NeutralModeValue.Brake 
            : NeutralModeValue.Coast;
        
        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = TurretConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = TurretConstants.ENABLE_CURRENT_LIMIT;
        
        config.CurrentLimits.StatorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = TurretConstants.ENABLE_CURRENT_LIMIT;
        
        // Use integrated encoder with gear ratio
        // SensorToMechanismRatio: how many sensor rotations (motor shaft) per mechanism rotation (turret)
        // With 10:1 gear ratio, motor makes 10 rotations for 1 turret rotation
        config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;
        
        // Hard limits for absolute protection (enabled in hardware)
        config.HardwareLimitSwitch.ForwardLimitEnable = TurretConstants.ENABLE_FORWARD_HARD_LIMIT;
        config.HardwareLimitSwitch.ReverseLimitEnable = TurretConstants.ENABLE_REVERSE_HARD_LIMIT;
        
        // Soft limits for cable management (in turret rotations)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_SOFT_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_SOFT_LIMIT;
        
        // PID configuration
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = TurretConstants.kP;
        slot0.kI = TurretConstants.kI;
        slot0.kD = TurretConstants.kD;
        slot0.kS = TurretConstants.kS;
        slot0.kV = TurretConstants.kV;
        slot0.kA = TurretConstants.kA;
        config.Slot0 = slot0;
        
        // Motion Magic configuration (smooth trapezoid motion)
        MotionMagicConfigs motionMagic = new MotionMagicConfigs();
        motionMagic.MotionMagicCruiseVelocity = TurretConstants.MAX_VELOCITY_RPS;
        motionMagic.MotionMagicAcceleration = TurretConstants.MAX_ACCELERATION_RPS2;
        motionMagic.MotionMagicJerk = TurretConstants.MAX_ACCELERATION_RPS2 * 10; // Jerk for smooth motion
        config.MotionMagic = motionMagic;
        
        // Apply configuration
        turretMotor.getConfigurator().apply(config);
        
        // Set initial position from constant (user must position turret manually before boot)
        turretMotor.setPosition(TurretConstants.INITIAL_ANGLE_DEGREES / 360.0);
    }
    
    /**
     * Sets the target angle for the turret (robot-relative)
     * Handles wraparound between 50° and 360° (wraps to 50° when exceeding 360°)
     * @param angle Target angle (0° = starting position, positive = clockwise)
     */
    public void setTargetAngle(Rotation2d angle) {
        // Normalize angle to 0-360 range
        double degrees = angle.getDegrees() % 360.0;
        if (degrees < 0) degrees += 360.0;
        
        // Handle wraparound: if angle goes below 50°, wrap to high end (near 360°)
        // if angle goes above 360°, wrap to low end (50°)
        if (degrees < 50.0) {
            degrees += 360.0; // Wrap to high end
        }
        
        // Convert to rotations (0.1389 = 50°, 1.0 = 360°)
        double targetRotations = degrees / 360.0;
        
        // Handle wraparound at 360° -> 50° (1.0 rotations wraps to 0.1389)
        if (targetRotations >= 1.0) {
            targetRotations = 0.1389; // Wrap to 50°
        }
        
        // Clamp to soft limits (50° to 360° range)
        targetRotations = Math.max(TurretConstants.MIN_SOFT_LIMIT, 
                                   Math.min(TurretConstants.MAX_SOFT_LIMIT, targetRotations));
        
        targetAngle = Rotation2d.fromDegrees(targetRotations * 360.0);
        manualControlActive = false;
    }
    
    /**
     * Nudges the turret clockwise by a small increment
     * Handles wraparound from 360° to 50°
     * @param incrementDegrees Amount to nudge in degrees (default 5.0°)
     */
    public void nudgeClockwise(double incrementDegrees) {
        double currentDegrees = targetAngle.getDegrees();
        double newAngle = currentDegrees + incrementDegrees;
        
        // Wraparound: if we exceed 360°, wrap to 50°
        if (newAngle >= 360.0) {
            newAngle = 50.0 + (newAngle - 360.0);
        }
        
        setTargetAngle(Rotation2d.fromDegrees(newAngle));
    }
    
    /**
     * Nudges the turret counterclockwise by a small increment
     * Handles wraparound from 50° to 360°
     * @param incrementDegrees Amount to nudge in degrees (default 5.0°)
     */
    public void nudgeCounterClockwise(double incrementDegrees) {
        double currentDegrees = targetAngle.getDegrees();
        double newAngle = currentDegrees - incrementDegrees;
        
        // Wraparound: if we go below 50°, wrap to near 360°
        if (newAngle < 50.0) {
            newAngle = 360.0 - (50.0 - newAngle);
        }
        
        setTargetAngle(Rotation2d.fromDegrees(newAngle));
    }
    
    /**
     * Points the turret at a specific field position
     * @param fieldPosition Target position on the field
     * @param robotPose Current robot pose
     */
    public void pointAtFieldPosition(Translation2d fieldPosition, Pose2d robotPose) {
        // Calculate vector from robot to target
        Translation2d robotToTarget = fieldPosition.minus(robotPose.getTranslation());
        
        // Calculate angle to target in field frame
        Rotation2d angleToTarget = new Rotation2d(robotToTarget.getX(), robotToTarget.getY());
        
        // Convert to robot-relative angle
        Rotation2d robotRelativeAngle = angleToTarget.minus(robotPose.getRotation());
        
        // Set target angle
        setTargetAngle(robotRelativeAngle);
    }
    
    /**
     * Sets manual control speed for the turret
     * @param speed Speed from -1.0 to 1.0 (positive = clockwise)
     */
    public void setManualControl(double speed) {
        manualControlActive = true;
        
        // Clamp speed
        speed = Math.max(-1.0, Math.min(1.0, speed));
        
        // Check if at limits
        double currentPos = getCurrentAngle().getRotations();
        if ((speed > 0 && currentPos >= TurretConstants.MAX_SOFT_LIMIT) ||
            (speed < 0 && currentPos <= TurretConstants.MIN_SOFT_LIMIT)) {
            speed = 0;
        }
        
        // Calculate target position based on speed
        double deltaRotations = speed * TurretConstants.MAX_VELOCITY_RPS * 0.02; // 20ms loop time
        double newTarget = currentPos + deltaRotations;
        
        // Clamp to limits
        newTarget = Math.max(TurretConstants.MIN_SOFT_LIMIT, 
                            Math.min(TurretConstants.MAX_SOFT_LIMIT, newTarget));
        
        targetAngle = Rotation2d.fromRotations(newTarget);
    }
    
    /**
     * Stops the turret
     */
    public void stop() {
        turretMotor.setControl(neutralRequest);
        manualControlActive = false;
    }
    
    /**
     * Gets the current angle of the turret
     * @return Current angle (0== = starting position, positive = clockwise)
     */
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
    }
    
    /**
     * Gets the current angular velocity of the turret
     * @return Velocity in rotations per second
     */
    public double getVelocityRPS() {
        return turretMotor.getVelocity().getValueAsDouble();
    }
    
    /**
     * Checks if the turret is at the target position
     * @return True if within tolerance
     */
    public boolean atTarget() {
        double error = Math.abs(getCurrentAngle().getRotations() - targetAngle.getRotations());
        double velocityRPS = Math.abs(getVelocityRPS());
        
        return error < TurretConstants.POSITION_TOLERANCE_ROTATIONS &&
               velocityRPS < TurretConstants.VELOCITY_TOLERANCE_RPS;
    }
    
    /**
     * Gets the target angle
     * @return Target angle
     */
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }
    
    /**
     * Checks if turret is in manual control mode
     * @return True if manual control is active
     */
    public boolean isManualControlActive() {
        return manualControlActive;
    }
    
    @Override
    public void periodic() {
        // Update target angle to track manual target point (if in manual mode)
        if (poseSupplier != null && manualTargetPoint != null) {
            // Recalculate the angle to the manual target every cycle
            // This automatically compensates for robot rotation
            targetAngle = getAngleToManualTarget();
            
            // Normalize to 0-360 and handle wraparound
            double degrees = targetAngle.getDegrees() % 360.0;
            if (degrees < 0) degrees += 360.0;
            if (degrees < 50.0) degrees += 360.0;
            if (degrees >= 360.0) degrees = 50.0 + (degrees - 360.0);
            targetAngle = Rotation2d.fromDegrees(degrees);
        }
        
        // Update motor position control
        if (!manualControlActive) {
            turretMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
        }
        
        // Update telemetry
        updateTelemetry();
    }
    
    /**
     * Updates telemetry to NetworkTables and SmartDashboard
     */
    private void updateTelemetry() {
        // Get values directly
        double currentDegrees = getCurrentAngle().getDegrees();
        double targetDegrees = targetAngle.getDegrees();
        double velocityRPS = turretMotor.getVelocity().getValueAsDouble();
        double currentAmps = turretMotor.getSupplyCurrent().getValueAsDouble();
        double voltageVolts = turretMotor.getMotorVoltage().getValueAsDouble();
        double motorPos = turretMotor.getPosition().getValueAsDouble();
        
        // Publish to NetworkTables
        currentAnglePublisher.set(currentDegrees);
        targetAnglePublisher.set(targetDegrees);
        velocityPublisher.set(velocityRPS);
        
        // Publish to SmartDashboard
        SmartDashboard.putNumber("Turret/Current Turret Angle", currentDegrees);
        SmartDashboard.putNumber("Turret/Target Angle", targetDegrees);
        SmartDashboard.putNumber("Turret/Velocity RPS", velocityRPS);
        SmartDashboard.putNumber("Turret/Current (A)", currentAmps);
        SmartDashboard.putNumber("Turret/Voltage (V)", voltageVolts);
        SmartDashboard.putNumber("Turret/Motor Position", motorPos);
        SmartDashboard.putBoolean("Turret/At Target", atTarget());
        SmartDashboard.putBoolean("Turret/Manual Control", manualControlActive);
        SmartDashboard.putNumber("Turret/Error (deg)", targetDegrees - currentDegrees);
        
        // Debug telemetry
        if (poseSupplier != null) {
            Pose2d robotPose = poseSupplier.get();
            
            // Calculate turret position in field space
            Translation2d turretPosition = robotPose.getTranslation().plus(
                new Translation2d(TurretConstants.OFFSET_X, TurretConstants.OFFSET_Y)
                    .rotateBy(robotPose.getRotation())
            );
            
            // Calculate turret's field-oriented angle
            Rotation2d turretFieldAngle = getCurrentAngle().plus(robotPose.getRotation());
            
            // Create 3D pose for turret (position + rotation)
            Pose3d turretPose3d = new Pose3d(
                turretPosition.getX(),
                turretPosition.getY(),
                TurretConstants.OFFSET_Z,
                new Rotation3d(0, 0, turretFieldAngle.getRadians())
            );
            
            // Publish turret 3D pose
            SmartDashboard.putString("TurretDebug/Turret3DPose", turretPose3d.toString());
            Logger.recordOutput("TurretDebug/Turret3DPose", turretPose3d);
            
            // Publish turret position components
            SmartDashboard.putNumber("TurretDebug/TurretFieldX", turretPosition.getX());
            SmartDashboard.putNumber("TurretDebug/TurretFieldY", turretPosition.getY());
            SmartDashboard.putNumber("TurretDebug/TurretFieldZ", TurretConstants.OFFSET_Z);
            SmartDashboard.putNumber("TurretDebug/TurretFieldAngleDeg", turretFieldAngle.getDegrees());
            
            // Publish manual target point if it exists
            if (manualTargetPoint != null) {
                Pose3d manualTargetPose3d = new Pose3d(
                    manualTargetPoint.getX(),
                    manualTargetPoint.getY(),
                    0.0, // Ground level
                    new Rotation3d()
                );
                
                SmartDashboard.putString("TurretDebug/ManualTargetPoint", manualTargetPoint.toString());
                SmartDashboard.putNumber("TurretDebug/ManualTargetX", manualTargetPoint.getX());
                SmartDashboard.putNumber("TurretDebug/ManualTargetY", manualTargetPoint.getY());
                Logger.recordOutput("TurretDebug/ManualTargetPoint", manualTargetPose3d);
                
                // Calculate distance to manual target
                double distanceToTarget = turretPosition.getDistance(manualTargetPoint);
                SmartDashboard.putNumber("TurretDebug/DistanceToManualTarget", distanceToTarget);
                
                // Calculate expected angle to target
                Translation2d toTarget = manualTargetPoint.minus(turretPosition);
                Rotation2d expectedFieldAngle = new Rotation2d(toTarget.getX(), toTarget.getY());
                Rotation2d expectedRobotAngle = expectedFieldAngle.minus(robotPose.getRotation());
                SmartDashboard.putNumber("TurretDebug/ExpectedAngleToTarget", expectedRobotAngle.getDegrees());
                SmartDashboard.putNumber("TurretDebug/AngleErrorToTarget", expectedRobotAngle.getDegrees() - currentDegrees);
            } else {
                SmartDashboard.putString("TurretDebug/ManualTargetPoint", "null");
            }
            
            // Robot pose for reference
            SmartDashboard.putNumber("TurretDebug/RobotX", robotPose.getX());
            SmartDashboard.putNumber("TurretDebug/RobotY", robotPose.getY());
            SmartDashboard.putNumber("TurretDebug/RobotRotationDeg", robotPose.getRotation().getDegrees());
        }
        
        // Log to AdvantageKit
        Logger.recordOutput("Turret/MotorPositionRotations", motorPos);
        Logger.recordOutput("Turret/CurrentTurretAngleDegrees", currentDegrees);
        Logger.recordOutput("Turret/TargetAngleDegrees", targetDegrees);
        Logger.recordOutput("Turret/VelocityRPS", velocityRPS);
        Logger.recordOutput("Turret/CurrentAmps", currentAmps);
        Logger.recordOutput("Turret/VoltageVolts", voltageVolts);
        Logger.recordOutput("Turret/AtTarget", atTarget());
    }
    
    /**
     * Command factory: Continuously track a field position
     */
    public Command trackFieldPosition(Translation2d position, java.util.function.Supplier<Pose2d> poseSupplier) {
        return run(() -> pointAtFieldPosition(position, poseSupplier.get()))
            .withName("TrackPosition");
    }
    
    /**
     * Command factory: Point to angle and wait until on target
     */
    public Command pointToAngleAndWait(Rotation2d angle) {
        return runOnce(() -> setTargetAngle(angle))
            .andThen(edu.wpi.first.wpilibj2.command.Commands.waitUntil(this::atTarget))
            .withName("PointToAngle");
    }
    
    /**
     * Command factory: Manual control with joystick
     */
    public Command manualControl(java.util.function.DoubleSupplier speedSupplier) {
        return run(() -> setManualControl(speedSupplier.getAsDouble()))
            .withName("ManualTurret");
    }
    
    /**
     * Command factory: Return to home position (0 degrees)
     */
    public Command returnToHome() {
        return pointToAngleAndWait(Rotation2d.fromDegrees(0))
            .withName("ReturnHome");
    }
}
