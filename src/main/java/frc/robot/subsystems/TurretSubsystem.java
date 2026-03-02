package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;
import frc.robot.util.ShootingCalculator;

import java.util.function.Supplier;

/**
 * Turret subsystem with full 360° rotation capability
 * Range: -50° to 312° (362° total with 2° overlap at 180°)
 * Features predictive tracking to compensate for robot motion
 */
public class TurretSubsystem extends SubsystemBase {
    
    private final CANBus canBus;
    private final TalonFX turretMotor;
    private final StatusSignal<?> positionSignal;
    private final StatusSignal<?> velocitySignal;
    private final MotionMagicVoltage positionRequest;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    
    // Target tracking
    private Translation3d currentTarget = null;
    
    // Motion compensation tuning
    private static final double TRANSLATION_COMPENSATION_FACTOR = 1.2; // 20% overcorrection for translation
    private static final double ROTATION_COMPENSATION_FACTOR = 1.3;     // 30% overcorrection for rotation
    private static final double LOOKAHEAD_TIME_SECONDS = 0.15;          // How far ahead to predict (150ms)
    
    public TurretSubsystem(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        
        canBus = new CANBus(TurretConstants.CANBUS_NAME);
        turretMotor = new TalonFX(TurretConstants.MOTOR_ID, canBus);
        positionSignal = turretMotor.getPosition();
        velocitySignal = turretMotor.getVelocity();
        positionSignal.setUpdateFrequency(100);
        velocitySignal.setUpdateFrequency(50);
        turretMotor.optimizeBusUtilization();
        positionRequest = new MotionMagicVoltage(0);
        
        configureTurretMotor();
    }
    
    private void configureTurretMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = TurretConstants.MOTOR_INVERTED ? 
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;
        
        // Software limits: -50° to 312° (full 360° coverage)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_SOFT_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_SOFT_LIMIT;
        
        // Motion Magic
        MotionMagicConfigs mmConfigs = config.MotionMagic;
        mmConfigs.MotionMagicCruiseVelocity = TurretConstants.MAX_VELOCITY_RPS;
        mmConfigs.MotionMagicAcceleration = TurretConstants.MAX_ACCELERATION_RPS2;
        mmConfigs.MotionMagicJerk = TurretConstants.MAX_ACCELERATION_RPS2 * 10.0;
        
        // PID
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = TurretConstants.kP;
        slot0.kI = TurretConstants.kI;
        slot0.kD = TurretConstants.kD;
        slot0.kV = TurretConstants.kV;
        
        turretMotor.getConfigurator().apply(config);
        turretMotor.setPosition(TurretConstants.INITIAL_ANGLE_DEGREES / 360.0);
    }
    
    // ==================== BASIC CONTROL ====================
    
    /**
     * Sets the turret to a specific angle
     * @param angle Target angle in degrees
     */
    public void setAngle(double degrees) {
        turretMotor.setControl(positionRequest.withPosition(degrees / 360.0));
    }
    
    /**
     * Gets the current turret angle
     * @return Current angle in degrees
     */
    public double getAngleDegrees() {
        return positionSignal.getValueAsDouble() * 360.0;
    }
    
    /**
     * Checks if turret is at target angle
     */
    public boolean atTarget() {
        double error = Math.abs(velocitySignal.getValueAsDouble());
        return error < TurretConstants.VELOCITY_TOLERANCE_RPS;
    }
    
    /**
     * Zeros the turret encoder at current position
     */
    public void zeroTurret() {
        turretMotor.setPosition(0);
    }
    
    /**
     * Sets encoder to specific value (for calibration)
     */
    public void setEncoderPosition(double degrees) {
        turretMotor.setPosition(degrees / 360.0);
    }
    
    /**
     * Sets encoder position using Rotation2d (for backward compatibility)
     */
    public void setEncoderPosition(Rotation2d angle) {
        setEncoderPosition(angle.getDegrees());
    }
    
    /**
     * Returns the turret to its initial position (180° - facing backward)
     * Called when robot is disabled
     */
    public void returnToInitialPosition() {
        setAngle(TurretConstants.INITIAL_ANGLE_DEGREES);
        // Disable tracking when returning to initial position
        setTarget(null);
    }
    
    // ==================== TARGET TRACKING ====================
    
    /**
     * Sets the target to track (null to disable tracking)
     */
    public void setTarget(Translation3d target) {
        this.currentTarget = target;
    }
    
    /**
     * Gets the current tracking target
     */
    public Translation3d getTarget() {
        return currentTarget;
    }
    
    /**
     * Checks if tracking is enabled
     */
    public boolean isTracking() {
        return currentTarget != null;
    }
    
    /**
     * Chassis rotation assist - always 0 with full 360° coverage
     */
    public double getChassisRotationAssist() {
        return 0.0;
    }
    
    /**
     * Updates turret to track current target with predictive motion compensation
     */
    private void updateTracking() {
        if (!isTracking()) {
            // Clear visualization
            Logger.recordOutput("TrackTarget/TargetPose", new Pose3d());
            Logger.recordOutput("TrackTarget/LineToTarget", new Pose3d[] {});
            Logger.recordOutput("TrackTarget/TurretAimLine", new Pose3d[] {});
            Logger.recordOutput("TrackTarget/PredictedPose", new Pose3d());
            return;
        }
        
        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds robotSpeeds = chassisSpeedsSupplier.get();
        Translation2d target2d = currentTarget.toTranslation2d();
        
        // ===== PREDICTIVE MOTION COMPENSATION =====
        // Predict where the robot will be based on current velocity
        double vx = robotSpeeds.vxMetersPerSecond;
        double vy = robotSpeeds.vyMetersPerSecond;
        double omega = robotSpeeds.omegaRadiansPerSecond;
        
        // Calculate translation velocity magnitude
        double translationSpeed = Math.sqrt(vx * vx + vy * vy);
        
        // Predict future robot pose with overcorrection
        double predictedX = robotPose.getX() + (vx * LOOKAHEAD_TIME_SECONDS * TRANSLATION_COMPENSATION_FACTOR);
        double predictedY = robotPose.getY() + (vy * LOOKAHEAD_TIME_SECONDS * TRANSLATION_COMPENSATION_FACTOR);
        double predictedRotation = robotPose.getRotation().getRadians() + 
                                   (omega * LOOKAHEAD_TIME_SECONDS * ROTATION_COMPENSATION_FACTOR);
        
        Pose2d predictedPose = new Pose2d(predictedX, predictedY, new Rotation2d(predictedRotation));
        
        // Calculate angle from predicted position to target
        Translation2d predictedToTarget = target2d.minus(predictedPose.getTranslation());
        Rotation2d fieldAngleToTarget = new Rotation2d(predictedToTarget.getX(), predictedToTarget.getY());
        
        // Convert to robot-relative angle (0° = forward)
        // Use predicted rotation for better compensation
        Rotation2d robotRelativeAngle = fieldAngleToTarget.minus(predictedPose.getRotation());
        double desiredAngle = robotRelativeAngle.getDegrees();
        
        // Get current turret position
        double currentAngle = getAngleDegrees();
        
        // Find the best angle within turret limits that points at target
        double commandAngle = findBestAngle(desiredAngle, currentAngle);
        
        // Command turret
        setAngle(commandAngle);
        
        // ===== VISUALIZATION =====
        Pose3d targetPose3d = new Pose3d(currentTarget, new Rotation3d());
        Logger.recordOutput("TrackTarget/TargetPose", targetPose3d);
        
        Pose3d robotPose3d = new Pose3d(robotPose);
        Pose3d predictedPose3d = new Pose3d(predictedPose);
        Logger.recordOutput("TrackTarget/PredictedPose", predictedPose3d);
        Logger.recordOutput("TrackTarget/LineToTarget", new Pose3d[] { robotPose3d, targetPose3d });
        
        // Turret aim line from predicted position
        Rotation2d turretFieldHeading = predictedPose.getRotation().plus(Rotation2d.fromDegrees(commandAngle));
        Translation2d aimEnd = predictedPose.getTranslation().plus(new Translation2d(5.0, turretFieldHeading));
        Pose3d aimEndPose = new Pose3d(aimEnd.getX(), aimEnd.getY(), currentTarget.getZ(), new Rotation3d());
        Logger.recordOutput("TrackTarget/TurretAimLine", new Pose3d[] { predictedPose3d, aimEndPose });
        
        // Motion telemetry
        Logger.recordOutput("TrackTarget/RobotVelocityX", vx);
        Logger.recordOutput("TrackTarget/RobotVelocityY", vy);
        Logger.recordOutput("TrackTarget/RobotOmega", Math.toDegrees(omega));
        Logger.recordOutput("TrackTarget/TranslationSpeed", translationSpeed);
        Logger.recordOutput("TrackTarget/RotationSpeed", Math.abs(Math.toDegrees(omega)));
        
        // Tracking telemetry
        Logger.recordOutput("TrackTarget/DesiredAngle", desiredAngle);
        Logger.recordOutput("TrackTarget/CommandAngle", commandAngle);
        Logger.recordOutput("TrackTarget/CurrentAngle", currentAngle);
        Logger.recordOutput("TrackTarget/Distance", predictedToTarget.getNorm());
        Logger.recordOutput("TrackTarget/AtTarget", atTarget());
    }
    
    /**
     * Finds the best angle to command given a desired direction and current position.
     * Handles the turret's -50° to 312° range and picks the shortest path.
     * 
     * @param desiredAngle The desired angle in degrees (can be any value -180 to 180)
     * @param currentAngle The current turret angle in degrees
     * @return The best angle to command within turret limits
     */
    private double findBestAngle(double desiredAngle, double currentAngle) {
        // Turret range: -50° to 312°
        // Since this covers 362°, any angle can be reached, but there are two ways:
        // 1. The angle directly (if in range)
        // 2. The angle +/- 360° (wrapped version)
        
        // Normalize desired angle to -180 to 180 range
        while (desiredAngle > 180.0) desiredAngle -= 360.0;
        while (desiredAngle <= -180.0) desiredAngle += 360.0;
        
        // Generate candidate angles (same direction, different representations)
        double[] candidates = {
            desiredAngle,           // -180 to 180 range
            desiredAngle + 360.0,   // 180 to 540 range
            desiredAngle - 360.0    // -540 to -180 range
        };
        
        // Find the candidate that's:
        // 1. Within limits (-50° to 312°)
        // 2. Closest to current position
        double bestAngle = desiredAngle;
        double bestDistance = Double.MAX_VALUE;
        
        for (double candidate : candidates) {
            // Check if within limits
            if (candidate >= TurretConstants.MIN_POSITION_DEGREES && 
                candidate <= TurretConstants.MAX_POSITION_DEGREES) {
                
                // Calculate distance from current position
                double distance = Math.abs(candidate - currentAngle);
                
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestAngle = candidate;
                }
            }
        }
        
        return bestAngle;
    }
    
    @Override
    public void periodic() {
        StatusSignal.refreshAll(positionSignal, velocitySignal);
        
        updateTracking();
        
        // Update smart shooting trajectory visualization continuously
        if (isTracking()) {
            Pose2d robotPose = robotPoseSupplier.get();
            var solution = ShootingCalculator.calculate(robotPose, currentTarget);
            // Solution automatically logs to AdvantageKit
        } else {
            // Clear shooting visualization when not tracking
            Logger.recordOutput("SmartShoot/Valid", false);
            Logger.recordOutput("SmartShoot/Trajectory", new Pose3d[0]);
        }
        
        // Turret pose visualization
        Pose2d robotPose = robotPoseSupplier.get();
        double angle = getAngleDegrees();
        Rotation2d turretFieldHeading = robotPose.getRotation().plus(Rotation2d.fromDegrees(angle));
        Pose3d turretPose = new Pose3d(
            robotPose.getX(), 
            robotPose.getY(), 
            TurretConstants.OFFSET_Z,
            new Rotation3d(0, 0, turretFieldHeading.getRadians())
        );
        Logger.recordOutput("Turret/Pose3d", turretPose);
        
        // Basic telemetry
        Logger.recordOutput("Turret/Angle", angle);
        Logger.recordOutput("Turret/Velocity", velocitySignal.getValueAsDouble());
        Logger.recordOutput("Turret/Tracking", isTracking());
        Logger.recordOutput("Turret/AtTarget", atTarget());
    }
}

