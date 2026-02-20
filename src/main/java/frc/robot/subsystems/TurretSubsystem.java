package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    
    private final TalonFX turretMotor;
    private final StatusSignal<?> positionSignal;
    private final StatusSignal<?> velocitySignal;
    private final MotionMagicVoltage positionRequest;
    private Rotation2d targetAngle = new Rotation2d();
    
    public TurretSubsystem() {
        turretMotor = new TalonFX(TurretConstants.MOTOR_ID, TurretConstants.CANBUS_NAME);
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
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = TurretConstants.MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;
        
        // Soft limits to prevent over-rotation (50° to 360°)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_SOFT_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_SOFT_LIMIT;
        
        // Motion Magic configuration
        MotionMagicConfigs mmConfigs = config.MotionMagic;
        mmConfigs.MotionMagicCruiseVelocity = TurretConstants.MAX_VELOCITY_RPS;
        mmConfigs.MotionMagicAcceleration = TurretConstants.MAX_ACCELERATION_RPS2;
        mmConfigs.MotionMagicJerk = TurretConstants.MAX_ACCELERATION_RPS2 * 10.0;
        
        // PID configuration
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = TurretConstants.kP;
        slot0.kI = TurretConstants.kI;
        slot0.kD = TurretConstants.kD;
        slot0.kV = TurretConstants.kV;
        
        turretMotor.getConfigurator().apply(config);
        turretMotor.setPosition(TurretConstants.INITIAL_ANGLE_DEGREES / 360.0);
        targetAngle = Rotation2d.fromDegrees(TurretConstants.INITIAL_ANGLE_DEGREES);
    }
    
    /**
     * Sets the target turret angle with enforced limits (50° to 360°)
     * @param angle Desired target angle
     */
    public void setTargetAngle(Rotation2d angle) {
        // Clamp angle between 50° and 360°
        double degrees = angle.getDegrees();
        double clampedDegrees = Math.max(50.0, Math.min(360.0, degrees));
        
        targetAngle = Rotation2d.fromDegrees(clampedDegrees);
        turretMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }
    
    public void stop() {
        turretMotor.stopMotor();
    }
    
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(positionSignal.getValueAsDouble());
    }
    
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }
    
    /**
     * Zeros the turret position to current location (forward = 0 degrees)
     */
    public void zeroTurret() {
        turretMotor.setPosition(0);
        targetAngle = Rotation2d.kZero;
    }
    
    /**
     * Checks if the given angle is within turret limits (50° to 360°)
     * @param angle Angle to check
     * @return True if angle is within limits
     */
    public boolean isAngleInLimits(Rotation2d angle) {
        double degrees = angle.getDegrees();
        return degrees >= 50.0 && degrees <= 360.0;
    }
    
    /**
     * Checks if turret is at minimum limit (50°)
     */
    public boolean atMinLimit() {
        return getCurrentAngle().getDegrees() <= 52.0; // Small margin
    }
    
    /**
     * Checks if turret is at maximum limit (360°)
     */
    public boolean atMaxLimit() {
        return getCurrentAngle().getDegrees() >= 358.0; // Small margin
    }
    
    public boolean atTarget() {
        double error = Math.abs(getCurrentAngle().minus(targetAngle).getRotations());
        return error < TurretConstants.POSITION_TOLERANCE_ROTATIONS;
    }
    
    private int telemetryCounter = 0;
    private static final int TELEMETRY_UPDATE_PERIOD = 25;
    
    @Override
    public void periodic() {
        StatusSignal.refreshAll(positionSignal, velocitySignal);
        telemetryCounter++;
        if (telemetryCounter >= TELEMETRY_UPDATE_PERIOD) {
            telemetryCounter = 0;
            // Telemetry - angles rounded to 0.1 degree
            SmartDashboard.putNumber("Turret/Current Angle", Math.round(getCurrentAngle().getDegrees() * 10.0) / 10.0);
            SmartDashboard.putNumber("Turret/Target Angle", Math.round(targetAngle.getDegrees() * 10.0) / 10.0);
            SmartDashboard.putBoolean("Turret/At Min Limit (50°)", atMinLimit());
            SmartDashboard.putBoolean("Turret/At Max Limit (360°)", atMaxLimit());
        }
    }
}
