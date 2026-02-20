package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HoodConstants;

/**
 * Subsystem for controlling the shooter hood angle
 */
public class HoodSubsystem extends SubsystemBase {
    
    private final TalonFX hoodMotor;
    private final MotionMagicVoltage positionRequest;
    private final NeutralOut neutralRequest;
    
    private Rotation2d targetAngle = Rotation2d.fromDegrees(HoodConstants.MIN_ANGLE_DEGREES);
    
    public HoodSubsystem() {
        hoodMotor = new TalonFX(HoodConstants.MOTOR_ID, HoodConstants.CANBUS_NAME);
        
        // Configure motor
        configureHood();
        
        // Initialize control requests
        positionRequest = new MotionMagicVoltage(0).withSlot(0);
        neutralRequest = new NeutralOut();
    }
    
    private void configureHood() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output
        config.MotorOutput.Inverted = HoodConstants.MOTOR_INVERTED 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = HoodConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = HoodConstants.ENABLE_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = HoodConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = HoodConstants.ENABLE_CURRENT_LIMIT;
        
        // Gear ratio
        config.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;
        
        // Hard limits
        config.HardwareLimitSwitch.ForwardLimitEnable = HoodConstants.ENABLE_FORWARD_HARD_LIMIT;
        config.HardwareLimitSwitch.ReverseLimitEnable = HoodConstants.ENABLE_REVERSE_HARD_LIMIT;
        
        // Soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.MAX_SOFT_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.MIN_SOFT_LIMIT;
        
        // Position PID
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = HoodConstants.kP;
        slot0.kI = HoodConstants.kI;
        slot0.kD = HoodConstants.kD;
        slot0.kS = HoodConstants.kS;
        slot0.kV = HoodConstants.kV;
        slot0.kA = HoodConstants.kA;
        config.Slot0 = slot0;
        
        // Motion Magic
        MotionMagicConfigs motionMagic = new MotionMagicConfigs();
        motionMagic.MotionMagicCruiseVelocity = HoodConstants.MAX_VELOCITY_RPS;
        motionMagic.MotionMagicAcceleration = HoodConstants.MAX_ACCELERATION_RPS2;
        motionMagic.MotionMagicJerk = HoodConstants.MAX_ACCELERATION_RPS2 * 10;
        config.MotionMagic = motionMagic;
        
        hoodMotor.getConfigurator().apply(config);
        
        // Set initial position
        hoodMotor.setPosition(HoodConstants.MIN_ANGLE_DEGREES / 360.0);
    }
    
    /**
     * Set the target hood angle
     * @param angle Target angle as Rotation2d
     */
    public void setAngle(Rotation2d angle) {
        double degrees = angle.getDegrees();
        degrees = Math.max(HoodConstants.MIN_ANGLE_DEGREES, 
                          Math.min(HoodConstants.MAX_ANGLE_DEGREES, degrees));
        targetAngle = Rotation2d.fromDegrees(degrees);
    }
    
    /**
     * Set the target hood angle in degrees
     * @param degrees Target angle in degrees
     */
    public void setAngleDegrees(double degrees) {
        setAngle(Rotation2d.fromDegrees(degrees));
    }
    
    /**
     * Get current hood angle
     * @return Current angle as Rotation2d
     */
    public Rotation2d getAngle() {
        double motorRotations = hoodMotor.getPosition().getValueAsDouble();
        double degrees = (motorRotations * 360.0) + HoodConstants.MOTOR_OFFSET_DEGREES;
        return Rotation2d.fromDegrees(degrees);
    }
    
    /**
     * Get current hood angle in degrees
     * @return Current angle in degrees
     */
    public double getAngleDegrees() {
        return getAngle().getDegrees();
    }
    
    /**
     * Get target hood angle
     * @return Target angle as Rotation2d
     */
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }
    
    /**
     * Check if hood is at target angle
     * @return True if within tolerance
     */
    public boolean atTargetAngle() {
        return Math.abs(getAngleDegrees() - targetAngle.getDegrees()) 
               < HoodConstants.ANGLE_TOLERANCE_DEGREES;
    }
    
    /**
     * Stop the hood motor
     */
    public void stop() {
        hoodMotor.setControl(neutralRequest);
    }
    
    @Override
    public void periodic() {
        // Update motor control
        hoodMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
        
        // Telemetry
        SmartDashboard.putNumber("Hood/Angle Degrees", getAngleDegrees());
        SmartDashboard.putNumber("Hood/Target Degrees", targetAngle.getDegrees());
        SmartDashboard.putBoolean("Hood/At Target", atTargetAngle());
    }
}
