package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.util.TelemetryManager;
import frc.robot.util.TelemetryThrottle;

/**
 * Subsystem for controlling the climber mechanism.
 * Uses a Kraken X60 motor with 100:1 reduction to spool/unspool a winch.
 * Includes a ratchet servo for locking the mechanism when deployed.
 */
public class ClimberSubsystem extends SubsystemBase {
    
    private final CANBus canBus;
    private final TalonFX climbMotor;
    private final Servo ratchetServo;
    private final TelemetryThrottle telemetryThrottle;
    
    // Cached status signals for performance
    private final StatusSignal<?> positionSignal;
    private final StatusSignal<?> velocitySignal;
    
    // Control request
    private final MotionMagicVoltage positionRequest;
    
    // State tracking
    private boolean isDeployed = false;
    
    public ClimberSubsystem() {
        // Initialize CAN bus
        canBus = new CANBus(ClimberConstants.CANBUS_NAME);
        
        // Initialize motor
        climbMotor = new TalonFX(ClimberConstants.CLIMB_MOTOR_ID, canBus);
        
        // Initialize servo
        ratchetServo = new Servo(ClimberConstants.RATCHET_SERVO_PWM);
        
        // Throttle telemetry to 5 Hz (200ms) to prevent loop overruns
        telemetryThrottle = new TelemetryThrottle(0.2);
        
        // Initialize cached status signals
        positionSignal = climbMotor.getPosition();
        velocitySignal = climbMotor.getVelocity();
        positionSignal.setUpdateFrequency(50); // 50Hz
        velocitySignal.setUpdateFrequency(50);
        
        // Optimize CAN bus utilization
        climbMotor.optimizeBusUtilization();
        
        // Initialize control request
        positionRequest = new MotionMagicVoltage(0);
        
        // Configure motor
        configureClimbMotor();
        
        // Set initial servo position (ratchet disabled)
        disableRatchet();
    }
    
    /**
     * Configure the climb motor for position control
     */
    private void configureClimbMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = ClimberConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        // Motion Magic configuration
        MotionMagicConfigs mmConfigs = config.MotionMagic;
        mmConfigs.MotionMagicAcceleration = ClimberConstants.MAX_ACCELERATION_RPSS;
        mmConfigs.MotionMagicJerk = ClimberConstants.MAX_JERK_RPSSS;
        
        // PID configuration (tune these values)
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 24.0;
        slot0.kI = 0.0;
        slot0.kD = 0.2;
        slot0.kV = 0.12; // Velocity feedforward
        
        // Soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.MAX_SOFT_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.MIN_SOFT_LIMIT;
        
        // Current Limits (prevent brownouts during climb)
        config.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = ClimberConstants.ENABLE_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = ClimberConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = ClimberConstants.ENABLE_CURRENT_LIMIT;
        
        climbMotor.getConfigurator().apply(config);
        
        // Set initial position to retracted
        climbMotor.setPosition(ClimberConstants.RETRACTED_POSITION_ROTATIONS);
    }
    
    /**
     * Deploy the climber (unspool winch)
     */
    public void deploy() {
        setTargetPosition(ClimberConstants.DEPLOYED_POSITION_ROTATIONS);
        isDeployed = true;
    }
    
    /**
     * Retract the climber (spool winch)
     */
    public void retract() {
        setTargetPosition(ClimberConstants.RETRACTED_POSITION_ROTATIONS);
        isDeployed = false;
    }
    
    /**
     * Set target position in motor rotations
     */
    public void setTargetPosition(double positionRotations) {
        climbMotor.setControl(positionRequest.withPosition(positionRotations));
    }
    
    /**
     * Enable the ratchet (lock mechanism)
     */
    public void enableRatchet() {
        ratchetServo.set(ClimberConstants.RATCHET_ENABLED_POSITION);
    }
    
    /**
     * Disable the ratchet (unlock mechanism)
     */
    public void disableRatchet() {
        ratchetServo.set(ClimberConstants.RATCHET_DISABLED_POSITION);
    }
    
    /**
     * Stop the climb motor
     */
    public void stop() {
        climbMotor.stopMotor();
    }
    
    /**
     * Get the current position in motor rotations
     */
    public double getPosition() {
        return positionSignal.getValueAsDouble();
    }
    
    /**
     * Get the current velocity in motor RPS
     */
    public double getVelocity() {
        return velocitySignal.getValueAsDouble();
    }
    
    /**
     * Check if at target position
     */
    public boolean atTargetPosition(double targetPosition) {
        return Math.abs(getPosition() - targetPosition) < ClimberConstants.POSITION_TOLERANCE_ROTATIONS;
    }
    
    /**
     * Check if deployed
     */
    public boolean isDeployed() {
        return isDeployed;
    }
    
    /**
     * Check if fully retracted
     */
    public boolean isRetracted() {
        return atTargetPosition(ClimberConstants.RETRACTED_POSITION_ROTATIONS);
    }
    
    /**
     * Check if fully deployed
     */
    public boolean isFullyDeployed() {
        return atTargetPosition(ClimberConstants.DEPLOYED_POSITION_ROTATIONS);
    }
    
    @Override
    public void periodic() {
        // Refresh cached signals
        positionSignal.refresh();
        velocitySignal.refresh();
        
        // Throttle telemetry updates to prevent loop overruns
        if (!telemetryThrottle.shouldUpdate()) {
            return; // Skip this update cycle
        }
        
        // Telemetry (if not disabled) - throttled to 5 Hz
        if (TelemetryManager.isEnabled()) {
            SmartDashboard.putNumber("Climber/Position (rotations)", getPosition());
            SmartDashboard.putNumber("Climber/Velocity (RPS)", getVelocity());
            SmartDashboard.putBoolean("Climber/Is Deployed", isDeployed());
            SmartDashboard.putBoolean("Climber/Is Retracted", isRetracted());
            SmartDashboard.putBoolean("Climber/Is Fully Deployed", isFullyDeployed());
        }
    }
}
