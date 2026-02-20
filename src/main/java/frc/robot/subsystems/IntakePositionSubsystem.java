package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakePositionConstants;

/**
 * Subsystem for controlling the intake deploy/position mechanism.
 * Controls the position of the intake (extended/stowed).
 */
public class IntakePositionSubsystem extends SubsystemBase {
    
    private final TalonFX deployMotor;
    
    // Cached status signal for performance (Phoenix 6 optimization)
    private final StatusSignal<?> positionSignal;
    
    // Control request
    private final MotionMagicVoltage positionRequest;
    
    // State tracking
    private boolean isDeployed = false;
    
    // Deploy motor override (disables deploy motor for testing)
    private boolean deployMotorDisabled = false;
    
    public IntakePositionSubsystem() {
        // Initialize motor
        deployMotor = new TalonFX(IntakePositionConstants.MOTOR_ID, IntakePositionConstants.CANBUS_NAME);
        
        // Initialize cached status signal for performance
        positionSignal = deployMotor.getPosition();
        positionSignal.setUpdateFrequency(50); // 50Hz for position
        
        // Optimize CAN bus utilization
        deployMotor.optimizeBusUtilization();
        
        // Initialize control request
        positionRequest = new MotionMagicVoltage(0);
        
        // Configure motor
        configureDeployMotor();
    }
    
    /**
     * Configure the intake deploy motor for position control
     */
    private void configureDeployMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IntakePositionConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        // Sensor to mechanism ratio (gear ratio)
        config.Feedback.SensorToMechanismRatio = IntakePositionConstants.GEAR_RATIO;
        
        // Motion Magic position configuration
        MotionMagicConfigs mmConfigs = config.MotionMagic;
        mmConfigs.MotionMagicCruiseVelocity = IntakePositionConstants.MAX_VELOCITY_RPS;
        mmConfigs.MotionMagicAcceleration = IntakePositionConstants.MAX_ACCELERATION_RPSS;
        mmConfigs.MotionMagicJerk = IntakePositionConstants.MAX_JERK_RPSSS;
        
        // Soft limits
        SoftwareLimitSwitchConfigs limitConfigs = config.SoftwareLimitSwitch;
        limitConfigs.ForwardSoftLimitEnable = true;
        limitConfigs.ForwardSoftLimitThreshold = IntakePositionConstants.MAX_SOFT_LIMIT;
        limitConfigs.ReverseSoftLimitEnable = true;
        limitConfigs.ReverseSoftLimitThreshold = IntakePositionConstants.MIN_SOFT_LIMIT;
        
        // Hard limits
        config.HardwareLimitSwitch.ForwardLimitEnable = IntakePositionConstants.ENABLE_FORWARD_HARD_LIMIT;
        config.HardwareLimitSwitch.ReverseLimitEnable = IntakePositionConstants.ENABLE_REVERSE_HARD_LIMIT;
        
        // PID configuration (placeholder values - tune these)
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 10.0;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kV = 0.0;
        
        deployMotor.getConfigurator().apply(config);
        deployMotor.setPosition(IntakePositionConstants.STOWED_POSITION_ROTATIONS);
    }
    
    /**
     * Deploy the intake to the extended position
     */
    public void deploy() {
        if (!deployMotorDisabled) {
            deployMotor.setControl(
                positionRequest.withPosition(IntakePositionConstants.EXTENDED_POSITION_ROTATIONS)
            );
            isDeployed = true;
        }
    }
    
    /**
     * Retract the intake to the stowed position
     */
    public void retract() {
        if (!deployMotorDisabled) {
            deployMotor.setControl(
                positionRequest.withPosition(IntakePositionConstants.STOWED_POSITION_ROTATIONS)
            );
            isDeployed = false;
        }
    }
    
    /**
     * Set intake to a specific position
     * @param positionRotations Position in rotations
     */
    public void setPosition(double positionRotations) {
        if (!deployMotorDisabled) {
            deployMotor.setControl(positionRequest.withPosition(positionRotations));
            isDeployed = positionRotations > IntakePositionConstants.STOWED_POSITION_ROTATIONS;
        }
    }
    
    /**
     * Check if the intake is deployed
     */
    public boolean isDeployed() {
        return isDeployed;
    }
    
    /**
     * Get the current deploy position in rotations
     */
    public double getPosition() {
        return positionSignal.getValueAsDouble();
    }
    
    /**
     * Check if at target position
     * @param targetPosition Target position in rotations
     * @param tolerance Tolerance in rotations
     */
    public boolean atTargetPosition(double targetPosition, double tolerance) {
        return Math.abs(getPosition() - targetPosition) < tolerance;
    }
    
    /**
     * Check if at extended position
     */
    public boolean atExtendedPosition() {
        return atTargetPosition(IntakePositionConstants.EXTENDED_POSITION_ROTATIONS, 0.01);
    }
    
    /**
     * Check if at stowed position
     */
    public boolean atStowedPosition() {
        return atTargetPosition(IntakePositionConstants.STOWED_POSITION_ROTATIONS, 0.01);
    }
    
    /**
     * Enable the deploy motor (normal operation)
     */
    public void enableDeployMotor() {
        deployMotorDisabled = false;
    }
    
    /**
     * Disable the deploy motor (for testing - saves wear and tear)
     * When disabled, deploy() and retract() commands are ignored.
     */
    public void disableDeployMotor() {
        deployMotorDisabled = true;
        deployMotor.stopMotor(); // Stop the motor when disabling
    }
    
    /**
     * Toggle the deploy motor enabled state
     */
    public void toggleDeployMotor() {
        if (deployMotorDisabled) {
            enableDeployMotor();
        } else {
            disableDeployMotor();
        }
    }
    
    /**
     * Check if the deploy motor is disabled
     */
    public boolean isDeployMotorDisabled() {
        return deployMotorDisabled;
    }
    
    // Telemetry counter for throttling
    private int telemetryCounter = 10; // Start at offset for staggering (different from roller)
    private static final int TELEMETRY_UPDATE_PERIOD = 25; // Update every 25 cycles (500ms)
    
    @Override
    public void periodic() {
        // Refresh cached signal
        positionSignal.refresh();
        
        // Throttle telemetry for performance
        telemetryCounter++;
        if (telemetryCounter >= TELEMETRY_UPDATE_PERIOD) {
            telemetryCounter = 0;
            SmartDashboard.putBoolean("IntakePosition/Deploy Motor Disabled", deployMotorDisabled);
            SmartDashboard.putBoolean("IntakePosition/Is Deployed", isDeployed);
            SmartDashboard.putNumber("IntakePosition/Position", getPosition());
            SmartDashboard.putBoolean("IntakePosition/At Extended", atExtendedPosition());
            SmartDashboard.putBoolean("IntakePosition/At Stowed", atStowedPosition());
        }
    }
}
