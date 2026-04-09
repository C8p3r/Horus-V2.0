package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeWinchConstants;

/**
 * Intake Winch Subsystem - Controls the winch motor that retracts the intake after match start
 * Uses a Kraken X60 motor with 80:1 reduction
 * Passive deployment at match start, then winch pulls it back up after alliance stations open
 */
public class IntakeWinchSubsystem extends SubsystemBase {
    
    private final TalonFX winchMotor;
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    
    private double targetPosition = IntakeWinchConstants.EXTENDED_POSITION_ROTATIONS;
    
    public IntakeWinchSubsystem() {
        // Initialize motor
        winchMotor = new TalonFX(IntakeWinchConstants.MOTOR_ID, IntakeWinchConstants.CANBUS_NAME);
        
        // Configure motor
        configureMotor();
        
        // Set initial position to extended (intake deployed)
        winchMotor.setPosition(IntakeWinchConstants.EXTENDED_POSITION_ROTATIONS);
    }
    
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // PID Configuration
        Slot0Configs pidConfigs = config.Slot0;
        pidConfigs.kP = IntakeWinchConstants.kP;
        pidConfigs.kI = IntakeWinchConstants.kI;
        pidConfigs.kD = IntakeWinchConstants.kD;
        pidConfigs.kS = IntakeWinchConstants.kS;
        pidConfigs.kV = IntakeWinchConstants.kV;
        pidConfigs.kA = IntakeWinchConstants.kA;
        
        // Motor Output Configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = IntakeWinchConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        // Soft Limits - DISABLED
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        
        // Current Limits
        config.CurrentLimits.SupplyCurrentLimit = IntakeWinchConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = IntakeWinchConstants.ENABLE_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = IntakeWinchConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = IntakeWinchConstants.ENABLE_CURRENT_LIMIT;
        
        // Apply configuration
        StatusCode status = winchMotor.getConfigurator().apply(config);
        if (!status.isOK()) {
            System.err.println("Failed to configure IntakeWinch motor: " + status);
        }
    }
    
    /**
     * Retract the intake (pull up via winch)
     */
    public void retract() {
        setTargetPosition(IntakeWinchConstants.RETRACTED_POSITION_ROTATIONS);
    }
    
    /**
     * Extend the intake (lower via winch)
     */
    public void extend() {
        setTargetPosition(IntakeWinchConstants.EXTENDED_POSITION_ROTATIONS);
    }
    
    /**
     * Set target position in rotations (after gear ratio)
     */
    public void setTargetPosition(double rotations) {
        targetPosition = rotations;
        winchMotor.setControl(positionRequest.withPosition(rotations));
    }
    
    /**
     * Get current position in rotations (after gear ratio)
     */
    public double getPosition() {
        return winchMotor.getPosition().getValueAsDouble();
    }
    
    /**
     * Check if winch is at target position
     */
    public boolean atTarget() {
        return Math.abs(getPosition() - targetPosition) < IntakeWinchConstants.POSITION_TOLERANCE_ROTATIONS;
    }
    
    /**
     * Check if intake is fully retracted
     */
    public boolean isRetracted() {
        return Math.abs(getPosition() - IntakeWinchConstants.RETRACTED_POSITION_ROTATIONS) 
            < IntakeWinchConstants.POSITION_TOLERANCE_ROTATIONS;
    }
    
    /**
     * Check if intake is fully extended
     */
    public boolean isExtended() {
        return Math.abs(getPosition() - IntakeWinchConstants.EXTENDED_POSITION_ROTATIONS) 
            < IntakeWinchConstants.POSITION_TOLERANCE_ROTATIONS;
    }
    
    /**
     * Stop the winch motor
     */
    public void stop() {
        winchMotor.stopMotor();
    }
    
    @Override
    public void periodic() {
        // Telemetry could go here if needed
    }
}
