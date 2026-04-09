package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeRollerConstants;

/**
 * Subsystem for controlling the intake roller motors.
 * NOW USES DUAL KRAKEN X60 MOTORS (upper and lower, opposed configuration)
 * Controls both rollers via duty cycle to intake or eject game pieces.
 */
public class IntakeRollerSubsystem extends SubsystemBase {
    
    private final TalonFX upperMotor;
    private final TalonFX lowerMotor;
    
    // Cached status signal for performance (Phoenix 6 optimization)
    private final StatusSignal<?> velocitySignal;
    
    // Control request
    private final VelocityVoltage velocityRequest;
    
    public IntakeRollerSubsystem() {
        // Initialize motors
        upperMotor = new TalonFX(IntakeRollerConstants.UPPER_MOTOR_ID, IntakeRollerConstants.CANBUS_NAME);
        lowerMotor = new TalonFX(IntakeRollerConstants.LOWER_MOTOR_ID, IntakeRollerConstants.CANBUS_NAME);
        
        // Initialize cached status signal for performance (use upper motor)
        velocitySignal = upperMotor.getVelocity();
        velocitySignal.setUpdateFrequency(50); // 50Hz for velocity
        
        // Optimize CAN bus utilization
        upperMotor.optimizeBusUtilization();
        lowerMotor.optimizeBusUtilization();
        
        // Initialize control request for velocity voltage control
        velocityRequest = new VelocityVoltage(0).withSlot(0);
        
        // Configure upper motor for velocity voltage control with PID
        configureMotor(upperMotor, IntakeRollerConstants.UPPER_MOTOR_INVERTED);
        
        // Configure lower motor as opposed follower of upper motor
        configureLowerAsFollower();
        
        System.out.println("[IntakeRoller] Dual X60 subsystem initialized with velocity voltage control and lower as opposed follower");
    }
    
    /**
     * Configure an intake roller motor for velocity voltage control with PID
     */
    private void configureMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        // Velocity PID Configuration (for velocity voltage control)
        config.Slot0.kP = IntakeRollerConstants.kP;
        config.Slot0.kI = IntakeRollerConstants.kI;
        config.Slot0.kD = IntakeRollerConstants.kD;
        config.Slot0.kS = IntakeRollerConstants.kS;
        config.Slot0.kV = IntakeRollerConstants.kV;
        config.Slot0.kA = IntakeRollerConstants.kA;
        
        // Current Limits (prevent brownouts)
        config.CurrentLimits.SupplyCurrentLimit = IntakeRollerConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = IntakeRollerConstants.ENABLE_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = IntakeRollerConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = IntakeRollerConstants.ENABLE_CURRENT_LIMIT;
        
        // NO software limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        
        motor.getConfigurator().apply(config);
        motor.setPosition(0);
    }
    
    /**
     * Configure lower motor as an opposed follower of the upper motor (NO PID)
     */
    private void configureLowerAsFollower() {
        // Clear any existing configuration first
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Factory default will clear all settings
        lowerMotor.getConfigurator().apply(config);
        
        // Now apply only the necessary settings for a follower
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        
        // Motor output settings
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Will be inverted by follower mode
        
        // Current Limits (prevent brownouts)
        followerConfig.CurrentLimits.SupplyCurrentLimit = IntakeRollerConstants.SUPPLY_CURRENT_LIMIT;
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = IntakeRollerConstants.ENABLE_CURRENT_LIMIT;
        followerConfig.CurrentLimits.StatorCurrentLimit = IntakeRollerConstants.STATOR_CURRENT_LIMIT;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = IntakeRollerConstants.ENABLE_CURRENT_LIMIT;
        
        // NO software limits - clear them explicitly
        followerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        followerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        
        // NO PID - Simple duty cycle only (follower uses parent's control)
        followerConfig.Slot0.kP = 0.0;
        followerConfig.Slot0.kI = 0.0;
        followerConfig.Slot0.kD = 0.0;
        followerConfig.Slot0.kS = 0.0;
        followerConfig.Slot0.kV = 0.0;
        followerConfig.Slot0.kA = 0.0;
        
        lowerMotor.getConfigurator().apply(followerConfig);
        
        // Configure as opposed follower of upper motor
        lowerMotor.setControl(new Follower(IntakeRollerConstants.UPPER_MOTOR_ID, MotorAlignmentValue.Aligned));
        
        System.out.println("[IntakeRoller] Lower motor configured as opposed follower (CAN ID " + IntakeRollerConstants.LOWER_MOTOR_ID + ") of upper motor (CAN ID " + IntakeRollerConstants.UPPER_MOTOR_ID + ")");
    }
    
    /**
     * Run both intake rollers at the specified velocity using velocity voltage control
     * Lower motor automatically follows upper with opposite direction
     * @param velocityRPS Target roller velocity in rotations per second (positive = intake)
     */
    public void setVelocity(double velocityRPS) {
        upperMotor.setControl(velocityRequest.withVelocity(velocityRPS));
        // Lower motor follows automatically as opposed follower
    }
    
    /**
     * Run intake rollers at the specified duty cycle (direct velocity mapping)
     * @param dutyCycle Roller duty cycle from -1.0 to 1.0 (maps to velocity RPS)
     */
    public void setDutyCycle(double dutyCycle) {
        // Map duty cycle to velocity RPS
        // Positive duty cycle = intake (use INTAKE_VELOCITY_RPS)
        // Negative duty cycle = eject (use EJECT_VELOCITY_RPS)
        double targetVelocityRPS = dutyCycle >= 0 
            ? dutyCycle * IntakeRollerConstants.INTAKE_VELOCITY_RPS
            : dutyCycle * Math.abs(IntakeRollerConstants.EJECT_VELOCITY_RPS);
        
        setVelocity(targetVelocityRPS);
    }
    
    /**
     * Legacy method for velocity control
     * @param velocityRPS Roller velocity in rotations per second (positive = intake)
     * @deprecated Use setVelocity instead
     */
    @Deprecated
    public void setVelocityLegacy(double velocityRPS) {
        // Convert RPS to approximate duty cycle (assuming max ~40 RPS at full power)
        double dutyCycle = velocityRPS / 40.0;
        setDutyCycle(dutyCycle);
    }
    
    /**
     * Stop both intake rollers
     */
    public void stop() {
        upperMotor.stopMotor();
        // Lower motor is follower, stops automatically
    }
    
    /**
     * Get the current upper roller velocity in RPS
     */
    public double getVelocity() {
        return velocitySignal.getValueAsDouble();
    }
    
    /**
     * Check if roller is at target velocity
     * @param targetVelocity Target velocity in RPS
     * @param tolerance Tolerance in RPS
     */
    public boolean atTargetVelocity(double targetVelocity, double tolerance) {
        return Math.abs(getVelocity() - targetVelocity) < tolerance;
    }
    
    // Telemetry counter for throttling
    private int telemetryCounter = 5; // Start at offset for staggering
    private static final int TELEMETRY_UPDATE_PERIOD = 25; // Update every 25 cycles (500ms)
    
    @Override
    public void periodic() {
        // Refresh cached signal
        velocitySignal.refresh();
        
        // Throttle telemetry for performance
        telemetryCounter++;
        if (telemetryCounter >= TELEMETRY_UPDATE_PERIOD) {
            telemetryCounter = 0;
            SmartDashboard.putNumber("IntakeRoller/UpperVelocity RPS", getVelocity());
        }
    }
}
