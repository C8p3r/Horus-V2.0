package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurboKickerConstants;

/**
 * TurboKicker subsystem - three motors for note indexing and kicking
 * - Left and Right X44 motors feed notes vertically into kicker
 * - Center kicker motor kicks notes into shooter at high speed
 */
public class TurboKickerSubsystem extends SubsystemBase {
    
    private final TalonFX leftFeedMotor;
    private final TalonFX rightFeedMotor;
    private final TalonFX kickerMotor;
    
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0).withEnableFOC(true);
    
    private double currentFeedDutyCycle = 0.0;
    private double currentKickerDutyCycle = 0.0;
    
    public TurboKickerSubsystem() {
        // Initialize motors
        leftFeedMotor = new TalonFX(TurboKickerConstants.LEFT_FEED_MOTOR_ID, TurboKickerConstants.CANBUS_NAME);
        rightFeedMotor = new TalonFX(TurboKickerConstants.RIGHT_FEED_MOTOR_ID, TurboKickerConstants.CANBUS_NAME);
        kickerMotor = new TalonFX(TurboKickerConstants.KICKER_MOTOR_ID, TurboKickerConstants.CANBUS_NAME);
        
        // Configure motors
        configureMotor(leftFeedMotor, TurboKickerConstants.LEFT_FEED_INVERTED);
        configureMotor(rightFeedMotor, TurboKickerConstants.RIGHT_FEED_INVERTED);
        configureMotor(kickerMotor, TurboKickerConstants.KICKER_INVERTED);
        
        System.out.println("[TurboKicker] Subsystem initialized with 3 motors");
    }
    
    /**
     * Configure a TurboKicker motor
     */
    private void configureMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Current limits
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = TurboKickerConstants.SUPPLY_CURRENT_LIMIT;
        currentLimits.SupplyCurrentLimitEnable = TurboKickerConstants.ENABLE_CURRENT_LIMIT;
        currentLimits.StatorCurrentLimit = TurboKickerConstants.STATOR_CURRENT_LIMIT;
        currentLimits.StatorCurrentLimitEnable = TurboKickerConstants.ENABLE_CURRENT_LIMIT;
        config.CurrentLimits = currentLimits;
        
        // Motor output
        config.MotorOutput.Inverted = inverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Apply configuration with retries
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        
        if (!status.isOK()) {
            System.out.println("[TurboKicker] Failed to configure motor " + motor.getDeviceID() + ": " + status);
        }
    }
    
    /**
     * Set duty cycle for feed motors (left/right X44s)
     */
    private void setFeedMotorsDutyCycle(double dutyCycle) {
        currentFeedDutyCycle = dutyCycle;
        leftFeedMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
        rightFeedMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }
    
    /**
     * Set duty cycle for kicker motor (center motor)
     */
    private void setKickerMotorDutyCycle(double dutyCycle) {
        currentKickerDutyCycle = dutyCycle;
        kickerMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }
    
    /**
     * Feed note at full power - runs all 3 motors
     * Feed motors bring note up, kicker kicks it into shooter
     */
    public void feed() {
        setFeedMotorsDutyCycle(TurboKickerConstants.FEED_MOTORS_DUTY_CYCLE);
        setKickerMotorDutyCycle(TurboKickerConstants.KICKER_FEED_DUTY_CYCLE);
    }
    
    /**
     * Hold note with light reverse pressure on both systems
     * Used during flywheel spinup to keep note from entering shooter prematurely
     */
    public void hold() {
        setFeedMotorsDutyCycle(TurboKickerConstants.FEED_MOTORS_HOLD_DUTY_CYCLE);
        setKickerMotorDutyCycle(TurboKickerConstants.KICKER_HOLD_DUTY_CYCLE);
    }
    
    /**
     * Reverse all motors to unjam
     */
    public void reverse() {
        setFeedMotorsDutyCycle(TurboKickerConstants.FEED_MOTORS_REVERSE_DUTY_CYCLE);
        setKickerMotorDutyCycle(TurboKickerConstants.KICKER_REVERSE_DUTY_CYCLE);
    }
    
    /**
     * Stop all motors
     */
    public void stop() {
        setFeedMotorsDutyCycle(TurboKickerConstants.OFF_DUTY_CYCLE);
        setKickerMotorDutyCycle(TurboKickerConstants.OFF_DUTY_CYCLE);
    }
    
    /**
     * Get current feed motors duty cycle
     */
    public double getFeedDutyCycle() {
        return currentFeedDutyCycle;
    }
    
    /**
     * Get current kicker motor duty cycle
     */
    public double getKickerDutyCycle() {
        return currentKickerDutyCycle;
    }
    
    @Override
    public void periodic() {
        // Telemetry could be added here if needed
    }
}
