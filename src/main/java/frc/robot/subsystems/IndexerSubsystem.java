package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import frc.robot.util.TelemetryThrottle;


/**
 * Subsystem for managing the fuel indexer mechanism.
 * Controls:
 * - Floor indexer motor (duty cycle control) - receives fuel from intake
 * - Fire indexer motor (duty cycle control) - feeds fuel to shooter
 */
public class IndexerSubsystem extends SubsystemBase {
    
    private final CANBus canBus;
    private final TalonFX floorIndexerMotor;
    private final TalonFX fireIndexerMotor;
    private final TelemetryThrottle telemetryThrottle;
    
    // Control requests
    private final DutyCycleOut floorDutyCycleRequest;
    private final DutyCycleOut fireDutyCycleRequest;
    
    public IndexerSubsystem() {
        // Initialize CAN bus
        canBus = new CANBus(IndexerConstants.CANBUS_NAME);
        
        // Initialize motors
        floorIndexerMotor = new TalonFX(IndexerConstants.FLOOR_MOTOR_ID, canBus);
        fireIndexerMotor = new TalonFX(IndexerConstants.FIRE_MOTOR_ID, canBus);
        
        // Initialize control requests
        floorDutyCycleRequest = new DutyCycleOut(0);
        fireDutyCycleRequest = new DutyCycleOut(0);
        
        // Throttle telemetry to 5 Hz (200ms) to prevent loop overruns
        telemetryThrottle = new TelemetryThrottle(0.2);
        
        // Configure motors
        configureFloorIndexerMotor();
        configureFireIndexerMotor();
    }
    
    /**
     * Configure the floor indexer motor for duty cycle control
     */
    private void configureFloorIndexerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IndexerConstants.FLOOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        // Current Limits (prevent brownouts)
        config.CurrentLimits.SupplyCurrentLimit = IndexerConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = IndexerConstants.ENABLE_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = IndexerConstants.ENABLE_CURRENT_LIMIT;
        
        floorIndexerMotor.getConfigurator().apply(config);
        floorIndexerMotor.setPosition(0);
    }
    
    /**
     * Configure the fire indexer motor for duty cycle control
     */
    private void configureFireIndexerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IndexerConstants.FIRE_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        // Current Limits (prevent brownouts)
        config.CurrentLimits.SupplyCurrentLimit = IndexerConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = IndexerConstants.ENABLE_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = IndexerConstants.ENABLE_CURRENT_LIMIT;
        
        fireIndexerMotor.getConfigurator().apply(config);
        fireIndexerMotor.setPosition(0);
    }
    
    /**
     * Run the floor indexer at the specified duty cycle
     * @param dutyCycle Duty cycle from -1.0 to 1.0 (positive = towards shooter)
     */
    public void setFloorIndexerDutyCycle(double dutyCycle) {
        floorIndexerMotor.setControl(floorDutyCycleRequest.withOutput(dutyCycle));
    }
    
    /**
     * Run the fire indexer at the specified duty cycle
     * @param dutyCycle Duty cycle from -1.0 to 1.0 (positive = feed to shooter)
     */
    public void setFireIndexerDutyCycle(double dutyCycle) {
        fireIndexerMotor.setControl(fireDutyCycleRequest.withOutput(dutyCycle));
    }
    
    /**
     * Legacy method for velocity control - converts RPS to duty cycle
     * @param velocityRPS Velocity in rotations per second (positive = towards shooter)
     * @deprecated Use setFloorIndexerDutyCycle instead
     */
    @Deprecated
    public void setFloorIndexerVelocity(double velocityRPS) {
        // Convert RPS to approximate duty cycle (assuming max ~60 RPS at full power)
        double dutyCycle = velocityRPS / 60.0;
        setFloorIndexerDutyCycle(dutyCycle);
    }
    
    /**
     * Legacy method for velocity control - converts RPS to duty cycle
     * @param velocityRPS Velocity in rotations per second (positive = feed to shooter)
     * @deprecated Use setFireIndexerDutyCycle instead
     */
    @Deprecated
    public void setFireIndexerVelocity(double velocityRPS) {
        // Convert RPS to approximate duty cycle (assuming max ~60 RPS at full power)
        double dutyCycle = velocityRPS / 60.0;
        setFireIndexerDutyCycle(dutyCycle);
    }
    
    /**
     * Stop the floor indexer
     */
    public void stopFloorIndexer() {
        floorIndexerMotor.stopMotor();
    }
    
    /**
     * Stop the fire indexer
     */
    public void stopFireIndexer() {
        fireIndexerMotor.stopMotor();
    }
    
    /**
     * Stop both indexers
     */
    public void stopAll() {
        stopFloorIndexer();
        stopFireIndexer();
    }
    
    /**
     * Run both indexers for intaking fuel from floor
     */
    public void intakeFromFloor() {
        setFloorIndexerVelocity(IndexerConstants.FLOOR_INTAKE_VELOCITY_RPS);
        setFireIndexerVelocity(0); // Don't feed to shooter during intake
    }
    
    /**
     * Feed fuel to the shooter
     */
    public void feedShooter() {
        setFloorIndexerVelocity(IndexerConstants.FLOOR_INTAKE_VELOCITY_RPS);
        setFireIndexerVelocity(IndexerConstants.FIRE_FEED_VELOCITY_RPS);
    }
    
    /**
     * Get the current floor indexer velocity in RPS
     */
    public double getFloorIndexerVelocity() {
        return floorIndexerMotor.getVelocity().getValueAsDouble();
    }
    
    /**
     * Get the current fire indexer velocity in RPS
     */
    public double getFireIndexerVelocity() {
        return fireIndexerMotor.getVelocity().getValueAsDouble();
    }
    
    @Override
    public void periodic() {
        // Throttle telemetry updates to prevent loop overruns
        if (!telemetryThrottle.shouldUpdate()) {
            return; // Skip this update cycle
        }
        
        // Display motor velocities on dashboard (throttled to 5 Hz)
        SmartDashboard.putNumber("Indexer/Floor Indexer Velocity RPS", getFloorIndexerVelocity());
        SmartDashboard.putNumber("Indexer/Fire Indexer Velocity RPS", getFireIndexerVelocity());
    }
}
