package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;


/**
 * Subsystem for managing the fuel indexer mechanism.
 * Controls:
 * - Floor indexer motor (velocity control) - receives fuel from intake
 * - Fire indexer motor (velocity control) - feeds fuel to shooter
 */
public class IndexerSubsystem extends SubsystemBase {
    
    private final TalonFX floorIndexerMotor;
    private final TalonFX fireIndexerMotor;
    
    // Control requests
    private final MotionMagicVelocityVoltage floorVelocityRequest;
    private final MotionMagicVelocityVoltage fireVelocityRequest;
    
    public IndexerSubsystem() {
        // Initialize motors
        floorIndexerMotor = new TalonFX(IndexerConstants.FLOOR_MOTOR_ID, IndexerConstants.CANBUS_NAME);
        fireIndexerMotor = new TalonFX(IndexerConstants.FIRE_MOTOR_ID, IndexerConstants.CANBUS_NAME);
        
        // Initialize control requests
        floorVelocityRequest = new MotionMagicVelocityVoltage(0);
        fireVelocityRequest = new MotionMagicVelocityVoltage(0);
        
        // Configure motors
        configureFloorIndexerMotor();
        configureFireIndexerMotor();
    }
    
    /**
     * Configure the floor indexer motor for velocity control
     */
    private void configureFloorIndexerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IndexerConstants.FLOOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        // Motion Magic velocity configuration
        MotionMagicConfigs mmConfigs = config.MotionMagic;
        mmConfigs.MotionMagicAcceleration = IndexerConstants.FLOOR_MAX_ACCELERATION_RPSS;
        mmConfigs.MotionMagicJerk = IndexerConstants.FLOOR_MAX_JERK_RPSSS;
        
        // PID configuration (placeholder values - tune these)
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 0.1;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kV = 0.12; // Velocity feedforward
        
        floorIndexerMotor.getConfigurator().apply(config);
        floorIndexerMotor.setPosition(0);
    }
    
    /**
     * Configure the fire indexer motor for velocity control
     */
    private void configureFireIndexerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IndexerConstants.FIRE_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        // Motion Magic velocity configuration
        MotionMagicConfigs mmConfigs = config.MotionMagic;
        mmConfigs.MotionMagicAcceleration = IndexerConstants.FIRE_MAX_ACCELERATION_RPSS;
        mmConfigs.MotionMagicJerk = IndexerConstants.FIRE_MAX_JERK_RPSSS;
        
        // PID configuration (placeholder values - tune these)
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 0.1;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kV = 0.12; // Velocity feedforward
        
        fireIndexerMotor.getConfigurator().apply(config);
        fireIndexerMotor.setPosition(0);
    }
    
    /**
     * Run the floor indexer at the specified velocity
     * @param velocityRPS Velocity in rotations per second (positive = towards shooter)
     */
    public void setFloorIndexerVelocity(double velocityRPS) {
        floorIndexerMotor.setControl(floorVelocityRequest.withVelocity(velocityRPS));
    }
    
    /**
     * Run the fire indexer at the specified velocity
     * @param velocityRPS Velocity in rotations per second (positive = feed to shooter)
     */
    public void setFireIndexerVelocity(double velocityRPS) {
        fireIndexerMotor.setControl(fireVelocityRequest.withVelocity(velocityRPS));
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
        // Display motor velocities on dashboard
        SmartDashboard.putNumber("Indexer/Floor Indexer Velocity RPS", getFloorIndexerVelocity());
        SmartDashboard.putNumber("Indexer/Fire Indexer Velocity RPS", getFireIndexerVelocity());
    }
}
