package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlywheelConstants;

/**
 * Subsystem for controlling the shooter flywheel velocity
 * Uses 3 Kraken X60 motors: 1 leader + 2 opposed followers
 */
public class FlywheelSubsystem extends SubsystemBase {
    
    private final CANBus canBus;
    private final TalonFX leaderMotor;
    private final TalonFX follower1Motor;
    private final TalonFX follower2Motor;
    private final MotionMagicVelocityVoltage velocityRequest;
    private final NeutralOut neutralRequest;
    
    // Cached status signals for performance
    private final StatusSignal<?> velocitySignal;
    private final StatusSignal<?> leaderTemperatureSignal;
    private final StatusSignal<?> follower1TemperatureSignal;
    private final StatusSignal<?> follower2TemperatureSignal;
    
    // Temperature threshold for overheating (Celsius)
    private static final double OVERHEAT_TEMPERATURE_C = 80.0;
    
    private double targetVelocityRPS = 0.0;
    
    public FlywheelSubsystem() {
        // Initialize CAN bus
        canBus = new CANBus(FlywheelConstants.CANBUS_NAME);
        
        leaderMotor = new TalonFX(FlywheelConstants.LEADER_MOTOR_ID, canBus);
        follower1Motor = new TalonFX(FlywheelConstants.FOLLOWER1_MOTOR_ID, canBus);
        follower2Motor = new TalonFX(FlywheelConstants.FOLLOWER2_MOTOR_ID, canBus);
        
        // Configure motors
        configureLeaderMotor();
        configureFollowerMotors();
        
        // Initialize cached status signals
        velocitySignal = leaderMotor.getVelocity();
        velocitySignal.setUpdateFrequency(50); // 50Hz
        
        leaderTemperatureSignal = leaderMotor.getDeviceTemp();
        leaderTemperatureSignal.setUpdateFrequency(4); // 4Hz (temperature changes slowly)
        
        follower1TemperatureSignal = follower1Motor.getDeviceTemp();
        follower1TemperatureSignal.setUpdateFrequency(4);
        
        follower2TemperatureSignal = follower2Motor.getDeviceTemp();
        follower2TemperatureSignal.setUpdateFrequency(4);
        
        leaderMotor.optimizeBusUtilization();
        follower1Motor.optimizeBusUtilization();
        follower2Motor.optimizeBusUtilization();
        
        // Initialize control requests
        velocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
        neutralRequest = new NeutralOut();
    }
    
    private void configureLeaderMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output
        config.MotorOutput.Inverted = FlywheelConstants.MOTOR_INVERTED 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Current Limits (prevent brownouts during shooting)
        config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = FlywheelConstants.ENABLE_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = FlywheelConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = FlywheelConstants.ENABLE_CURRENT_LIMIT;
        
        // Gear ratio
        config.Feedback.SensorToMechanismRatio = FlywheelConstants.GEAR_RATIO;
        
        // Velocity PID
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = FlywheelConstants.kP;
        slot0.kI = FlywheelConstants.kI;
        slot0.kD = FlywheelConstants.kD;
        slot0.kS = FlywheelConstants.kS;
        slot0.kV = FlywheelConstants.kV;
        slot0.kA = FlywheelConstants.kA;
        config.Slot0 = slot0;
        
        // Motion Magic
        MotionMagicConfigs motionMagic = new MotionMagicConfigs();
        motionMagic.MotionMagicAcceleration = FlywheelConstants.MAX_ACCELERATION_RPSS;
        motionMagic.MotionMagicJerk = FlywheelConstants.MAX_JERK_RPSSS;
        config.MotionMagic = motionMagic;
        
        leaderMotor.getConfigurator().apply(config);
    }
    
    private void configureFollowerMotors() {
        // Strip followers to default configuration before setting them up
        TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
        follower1Motor.getConfigurator().apply(defaultConfig);
        follower2Motor.getConfigurator().apply(defaultConfig);
        
        // Configure follower motors with opposed direction
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        
        // Match neutral mode to leader
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Invert for opposed direction
        followerConfig.MotorOutput.Inverted = !FlywheelConstants.MOTOR_INVERTED 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        
        // Current Limits (same as leader)
        followerConfig.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.SUPPLY_CURRENT_LIMIT;
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = FlywheelConstants.ENABLE_CURRENT_LIMIT;
        followerConfig.CurrentLimits.StatorCurrentLimit = FlywheelConstants.STATOR_CURRENT_LIMIT;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = FlywheelConstants.ENABLE_CURRENT_LIMIT;
        
        follower1Motor.getConfigurator().apply(followerConfig);
        follower2Motor.getConfigurator().apply(followerConfig);
        
    // Set followers to follow leader (alignment handled by Follower control)
    follower1Motor.setControl(new Follower(FlywheelConstants.LEADER_MOTOR_ID, MotorAlignmentValue.Aligned));
    follower2Motor.setControl(new Follower(FlywheelConstants.LEADER_MOTOR_ID, MotorAlignmentValue.Aligned));
    }
    
    /**
     * Set the target flywheel velocity
     * @param rps Target velocity in rotations per second
     */
    public void setVelocity(double rps) {
        targetVelocityRPS = Math.max(0, Math.min(FlywheelConstants.MAX_VELOCITY_RPS, rps));
    }
    
    /**
     * Get current flywheel velocity
     * @return Current velocity in RPS
     */
    public double getVelocityRPS() {
        return velocitySignal.getValueAsDouble();
    }
    
    /**
     * Get target flywheel velocity
     * @return Target velocity in RPS
     */
    public double getTargetVelocityRPS() {
        return targetVelocityRPS;
    }
    
    /**
     * Check if flywheel is at target velocity
     * @return True if within tolerance
     */
    public boolean atTargetVelocity() {
        return Math.abs(getVelocityRPS() - targetVelocityRPS) 
               < FlywheelConstants.VELOCITY_TOLERANCE_RPS;
    }
    
    /**
     * Stop the flywheel
     */
    public void stop() {
        targetVelocityRPS = 0;
        leaderMotor.setControl(neutralRequest);
        // Followers automatically follow leader to neutral
    }
    
    /**
     * Get current motor temperature in Celsius
     * @return Maximum temperature across all three motors
     */
    public double getTemperatureCelsius() {
        return Math.max(
            Math.max(leaderTemperatureSignal.getValueAsDouble(), 
                     follower1TemperatureSignal.getValueAsDouble()),
            follower2TemperatureSignal.getValueAsDouble()
        );
    }
    
    /**
     * Check if flywheel motor is overheating
     * @return True if temperature exceeds threshold
     */
    public boolean isOverheating() {
        return getTemperatureCelsius() >= OVERHEAT_TEMPERATURE_C;
    }
    
    // Telemetry counter for throttling
    private int telemetryCounter = 10; // Start at offset for staggering
    private static final int TELEMETRY_UPDATE_PERIOD = 25; // Update every 25 cycles (500ms)
    
    @Override
    public void periodic() {
        // Update motor control - targetVelocityRPS is set by default command or manual control
        leaderMotor.setControl(velocityRequest.withVelocity(targetVelocityRPS));
        // Followers automatically follow leader
        
        // Refresh cached signals
        velocitySignal.refresh();
        leaderTemperatureSignal.refresh();
        follower1TemperatureSignal.refresh();
        follower2TemperatureSignal.refresh();
        
        // Throttle telemetry for performance
        telemetryCounter++;
        if (telemetryCounter >= TELEMETRY_UPDATE_PERIOD) {
            telemetryCounter = 0;
            SmartDashboard.putNumber("Flywheel/Velocity RPS", getVelocityRPS());
            SmartDashboard.putNumber("Flywheel/Target RPS", targetVelocityRPS);
            SmartDashboard.putBoolean("Flywheel/At Target", atTargetVelocity());
            SmartDashboard.putNumber("Flywheel/Max Temperature °C", Math.round(getTemperatureCelsius() * 10.0) / 10.0);
            SmartDashboard.putBoolean("Flywheel/OVERHEATING", isOverheating());
        }
    }
}
