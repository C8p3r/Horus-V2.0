package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlywheelConstants;

/**
 * Subsystem for controlling the shooter flywheel velocity
 */
public class FlywheelSubsystem extends SubsystemBase {
    
    private final TalonFX flywheelMotor;
    private final MotionMagicVelocityVoltage velocityRequest;
    private final NeutralOut neutralRequest;
    
    // Cached status signals for performance
    private final StatusSignal<?> velocitySignal;
    private final StatusSignal<?> temperatureSignal;
    
    // Temperature threshold for overheating (Celsius)
    private static final double OVERHEAT_TEMPERATURE_C = 80.0;
    
    private double targetVelocityRPS = 0.0;
    
    public FlywheelSubsystem() {
        flywheelMotor = new TalonFX(FlywheelConstants.MOTOR_ID, FlywheelConstants.CANBUS_NAME);
        
        // Configure motor
        configureFlywheel();
        
        // Initialize cached status signals
        velocitySignal = flywheelMotor.getVelocity();
        velocitySignal.setUpdateFrequency(50); // 50Hz
        
        temperatureSignal = flywheelMotor.getDeviceTemp();
        temperatureSignal.setUpdateFrequency(4); // 4Hz (temperature changes slowly)
        
        flywheelMotor.optimizeBusUtilization();
        
        // Initialize control requests
        velocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
        neutralRequest = new NeutralOut();
    }
    
    private void configureFlywheel() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output
        config.MotorOutput.Inverted = FlywheelConstants.MOTOR_INVERTED 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Current limits
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
        
        flywheelMotor.getConfigurator().apply(config);
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
        flywheelMotor.setControl(neutralRequest);
    }
    
    /**
     * Get current motor temperature in Celsius
     * @return Temperature in degrees Celsius
     */
    public double getTemperatureCelsius() {
        return temperatureSignal.getValueAsDouble();
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
        // Update motor control
        flywheelMotor.setControl(velocityRequest.withVelocity(targetVelocityRPS));
        
        // Refresh cached signals
        velocitySignal.refresh();
        temperatureSignal.refresh();
        
        // Throttle telemetry for performance
        telemetryCounter++;
        if (telemetryCounter >= TELEMETRY_UPDATE_PERIOD) {
            telemetryCounter = 0;
            SmartDashboard.putNumber("Flywheel/Velocity RPS", getVelocityRPS());
            SmartDashboard.putNumber("Flywheel/Target RPS", targetVelocityRPS);
            SmartDashboard.putBoolean("Flywheel/At Target", atTargetVelocity());
            SmartDashboard.putNumber("Flywheel/Temperature °C", Math.round(getTemperatureCelsius() * 10.0) / 10.0);
            SmartDashboard.putBoolean("Flywheel/OVERHEATING", isOverheating());
        }
    }
}
