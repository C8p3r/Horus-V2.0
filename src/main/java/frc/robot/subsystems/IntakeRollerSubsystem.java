package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeRollerConstants;

/**
 * Subsystem for controlling the intake roller motor.
 * Controls the roller velocity to intake or eject game pieces.
 */
public class IntakeRollerSubsystem extends SubsystemBase {
    
    private final TalonFX rollerMotor;
    
    // Cached status signal for performance (Phoenix 6 optimization)
    private final StatusSignal<?> velocitySignal;
    
    // Control request
    private final MotionMagicVelocityVoltage velocityRequest;
    
    public IntakeRollerSubsystem() {
        // Initialize motor
        rollerMotor = new TalonFX(IntakeRollerConstants.MOTOR_ID, IntakeRollerConstants.CANBUS_NAME);
        
        // Initialize cached status signal for performance
        velocitySignal = rollerMotor.getVelocity();
        velocitySignal.setUpdateFrequency(50); // 50Hz for velocity
        
        // Optimize CAN bus utilization
        rollerMotor.optimizeBusUtilization();
        
        // Initialize control request
        velocityRequest = new MotionMagicVelocityVoltage(0);
        
        // Configure motor
        configureRollerMotor();
    }
    
    /**
     * Configure the intake roller motor for velocity control
     */
    private void configureRollerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IntakeRollerConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        // Motion Magic velocity configuration
        MotionMagicConfigs mmConfigs = config.MotionMagic;
        mmConfigs.MotionMagicAcceleration = IntakeRollerConstants.MAX_ACCELERATION_RPSS;
        mmConfigs.MotionMagicJerk = IntakeRollerConstants.MAX_JERK_RPSSS;
        
        // PID configuration (placeholder values - tune these)
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 0.1;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kV = 0.12; // Velocity feedforward
        
        rollerMotor.getConfigurator().apply(config);
        rollerMotor.setPosition(0);
    }
    
    /**
     * Run the intake roller at the specified velocity
     * @param velocityRPS Roller velocity in rotations per second (positive = intake)
     */
    public void setVelocity(double velocityRPS) {
        rollerMotor.setControl(velocityRequest.withVelocity(velocityRPS));
    }
    
    /**
     * Stop the intake roller
     */
    public void stop() {
        rollerMotor.stopMotor();
    }
    
    /**
     * Get the current roller velocity in RPS
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
            SmartDashboard.putNumber("IntakeRoller/Velocity RPS", getVelocity());
        }
    }
}
