package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeRollerConstants;

/**
 * Subsystem for controlling the intake roller motor.
 * Controls the roller duty cycle to intake or eject game pieces.
 */
public class IntakeRollerSubsystem extends SubsystemBase {
    
    private final CANBus canBus;
    private final TalonFX rollerMotor;
    
    // Cached status signal for performance (Phoenix 6 optimization)
    private final StatusSignal<?> velocitySignal;
    
    // Control request
    private final DutyCycleOut dutyCycleRequest;
    
    public IntakeRollerSubsystem() {
        // Initialize CAN bus
        canBus = new CANBus(IntakeRollerConstants.CANBUS_NAME);
        
        // Initialize motor
        rollerMotor = new TalonFX(IntakeRollerConstants.MOTOR_ID, canBus);
        
        // Initialize cached status signal for performance
        velocitySignal = rollerMotor.getVelocity();
        velocitySignal.setUpdateFrequency(50); // 50Hz for velocity
        
        // Optimize CAN bus utilization
        rollerMotor.optimizeBusUtilization();
        
        // Initialize control request
        dutyCycleRequest = new DutyCycleOut(0);
        
        // Configure motor
        configureRollerMotor();
    }
    
    /**
     * Configure the intake roller motor for duty cycle control
     */
    private void configureRollerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IntakeRollerConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        // Current Limits (prevent brownouts)
        config.CurrentLimits.SupplyCurrentLimit = IntakeRollerConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = IntakeRollerConstants.ENABLE_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = IntakeRollerConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = IntakeRollerConstants.ENABLE_CURRENT_LIMIT;
        
        rollerMotor.getConfigurator().apply(config);
        rollerMotor.setPosition(0);
    }
    
    /**
     * Run the intake roller at the specified duty cycle
     * @param dutyCycle Roller duty cycle from -1.0 to 1.0 (positive = intake)
     */
    public void setDutyCycle(double dutyCycle) {
        rollerMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }
    
    /**
     * Legacy method for velocity control - converts RPS to duty cycle
     * @param velocityRPS Roller velocity in rotations per second (positive = intake)
     * @deprecated Use setDutyCycle instead
     */
    @Deprecated
    public void setVelocity(double velocityRPS) {
        // Convert RPS to approximate duty cycle (assuming max ~40 RPS at full power)
        double dutyCycle = velocityRPS / 40.0;
        setDutyCycle(dutyCycle);
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
