package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
    private final DutyCycleOut dutyCycleRequest;
    
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
        
        // Initialize control request
        dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);
        
        // Configure motors
        configureMotor(upperMotor, IntakeRollerConstants.UPPER_MOTOR_INVERTED);
        configureMotor(lowerMotor, IntakeRollerConstants.LOWER_MOTOR_INVERTED);
        
        System.out.println("[IntakeRoller] Dual X60 subsystem initialized");
    }
    
    /**
     * Configure an intake roller motor for duty cycle control
     */
    private void configureMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        // Current Limits (prevent brownouts)
        config.CurrentLimits.SupplyCurrentLimit = IntakeRollerConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = IntakeRollerConstants.ENABLE_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = IntakeRollerConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = IntakeRollerConstants.ENABLE_CURRENT_LIMIT;
        
        motor.getConfigurator().apply(config);
        motor.setPosition(0);
    }
    
    /**
     * Run both intake rollers at the specified duty cycle
     * @param dutyCycle Roller duty cycle from -1.0 to 1.0 (positive = intake)
     */
    public void setDutyCycle(double dutyCycle) {
        upperMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
        lowerMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
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
     * Stop both intake rollers
     */
    public void stop() {
        upperMotor.stopMotor();
        lowerMotor.stopMotor();
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
