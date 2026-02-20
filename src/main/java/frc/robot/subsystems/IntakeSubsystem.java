package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;


/**
 * Subsystem for managing the intake mechanism.
 * Controls:
 * - Intake roller motor (velocity control) - spins to grab fuel
 * - Intake deploy motor (position control) - extends/retracts intake
 */
public class IntakeSubsystem extends SubsystemBase {
    
    private final TalonFX intakeRollerMotor;
    private final TalonFX intakeDeployMotor;
    
    // Control requests
    private final MotionMagicVelocityVoltage rollerVelocityRequest;
    private final MotionMagicVoltage deployPositionRequest;
    
    // State tracking
    private boolean isDeployed = false;
    
    // Deploy motor override (disables deploy motor for testing)
    private boolean deployMotorDisabled = false;
    
    public IntakeSubsystem() {
        // Initialize motors
        intakeRollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, IntakeConstants.CANBUS_NAME);
        intakeDeployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID, IntakeConstants.CANBUS_NAME);
        
        // Initialize control requests
        rollerVelocityRequest = new MotionMagicVelocityVoltage(0);
        deployPositionRequest = new MotionMagicVoltage(0);
        
        // Configure motors
        configureIntakeRollerMotor();
        configureIntakeDeployMotor();
    }
    
    /**
     * Configure the intake roller motor for velocity control
     */
    private void configureIntakeRollerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IntakeConstants.ROLLER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        // Motion Magic velocity configuration
        MotionMagicConfigs mmConfigs = config.MotionMagic;
        mmConfigs.MotionMagicAcceleration = IntakeConstants.ROLLER_MAX_ACCELERATION_RPSS;
        mmConfigs.MotionMagicJerk = IntakeConstants.ROLLER_MAX_JERK_RPSSS;
        
        // PID configuration (placeholder values - tune these)
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 0.1;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kV = 0.12; // Velocity feedforward
        
        intakeRollerMotor.getConfigurator().apply(config);
        intakeRollerMotor.setPosition(0);
    }
    
    /**
     * Configure the intake deploy motor for position control
     */
    private void configureIntakeDeployMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IntakeConstants.DEPLOY_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        // Sensor to mechanism ratio (gear ratio)
        config.Feedback.SensorToMechanismRatio = IntakeConstants.DEPLOY_GEAR_RATIO;
        
        // Motion Magic position configuration
        MotionMagicConfigs mmConfigs = config.MotionMagic;
        mmConfigs.MotionMagicCruiseVelocity = IntakeConstants.DEPLOY_MAX_VELOCITY_RPS;
        mmConfigs.MotionMagicAcceleration = IntakeConstants.DEPLOY_MAX_ACCELERATION_RPSS;
        mmConfigs.MotionMagicJerk = IntakeConstants.DEPLOY_MAX_JERK_RPSSS;
        
        // Soft limits
        SoftwareLimitSwitchConfigs limitConfigs = config.SoftwareLimitSwitch;
        limitConfigs.ForwardSoftLimitEnable = true;
        limitConfigs.ForwardSoftLimitThreshold = IntakeConstants.DEPLOY_MAX_SOFT_LIMIT;
        limitConfigs.ReverseSoftLimitEnable = true;
        limitConfigs.ReverseSoftLimitThreshold = IntakeConstants.DEPLOY_MIN_SOFT_LIMIT;
        
        // Hard limits
        config.HardwareLimitSwitch.ForwardLimitEnable = IntakeConstants.ENABLE_DEPLOY_FORWARD_HARD_LIMIT;
        config.HardwareLimitSwitch.ReverseLimitEnable = IntakeConstants.ENABLE_DEPLOY_REVERSE_HARD_LIMIT;
        
        // PID configuration (placeholder values - tune these)
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 10.0;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kV = 0.0;
        
        intakeDeployMotor.getConfigurator().apply(config);
        intakeDeployMotor.setPosition(IntakeConstants.STOWED_POSITION_ROTATIONS);
    }
    
    /**
     * Deploy the intake to the extended position
     */
    public void deploy() {
        if (!deployMotorDisabled) {
            intakeDeployMotor.setControl(
                deployPositionRequest.withPosition(IntakeConstants.EXTENDED_POSITION_ROTATIONS)
            );
            isDeployed = true;
        }
    }
    
    /**
     * Retract the intake to the stowed position
     */
    public void retract() {
        if (!deployMotorDisabled) {
            intakeDeployMotor.setControl(
                deployPositionRequest.withPosition(IntakeConstants.STOWED_POSITION_ROTATIONS)
            );
            isDeployed = false;
        }
    }
    
    /**
     * Run the intake roller at the specified velocity
     * @param velocityRPS Roller velocity in rotations per second (positive = intake)
     */
    public void setRollerVelocity(double velocityRPS) {
        intakeRollerMotor.setControl(rollerVelocityRequest.withVelocity(velocityRPS));
    }
    
    /**
     * Stop the intake roller
     */
    public void stopRoller() {
        intakeRollerMotor.stopMotor();
    }
    
    /**
     * Run the intake (deploy and spin roller)
     * @param velocityRPS Roller velocity in rotations per second
     */
    public void intake(double velocityRPS) {
        deploy();
        setRollerVelocity(velocityRPS);
    }
    
    /**
     * Stop the intake (stop roller and retract)
     */
    public void stop() {
        stopRoller();
        retract();
    }
    
    /**
     * Check if the intake is deployed
     */
    public boolean isDeployed() {
        return isDeployed;
    }
    
    /**
     * Get the current deploy position in rotations
     */
    public double getDeployPosition() {
        return intakeDeployMotor.getPosition().getValueAsDouble();
    }
    
    /**
     * Get the current roller velocity in RPS
     */
    public double getRollerVelocity() {
        return intakeRollerMotor.getVelocity().getValueAsDouble();
    }
    
    /**
     * Enable the deploy motor (normal operation)
     */
    public void enableDeployMotor() {
        deployMotorDisabled = false;
    }
    
    /**
     * Disable the deploy motor (for testing - saves wear and tear)
     * When disabled, deploy() and retract() commands are ignored,
     * but the roller can still operate normally.
     */
    public void disableDeployMotor() {
        deployMotorDisabled = true;
        intakeDeployMotor.stopMotor(); // Stop the motor when disabling
    }
    
    /**
     * Toggle the deploy motor enabled state
     */
    public void toggleDeployMotor() {
        if (deployMotorDisabled) {
            enableDeployMotor();
        } else {
            disableDeployMotor();
        }
    }
    
    /**
     * Check if the deploy motor is disabled
     */
    public boolean isDeployMotorDisabled() {
        return deployMotorDisabled;
    }
    
    @Override
    public void periodic() {
        // Display deploy motor override status on dashboard
        SmartDashboard.putBoolean("Intake/Deploy Motor Disabled", deployMotorDisabled);
        SmartDashboard.putBoolean("Intake/Is Deployed", isDeployed);
        SmartDashboard.putNumber("Intake/Deploy Position", getDeployPosition());
        SmartDashboard.putNumber("Intake/Roller Velocity RPS", getRollerVelocity());
    }
}
