package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.IntakeRollerConstants;
import frc.robot.constants.TurboKickerConstants;
import frc.robot.subsystems.TurboKickerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;

/**
 * Intake control commands:
 * - Intake: Run at full speed when button held
 * - Eject: Reverse when button held
 * - Idle: Low speed continuous run when no buttons pressed
 */
public class IntakeCommand {
    
    /**
     * Create an intake command that runs while held (full speed forward)
     * 
     * Runs ONLY intake rollers at full speed. Belt floor, feed motors, and kicker motor are off.
     * All turbokicker components remain off unless actively shooting.
     * 
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param turboKickerSubsystem The TurboKicker subsystem
     * @return The intake command
     */
    public static Command intake(
            IntakeRollerSubsystem intakeRollerSubsystem,
            TurboKickerSubsystem turboKickerSubsystem) {
        
        return Commands.run(() -> {
            // Run ONLY intake rollers at full speed
            intakeRollerSubsystem.setDutyCycle(IntakeRollerConstants.INTAKE_DUTY_CYCLE);
            // Belt floor OFF during intake
            turboKickerSubsystem.setBeltFloorDutyCycle(0.0);
            // Kicker motors OFF during intake
            turboKickerSubsystem.setFeedMotorsDutyCycle(0.0);  // Feed motors off
            turboKickerSubsystem.setKickerMotorDutyCycle(0.0); // Kicker off
        }, intakeRollerSubsystem, turboKickerSubsystem)
        .withName("Intake Full Speed");
    }
    
    /**
     * Eject game piece (reverse intake rollers only)
     * 
     * Belt floor, feed motors, and kicker motor remain off.
     * 
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param turboKickerSubsystem The TurboKicker subsystem
     * @return The eject command
     */
    public static Command eject(
            IntakeRollerSubsystem intakeRollerSubsystem,
            TurboKickerSubsystem turboKickerSubsystem) {
        
        return Commands.run(() -> {
            // Reverse ONLY intake rollers
            intakeRollerSubsystem.setDutyCycle(IntakeRollerConstants.EJECT_DUTY_CYCLE);
            // Belt floor BACKWARD to help unjam
            turboKickerSubsystem.setBeltFloorDutyCycle(-0.5);
            // Kicker motors OFF during eject
            turboKickerSubsystem.setFeedMotorsDutyCycle(0.0);  // Feed motors off
            turboKickerSubsystem.setKickerMotorDutyCycle(0.0); // Kicker off
        }, intakeRollerSubsystem, turboKickerSubsystem)
        .withName("Intake Eject");
    }
    
    /**
     * Idle intake at low speed (continuous background operation)
     * Only runs belt floor at 25%, kicker motors are stopped
     * 
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param turboKickerSubsystem The TurboKicker subsystem
     * @return The idle command
     */
    public static Command idle(
            IntakeRollerSubsystem intakeRollerSubsystem,
            TurboKickerSubsystem turboKickerSubsystem) {
        
        return Commands.run(() -> {
            // All TurboKicker motors off in idle - only run when shoot trigger pressed
            turboKickerSubsystem.setBeltFloorDutyCycle(0.0);
            turboKickerSubsystem.setFeedMotorsDutyCycle(0.0);
            turboKickerSubsystem.setKickerMotorDutyCycle(0.0);
            // Intake rollers at 25%
            intakeRollerSubsystem.setDutyCycle(0.25);
        }, intakeRollerSubsystem, turboKickerSubsystem)
        .withName("Intake Idle");
    }
}
