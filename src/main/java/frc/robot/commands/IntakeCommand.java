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
     * Runs motors at full speed. TurboKicker runs in hold mode to prevent jamming.
     * 
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param turboKickerSubsystem The TurboKicker subsystem
     * @return The intake command
     */
    public static Command intake(
            IntakeRollerSubsystem intakeRollerSubsystem,
            TurboKickerSubsystem turboKickerSubsystem) {
        
        return Commands.run(() -> {
            intakeRollerSubsystem.setDutyCycle(IntakeRollerConstants.INTAKE_DUTY_CYCLE);
            turboKickerSubsystem.hold(); // Hold mode prevents jamming
        }, intakeRollerSubsystem, turboKickerSubsystem)
        .withName("Intake Full Speed");
    }
    
    /**
     * Eject game piece (reverse intake and TurboKicker)
     * 
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param turboKickerSubsystem The TurboKicker subsystem
     * @return The eject command
     */
    public static Command eject(
            IntakeRollerSubsystem intakeRollerSubsystem,
            TurboKickerSubsystem turboKickerSubsystem) {
        
        return Commands.run(() -> {
            intakeRollerSubsystem.setDutyCycle(IntakeRollerConstants.EJECT_DUTY_CYCLE);
            turboKickerSubsystem.reverse(); // Reverse TurboKicker for ejection
        }, intakeRollerSubsystem, turboKickerSubsystem)
        .withName("Intake Eject");
    }
    
    /**
     * Idle intake at low speed (continuous background operation)
     * 
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param turboKickerSubsystem The TurboKicker subsystem
     * @return The idle command
     */
    public static Command idle(
            IntakeRollerSubsystem intakeRollerSubsystem,
            TurboKickerSubsystem turboKickerSubsystem) {
        
        return Commands.run(() -> {
            // Very low speed idle - 5% of intake speed (reduced from 20%)
            intakeRollerSubsystem.setDutyCycle(IntakeRollerConstants.INTAKE_DUTY_CYCLE * 0.05);
            turboKickerSubsystem.hold(); // Light hold to keep note in place
        }, intakeRollerSubsystem, turboKickerSubsystem)
        .withName("Intake Idle");
    }
}
