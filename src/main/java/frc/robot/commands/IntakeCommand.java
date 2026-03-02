package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.IntakeRollerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;

/**
 * Choreographed intake sequence:
 * 1. Deploy intake
 * 2. Run intake roller and floor indexer
 * 3. On end: Stop motors and retract intake
 */
public class IntakeCommand {
    
    // Duty cycle constants (0.0 to 1.0, positive = forward)
    private static final double FLOOR_INDEXER_DUTY_CYCLE = 0.5;   // 50% power (was 30 RPS / 60 = 0.5)
    private static final double FIRE_INDEXER_REVERSE_DUTY_CYCLE = -0.5;  // 50% reverse (was -30 RPS / 60 = -0.5)
    private static final double EJECT_DUTY_CYCLE = -0.5;  // 50% reverse for eject (was -30 RPS / 60 = -0.5)
    
    /**
     * Create an intake command that runs while held
     * 
     * Deploys intake and runs motors while held. Does NOT retract when released.
     * Fire indexer runs in reverse to prevent jamming.
     * Intake stays deployed until shooting command retracts it.
     * 
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param intakePositionSubsystem The intake position subsystem
     * @param indexerSubsystem The indexer subsystem
     * @return The intake command
     */
    public static Command intake(
            IntakeRollerSubsystem intakeRollerSubsystem,
            IntakePositionSubsystem intakePositionSubsystem,
            IndexerSubsystem indexerSubsystem) {
        
        return Commands.sequence(
            // Phase 1: Deploy intake
            Commands.runOnce(() -> intakePositionSubsystem.deploy()),
            Commands.waitSeconds(0.3), // Wait for deployment
            
            // Phase 2: Run motors (fire indexer in reverse)
            Commands.run(() -> {
                intakeRollerSubsystem.setDutyCycle(IntakeRollerConstants.INTAKE_DUTY_CYCLE);
                indexerSubsystem.setFloorIndexerDutyCycle(FLOOR_INDEXER_DUTY_CYCLE);
                indexerSubsystem.setFireIndexerDutyCycle(FIRE_INDEXER_REVERSE_DUTY_CYCLE); // Reverse to prevent jamming
            }, intakeRollerSubsystem, indexerSubsystem)
        ).finallyDo(() -> {
            // On end: Stop motors only (do NOT retract - stays deployed)
            intakeRollerSubsystem.stop();
            indexerSubsystem.stopAll();
        });
    }
    
    /**
     * Create a simple intake command without deployment control
     * (useful if intake position is controlled separately)
     * 
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param indexerSubsystem The indexer subsystem
     * @return The simple intake command
     */
    public static Command intakeSimple(
            IntakeRollerSubsystem intakeRollerSubsystem,
            IndexerSubsystem indexerSubsystem) {
        
        return Commands.run(() -> {
            intakeRollerSubsystem.setDutyCycle(IntakeRollerConstants.INTAKE_DUTY_CYCLE);
            indexerSubsystem.setFloorIndexerDutyCycle(FLOOR_INDEXER_DUTY_CYCLE);
            indexerSubsystem.stopFireIndexer();
        }, intakeRollerSubsystem, indexerSubsystem)
        .finallyDo(() -> {
            intakeRollerSubsystem.stop();
            indexerSubsystem.stopAll();
        });
    }
    
    /**
     * Eject game piece (reverse intake)
     * 
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param indexerSubsystem The indexer subsystem
     * @return The eject command
     */
    public static Command eject(
            IntakeRollerSubsystem intakeRollerSubsystem,
            IndexerSubsystem indexerSubsystem) {
        
        return Commands.run(() -> {
            intakeRollerSubsystem.setDutyCycle(IntakeRollerConstants.EJECT_DUTY_CYCLE);
            indexerSubsystem.setFloorIndexerDutyCycle(-FLOOR_INDEXER_DUTY_CYCLE);
            indexerSubsystem.setFireIndexerDutyCycle(EJECT_DUTY_CYCLE);
        }, intakeRollerSubsystem, indexerSubsystem)
        .finallyDo(() -> {
            intakeRollerSubsystem.stop();
            indexerSubsystem.stopAll();
        });
    }
    
    /**
     * Three-state intake toggle:
     * State 1 (retracted/off): Deploy and turn on at full speed
     * State 2 (deployed/on): Turn off but stay deployed
     * State 3 (deployed/off): Retract and turn off
     * 
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param intakePositionSubsystem The intake position subsystem
     * @param indexerSubsystem The indexer subsystem
     * @return The toggle command
     */
    public static Command toggleIntake(
            IntakeRollerSubsystem intakeRollerSubsystem,
            IntakePositionSubsystem intakePositionSubsystem,
            IndexerSubsystem indexerSubsystem) {
        
        return Commands.either(
            // If deployed, check if motors are running
            Commands.either(
                // Motors running: State 2 (deployed/on) -> State 3 (deployed/off) - turn off motors
                Commands.runOnce(() -> {
                    intakeRollerSubsystem.stop();
                    indexerSubsystem.stopAll();
                }),
                // Motors stopped: State 3 (deployed/off) -> State 1 (retracted/off) - retract
                Commands.runOnce(() -> intakePositionSubsystem.retract()),
                // Check if roller motor is running
                () -> Math.abs(intakeRollerSubsystem.getVelocity()) > 1.0 // > 1 RPS means running
            ),
            // If retracted: State 1 -> State 2 - deploy and turn on
            Commands.sequence(
                Commands.runOnce(() -> intakePositionSubsystem.deploy()),
                Commands.waitSeconds(0.3), // Wait for deployment
                Commands.runOnce(() -> {
                    intakeRollerSubsystem.setDutyCycle(IntakeRollerConstants.INTAKE_DUTY_CYCLE);
                    indexerSubsystem.setFloorIndexerDutyCycle(FLOOR_INDEXER_DUTY_CYCLE);
                    indexerSubsystem.setFireIndexerDutyCycle(FIRE_INDEXER_REVERSE_DUTY_CYCLE);
                })
            ),
            // Check if deployed
            intakePositionSubsystem::isDeployed
        );
    }
}
