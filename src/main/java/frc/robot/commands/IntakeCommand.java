package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    
    // Velocity constants
    private static final double INTAKE_ROLLER_SPEED = 40.0; // RPS
    private static final double FLOOR_INDEXER_SPEED = 30.0; // RPS
    
    /**
     * Create an intake command that runs while held
     * 
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param intakePositionSubsystem The intake position subsystem
     * @param indexerSubsystem The indexer subsystem
     * @return The choreographed intake command
     */
    public static Command intake(
            IntakeRollerSubsystem intakeRollerSubsystem,
            IntakePositionSubsystem intakePositionSubsystem,
            IndexerSubsystem indexerSubsystem) {
        
        return Commands.sequence(
            // Phase 1: Deploy intake
            Commands.runOnce(() -> intakePositionSubsystem.deploy()),
            Commands.waitSeconds(0.3), // Wait for deployment
            
            // Phase 2: Run intake and indexer continuously
            Commands.run(() -> {
                intakeRollerSubsystem.setVelocity(INTAKE_ROLLER_SPEED);
                indexerSubsystem.setFloorIndexerVelocity(FLOOR_INDEXER_SPEED);
                indexerSubsystem.stopFireIndexer(); // Don't feed to shooter
            }, intakeRollerSubsystem, indexerSubsystem)
        ).finallyDo(() -> {
            // On end: Stop motors and retract
            intakeRollerSubsystem.stop();
            indexerSubsystem.stopAll();
            intakePositionSubsystem.retract();
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
            intakeRollerSubsystem.setVelocity(INTAKE_ROLLER_SPEED);
            indexerSubsystem.setFloorIndexerVelocity(FLOOR_INDEXER_SPEED);
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
            intakeRollerSubsystem.setVelocity(-INTAKE_ROLLER_SPEED);
            indexerSubsystem.setFloorIndexerVelocity(-FLOOR_INDEXER_SPEED);
            indexerSubsystem.setFireIndexerVelocity(-30.0);
        }, intakeRollerSubsystem, indexerSubsystem)
        .finallyDo(() -> {
            intakeRollerSubsystem.stop();
            indexerSubsystem.stopAll();
        });
    }
}
