package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Commands for climber deployment and storage
 */
public class ClimberCommand {
    
    /**
     * Deploy the climber (unspool winch)
     * 
     * Sequence:
     * 1. Disable all non-drivetrain, non-LED subsystems
     * 2. Retract intake
     * 3. Disable ratchet to allow free extension
     * 4. Deploy climber to full extension (300 rotations)
     * 5. Wait for full deployment
     * 6. Enable ratchet to allow ratcheting on the way down
     * 
     * @param climberSubsystem The climber subsystem
     * @param intakePositionSubsystem The intake position subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param indexerSubsystem The indexer subsystem
     * @param flywheelSubsystem The flywheel subsystem
     * @param turretSubsystem The turret subsystem
     * @param hoodSubsystem The hood subsystem
     * @return The deploy command
     */
    public static Command deployClimber(
            ClimberSubsystem climberSubsystem,
            IntakePositionSubsystem intakePositionSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            IndexerSubsystem indexerSubsystem,
            FlywheelSubsystem flywheelSubsystem,
            TurretSubsystem turretSubsystem,
            HoodSubsystem hoodSubsystem) {
        
        return Commands.sequence(
            // Phase 1: Stop all non-drivetrain subsystems
            Commands.runOnce(() -> {
                intakeRollerSubsystem.stop();
                indexerSubsystem.stopAll();
                flywheelSubsystem.stop();
            }),
            
            // Phase 2: Retract intake for safety
            Commands.runOnce(() -> intakePositionSubsystem.retract()),
            Commands.waitSeconds(0.5), // Wait for retraction
            
            // Phase 3: Disable ratchet and deploy
            Commands.runOnce(() -> {
                climberSubsystem.disableRatchet();
                climberSubsystem.deploy();
            }),
            
            // Phase 4: Wait for full deployment
            new WaitUntilCommand(climberSubsystem::isFullyDeployed)
                .withTimeout(10.0), // Safety timeout
            
            // Phase 5: Enable ratchet once fully deployed
            Commands.runOnce(() -> climberSubsystem.enableRatchet())
        ).finallyDo(() -> {
            // Ensure ratchet is enabled when done (allows ratcheting down)
            climberSubsystem.enableRatchet();
        });
    }
    
    /**
     * Store the climber - Engages ratchet to allow manual climbing down
     * 
     * Sequence:
     * 1. Enable ratchet immediately (allows robot to climb down manually)
     * 2. Disable all non-drivetrain, non-LED subsystems
     * 3. Retract intake for safety
     * 
     * The ratchet remains ENABLED after this command, allowing the robot
     * to manually climb down the bar with the ratchet preventing backsliding.
     * The ratchet will only be disabled when deployClimber is called again.
     * 
     * @param climberSubsystem The climber subsystem
     * @param intakePositionSubsystem The intake position subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param indexerSubsystem The indexer subsystem
     * @param flywheelSubsystem The flywheel subsystem
     * @param turretSubsystem The turret subsystem
     * @param hoodSubsystem The hood subsystem
     * @return The store/climb command
     */
    public static Command storeClimber(
            ClimberSubsystem climberSubsystem,
            IntakePositionSubsystem intakePositionSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            IndexerSubsystem indexerSubsystem,
            FlywheelSubsystem flywheelSubsystem,
            TurretSubsystem turretSubsystem,
            HoodSubsystem hoodSubsystem) {
        
        return Commands.sequence(
            // Phase 1: Enable ratchet immediately (allows manual climbing down)
            Commands.runOnce(() -> climberSubsystem.enableRatchet()),
            
            // Phase 2: Stop all non-drivetrain subsystems
            Commands.runOnce(() -> {
                intakeRollerSubsystem.stop();
                indexerSubsystem.stopAll();
                flywheelSubsystem.stop();
            }),
            
            // Phase 3: Retract intake for safety
            Commands.runOnce(() -> intakePositionSubsystem.retract()),
            Commands.waitSeconds(0.5) // Wait for retraction
            
            // Ratchet remains enabled - robot can now manually climb down using ratchet
        ).finallyDo(() -> {
            // Ensure ratchet stays enabled even if interrupted
            climberSubsystem.enableRatchet();
        });
    }
}
