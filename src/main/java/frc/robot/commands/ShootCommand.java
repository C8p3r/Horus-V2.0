package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Choreographed shooting sequence:
 * 1. Spin up flywheel while running floor indexer at low speed and fire indexer in reverse
 * 2. Wait for flywheel to reach target velocity, then retract intake
 * 3. Stop floor indexer briefly
 * 4. Run fire indexer at high speed (60 RPS)
 * 5. Run floor indexer at stable speed to feed continuously
 * 6. On end: Run fire indexer backwards briefly to clear jam
 */
public class ShootCommand {
    
    // Duty cycle constants (-1.0 to 1.0, positive = feed to shooter)
    private static final double FLOOR_LOW_DUTY_CYCLE = 0.083;  // ~8% - slow feed during spinup (was 5 RPS / 60 = 0.083)
    private static final double FIRE_HIGH_DUTY_CYCLE = 1.0;  // 100% - full speed shooting (was 60 RPS / 60 = 1.0)
    private static final double FIRE_REVERSE_DUTY_CYCLE = -0.33; // 33% reverse to clear (was -20 RPS / 60 = -0.33)
    private static final double FIRE_SPINUP_REVERSE_DUTY_CYCLE = -0.25; // 25% reverse during spinup (was -15 RPS / 60 = -0.25)
    private static final double REVERSE_DURATION = 0.15; // seconds
    
    /**
     * Create a shooting command with the specified flywheel velocity, turret angle, and hood angle
     * 
     * @param flywheelSubsystem The flywheel subsystem
     * @param indexerSubsystem The indexer subsystem
     * @param turretSubsystem The turret subsystem
     * @param hoodSubsystem The hood subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param intakePositionSubsystem The intake position subsystem
     * @param targetVelocityRPS Target flywheel velocity in RPS
     * @param turretAngle Target turret angle
     * @param hoodAngle Target hood angle
     * @return The choreographed shoot command
     */
    public static Command shoot(
            FlywheelSubsystem flywheelSubsystem,
            IndexerSubsystem indexerSubsystem,
            TurretSubsystem turretSubsystem,
            HoodSubsystem hoodSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            IntakePositionSubsystem intakePositionSubsystem,
            double targetVelocityRPS,
            Rotation2d turretAngle,
            Rotation2d hoodAngle) {
        
        return Commands.sequence(
            // Phase 0: Set turret and hood angles
            Commands.parallel(
                Commands.runOnce(() -> turretSubsystem.setTargetAngle(turretAngle), turretSubsystem),
                Commands.runOnce(() -> hoodSubsystem.setAngle(hoodAngle), hoodSubsystem)
            ),
            
            // Phase 1: Spin up flywheel with low floor speed and fire indexer in REVERSE
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS)),
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerDutyCycle(FLOOR_LOW_DUTY_CYCLE)),
                Commands.runOnce(() -> indexerSubsystem.setFireIndexerDutyCycle(FIRE_SPINUP_REVERSE_DUTY_CYCLE)) // Reverse during spinup
            ),
            
            // Phase 2: Wait for flywheel to reach target (hold hood and turret position), then retract intake if enabled
            new WaitUntilCommand(flywheelSubsystem::atTargetVelocity)
                .withTimeout(3.0) // Safety timeout
                .alongWith(Commands.run(() -> {}, hoodSubsystem, turretSubsystem)), // Hold position
            Commands.runOnce(() -> {
                if (RobotContainer.RETRACT_INTAKE_WHILE_SHOOTING) {
                    intakePositionSubsystem.retract();
                }
            }), // Conditionally retract once flywheel is up to speed
            
            // Phase 3: Stop floor briefly (momentary pause)
            Commands.runOnce(() -> indexerSubsystem.stopFloorIndexer()),
            Commands.waitSeconds(0.05), // 50ms pause
            
            // Phase 4: Start fire indexer at high speed
            Commands.runOnce(() -> indexerSubsystem.setFireIndexerDutyCycle(FIRE_HIGH_DUTY_CYCLE)),
            Commands.waitSeconds(0.1), // Let fire motor get up to speed
            
            // Phase 5: Run floor at full power to feed continuously (hold hood and turret)
            Commands.parallel(
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerDutyCycle(FIRE_HIGH_DUTY_CYCLE)),
                Commands.run(() -> {}, hoodSubsystem, turretSubsystem) // Hold position
            )
        ).finallyDo(() -> {
            // On end: Reverse fire motor briefly to clear any jam
            indexerSubsystem.setFireIndexerDutyCycle(FIRE_REVERSE_DUTY_CYCLE);
            try {
                Thread.sleep((long)(REVERSE_DURATION * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            indexerSubsystem.stopAll();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
        }).andThen(
            // After cleanup: Redeploy intake with wait time if retraction is enabled
            Commands.runOnce(() -> {
                if (RobotContainer.RETRACT_INTAKE_WHILE_SHOOTING) {
                    intakePositionSubsystem.deploy();
                }
            }),
            Commands.waitSeconds(0.3) // Wait for deployment to complete
        );
    }
    
    /**
     * Create a shooting command with the specified flywheel velocity (no turret/hood control)
     * 
     * @param flywheelSubsystem The flywheel subsystem
     * @param indexerSubsystem The indexer subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param intakePositionSubsystem The intake position subsystem
     * @param targetVelocityRPS Target flywheel velocity in RPS
     * @return The choreographed shoot command
     */
    public static Command shoot(
            FlywheelSubsystem flywheelSubsystem,
            IndexerSubsystem indexerSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            IntakePositionSubsystem intakePositionSubsystem,
            double targetVelocityRPS) {
        
        return Commands.sequence(
            // Phase 1: Spin up flywheel with low floor speed and fire indexer in REVERSE
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS)),
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerDutyCycle(FLOOR_LOW_DUTY_CYCLE)),
                Commands.runOnce(() -> indexerSubsystem.setFireIndexerDutyCycle(FIRE_SPINUP_REVERSE_DUTY_CYCLE)) // Reverse during spinup
            ),
            
            // Phase 2: Wait for flywheel to reach target, then retract intake if enabled
            new WaitUntilCommand(flywheelSubsystem::atTargetVelocity)
                .withTimeout(3.0), // Safety timeout
            Commands.runOnce(() -> {
                if (RobotContainer.RETRACT_INTAKE_WHILE_SHOOTING) {
                    intakePositionSubsystem.retract();
                }
            }), // Conditionally retract once flywheel is up to speed
            
            // Phase 3: Stop floor briefly (momentary pause)
            Commands.runOnce(() -> indexerSubsystem.stopFloorIndexer()),
            Commands.waitSeconds(0.05), // 50ms pause
            
            // Phase 4: Start fire indexer at high speed
            Commands.runOnce(() -> indexerSubsystem.setFireIndexerDutyCycle(FIRE_HIGH_DUTY_CYCLE)),
            Commands.waitSeconds(0.1), // Let fire motor get up to speed
            
            // Phase 5: Run floor at full power to feed continuously
            Commands.runOnce(() -> indexerSubsystem.setFloorIndexerDutyCycle(FIRE_HIGH_DUTY_CYCLE))
        ).finallyDo(() -> {
            // On end: Reverse fire motor briefly to clear any jam
            indexerSubsystem.setFireIndexerDutyCycle(FIRE_REVERSE_DUTY_CYCLE);
            try {
                Thread.sleep((long)(REVERSE_DURATION * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            indexerSubsystem.stopAll();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
        }).andThen(
            // After cleanup: Redeploy intake with wait time if retraction is enabled
            Commands.runOnce(() -> {
                if (RobotContainer.RETRACT_INTAKE_WHILE_SHOOTING) {
                    intakePositionSubsystem.deploy();
                }
            }),
            Commands.waitSeconds(0.3) // Wait for deployment to complete
        );
    }
    
    /**
     * Create a shooting command that runs indefinitely until interrupted
     * (e.g., while holding a button) with turret and hood angle control
     * Floor and fire indexers run at full duty cycle
     * 
     * @param flywheelSubsystem The flywheel subsystem
     * @param indexerSubsystem The indexer subsystem
     * @param turretSubsystem The turret subsystem
     * @param hoodSubsystem The hood subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param intakePositionSubsystem The intake position subsystem
     * @param targetVelocityRPS Target flywheel velocity in RPS
     * @param turretAngle Target turret angle
     * @param hoodAngle Target hood angle
     * @return The choreographed shoot command
     */
    public static Command shootContinuous(
            FlywheelSubsystem flywheelSubsystem,
            IndexerSubsystem indexerSubsystem,
            TurretSubsystem turretSubsystem,
            HoodSubsystem hoodSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            IntakePositionSubsystem intakePositionSubsystem,
            double targetVelocityRPS,
            Rotation2d turretAngle,
            Rotation2d hoodAngle) {
        
        return Commands.sequence(
            // Phase 0: Set turret and hood angles
            Commands.parallel(
                Commands.runOnce(() -> turretSubsystem.setTargetAngle(turretAngle), turretSubsystem),
                Commands.runOnce(() -> hoodSubsystem.setAngle(hoodAngle), hoodSubsystem)
            ),
            
            // Phase 1: Spin up flywheel with low floor speed and fire indexer in REVERSE
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS)),
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerDutyCycle(FLOOR_LOW_DUTY_CYCLE)),
                Commands.runOnce(() -> indexerSubsystem.setFireIndexerDutyCycle(FIRE_SPINUP_REVERSE_DUTY_CYCLE)) // Reverse during spinup
            ),
            
            // Phase 2: Wait for flywheel to reach target, then retract intake if enabled
            new WaitUntilCommand(flywheelSubsystem::atTargetVelocity)
                .withTimeout(3.0),
            Commands.runOnce(() -> {
                if (RobotContainer.RETRACT_INTAKE_WHILE_SHOOTING) {
                    intakePositionSubsystem.retract();
                }
            }), // Conditionally retract once flywheel is up to speed
            
            // Phase 3: Stop floor briefly
            Commands.runOnce(() -> indexerSubsystem.stopFloorIndexer()),
            Commands.waitSeconds(0.05),
            
            // Phase 4: Start fire indexer
            Commands.runOnce(() -> indexerSubsystem.setFireIndexerDutyCycle(FIRE_HIGH_DUTY_CYCLE)),
            Commands.waitSeconds(0.1),
            
            // Phase 5: Run both indexers continuously - holds hood and turret in position
            Commands.parallel(
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerDutyCycle(FIRE_HIGH_DUTY_CYCLE)),
                Commands.run(() -> {
                    indexerSubsystem.setFireIndexerDutyCycle(FIRE_HIGH_DUTY_CYCLE);
                    indexerSubsystem.setFloorIndexerDutyCycle(FIRE_HIGH_DUTY_CYCLE);
                }, indexerSubsystem, hoodSubsystem, turretSubsystem) // Require hood and turret to hold position
            )
        ).finallyDo(() -> {
            // On interrupt: Reverse fire motor briefly
            indexerSubsystem.setFireIndexerDutyCycle(FIRE_REVERSE_DUTY_CYCLE);
            try {
                Thread.sleep((long)(REVERSE_DURATION * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            indexerSubsystem.stopAll();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
        }).andThen(
            // After cleanup: Redeploy intake with wait time if retraction is enabled
            Commands.runOnce(() -> {
                if (RobotContainer.RETRACT_INTAKE_WHILE_SHOOTING) {
                    intakePositionSubsystem.deploy();
                }
            }),
            Commands.waitSeconds(0.3) // Wait for deployment to complete
        );
    }
    
    /**
     * Create a shooting command that runs indefinitely until interrupted
     * (e.g., while holding a button) - no turret/hood control
     * Floor and fire indexers run at full duty cycle
     * 
     * @param flywheelSubsystem The flywheel subsystem
     * @param indexerSubsystem The indexer subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param intakePositionSubsystem The intake position subsystem
     * @param targetVelocityRPS Target flywheel velocity in RPS
     * @return The choreographed shoot command
     */
    public static Command shootContinuous(
            FlywheelSubsystem flywheelSubsystem,
            IndexerSubsystem indexerSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            IntakePositionSubsystem intakePositionSubsystem,
            double targetVelocityRPS) {
        
        return Commands.sequence(
            // Phase 1: Spin up flywheel with low floor speed and fire indexer in REVERSE
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS)),
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerDutyCycle(FLOOR_LOW_DUTY_CYCLE)),
                Commands.runOnce(() -> indexerSubsystem.setFireIndexerDutyCycle(FIRE_SPINUP_REVERSE_DUTY_CYCLE)) // Reverse during spinup
            ),
            
            // Phase 2: Wait for flywheel to reach target, then retract intake if enabled
            new WaitUntilCommand(flywheelSubsystem::atTargetVelocity)
                .withTimeout(3.0),
            Commands.runOnce(() -> {
                if (RobotContainer.RETRACT_INTAKE_WHILE_SHOOTING) {
                    intakePositionSubsystem.retract();
                }
            }), // Conditionally retract once flywheel is up to speed
            
            // Phase 3: Stop floor briefly
            Commands.runOnce(() -> indexerSubsystem.stopFloorIndexer()),
            Commands.waitSeconds(0.05),
            
            // Phase 4: Start fire indexer
            Commands.runOnce(() -> indexerSubsystem.setFireIndexerDutyCycle(FIRE_HIGH_DUTY_CYCLE)),
            Commands.waitSeconds(0.1),
            
            // Phase 5: Run both indexers at full duty cycle
            Commands.parallel(
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerDutyCycle(FIRE_HIGH_DUTY_CYCLE)),
                Commands.run(() -> {
                    indexerSubsystem.setFireIndexerDutyCycle(FIRE_HIGH_DUTY_CYCLE);
                    indexerSubsystem.setFloorIndexerDutyCycle(FIRE_HIGH_DUTY_CYCLE);
                }, indexerSubsystem)
            )
        ).finallyDo(() -> {
            // On interrupt: Reverse fire motor briefly
            indexerSubsystem.setFireIndexerDutyCycle(FIRE_REVERSE_DUTY_CYCLE);
            try {
                Thread.sleep((long)(REVERSE_DURATION * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            indexerSubsystem.stopAll();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
        }).andThen(
            // After cleanup: Redeploy intake with wait time if retraction is enabled
            Commands.runOnce(() -> {
                if (RobotContainer.RETRACT_INTAKE_WHILE_SHOOTING) {
                    intakePositionSubsystem.deploy();
                }
            }),
            Commands.waitSeconds(0.3) // Wait for deployment to complete
        );
    }
}
