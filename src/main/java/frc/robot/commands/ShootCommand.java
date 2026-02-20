package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Choreographed shooting sequence:
 * 1. Spin up flywheel while running floor indexer at low speed
 * 2. Wait for flywheel to reach target velocity
 * 3. Stop floor indexer briefly
 * 4. Run fire indexer at high speed
 * 5. Run floor indexer at stable 60 RPS and intake at 30% velocity
 * 6. On end: Run fire indexer backwards briefly to clear jam
 */
public class ShootCommand {
    
    // Velocity constants
    private static final double FLOOR_LOW_SPEED = 5.0;  // RPS - slow feed during spinup
    private static final double FIRE_HIGH_SPEED = 40.0;  // RPS - reduced feed speed
    private static final double FIRE_REVERSE_SPEED = -20.0; // RPS - reverse to clear
    private static final double REVERSE_DURATION = 0.15; // seconds
    private static final double INTAKE_SHOOT_SPEED = 12.0; // RPS - 30% of 40 RPS max
    
    /**
     * Create a shooting command with the specified flywheel velocity, turret angle, and hood angle
     * 
     * @param flywheelSubsystem The flywheel subsystem
     * @param indexerSubsystem The indexer subsystem
     * @param turretSubsystem The turret subsystem
     * @param hoodSubsystem The hood subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
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
            double targetVelocityRPS,
            Rotation2d turretAngle,
            Rotation2d hoodAngle) {
        
        return Commands.sequence(
            // Phase 0: Set turret and hood angles
            Commands.parallel(
                Commands.runOnce(() -> turretSubsystem.setTargetAngle(turretAngle), turretSubsystem),
                Commands.runOnce(() -> hoodSubsystem.setAngle(hoodAngle), hoodSubsystem)
            ),
            
            // Phase 1: Spin up flywheel with low floor speed
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS)),
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerVelocity(FLOOR_LOW_SPEED))
            ),
            
            // Phase 2: Wait for flywheel to reach target (hold hood and turret position)
            new WaitUntilCommand(flywheelSubsystem::atTargetVelocity)
                .withTimeout(3.0) // Safety timeout
                .alongWith(Commands.run(() -> {}, hoodSubsystem, turretSubsystem)), // Hold position
            
            // Phase 3: Stop floor briefly (momentary pause)
            Commands.runOnce(() -> indexerSubsystem.stopFloorIndexer()),
            Commands.waitSeconds(0.05), // 50ms pause
            
            // Phase 4: Start fire indexer at high speed and intake at 30%
            Commands.parallel(
                Commands.runOnce(() -> indexerSubsystem.setFireIndexerVelocity(FIRE_HIGH_SPEED)),
                Commands.runOnce(() -> intakeRollerSubsystem.setVelocity(INTAKE_SHOOT_SPEED))
            ),
            Commands.waitSeconds(0.1), // Let fire motor get up to speed
            
            // Phase 5: Run floor at 60 RPS to feed continuously (hold hood and turret)
            Commands.parallel(
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerVelocity(FIRE_HIGH_SPEED)),
                Commands.run(() -> {}, hoodSubsystem, turretSubsystem) // Hold position
            )
        ).finallyDo(() -> {
            // On end: Reverse fire motor briefly to clear any jam
            indexerSubsystem.setFireIndexerVelocity(FIRE_REVERSE_SPEED);
            try {
                Thread.sleep((long)(REVERSE_DURATION * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            indexerSubsystem.stopAll();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
        });
    }
    
    /**
     * Create a shooting command with the specified flywheel velocity (no turret/hood control)
     * 
     * @param flywheelSubsystem The flywheel subsystem
     * @param indexerSubsystem The indexer subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param targetVelocityRPS Target flywheel velocity in RPS
     * @return The choreographed shoot command
     */
    public static Command shoot(
            FlywheelSubsystem flywheelSubsystem,
            IndexerSubsystem indexerSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            double targetVelocityRPS) {
        
        return Commands.sequence(
            // Phase 1: Spin up flywheel with low floor speed
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS)),
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerVelocity(FLOOR_LOW_SPEED))
            ),
            
            // Phase 2: Wait for flywheel to reach target
            new WaitUntilCommand(flywheelSubsystem::atTargetVelocity)
                .withTimeout(3.0), // Safety timeout
            
            // Phase 3: Stop floor briefly (momentary pause)
            Commands.runOnce(() -> indexerSubsystem.stopFloorIndexer()),
            Commands.waitSeconds(0.05), // 50ms pause
            
            // Phase 4: Start fire indexer at high speed and intake at 30%
            Commands.parallel(
                Commands.runOnce(() -> indexerSubsystem.setFireIndexerVelocity(FIRE_HIGH_SPEED)),
                Commands.runOnce(() -> intakeRollerSubsystem.setVelocity(INTAKE_SHOOT_SPEED))
            ),
            Commands.waitSeconds(0.1), // Let fire motor get up to speed
            
            // Phase 5: Run floor at 60 RPS to feed continuously
            Commands.runOnce(() -> indexerSubsystem.setFloorIndexerVelocity(FIRE_HIGH_SPEED))
        ).finallyDo(() -> {
            // On end: Reverse fire motor briefly to clear any jam
            indexerSubsystem.setFireIndexerVelocity(FIRE_REVERSE_SPEED);
            try {
                Thread.sleep((long)(REVERSE_DURATION * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            indexerSubsystem.stopAll();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
        });
    }
    
    /**
     * Create a shooting command that runs indefinitely until interrupted
     * (e.g., while holding a button) with turret and hood angle control
     * Floor and fire indexers run at stable 60 RPS, intake at 30%
     * 
     * @param flywheelSubsystem The flywheel subsystem
     * @param indexerSubsystem The indexer subsystem
     * @param turretSubsystem The turret subsystem
     * @param hoodSubsystem The hood subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
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
            double targetVelocityRPS,
            Rotation2d turretAngle,
            Rotation2d hoodAngle) {
        
        return Commands.sequence(
            // Phase 0: Set turret and hood angles
            Commands.parallel(
                Commands.runOnce(() -> turretSubsystem.setTargetAngle(turretAngle), turretSubsystem),
                Commands.runOnce(() -> hoodSubsystem.setAngle(hoodAngle), hoodSubsystem)
            ),
            
            // Phase 1: Spin up flywheel with low floor speed
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS)),
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerVelocity(FLOOR_LOW_SPEED))
            ),
            
            // Phase 2: Wait for flywheel to reach target
            new WaitUntilCommand(flywheelSubsystem::atTargetVelocity)
                .withTimeout(3.0),
            
            // Phase 3: Stop floor briefly
            Commands.runOnce(() -> indexerSubsystem.stopFloorIndexer()),
            Commands.waitSeconds(0.05),
            
            // Phase 4: Start fire indexer and intake
            Commands.parallel(
                Commands.runOnce(() -> indexerSubsystem.setFireIndexerVelocity(FIRE_HIGH_SPEED)),
                Commands.runOnce(() -> intakeRollerSubsystem.setVelocity(INTAKE_SHOOT_SPEED))
            ),
            Commands.waitSeconds(0.1),
            
            // Phase 5: Run both indexers continuously - holds hood and turret in position
            Commands.parallel(
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerVelocity(FIRE_HIGH_SPEED)),
                Commands.run(() -> {
                    indexerSubsystem.setFireIndexerVelocity(FIRE_HIGH_SPEED);
                    indexerSubsystem.setFloorIndexerVelocity(FIRE_HIGH_SPEED);
                    intakeRollerSubsystem.setVelocity(INTAKE_SHOOT_SPEED);
                }, indexerSubsystem, hoodSubsystem, turretSubsystem) // Require hood and turret to hold position
            )
        ).finallyDo(() -> {
            // On interrupt: Reverse fire motor briefly
            indexerSubsystem.setFireIndexerVelocity(FIRE_REVERSE_SPEED);
            try {
                Thread.sleep((long)(REVERSE_DURATION * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            indexerSubsystem.stopAll();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
        });
    }
    
    /**
     * Create a shooting command that runs indefinitely until interrupted
     * (e.g., while holding a button) - no turret/hood control
     * Floor and fire indexers run at stable 60 RPS, intake at 30%
     * 
     * @param flywheelSubsystem The flywheel subsystem
     * @param indexerSubsystem The indexer subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param targetVelocityRPS Target flywheel velocity in RPS
     * @return The choreographed shoot command
     */
    public static Command shootContinuous(
            FlywheelSubsystem flywheelSubsystem,
            IndexerSubsystem indexerSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            double targetVelocityRPS) {
        
        return Commands.sequence(
            // Phase 1: Spin up flywheel with low floor speed
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS)),
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerVelocity(FLOOR_LOW_SPEED))
            ),
            
            // Phase 2: Wait for flywheel to reach target
            new WaitUntilCommand(flywheelSubsystem::atTargetVelocity)
                .withTimeout(3.0),
            
            // Phase 3: Stop floor briefly
            Commands.runOnce(() -> indexerSubsystem.stopFloorIndexer()),
            Commands.waitSeconds(0.05),
            
            // Phase 4: Start fire indexer and intake
            Commands.parallel(
                Commands.runOnce(() -> indexerSubsystem.setFireIndexerVelocity(FIRE_HIGH_SPEED)),
                Commands.runOnce(() -> intakeRollerSubsystem.setVelocity(INTAKE_SHOOT_SPEED))
            ),
            Commands.waitSeconds(0.1),
            
            // Phase 5: Run both indexers at stable 60 RPS
            Commands.parallel(
                Commands.runOnce(() -> indexerSubsystem.setFloorIndexerVelocity(FIRE_HIGH_SPEED)),
                Commands.run(() -> {
                    indexerSubsystem.setFireIndexerVelocity(FIRE_HIGH_SPEED);
                    indexerSubsystem.setFloorIndexerVelocity(FIRE_HIGH_SPEED);
                    intakeRollerSubsystem.setVelocity(INTAKE_SHOOT_SPEED);
                }, indexerSubsystem)
            )
        ).finallyDo(() -> {
            // On interrupt: Reverse fire motor briefly
            indexerSubsystem.setFireIndexerVelocity(FIRE_REVERSE_SPEED);
            try {
                Thread.sleep((long)(REVERSE_DURATION * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            indexerSubsystem.stopAll();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
        });
    }
}
