package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TurboKickerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Choreographed shooting sequence:
 * 1. Spin up flywheel while holding turbokicker in reverse
 * 2. Wait for flywheel to reach target velocity
 * 3. Feed note through turbokicker at full speed
 * 4. On end: Stop turbokicker
 */
public class ShootCommand {
    
    /**
     * Create a shooting command with the specified flywheel velocity and turret angle
     * NOTE: Hood is fixed at 14° mechanically
     * 
     * @param flywheelSubsystem The flywheel subsystem
     * @param turboKickerSubsystem The turbokicker subsystem
     * @param turretSubsystem The turret subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param targetVelocityRPS Target flywheel velocity in RPS
     * @param turretAngle Target turret angle
     * @return The choreographed shoot command
     */
    public static Command shoot(
            FlywheelSubsystem flywheelSubsystem,
            TurboKickerSubsystem turboKickerSubsystem,
            TurretSubsystem turretSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            double targetVelocityRPS,
            Rotation2d turretAngle) {
        
        return Commands.sequence(
            // Phase 0: Set turret angle
            Commands.runOnce(() -> turretSubsystem.setAngle(turretAngle.getDegrees()), turretSubsystem),
            
            // Phase 1: Spin up flywheel while holding turbokicker in reverse and maintaining turret position
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS), flywheelSubsystem),
                Commands.run(() -> turboKickerSubsystem.hold(), turboKickerSubsystem),
                Commands.run(() -> turretSubsystem.setAngle(turretAngle.getDegrees()), turretSubsystem)
            ).withTimeout(0.1), // Brief moment to start motors
            
            // Phase 2: Wait for flywheel to reach target (hold turret position and turbokicker)
            Commands.parallel(
                new WaitUntilCommand(flywheelSubsystem::atTargetVelocity).withTimeout(3.0),
                Commands.run(() -> turboKickerSubsystem.hold(), turboKickerSubsystem),
                Commands.run(() -> turretSubsystem.setAngle(turretAngle.getDegrees()), turretSubsystem)
            ),
            
            // Phase 3: Feed note through turbokicker while holding turret (0.5 second feed)
            Commands.parallel(
                Commands.run(() -> turboKickerSubsystem.feed(), turboKickerSubsystem),
                Commands.run(() -> turretSubsystem.setAngle(turretAngle.getDegrees()), turretSubsystem)
            ).withTimeout(0.5)
        ).finallyDo(() -> {
            turboKickerSubsystem.stop();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
        });
    }
    
    /**
     * Create a shooting command with the specified flywheel velocity (no turret control)
     * NOTE: Hood is fixed at 14° mechanically
     * 
     * @param flywheelSubsystem The flywheel subsystem
     * @param turboKickerSubsystem The turbokicker subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param targetVelocityRPS Target flywheel velocity in RPS
     * @return The choreographed shoot command
     */
    public static Command shoot(
            FlywheelSubsystem flywheelSubsystem,
            TurboKickerSubsystem turboKickerSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            double targetVelocityRPS) {
        
        return Commands.sequence(
            // Phase 1: Spin up flywheel while holding turbokicker
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS), flywheelSubsystem),
                Commands.run(() -> turboKickerSubsystem.hold(), turboKickerSubsystem)
            ).withTimeout(0.1), // Brief moment to start motors
            
            // Phase 2: Wait for flywheel to reach target while holding turbokicker
            Commands.parallel(
                new WaitUntilCommand(flywheelSubsystem::atTargetVelocity).withTimeout(3.0),
                Commands.run(() -> turboKickerSubsystem.hold(), turboKickerSubsystem)
            ),
            
            // Phase 3: Feed note through turbokicker (0.5 second feed)
            Commands.run(() -> turboKickerSubsystem.feed(), turboKickerSubsystem)
                .withTimeout(0.5)
        ).finallyDo(() -> {
            turboKickerSubsystem.stop();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
        });
    }
    
    /**
     * Create a shooting command that runs indefinitely until interrupted
     * (e.g., while holding a button) with turret angle control
     * NOTE: Hood is fixed at 14° mechanically
     * 
     * @param flywheelSubsystem The flywheel subsystem
     * @param turboKickerSubsystem The turbokicker subsystem
     * @param turretSubsystem The turret subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param targetVelocityRPS Target flywheel velocity in RPS
     * @param turretAngle Target turret angle
     * @return The choreographed shoot command
     */
    public static Command shootContinuous(
            FlywheelSubsystem flywheelSubsystem,
            TurboKickerSubsystem turboKickerSubsystem,
            TurretSubsystem turretSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            double targetVelocityRPS,
            Rotation2d turretAngle) {
        
        return Commands.sequence(
            // Phase 0: Set turret angle
            Commands.runOnce(() -> turretSubsystem.setAngle(turretAngle.getDegrees()), turretSubsystem),
            
            // Phase 1: Spin up flywheel while holding turbokicker and maintaining turret
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS), flywheelSubsystem),
                Commands.run(() -> turboKickerSubsystem.hold(), turboKickerSubsystem),
                Commands.run(() -> turretSubsystem.setAngle(turretAngle.getDegrees()), turretSubsystem)
            ).withTimeout(0.1),
            
            // Phase 2: Wait for flywheel to reach target while maintaining turret and turbokicker
            Commands.parallel(
                new WaitUntilCommand(flywheelSubsystem::atTargetVelocity).withTimeout(3.0),
                Commands.run(() -> turboKickerSubsystem.hold(), turboKickerSubsystem),
                Commands.run(() -> turretSubsystem.setAngle(turretAngle.getDegrees()), turretSubsystem)
            ),
            
            // Phase 3: Feed note continuously through turbokicker while holding turret
            Commands.parallel(
                Commands.run(() -> turboKickerSubsystem.feed(), turboKickerSubsystem),
                Commands.run(() -> turretSubsystem.setAngle(turretAngle.getDegrees()), turretSubsystem)
            )
        ).finallyDo(() -> {
            turboKickerSubsystem.stop();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
        });
    }
    
    /**
     * Create a shooting command that runs indefinitely until interrupted
     * (e.g., while holding a button) - no turret control
     * NOTE: Hood is fixed at 14° mechanically
     * 
     * @param flywheelSubsystem The flywheel subsystem
     * @param turboKickerSubsystem The turbokicker subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param targetVelocityRPS Target flywheel velocity in RPS
     * @return The choreographed shoot command
     */
    public static Command shootContinuous(
            FlywheelSubsystem flywheelSubsystem,
            TurboKickerSubsystem turboKickerSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            double targetVelocityRPS) {
        
        return Commands.sequence(
            // Phase 1: Spin up flywheel while holding turbokicker
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS), flywheelSubsystem),
                Commands.run(() -> turboKickerSubsystem.hold(), turboKickerSubsystem)
            ).withTimeout(0.1),
            
            // Phase 2: Wait for flywheel to reach target while holding turbokicker
            Commands.parallel(
                new WaitUntilCommand(flywheelSubsystem::atTargetVelocity).withTimeout(3.0),
                Commands.run(() -> turboKickerSubsystem.hold(), turboKickerSubsystem)
            ),
            
            // Phase 3: Feed note continuously through turbokicker
            Commands.run(() -> turboKickerSubsystem.feed(), turboKickerSubsystem)
        ).finallyDo(() -> {
            turboKickerSubsystem.stop();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
        });
    }
}
