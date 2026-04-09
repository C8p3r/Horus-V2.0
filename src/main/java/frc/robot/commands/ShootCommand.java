package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TurboKickerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.IntakeWinchSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.constants.TurboKickerConstants;

/**
 * Choreographed shooting sequence with locking phase:
 * 
 * PHASE 0 - LOCK ON (Purple LEDs):
 *   1. Set turret angle and initialize belt floor
 *   2. Spin up flywheel while holding turbokicker in reverse
 *   3. Wait for flywheel to reach target velocity
 *   4. Prepare intake winch
 * 
 * PHASE 1 - EXECUTE (Yellow LEDs):
 *   1. Enter X-stance (wheels locked, immovable)
 *   2. Feed note through turbokicker
 *   3. Remain stationary regardless of external forces or pose data
 * 
 * On end: Stop all motors, exit X-stance
 */
public class ShootCommand {
    
    /**
     * Create a shooting command with locking phase, X-stance, and LED feedback
     * NOTE: Hood is fixed at 14° mechanically
     * 
     * @param flywheelSubsystem The flywheel subsystem
     * @param turboKickerSubsystem The turbokicker subsystem
     * @param turretSubsystem The turret subsystem
     * @param intakeRollerSubsystem The intake roller subsystem
     * @param intakeWinchSubsystem The intake winch subsystem
     * @param drivetrainSubsystem The drivetrain subsystem for X-stance
     * @param candleSubsystem The LED subsystem
     * @param targetVelocityRPS Target flywheel velocity in RPS
     * @param turretAngle Target turret angle
     * @return The choreographed shoot command with locking and X-stance phases
     */
    public static Command shoot(
            FlywheelSubsystem flywheelSubsystem,
            TurboKickerSubsystem turboKickerSubsystem,
            TurretSubsystem turretSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            IntakeWinchSubsystem intakeWinchSubsystem,
            CommandSwerveDrivetrain drivetrainSubsystem,
            CANdleSubsystem candleSubsystem,
            double targetVelocityRPS,
            Rotation2d turretAngle) {
        
        return Commands.sequence(
            // DEBUG: Confirm ShootCommand was invoked
            Commands.runOnce(() -> System.out.println("[ShootCommand] INITIATED - shoot command starting")),
            
            // ============ PHASE 0: LOCK ON (Purple LEDs) ============
            
            // Lock the LEDs so default command doesn't override our state
            Commands.runOnce(() -> {
                System.out.println("[ShootCommand] LOCKING LEDs to prevent default command override");
                candleSubsystem.lockLEDs();
            }, candleSubsystem),
            
            // DEBUG: Print that we're entering Phase 0
            Commands.runOnce(() -> {
                System.out.println("[ShootCommand] PHASE 0 START - Setting up for locking");
            }),
            
            // Set LEDs to purple (acquiring target, not yet locked)
            Commands.runOnce(() -> {
                System.out.println("[ShootCommand] Setting LEDs to PURPLE (locking phase)");
                candleSubsystem.setShootingLocking();
            }, candleSubsystem),
            
            // Phase 0a: Set turret angle and initialize belt floor
            Commands.runOnce(() -> {
                System.out.println("[ShootCommand] Phase 0a: Setting turret angle to " + turretAngle.getDegrees() + "°");
                turretSubsystem.setAngle(turretAngle.getDegrees());
                turboKickerSubsystem.setBeltFloorDutyCycle(TurboKickerConstants.BELT_FLOOR_FEED_DUTY_CYCLE);
            }, turretSubsystem, turboKickerSubsystem),
            
            // Phase 0b: Spin up flywheel while holding turbokicker in reverse and maintaining turret position
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS), flywheelSubsystem),
                Commands.run(() -> turboKickerSubsystem.hold(), turboKickerSubsystem),
                Commands.run(() -> turretSubsystem.setAngle(turretAngle.getDegrees()), turretSubsystem)
            ).withTimeout(0.1), // Brief moment to start motors
            
            // Delay 1 second then move winch to retracted position
            Commands.waitSeconds(0.9),
            Commands.runOnce(() -> intakeWinchSubsystem.setTargetPosition(8.0), intakeWinchSubsystem),
            
            // Phase 0c: Wait for flywheel to reach target (hold turret position and turbokicker)
            // LEDs remain purple until this completes
            Commands.parallel(
                new WaitUntilCommand(flywheelSubsystem::atTargetVelocity).withTimeout(3.0),
                Commands.run(() -> turboKickerSubsystem.hold(), turboKickerSubsystem),
                Commands.run(() -> turretSubsystem.setAngle(turretAngle.getDegrees()), turretSubsystem)
            ),
            
            // DEBUG: Print that we're entering Phase 1
            Commands.runOnce(() -> {
                System.out.println("[ShootCommand] PHASE 0 COMPLETE - Flywheel at target");
                System.out.println("[ShootCommand] PHASE 1 START - Entering X-stance and setting LEDs to YELLOW");
            }),
            
            // ============ PHASE 1: EXECUTE WITH X-STANCE (Yellow LEDs) ============
            
            // Enter X-stance FIRST: Lock wheels in place (SwerveDriveBrake prevents any movement)
            Commands.runOnce(() -> {
                System.out.println("[ShootCommand] ENTERING X-STANCE - Robot is now IMMOBILE");
                // Cancel drivetrain default command so it doesn't override our X-stance control
                CommandScheduler.getInstance().cancel(drivetrainSubsystem.getDefaultCommand());
                // Now set X-stance control
                drivetrainSubsystem.setControl(new SwerveRequest.SwerveDriveBrake());
            }, drivetrainSubsystem),
            
            // Set LEDs to yellow (locked and shooting)
            Commands.runOnce(() -> {
                System.out.println("[ShootCommand] Setting LEDs to YELLOW (locked and shooting)");
                candleSubsystem.setShootingReady();
            }, candleSubsystem),
            
            // Feed note through turbokicker while locked in X-stance (0.5 second feed)
            // The robot will NOT respond to external forces or movement commands during this phase
            Commands.run(() -> turboKickerSubsystem.feed(), turboKickerSubsystem)
                .withTimeout(0.5),
            
            // DEBUG: Print that we're ending
            Commands.runOnce(() -> {
                System.out.println("[ShootCommand] PHASE 1 COMPLETE - Feeding done, exiting X-stance");
            })
        ).finallyDo(() -> {
            System.out.println("[ShootCommand] CLEANUP - Stopping all motors and exiting X-stance");
            // Cleanup: Stop all motors and exit X-stance
            turboKickerSubsystem.stop();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
            intakeWinchSubsystem.setTargetPosition(0.0); // Return to position 0
            
            // Exit X-stance by returning control to idle
            System.out.println("[ShootCommand] Returning drivetrain to IDLE state");
            drivetrainSubsystem.setControl(new SwerveRequest.Idle());
            
            // Restore drivetrain default command so normal drive controls work again
            System.out.println("[ShootCommand] Restoring drivetrain default command");
            drivetrainSubsystem.setDefaultCommand(drivetrainSubsystem.getDefaultCommand());
            
            // Unlock LEDs so default command can resume control
            System.out.println("[ShootCommand] UNLOCKING LEDs - default command can now control");
            candleSubsystem.unlockLEDs();
            
            System.out.println("[ShootCommand] SHOOT COMMAND COMPLETE");
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
            // Phase 0: Reversed burst on feed motors to dislodge pieces (0.2 seconds)
            Commands.run(() -> turboKickerSubsystem.setFeedMotorsDutyCycle(-0.5), turboKickerSubsystem)
                .withTimeout(0.2),
            
            // Phase 1: Spin up flywheel and kicker motor while feed motors run at 90%
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS), flywheelSubsystem),
                Commands.run(() -> {
                    turboKickerSubsystem.setFeedMotorsDutyCycle(0.90);  // Feed motors at 90%
                    turboKickerSubsystem.setKickerMotorDutyCycle(0.90); // Kicker at 90%
                    turboKickerSubsystem.setBeltFloorDutyCycle(TurboKickerConstants.BELT_FLOOR_FEED_DUTY_CYCLE);   // Belt floor at 80%
                }, turboKickerSubsystem)
            ).withTimeout(0.1), // Brief moment to start motors
            
            // Phase 2: Wait for flywheel to reach target while keeping all motors at 90%
            Commands.parallel(
                new WaitUntilCommand(flywheelSubsystem::atTargetVelocity).withTimeout(3.0),
                Commands.run(() -> {
                    turboKickerSubsystem.setFeedMotorsDutyCycle(0.90);  // Feed motors at 90%
                    turboKickerSubsystem.setKickerMotorDutyCycle(0.90); // Kicker at 90%
                    turboKickerSubsystem.setBeltFloorDutyCycle(TurboKickerConstants.BELT_FLOOR_FEED_DUTY_CYCLE);   // Belt floor at 80%
                }, turboKickerSubsystem)
            ),
            
            // Phase 3: Continue feeding at 90% (0.5 second)
            Commands.run(() -> {
                turboKickerSubsystem.setFeedMotorsDutyCycle(0.90);  // Feed motors at 90%
                turboKickerSubsystem.setKickerMotorDutyCycle(0.90); // Kicker at 90%
                turboKickerSubsystem.setBeltFloorDutyCycle(TurboKickerConstants.BELT_FLOOR_FEED_DUTY_CYCLE);   // Belt floor at 80%
            }, turboKickerSubsystem).withTimeout(0.5)
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
            
            // Phase 1: Reversed burst on feed motors to dislodge pieces (0.2 seconds)
            Commands.parallel(
                Commands.run(() -> turboKickerSubsystem.setFeedMotorsDutyCycle(-0.5), turboKickerSubsystem),
                Commands.run(() -> turretSubsystem.setAngle(turretAngle.getDegrees()), turretSubsystem)
            ).withTimeout(0.2),
            
            // Phase 2: Spin up flywheel and run all motors at 90% while maintaining turret
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS), flywheelSubsystem),
                Commands.run(() -> {
                    turboKickerSubsystem.setFeedMotorsDutyCycle(0.90);  // Feed motors at 90%
                    turboKickerSubsystem.setKickerMotorDutyCycle(0.90); // Kicker at 90%
                    turboKickerSubsystem.setBeltFloorDutyCycle(TurboKickerConstants.BELT_FLOOR_FEED_DUTY_CYCLE);   // Belt floor at 80%
                }, turboKickerSubsystem),
                Commands.run(() -> turretSubsystem.setAngle(turretAngle.getDegrees()), turretSubsystem)
            ).withTimeout(0.1),
            
            // Phase 3: Wait for flywheel to reach target while maintaining 90% and turret
            Commands.parallel(
                new WaitUntilCommand(flywheelSubsystem::atTargetVelocity).withTimeout(3.0),
                Commands.run(() -> {
                    turboKickerSubsystem.setFeedMotorsDutyCycle(0.90);  // Feed motors at 90%
                    turboKickerSubsystem.setKickerMotorDutyCycle(0.90); // Kicker at 90%
                    turboKickerSubsystem.setBeltFloorDutyCycle(TurboKickerConstants.BELT_FLOOR_FEED_DUTY_CYCLE);   // Belt floor at 80%
                }, turboKickerSubsystem),
                Commands.run(() -> turretSubsystem.setAngle(turretAngle.getDegrees()), turretSubsystem)
            ),
            
            // Phase 4: Feed note continuously at 90% while holding turret (runs until interrupted)
            Commands.parallel(
                Commands.run(() -> {
                    turboKickerSubsystem.setFeedMotorsDutyCycle(0.90);  // Feed motors at 90%
                    turboKickerSubsystem.setKickerMotorDutyCycle(0.90); // Kicker at 90%
                    turboKickerSubsystem.setBeltFloorDutyCycle(TurboKickerConstants.BELT_FLOOR_FEED_DUTY_CYCLE);   // Belt floor at 80%
                }, turboKickerSubsystem),
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
            // Phase 0: Reversed burst on feed motors to dislodge pieces (0.2 seconds)
            Commands.run(() -> turboKickerSubsystem.setFeedMotorsDutyCycle(-0.5), turboKickerSubsystem)
                .withTimeout(0.2),
            
            // Phase 1: Spin up flywheel and run all motors at 90%
            Commands.parallel(
                Commands.runOnce(() -> flywheelSubsystem.setVelocity(targetVelocityRPS), flywheelSubsystem),
                Commands.run(() -> {
                    turboKickerSubsystem.setFeedMotorsDutyCycle(0.90);  // Feed motors at 90%
                    turboKickerSubsystem.setKickerMotorDutyCycle(0.90); // Kicker at 90%
                    turboKickerSubsystem.setBeltFloorDutyCycle(TurboKickerConstants.BELT_FLOOR_FEED_DUTY_CYCLE);   // Belt floor at 80%
                }, turboKickerSubsystem)
            ).withTimeout(0.1),
            
            // Phase 2: Wait for flywheel to reach target while maintaining 90%
            Commands.parallel(
                new WaitUntilCommand(flywheelSubsystem::atTargetVelocity).withTimeout(3.0),
                Commands.run(() -> {
                    turboKickerSubsystem.setFeedMotorsDutyCycle(0.90);  // Feed motors at 90%
                    turboKickerSubsystem.setKickerMotorDutyCycle(0.90); // Kicker at 90%
                    turboKickerSubsystem.setBeltFloorDutyCycle(TurboKickerConstants.BELT_FLOOR_FEED_DUTY_CYCLE);   // Belt floor at 80%
                }, turboKickerSubsystem)
            ),
            
            // Phase 3: Feed note continuously at 90% (runs until interrupted)
            Commands.run(() -> {
                turboKickerSubsystem.setFeedMotorsDutyCycle(0.90);  // Feed motors at 90%
                turboKickerSubsystem.setKickerMotorDutyCycle(0.90); // Kicker at 90%
                turboKickerSubsystem.setBeltFloorDutyCycle(TurboKickerConstants.BELT_FLOOR_FEED_DUTY_CYCLE);   // Belt floor at 80%
            }, turboKickerSubsystem)
        ).finallyDo(() -> {
            turboKickerSubsystem.stop();
            flywheelSubsystem.stop();
            intakeRollerSubsystem.stop();
        });
    }
}
