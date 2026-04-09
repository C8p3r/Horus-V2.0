// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.util.FuelSim;
import frc.robot.util.ShootingCalculator;
import frc.robot.util.MatchTimer;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    
    // Track if we've initialized pose from vision when first enabled
    private boolean poseInitializedFromVision = false;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        // Initialize AdvantageKit Logger
        if (!RobotContainer.DISABLE_ALL_TELEMETRY) {
            Logger.recordMetadata("ProjectName", "Horus-2026");
            Logger.recordMetadata("RobotName", "Horus");
            
            if (isReal()) {
                // Running on real robot - log to USB stick and publish to NetworkTables
                Logger.addDataReceiver(new WPILOGWriter()); // Log to USB
                Logger.addDataReceiver(new NT4Publisher()); // Publish to NetworkTables for AdvantageScope
            } else {
                // Running in simulation - only publish to NetworkTables
                Logger.addDataReceiver(new NT4Publisher());
            }
            
            // Start logging
            Logger.start();
        }
        
        // Publish target positions to NetworkTables for easy tuning
        if (!RobotContainer.DISABLE_ALL_TELEMETRY) {
            frc.robot.constants.FieldConstants.publishToNetworkTables();
        }
        
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        
        // Update match timer for driver station
        MatchTimer.updateDashboard();
        
        // Update controller telemetry
        m_robotContainer.periodic();
        
        // Publish current robot pose for debugging
        Pose2d currentPose = m_robotContainer.getDrivetrain().getPose();
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Robot/CurrentX", currentPose.getX());
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Robot/CurrentY", currentPose.getY());
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Robot/CurrentHeading", currentPose.getRotation().getDegrees());
    }

    @Override
    public void disabledInit() {
        // Reset the flag when entering disabled
        poseInitializedFromVision = false;
        
        // Return turret to initial position (180° - facing backward)
        m_robotContainer.getTurretSubsystem().returnToInitialPosition();
    }

    @Override
    public void disabledPeriodic() {
        // Initialize robot pose from vision ONCE while disabled (when first tags are seen)
        // After that, let normal vision fusion handle updates
        if (!poseInitializedFromVision) {
            Pose2d visionPose = m_robotContainer.getVisionSubsystem().getBestVisionPose();
            if (visionPose != null) {
                m_robotContainer.getDrivetrain().initializePoseFromVision(visionPose);
                poseInitializedFromVision = true;
                System.out.println("Pose initialized from vision: " + visionPose);
            }
        }
    }

    @Override
    public void disabledExit() {
        // When exiting disabled (entering auto or teleop), the pose is already set from vision
        // Wheel odometry will now supplement vision measurements
    }

    @Override
    public void autonomousInit() {
        // Pose already initialized from vision during disabled
        System.out.println("[Robot] autonomousInit() called");
        
        // Remove drivetrain default command so PathPlanner has full control
        Command driveDefaultCmd = m_robotContainer.drivetrain.getDefaultCommand();
        if (driveDefaultCmd != null) {
            System.out.println("[Robot] Removing drivetrain default command");
            CommandScheduler.getInstance().cancel(driveDefaultCmd);
            m_robotContainer.drivetrain.removeDefaultCommand();
            System.out.println("[Robot] Removed drivetrain default command");
        }
        
        // Remove intake default command so named commands have full control
        Command intakeDefaultCmd = m_robotContainer.intakeRollerSubsystem.getDefaultCommand();
        if (intakeDefaultCmd != null) {
            System.out.println("[Robot] Removing intake default command");
            CommandScheduler.getInstance().cancel(intakeDefaultCmd);
            m_robotContainer.intakeRollerSubsystem.removeDefaultCommand();
            System.out.println("[Robot] Removed intake default command");
        }
        
        // Remove TurboKicker default command so shooting commands have full control
        Command turboKickerDefaultCmd = m_robotContainer.turboKickerSubsystem.getDefaultCommand();
        if (turboKickerDefaultCmd != null) {
            System.out.println("[Robot] Removing TurboKicker default command");
            CommandScheduler.getInstance().cancel(turboKickerDefaultCmd);
            m_robotContainer.turboKickerSubsystem.removeDefaultCommand();
            System.out.println("[Robot] Removed TurboKicker default command");
        }
        
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        System.out.println("[Robot] Got autonomous command: " + (m_autonomousCommand != null ? m_autonomousCommand.getName() : "null"));

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
            System.out.println("[Robot] Scheduled autonomous command");
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        System.out.println("[Robot] autonomousExit() - autonomous has ended");
        
        // Cancel the autonomous command if it's still running
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
            m_autonomousCommand = null;
            System.out.println("[Robot] Cancelled autonomous command");
        }
        
        System.out.println("[Robot] Ready for teleop");
    }

    @Override
    public void teleopInit() {
        System.out.println("[Robot] teleopInit() - transitioning from autonomous to teleop");
        
        // Immediately cancel autonomous command
        if (m_autonomousCommand != null) {
            System.out.println("[Robot] Cancelling autonomous command");
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
            m_autonomousCommand = null;
        }
        
        // Clear any subsystem that might have lingering commands
        System.out.println("[Robot] Clearing subsystem commands");
        CommandScheduler.getInstance().cancel(m_robotContainer.drivetrain.getCurrentCommand());
        CommandScheduler.getInstance().cancel(m_robotContainer.intakeRollerSubsystem.getCurrentCommand());
        
        // Small delay to ensure clearing completes
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        
        // Restore all default commands
        System.out.println("[Robot] Restoring default commands");
        m_robotContainer.configureDefaultCommands();
        System.out.println("[Robot] Default commands restored successfully");
        
        // Reset shooter calibration values to initial state (hood=14°, flywheel=0 RPS)
        ShootingCalculator.getCalibration().resetToInitialValues();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        // Update FuelSim physics simulation
        FuelSim.getInstance().updateSim();
    }
}
