// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.util.FuelSim;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

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
        
        // Update controller telemetry
        m_robotContainer.periodic();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
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
