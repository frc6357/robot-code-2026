// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Ports.DriverPorts.kDriver;
import static frc.robot.Ports.OperatorPorts.kOperator;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the LoggedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
    public static enum RobotMode {
        CONTROLLED, REPLAY
    }
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public static RobotMode Mode = RobotMode.CONTROLLED;

    public Robot() {
        this(RobotMode.CONTROLLED);
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot(RobotMode mode) {
        /* AdvantageKit Logging Initialization */
        Mode = mode;
        switch(mode) {
            case CONTROLLED: 
                if (isReal()) {
                    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
                    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
                }
                else {
                    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
                }
                break;
            case REPLAY:
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
                break;
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        
        DriverStation.silenceJoystickConnectionWarning(true);
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        //get the saved elastic dashboard layout
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        kDriver.setRumble(RumbleType.kBothRumble, 0.0);
        kOperator.setRumble(RumbleType.kBothRumble, 0.0);


        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand().withName("PathPlannerWarmup"));

        
        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putNumber("DS Match Time", DriverStation.getMatchTime());
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
            }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
