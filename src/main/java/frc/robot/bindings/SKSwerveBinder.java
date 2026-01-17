package frc.robot.bindings;

import static frc.robot.Konstants.OIConstants.kJoystickDeadband;
import static frc.robot.Ports.DriverPorts.kFastMode;
import static frc.robot.Ports.DriverPorts.kResetGyroPos;
import static frc.robot.Ports.DriverPorts.kRobotCentricMode;
import static frc.robot.Ports.DriverPorts.kSlowMode;
import static frc.robot.Ports.DriverPorts.kTranslationXPort;
import static frc.robot.Ports.DriverPorts.kTranslationYPort;
import static frc.robot.Ports.DriverPorts.kVelocityOmegaPort;
import static frc.robot.Ports.DriverPorts.kBumpAlign;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
import frc.lib.utils.filters.LinearDeadbandFilter;
import frc.lib.utils.Field;
import frc.lib.utils.filters.DriveStickFilter;
import frc.robot.commands.AlignForBumpJump;
import frc.robot.subsystems.drive.DriveRequests;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.drive.SKTargetPoint;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignAroundPoint;

import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kBlueHub;
import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kOperatorControlled;
import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kRedHub;
import static frc.robot.Konstants.TargetPointConstants.targetPoints;

@SuppressWarnings("unused")
public class SKSwerveBinder implements CommandBinder{
    Optional<SKSwerve>  m_drive;
    DriveStickFilter translationXFilter;
    DriveStickFilter translationYFilter;
    DriveStickFilter rotationFilter;
    boolean slowModeStatus;

    //Allow alterable slew rates from the dashboard.
     Pref<Double> driverTranslationSlewPref = SKPreferences.attach("driverTranslSlew", 4.0)
                 .onChange((newValue) -> {
                     translationXFilter.setSlewRate(newValue);
                     translationYFilter.setSlewRate(newValue);
                 });
    

    //Allow alterable slew rates from the dashboard.
    Pref<Double> driverRotationSlewPref = SKPreferences.attach("driverRotSlew", 4.0)
                .onChange((newValue) -> {
                    rotationFilter.setSlewRate(newValue);
                });

    //Driver buttons
    private final Trigger robotCentric = kRobotCentricMode.button;
    private final Trigger slowmode = kSlowMode.button;
    private final Trigger resetButton = kResetGyroPos.button;
    private final Trigger fastmode = kFastMode.button;
    private final Trigger hubAlign = kBumpAlign.button;


    public SKSwerveBinder(Optional<SKSwerve> m_drive) {
        this.m_drive = m_drive;

        this.translationXFilter = new DriveStickFilter(
            driverTranslationSlewPref.get(),
            kJoystickDeadband);
        this.translationYFilter = new DriveStickFilter(
            driverTranslationSlewPref.get(), 
            kJoystickDeadband);
        this.rotationFilter = new DriveStickFilter(
            driverRotationSlewPref.get(), 
            kJoystickDeadband);
    }

    @Override
    public void bindButtons()
    {
        if (!m_drive.isPresent())
        {
            return;
        }

        SKSwerve drive = m_drive.get();

        // TODO: Might need to uncomment this later? It caused a weird issue that made the robot drift like it's in space.
        // Sets filters for driving axes
        kTranslationXPort.setFilter(new LinearDeadbandFilter(kJoystickDeadband, 1.0));
        kTranslationYPort.setFilter(new LinearDeadbandFilter(kJoystickDeadband, 1.0));
        kVelocityOmegaPort.setFilter(new LinearDeadbandFilter(kJoystickDeadband, 1.0));

        robotCentric.whileTrue(
            drive.followSwerveRequestCommand(
                DriveRequests.robotCentricTeleopRequest, 
                DriveRequests.getRobotCentricTeleopRequestUpdater(
                        () -> -kTranslationXPort.getFilteredAxis(), 
                        () -> -kTranslationYPort.getFilteredAxis(), 
                        () -> -kVelocityOmegaPort.getFilteredAxis(), 
                        () -> slowmode.getAsBoolean(), 
                        () -> fastmode.getAsBoolean())
        ));
        
        // Resets gyro angles / robot oreintation
        resetButton.onTrue(new InstantCommand(() -> {drive.resetOrientation();} ));

        // bumpAlign.whileTrue(new AlignForBumpJump(drive));
        hubAlign.whileTrue(
            new AlignAroundPoint(
                drive, 
                (Field.isBlue() ? targetPoints[kBlueHub.ordinal()] : targetPoints[kRedHub.ordinal()])));

        drive.setDefaultCommand(
            drive.followSwerveRequestCommand(
                DriveRequests.teleopRequest, 
                DriveRequests.getTeleopRequestUpdater(
                        () -> -kTranslationXPort.getFilteredAxis(), 
                        () -> -kTranslationYPort.getFilteredAxis(), 
                        () -> -kVelocityOmegaPort.getFilteredAxis(), 
                        () -> slowmode.getAsBoolean(), 
                        () -> fastmode.getAsBoolean())
            )
        );
    }
}