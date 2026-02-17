package frc.robot.bindings;

import static frc.robot.Konstants.OIConstants.kJoystickDeadband;
import static frc.robot.Ports.DriverPorts.kLSbutton;
import static frc.robot.Ports.DriverPorts.kRSbutton;
import static frc.robot.Ports.DriverPorts.kRBbutton;
import static frc.robot.Ports.DriverPorts.kLBbutton;
import static frc.robot.Ports.DriverPorts.kLeftStickY;
import static frc.robot.Ports.DriverPorts.kLeftStickX;
import static frc.robot.Ports.DriverPorts.kRightStickX;
import static frc.robot.Ports.DriverPorts.kLTrigger;

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
    private final Trigger robotCentric = kRBbutton.button;
    private final Trigger slowmode = kLBbutton.button;
    private final Trigger resetButton = kRSbutton.button;
    private final Trigger fastmode = kLSbutton.button;
    private final Trigger hubAlign = kLTrigger.button;


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

        // Sets filters for driving axes
        kLeftStickY.setFilter(new LinearDeadbandFilter(kJoystickDeadband, 1.0));
        kLeftStickX.setFilter(new LinearDeadbandFilter(kJoystickDeadband, 1.0));
        kRightStickX.setFilter(new LinearDeadbandFilter(kJoystickDeadband, 1.0));

        robotCentric.whileTrue(
            drive.followSwerveRequestCommand(
                DriveRequests.robotCentricTeleopRequest, 
                DriveRequests.getRobotCentricTeleopRequestUpdater(
                        () -> -kLeftStickY.getFilteredAxis(), 
                        () -> -kLeftStickX.getFilteredAxis(), 
                        () -> -kRightStickX.getFilteredAxis(), 
                        () -> slowmode.getAsBoolean(), 
                        () -> fastmode.getAsBoolean())).withName("SwerveRobotCentricDrive")
        );
        
        // Resets gyro angles / robot oreintation
        resetButton.onTrue(new InstantCommand(() -> {drive.resetOrientation();} ));

        // bumpAlign.whileTrue(new AlignForBumpJump(drive));
        hubAlign.whileTrue(
            new AlignAroundPoint(
                drive, 
                (Field.isBlue() ? kBlueHub.point : kRedHub.point)).withName("SwerveHubAlign"));

        drive.setDefaultCommand(
            drive.followSwerveRequestCommand(
                DriveRequests.teleopRequest, 
                DriveRequests.getTeleopRequestUpdater(
                        () -> -kLeftStickY.getFilteredAxis(), 
                        () -> -kLeftStickX.getFilteredAxis(), 
                        () -> -kRightStickX.getFilteredAxis(), 
                        () -> slowmode.getAsBoolean(), 
                        () -> fastmode.getAsBoolean())
            ).withName("SwerveTeleopDrive")
        );
    }
}