package frc.robot.bindings;

import static frc.robot.Konstants.AutoConstants.kDefaultPathfindingConstraints;
import static frc.robot.Konstants.AutoConstants.kTrenchPathfindingConstraints;
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
import java.util.Set;

import com.pathplanner.lib.commands.PathfindThenFollowPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.auto.Pathfinder;
import frc.lib.bindings.CommandBinder;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
import frc.lib.utils.filters.LinearDeadbandFilter;
import frc.lib.utils.Field;
import frc.lib.utils.filters.DriveStickFilter;
import frc.robot.commands.automatedDriving.AlignAroundPoint;
import frc.robot.commands.automatedDriving.AlignForBumpJump;
import frc.robot.commands.automatedDriving.TravelUnderNearestTrench;
import frc.robot.subsystems.drive.DriveRequests;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.drive.SKTargetPoint;
import frc.robot.RobotContainer;
import frc.robot.Ports.DriverPorts;


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
        kLeftStickY.setFilter(translationXFilter);
        kLeftStickX.setFilter(translationYFilter);
        kRightStickX.setFilter(rotationFilter);

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

        // DriverPorts.kXbutton.button.toggleOnTrue(
        //     Pathfinder.PathfindToPoseCommand(new Pose2d(Field.flipIfRed(new Translation2d(5.838, 7.431)), drive.getRobotRotation()), kDefaultPathfindingConstraints, 3.5).withName("PathfindUnderLeftTrench")
        // );
        // DriverPorts.kBbutton.button.toggleOnTrue(
        //     Pathfinder.PathfindToPoseCommand(new Pose2d(Field.flipIfRed(new Translation2d(5.838, 0.613)), drive.getRobotRotation()), kDefaultPathfindingConstraints, 3.5).withName("PathfindUnderRightTrench")
        // );

        // DriverPorts.kXbutton.button.toggleOnTrue(
        //     Pathfinder.PathfindThenFollowPathCommand("PassUnderLTrench", kDefaultPathfindingConstraints)
        // );
        // DriverPorts.kBbutton.button.toggleOnTrue(
        //     Pathfinder.PathfindThenFollowPathCommand("PassUnderRTrench", kDefaultPathfindingConstraints)
        // );

        // DriverPorts.kYbutton.button.toggleOnTrue(
        //     Commands.defer(
        //         () -> TravelUnderNearestTrench.createCommand(
        //             drive.getRobotPose(),
        //             drive.getVelocity(true),
        //             kTrenchPathfindingConstraints),
        //         Set.of(drive))
        // );

        // bumpAlign.whileTrue(new AlignForBumpJump(drive));
        // hubAlign.whileTrue(
        //     new AlignAroundPoint(
        //         drive, 
        //         (Field.isBlue() ? kBlueHub.point : kRedHub.point)).withName("SwerveHubAlign"));

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