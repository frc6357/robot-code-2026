package frc.robot.bindings;

import java.util.Optional;

import frc.lib.bindings.CommandBinder;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.vision.SKVision;

import static frc.robot.Ports.DriverPorts.kYbutton;
import static frc.robot.Ports.DriverPorts.kBbutton;
import static frc.robot.Ports.DriverPorts.kDownDpad;
import static frc.robot.Ports.DriverPorts.kUpDpad;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SKVisionBinder implements CommandBinder {
    Optional<SKVision> m_visionContainer;
    Optional<SKSwerve> m_swerveContainer;

    Trigger forceResetPoseToVision;
    Trigger resetPoseToVision;
    Trigger visionOff;
    Trigger visionOn;
    Trigger visionEnabled;
    Trigger attemptStationaryPoseReset;

    public SKVisionBinder(Optional<SKVision> m_visionContainer, Optional<SKSwerve> m_swerveContainer) {
        this.m_visionContainer = m_visionContainer;
        this.m_swerveContainer = m_swerveContainer;

        this.resetPoseToVision = kBbutton.button;
        this.forceResetPoseToVision = kYbutton.button;
        this.visionOn = kUpDpad.button;
        this.visionOff = kDownDpad.button;
        if(m_swerveContainer.isPresent()) {
            attemptStationaryPoseReset = new Trigger(() ->
                {
                    ChassisSpeeds speeds = m_swerveContainer.get().getVelocity(false);
                    return (speeds.omegaRadiansPerSecond < (3.14/6.0)) && 
                            (speeds.vxMetersPerSecond < 0.1) &&
                            (speeds.vyMetersPerSecond < 0.1);
                }
            );
        }
    }

    public void bindButtons() {
        if(m_visionContainer.isPresent() && m_swerveContainer.isPresent()) {
            // The specific swerve instance is needed in order to control the robot
            // while the vision commands are all static since vision doesn't need one specific
            // instance to be controlled. Vision should be able to run multiple commands
            // either in sequence or parallel with itself.
            @SuppressWarnings("unused")
            SKSwerve m_swerve = m_swerveContainer.get();
            SKVision m_vision = m_visionContainer.get();

            forceResetPoseToVision.onTrue(new InstantCommand(() -> m_vision.forcePoseToVision())
                .withName("VisionForceResetPose").ignoringDisable(true));
            resetPoseToVision.onTrue(new InstantCommand(() -> m_vision.resetPoseToVision())
                .withName("VisionResetPose").ignoringDisable(true));
            attemptStationaryPoseReset.debounce(0.5).whileTrue(
                (new InstantCommand(() -> m_vision.stationaryDrivetrainUpdate(), m_vision)
                .andThen(Commands.waitSeconds(2)))
                .repeatedly().withName("VisionStationaryDrivetrainUpdate")
                .ignoringDisable(true)
            );

            // visionOff.onTrue(new InstantCommand())
        }
    }
}
