package frc.robot.bindings;

import java.util.Optional;

import frc.lib.bindings.CommandBinder;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.vision.SKVision;

import static frc.robot.Ports.DriverPorts.kYbutton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SKVisionBinder implements CommandBinder {
    Optional<SKVision> m_visionContainer;
    Optional<SKSwerve> m_swerveContainer;

    Trigger forceResetPoseToVision;
    Trigger attemptStationaryPoseReset;

    public SKVisionBinder(Optional<SKVision> m_visionContainer, Optional<SKSwerve> m_swerveContainer) {
        this.m_visionContainer = m_visionContainer;
        this.m_swerveContainer = m_swerveContainer;

        this.forceResetPoseToVision = kYbutton.button;
        if(m_swerveContainer.isPresent()) {
            attemptStationaryPoseReset = new Trigger(() ->
                {
                    ChassisSpeeds speeds = m_swerveContainer.get().getVelocity(false);
                    return (Math.abs(speeds.omegaRadiansPerSecond) < (3.14/6.0)) && 
                            (Math.abs(speeds.vxMetersPerSecond) < 0.1) &&
                            (Math.abs(speeds.vyMetersPerSecond) < 0.1);
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
            SKVision m_vision = m_visionContainer.get();

            forceResetPoseToVision.onTrue(new InstantCommand(() -> m_vision.forcePoseToVision())
                .withName("VisionForceResetPose").ignoringDisable(true));
            attemptStationaryPoseReset.debounce(0.5).whileTrue(
                (new InstantCommand(() -> m_vision.stationaryDrivetrainUpdate(), m_vision)
                .andThen(Commands.waitSeconds(2)))
                .repeatedly().withName("VisionStationaryDrivetrainUpdate")
                .ignoringDisable(true)
            );

        }
    }
}
