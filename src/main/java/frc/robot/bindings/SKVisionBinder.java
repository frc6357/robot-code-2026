package frc.robot.bindings;

import java.util.Optional;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.vision.SKVision;

import static frc.robot.Ports.DriverPorts.kDriverYbutton;
import static frc.robot.Ports.DriverPorts.kDriverBbutton;
import static frc.robot.Ports.DriverPorts.kDownDpad;
import static frc.robot.Ports.DriverPorts.kUpDpad;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SKVisionBinder implements CommandBinder {
    Optional<SKVision> m_visionContainer;
    Optional<SKSwerve> m_swerveContainer;

    Trigger alignToReef;
    Trigger leftReef;
    Trigger rightReef;
    Trigger forceResetPoseToVision;
    Trigger resetPoseToVision;
    Trigger visionOff;
    Trigger visionOn;
    Trigger visionEnabled;

    public SKVisionBinder(Optional<SKVision> m_visionContainer, Optional<SKSwerve> m_swerveContainer) {
        this.m_visionContainer = m_visionContainer;
        this.m_swerveContainer = m_swerveContainer;

        this.resetPoseToVision = kDriverBbutton.button;
        this.forceResetPoseToVision = kDriverYbutton.button;
        this.visionOn = kUpDpad.button;
        this.visionOff = kDownDpad.button;
    }

    public void bindButtons() {
        if(m_visionContainer.isPresent() && m_swerveContainer.isPresent()) {
            // The specific swerve instance is needed in order to control the robot
            // while the vision commands are all static since vision doesn't need one specific
            // instance to be controlled. Vision should be able to run multiple commands
            // either in sequence or parallel with itself.
            SKSwerve m_swerve = m_swerveContainer.get();
            SKVision m_vision = m_visionContainer.get();

            forceResetPoseToVision.onTrue(new InstantCommand(() -> m_vision.forcePoseToVision())
                .withName("VisionForceResetPose"));
            resetPoseToVision.onTrue(new InstantCommand(() -> m_vision.resetPoseToVision())
                .withName("VisionResetPose"));

            // visionOff.onTrue(new InstantCommand())
        }
    }
}
