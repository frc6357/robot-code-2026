package frc.robot.bindings;

import java.util.Optional;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.vision.SKVision;

import static frc.robot.Ports.DriverPorts.kForceResetPoseToVision;
import static frc.robot.Ports.DriverPorts.kResetPoseToVision;
import static frc.robot.Ports.DriverPorts.kVisionOff;
import static frc.robot.Ports.DriverPorts.kVisionOn;

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

        this.resetPoseToVision = kResetPoseToVision.button;
        this.forceResetPoseToVision = kForceResetPoseToVision.button;
        this.visionOn = kVisionOn.button;
        this.visionOff = kVisionOff.button;
    }

    public void bindButtons() {
        if(m_visionContainer.isPresent() && m_swerveContainer.isPresent()) {
            // The specific swerve instance is needed in order to control the robot
            // while the vision commands are all static since vision doesn't need one specific
            // instance to be controlled. Vision should be able to run multiple commands
            // either in sequence or parallel with itself.
            SKSwerve m_swerve = m_swerveContainer.get();
            SKVision m_vision = m_visionContainer.get();

            forceResetPoseToVision.onTrue(new InstantCommand(() -> m_vision.forcePoseToVision()));
            resetPoseToVision.onTrue(new InstantCommand(() -> m_vision.resetPoseToVision()));

            // visionOff.onTrue(new InstantCommand())
        }
    }
}
