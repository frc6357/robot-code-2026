package frc.robot.commands;

import static frc.robot.Konstants.DriveConstants.RotationAligningConstants.kD;
import static frc.robot.Konstants.DriveConstants.RotationAligningConstants.kI;
import static frc.robot.Konstants.DriveConstants.RotationAligningConstants.kP;
import static frc.robot.Ports.DriverPorts.kFastMode;
import static frc.robot.Ports.DriverPorts.kSlowMode;
import static frc.robot.Ports.DriverPorts.kTranslationXPort;
import static frc.robot.Ports.DriverPorts.kTranslationYPort;
import frc.robot.subsystems.drive.DriveRequests;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.drive.SKTargetPoint;

public class AlignAroundPoint extends Command{
    private SKSwerve m_drive;
    private SKTargetPoint targetPoint;
    private PIDController alignController = new PIDController(kP, kI, kD);
    private Rotation2d targetAngle;

    public AlignAroundPoint(SKSwerve m_drive, SKTargetPoint targetPoint) {
        this.targetPoint = targetPoint;
        this.m_drive = m_drive;
        addRequirements(m_drive);

        alignController.enableContinuousInput(-180, 180);
        alignController.setTolerance(2); // 2 degree tolerance for alignment
    }

    @Override
    public void initialize() {
        alignController.reset();
    }

    @Override
    public void execute() {
        double desiredAngle = Math.toDegrees(
            Math.signum(m_drive.getRobotPose().getTranslation().getY() - targetPoint.getTargetPoint().getY()) *
            Math.acos(
                (m_drive.getRobotPose().getTranslation().getX() - targetPoint.getTargetPoint().getX()) / 
                m_drive.getRobotPose().getTranslation().getDistance(targetPoint.getTargetPoint())))
                 + 180; // Instead of matching the angle directly, face opposite of it (towards the point)
        if(Double.isNaN(desiredAngle)) {
            desiredAngle = m_drive.getRobotRotation().getDegrees();
        }
        double output = alignController.calculate(m_drive.getRobotRotation().getDegrees(), desiredAngle);
        
        m_drive.setSwerveRequest(
            DriveRequests.getTeleopRequestUpdater(
                () -> -kTranslationXPort.getFilteredAxis(), 
                () -> -kTranslationYPort.getFilteredAxis(), 
                () -> Math.toRadians(output), 
                () -> kSlowMode.button.getAsBoolean(), 
                () -> kFastMode.button.getAsBoolean())
            .apply(DriveRequests.teleopRequest)
        );

        SmartDashboard.putNumber("PointAlign/Rottarget", desiredAngle);
        SmartDashboard.putNumber("PointAlign/RotOutput", output);
        SmartDashboard.putNumber("PointAlign/CurrentRot", m_drive.getRobotRotation().getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
}
