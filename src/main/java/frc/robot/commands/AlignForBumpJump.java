package frc.robot.commands;

import static frc.robot.Konstants.DriveConstants.RotationAligningConstants.kBumpJumpAngles;
import static frc.robot.Konstants.DriveConstants.RotationAligningConstants.kD;
import static frc.robot.Konstants.DriveConstants.RotationAligningConstants.kI;
import static frc.robot.Konstants.DriveConstants.RotationAligningConstants.kP;
import static frc.robot.Ports.DriverPorts.kFastMode;
import static frc.robot.Ports.DriverPorts.kSlowMode;
import static frc.robot.Ports.DriverPorts.kTranslationXPort;
import static frc.robot.Ports.DriverPorts.kTranslationYPort;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveRequests;
import frc.robot.subsystems.drive.SKSwerve;

public class AlignForBumpJump extends Command {
    private SKSwerve m_drive;
    private PIDController alignController = new PIDController(kP, kI, kD);
    private Rotation2d targetAngle;

    public AlignForBumpJump(SKSwerve drive) {
        m_drive = drive;
        addRequirements(m_drive);

        alignController.enableContinuousInput(-180, 180);
        alignController.setTolerance(10); // 10 degree tolerance since the bump is forgiving
    }
    
    @Override
    public void initialize() {
        alignController.reset();
        targetAngle = findClosestAngle();
        alignController.setSetpoint(targetAngle.getDegrees());
    }

    @Override
    public void execute() {
        double output = alignController.calculate(m_drive.getRobotRotation().getDegrees());
        m_drive.setSwerveRequest(
            DriveRequests.getTeleopRequestUpdater(
                () -> -kTranslationXPort.getFilteredAxis(), 
                () -> -kTranslationYPort.getFilteredAxis(), 
                () -> Math.toRadians(output), 
                () -> kSlowMode.button.getAsBoolean(), 
                () -> kFastMode.button.getAsBoolean())
            .apply(DriveRequests.teleopRequest)
        );
        SmartDashboard.putNumber("BumpAlign/Rottarget", targetAngle.getDegrees());
        SmartDashboard.putNumber("BumpAlign/RotOutput", output);
        SmartDashboard.putNumber("BumpAlign/CurrentRot", m_drive.getRobotRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

     @Override
     public void end(boolean interrupted) {

     }

     private Rotation2d findClosestAngle() {
        Rotation2d currentAngle = m_drive.getRobotRotation();

        for (Rotation2d angle : kBumpJumpAngles) {
            if (Math.abs(currentAngle.minus(angle).getDegrees()) < 45) { 
                return angle;
            }
        }

        return kBumpJumpAngles[0]; // default to first angle if somehow no angles are within 45 degrees (literally impossible but the compiler needs it)
     }
    
}
