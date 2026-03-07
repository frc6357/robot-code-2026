package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.RobotContainer.m_field;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * Subsystem to manage a target point in the field for the robot to interact with or align around.
 */
public class SKTargetPoint extends SubsystemBase{
    private Pose2d targetPoint;
    private final String name; // Used for identification if multiple target points are used


    public SKTargetPoint(Translation2d targetPoint, String name) {
        this.targetPoint = new Pose2d(targetPoint, Rotation2d.kZero);
        this.name = name;
    }

    public SKTargetPoint(Pose2d targetPoint, String name) {
        this.targetPoint = targetPoint;
        this.name = name;
    }

    public void setTargetPoint(Translation2d point) {
        this.targetPoint = new Pose2d(point, Rotation2d.kZero);
    }

    public Translation2d getTargetPoint() {
        return targetPoint.getTranslation();
    }

    public void setTargetPose(Pose2d pose) {
        this.targetPoint = pose;
    }

    public Pose2d getTargetPose() {
        return targetPoint;
    }

    /**
     * Move the target point by a given delta in X and Y directions.
     * @param deltaX Delta in X direction
     * @param deltaY Delta in Y direction
     */
    public void moveTargetPoint(double deltaX, double deltaY) {
        targetPoint = new Pose2d(
            targetPoint.getTranslation().getX() + deltaX,
            targetPoint.getTranslation().getY() + deltaY,
            targetPoint.getRotation()
        );
    }

    /**
     * Rotate the target point by a given angle in degrees. (CCW positive)
     * @param deltaTheta Angle in degrees to rotate the target point.
     */
    public void rotateTargetPoint(double deltaTheta) {
        targetPoint = new Pose2d(
            targetPoint.getTranslation(),
            targetPoint.getRotation().plus(Rotation2d.fromDegrees(deltaTheta))
        );
    }

    public Command movePointCommand(Supplier<Double> deltaX, Supplier<Double> deltaY) {
        return this.run(() -> moveTargetPoint(deltaX.get(), deltaY.get()));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("TargetPoint-" + name, targetPoint);
        m_field.getObject("TargetPoint-" + name).setPose(targetPoint);
    }
}
