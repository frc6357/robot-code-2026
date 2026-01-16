package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.OperatorPorts.kMoveTargetX;
import static frc.robot.Ports.OperatorPorts.kMoveTargetY;
import static frc.robot.RobotContainer.m_field;

/**
 * Subsystem to manage a target point in the field for the robot to interact with or align around.
 */
public class SKTargetPoint extends SubsystemBase{
    private Pose2d targetPoint;
    private String name; // Used for identification if multiple target points are used

    // Publisher for network tables (especially helpful when displayed in AdvantageScope)
    private StructPublisher<Pose2d> targetPublisher;

    public SKTargetPoint(Translation2d targetPoint, String name) {
        this.targetPoint = new Pose2d(targetPoint, Rotation2d.kZero);
        this.name = name;

        targetPublisher = NetworkTableInstance.getDefault()
            .getTable("TargetPoints/" + name)
            .getStructTopic("Pose", Pose2d.struct).publish();
    }

    public SKTargetPoint(Pose2d targetPoint, String name) {
        this.targetPoint = targetPoint;
        this.name = name;

        targetPublisher = NetworkTableInstance.getDefault()
            .getTable("TargetPoints/" + name)
            .getStructTopic("Pose", Pose2d.struct).publish();
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

    public Command movePointCommand(double deltaX, double deltaY) {
        return this.runOnce(() -> this.moveTargetPoint(deltaX, deltaY)).ignoringDisable(true);
    }

    @Override
    public void periodic() {
        targetPublisher.set(targetPoint);
        SmartDashboard.putNumber("OperatorStickX", kMoveTargetX.getFilteredAxis());
        SmartDashboard.putNumber("OperatorStickY", kMoveTargetY.getFilteredAxis());
        m_field.getObject("TargetPoint-" + name).setPose(targetPoint);
    }
}
