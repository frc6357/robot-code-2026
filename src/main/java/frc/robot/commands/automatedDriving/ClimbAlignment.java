package frc.robot.commands.automatedDriving;

import static frc.robot.Konstants.ClimbConstants.kAlignmentMaxAcceleration;
import static frc.robot.Konstants.ClimbConstants.kAlignmentMaxVelocity;
import static frc.robot.Konstants.ClimbConstants.kAlignmentP;
import static frc.robot.Konstants.ClimbConstants.kAlignmentI;
import static frc.robot.Konstants.ClimbConstants.kAlignmentD;
import static frc.robot.Konstants.ClimbConstants.kAlignmentToleranceMeters;
import static frc.robot.Konstants.ClimbConstants.kAlignmentYMaxAcceleration;
import static frc.robot.Konstants.ClimbConstants.kAlignmentYMaxVelocity;
import static frc.robot.Konstants.ClimbConstants.kAlignmentYP;
import static frc.robot.Konstants.ClimbConstants.kAlignmentYI;
import static frc.robot.Konstants.ClimbConstants.kAlignmentYD;
import static frc.robot.Konstants.ClimbConstants.kAlignmentYToleranceMeters;
import static frc.robot.Konstants.ClimbConstants.kAlignmentXEnableYErrorThreshold;
import static frc.robot.Konstants.ClimbConstants.kAlignmentRotMaxAcceleration;
import static frc.robot.Konstants.ClimbConstants.kAlignmentRotMaxVelocity;
import static frc.robot.Konstants.ClimbConstants.kAlignmentRotP;
import static frc.robot.Konstants.ClimbConstants.kAlignmentRotI;
import static frc.robot.Konstants.ClimbConstants.kAlignmentRotD;
import static frc.robot.Konstants.ClimbConstants.kAlignmentRotToleranceRadians;
import static frc.robot.Konstants.ClimbConstants.kTowerLeftRungBlue;
import static frc.robot.Konstants.ClimbConstants.kTowerRightRungBlue;
import static frc.robot.Konstants.ClimbConstants.kTowerLeftRungRed;
import static frc.robot.Konstants.ClimbConstants.kTowerRightRungRed;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utils.Field;
import frc.robot.subsystems.drive.DriveRequests;
import frc.robot.subsystems.drive.SKSwerve;

/**
 * Command that precisely aligns the robot to the closest tower rung using
 * ProfiledPIDControllers for X, Y, and rotational axes.
 *
 * <p>The X controller is only enabled once the Y error is within a threshold,
 * ensuring the robot first corrects its lateral position before driving forward.
 * The command finishes when all three controllers are at their setpoints.
 */
public class ClimbAlignment extends Command {

    private final SKSwerve drive;

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController rotController;

    private double targetX;
    private double targetY;
    private double targetRotRadians;
    private boolean flipPIDOutput;

    /**
     * Creates a new ClimbAlignment command.
     *
     * @param drive The swerve drive subsystem
     */
    public ClimbAlignment(SKSwerve drive) {
        this.drive = drive;
        addRequirements(drive);
        setName("ClimbXYRotAlignment");
    }

    @Override
    public void initialize() {
        flipPIDOutput = Field.isRed();

        Pose2d initPose = drive.getRobotPose();
        ChassisSpeeds initSpeeds = drive.getVelocity(true);

        // --- X controller ---
        xController = new ProfiledPIDController(
            kAlignmentP, kAlignmentI, kAlignmentD,
            new TrapezoidProfile.Constraints(kAlignmentMaxVelocity, kAlignmentMaxAcceleration)
        );
        xController.setTolerance(kAlignmentToleranceMeters);
        xController.reset(new State(initPose.getX(), initSpeeds.vxMetersPerSecond));

        // --- Y controller ---
        yController = new ProfiledPIDController(
            kAlignmentYP, kAlignmentYI, kAlignmentYD,
            new TrapezoidProfile.Constraints(kAlignmentYMaxVelocity, kAlignmentYMaxAcceleration)
        );
        yController.setTolerance(kAlignmentYToleranceMeters);
        yController.reset(new State(initPose.getY(), initSpeeds.vyMetersPerSecond));

        // --- Rotation controller ---
        rotController = new ProfiledPIDController(
            kAlignmentRotP, kAlignmentRotI, kAlignmentRotD,
            new TrapezoidProfile.Constraints(
                Math.toRadians(kAlignmentRotMaxVelocity),
                Math.toRadians(kAlignmentRotMaxAcceleration))
        );
        rotController.setTolerance(kAlignmentRotToleranceRadians);
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        rotController.reset(new State(initPose.getRotation().getRadians(), initSpeeds.omegaRadiansPerSecond));

        // --- Target: closest rung position ---
        Translation2d targetRung = getClosestTowerUpright(drive.getRobotPose().getY());
        targetX = targetRung.getX();
        targetY = targetRung.getY();
        targetRotRadians = Field.isBlue() ? 0.0 : Math.PI;

        xController.setGoal(new State(targetX, 0.0));
        yController.setGoal(new State(targetY, 0.0));
        rotController.setGoal(new State(targetRotRadians, 0.0));
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getRobotPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotRotRadians = robotPose.getRotation().getRadians();

        double yError = Math.abs(robotY - targetY);
        boolean xEnabled = yError <= kAlignmentXEnableYErrorThreshold;

        // Only run X controller when Y error is within threshold
        double xVelocity = xEnabled ? xController.calculate(robotX) : 0.0;
        double yVelocity = yController.calculate(robotY);
        double rotVelocity = rotController.calculate(robotRotRadians);

        // Apply velocity as field-centric X, Y, and rotational movement
        drive.setSwerveRequest(
            DriveRequests.getPidRequestUpdater(
                () -> flipPIDOutput ? -xVelocity : xVelocity,
                () -> flipPIDOutput ? -yVelocity : yVelocity,
                () -> rotVelocity)
                .apply(DriveRequests.pidRequest)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.setSwerveRequest(
            DriveRequests.getPidRequestUpdater(() -> 0.0, () -> 0.0, () -> 0.0)
                .apply(DriveRequests.pidRequest)
        );
    }

    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && rotController.atGoal();
    }

    /**
     * Determines the target tower post based on which one the robot is closer to.
     *
     * @param robotY Current robot Y position (meters)
     * @return The Translation2d of the closest tower upright (left or right)
     */
    private static Translation2d getClosestTowerUpright(double robotY) {
        Translation2d leftUpright;
        Translation2d rightUpright;

        if (Field.isBlue()) {
            leftUpright = kTowerLeftRungBlue;
            rightUpright = kTowerRightRungBlue;
        } else {
            leftUpright = kTowerLeftRungRed;
            rightUpright = kTowerRightRungRed;
        }

        double distToLeft = Math.abs(robotY - leftUpright.getY());
        double distToRight = Math.abs(robotY - rightUpright.getY());

        return distToLeft < distToRight ? leftUpright : rightUpright;
    }
}
