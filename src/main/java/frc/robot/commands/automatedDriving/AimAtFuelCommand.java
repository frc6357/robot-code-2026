package frc.robot.commands.automatedDriving;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Konstants.DriveConstants.kMaxSpeed;
import static frc.robot.Konstants.DriveConstants.kMaxSpeedFAST;
import static frc.robot.Konstants.DriveConstants.kMaxSpeedSLOW;
import static frc.robot.Ports.DriverPorts.kLBbutton;
import static frc.robot.Ports.DriverPorts.kLSbutton;
import static frc.robot.Ports.DriverPorts.kLeftStickX;
import static frc.robot.Ports.DriverPorts.kLeftStickY;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.fueldetection.TrackedFuel;

import org.littletonrobotics.junction.Logger;

/**
 * While held, locks the robot's heading toward the <b>closest</b> confirmed
 * fuel using {@link SwerveRequest.RobotCentricFacingAngle}.  The driver's
 * left stick controls robot-centric translation — just push forward and the
 * robot drives straight at the ball while CTRE's heading controller keeps
 * the front of the robot pointed at it.
 *
 * <p>Slow-mode and fast-mode modifiers still work as normal.
 *
 * <p>If no confirmed fuel exists, the robot holds its current heading
 * so the driver doesn't lose control.
 */
public class AimAtFuelCommand extends Command {

    private final SKSwerve      drive;
    private final FuelDetection fuelDetection;

    /** CTRE request — handles the heading PID internally. Gains are
     *  tuned faster than the shared RotationAligningConstants. */
    private final SwerveRequest.RobotCentricFacingAngle request =
            new SwerveRequest.RobotCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withHeadingPID(4.0, 0.0, 0.1)
                .withDeadband(kMaxSpeed.times(0.05).in(MetersPerSecond));

    /** The last target we sent so we can hold it when fuel disappears. */
    private Rotation2d lastTarget;

    public AimAtFuelCommand(SKSwerve drive, FuelDetection fuelDetection) {
        this.drive         = drive;
        this.fuelDetection = fuelDetection;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Start by holding the current heading
        lastTarget = drive.getRobotRotation();
    }

    /** Max angle (degrees) off the robot's heading to consider a fuel target. */
    private static final double MAX_ANGLE_DEG = 90.0;

    @Override
    public void execute() {
        // --- Find the closest confirmed fuel in front of the robot ---
        Translation2d robot   = drive.getRobotPose().getTranslation();
        Rotation2d    heading = drive.getRobotRotation();
        List<TrackedFuel> confirmed = fuelDetection.getFuelMap().getConfirmedFuels();

        TrackedFuel closest = null;
        double closestDist = Double.MAX_VALUE;
        for (TrackedFuel f : confirmed) {
            Translation2d fuelPos = f.getFieldPosition();
            double dx = fuelPos.getX() - robot.getX();
            double dy = fuelPos.getY() - robot.getY();

            // Angle from robot to this fuel, in field space
            Rotation2d toFuel = new Rotation2d(dx, dy);
            // How far off our current heading is this fuel?
            double angleDiff = Math.abs(toFuel.minus(heading).getDegrees());

            if (angleDiff > MAX_ANGLE_DEG) continue; // behind us — skip

            double dist = fuelPos.getDistance(robot);
            if (dist < closestDist) {
                closestDist = dist;
                closest = f;
            }
        }

        boolean hasTarget = closest != null;
        if (hasTarget) {
            Translation2d target = closest.getFieldPosition();
            lastTarget = new Rotation2d(
                    target.getX() - robot.getX(),
                    target.getY() - robot.getY());
        }
        // else: keep lastTarget so the robot holds its last known heading

        // --- Determine speed multiplier (slow / fast / normal) ---
        double maxSpeed;
        if (kLBbutton.button.getAsBoolean()) {
            maxSpeed = kMaxSpeedSLOW.in(MetersPerSecond);
        } else if (kLSbutton.button.getAsBoolean()) {
            maxSpeed = kMaxSpeedFAST.in(MetersPerSecond);
        } else {
            maxSpeed = kMaxSpeed.in(MetersPerSecond);
        }

        // --- Robot-centric velocities from driver sticks ---
        double vx = -kLeftStickY.getFilteredAxis() * maxSpeed;
        double vy = -kLeftStickX.getFilteredAxis() * maxSpeed;

        // --- Send the request ---
        drive.setSwerveRequest(
            request.withVelocityX(vx)
                   .withVelocityY(vy)
                   .withTargetDirection(lastTarget)
        );

        // --- Logging ---
        Logger.recordOutput("FuelAim/TargetAngle",  lastTarget.getDegrees());
        Logger.recordOutput("FuelAim/HasTarget",     hasTarget);
    }

    @Override
    public void end(boolean interrupted) {
        // Default teleop command resumes automatically
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
