package frc.robot.commands.automatedDriving;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Konstants.DriveConstants.kMaxSpeed;
import static frc.robot.Konstants.DriveConstants.kMaxSpeedFAST;
import static frc.robot.Konstants.DriveConstants.kMaxSpeedSLOW;
import static frc.robot.Konstants.DriveConstants.RotationAligningConstants.kD;
import static frc.robot.Konstants.DriveConstants.RotationAligningConstants.kI;
import static frc.robot.Konstants.DriveConstants.RotationAligningConstants.kP;
import static frc.robot.Ports.DriverPorts.kLBbutton;
import static frc.robot.Ports.DriverPorts.kLSbutton;
import static frc.robot.Ports.DriverPorts.kLeftStickX;
import static frc.robot.Ports.DriverPorts.kLeftStickY;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.fueldetection.FuelCluster;
import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.fueldetection.FuelScorer;

import org.littletonrobotics.junction.Logger;

/**
 * While held, locks the robot's heading toward the best fuel cluster using
 * {@link SwerveRequest.RobotCentricFacingAngle}.  The driver's left stick
 * controls robot-centric translation — just push forward and the robot
 * drives straight at the balls while CTRE's heading controller keeps the
 * front of the robot pointed at them.
 *
 * <p>Slow-mode and fast-mode modifiers still work as normal.
 *
 * <p>If no fuel cluster is confirmed, the robot holds its current heading
 * so the driver doesn't lose control.
 */
public class AimAtFuelCommand extends Command {

    private final SKSwerve      drive;
    private final FuelDetection fuelDetection;

    /** CTRE request — handles the heading PID internally. */
    private final SwerveRequest.RobotCentricFacingAngle request =
            new SwerveRequest.RobotCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withHeadingPID(kP, kI, kD)
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

    @Override
    public void execute() {
        // --- Determine the desired field heading ---
        Optional<FuelCluster> best = FuelScorer.bestCluster(
                fuelDetection.getFuelMap().getConfirmedFuels(),
                drive.getRobotPose().getTranslation());

        if (best.isPresent()) {
            Translation2d target = best.get().getCentroid();
            Translation2d robot  = drive.getRobotPose().getTranslation();
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
        Logger.recordOutput("FuelAim/HasTarget",     best.isPresent());
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
