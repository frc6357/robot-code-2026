package frc.robot.subsystems.drive;

import static frc.robot.Konstants.AutoConstants.pathConfig;
import static frc.robot.RobotContainer.m_field;

import java.util.function.UnaryOperator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Konstants.DriveConstants;
import frc.robot.Robot;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class SKSwerve extends SubsystemBase {
    private SwerveDriveState lastReadState;
    private final GeneratedDrivetrain drivetrain = GeneratedConstants.createDrivetrain();
    private SwerveDrivePoseEstimator poseEstimator;
    private final GeneratedTelemetry telemetry = new GeneratedTelemetry(DriveConstants.kMaxSpeed.baseUnitMagnitude());
    private SwerveRequest currentRequest = DriveRequests.teleopRequest;

    private Pose2d[] emptyPath = new Pose2d[0];
            
    public void setSwerveRequest(SwerveRequest request) {
        // Only allows PathPlanner to control the drivetrain during auto period through its own request
        if(DriverStation.isAutonomousEnabled() && !request.equals(DriveRequests.pathPlannerRequest)) {
            return;
        }
		currentRequest = request;
	}

    /**
     * Creates a command that will continuously update the drivetrain's
     * target swerve request based on the given request and updater function.
     * @param request The specific request to follow.
     * @param updater The corresponding request updater. Make sure you use the corrrect updater.
     * @return The command.
     */
    public Command followSwerveRequestCommand(
        SwerveRequest.FieldCentric request, 
        UnaryOperator<SwerveRequest.FieldCentric> updater) {
        return run(() -> setSwerveRequest(updater.apply(request)))
                .handleInterrupt(() -> setSwerveRequest(new SwerveRequest.FieldCentric()));
    }

    /**
     * Creates a command that will continuously update the drivetrain's
     * target swerve request based on the given request and updater function.
     * The same as the FieldCentric version, but for RobotCentric requests.
     * @param request The specific request to follow.
     * @param updater The corresponding request updater. Make sure you use the corrrect updater.
     * @return The command.
     */
    public Command followSwerveRequestCommand(
        SwerveRequest.RobotCentric request, 
        UnaryOperator<SwerveRequest.RobotCentric> updater) {
        return run(() -> setSwerveRequest(updater.apply(request)))
                .handleInterrupt(() -> setSwerveRequest(new SwerveRequest.RobotCentric()));
    }

    
    public SKSwerve() {
        lastReadState = drivetrain.getState();
        
        setupPoseEstimator();
        configureAutoBuilder();
        
        PathPlannerLogging.setLogActivePathCallback((activePath) -> Logger.recordOutput("Drive/ActivePath", activePath.toArray(emptyPath)));

        drivetrain.setDefaultCommand(drivetrain.applyRequest(()-> currentRequest).withName("DrivetrainRequestApplier"));
        SmartDashboard.putData("Elastic Field 2D", m_field);
        // SmartDashboard.putData("Drive", this);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroRotation(), drivetrain.getState().ModulePositions);
        lastReadState = drivetrain.getState();

        outputTelemetry();

        Logger.recordOutput("PathPlanner/Active Path Name", PathPlannerAuto.currentPathName);
    }

    public void outputTelemetry() {
		telemetry.telemeterize(lastReadState);
        telemeterizeDevices();
		m_field.setRobotPose(getRobotPose());
	}

	public void telemeterizeDevices() {
		Logger.recordOutput(
				"Drive/Pitch Velocity Degrees Per Second",
				drivetrain
						.getPigeon2()
						.getAngularVelocityYDevice()
						.getValue()
						.in(Units.DegreesPerSecond));
		Logger.recordOutput(
				"Drive/Pitch Degrees",
				drivetrain.getPigeon2().getPitch().getValue().in(Units.Degrees));

		Logger.recordOutput(
				"Drive/Roll Velocity Degrees Per Second",
				drivetrain
						.getPigeon2()
						.getAngularVelocityXDevice()
						.getValue()
						.in(Units.DegreesPerSecond));
		Logger.recordOutput(
				"Drive/Roll Degrees",
				drivetrain.getPigeon2().getRoll().getValue().in(Units.Degrees));
            
		addModuleToLogger(0);
		addModuleToLogger(1);
		addModuleToLogger(2);
		addModuleToLogger(3);
	}

	private void addModuleToLogger(int module) {
		Logger.recordOutput(
				"Drive/ModuleStatesInfo/" + module + "/Drive/Volts",
				drivetrain
                    .getModules()[module]
                    .getDriveMotor()
                    .getMotorVoltage()
                    .getValue()
                    .in(Units.Volts));

		Logger.recordOutput(
				"Drive/ModuleStatesInfo/" + module + "/Rotation/Volts",
				drivetrain
                    .getModules()[module]
                    .getSteerMotor()
                    .getMotorVoltage()
                    .getValue()
                    .in(Units.Volts));

		Logger.recordOutput(
				"Drive/ModuleStatesInfo/" + module + "/Drive/Stator Current",
				drivetrain
                    .getModules()[module]
                    .getDriveMotor()
                    .getStatorCurrent()
                    .getValue()
                    .in(Units.Amps));

		Logger.recordOutput(
				"Drive/ModuleStatesInfo/" + module + "/Drive/Temperature Celsius",
				drivetrain
                    .getModules()[module]
                    .getDriveMotor()
                    .getDeviceTemp()
                    .getValue()
                    .in(Units.Celsius));

		Logger.recordOutput(
				"Drive/ModuleStatesInfo/" + module + "/Rotation/Stator Current",
                drivetrain
                    .getModules()[module]
                    .getSteerMotor()
                    .getStatorCurrent()
                    .getValue()
                    .in(Units.Amps));

		Logger.recordOutput(
				"Drive/ModuleStatesInfo/" + module + "/Drive/Supply Current",
				drivetrain
                    .getModules()[module]
                    .getDriveMotor()
                    .getSupplyCurrent()
                    .getValue()
                    .in(Units.Amps));

		Logger.recordOutput(
				"Drive/ModuleStatesInfo/" + module + "/Rotation/Supply Current",
				drivetrain
                    .getModules()[module]
                    .getSteerMotor()
                    .getSupplyCurrent()
                    .getValue()
                    .in(Units.Amps));

		Logger.recordOutput(
				"Drive/ModuleStatesInfo/" + module + "/Rotation/Temperature Celsius",
				drivetrain
                    .getModules()[module]
                    .getSteerMotor()
                    .getDeviceTemp()
                    .getValue()
                    .in(Units.Celsius));
	}


    public GeneratedDrivetrain getDrivetrain() {
        return drivetrain;
    }

    private void setupPoseEstimator() {
        poseEstimator = 
            new SwerveDrivePoseEstimator(
                drivetrain.getKinematics(), 
                new Rotation2d(drivetrain.getPigeon2().getYaw().getValue()), 
                drivetrain.getState().ModulePositions, 
                new Pose2d());
    }

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to
     * change trust in vision measurements after the autonomous period, or to change
     * trust as distance to a vision target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision
     *                                 measurements. Increase these numbers to
     *                                 trust global measurements from vision less.
     *                                 This matrix is in the form [x, y, theta]ᵀ,
     *                                 with units in meters and radians.
     */
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /* 
     *
     * Autonomous
     * 
     */

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getRobotPose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> drivetrain.getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setSwerveRequest(
                    DriveRequests.getPathPlannerRequestUpdater(() -> speeds, () -> feedforwards).apply(DriveRequests.pathPlannerRequest)
                ),
                pathConfig,
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public void simulateCollision() {
        if(!Robot.isReal()) {
            resetPose(getRobotPose().plus(new Transform2d(-0.4, 0.5, getRobotRotation().plus(Rotation2d.fromDegrees(30)))));
        }
    }

    /**
    * Set chassis speeds of robot to drive it robot oreintedly.
    * @param chassisSpeeds Chassis Speeds to set.
    */
    public void chassisSpeedsDrive(ChassisSpeeds chassisSpeeds, DriveFeedforwards ff)
    {
        SwerveRequest chassisSpeed = new SwerveRequest.ApplyRobotSpeeds().withSpeeds(chassisSpeeds);
        drivetrain.setControl(chassisSpeed);
    }

    // public void robotRelativeDrive(ChassisSpeeds speeds)
    // {
    //     final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();
    //     this.applyRequest(() ->
    //             robotCentricDrive.withVelocityX(speeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
    //                 .withVelocityY(speeds.vyMetersPerSecond) // Drive left with negative X (left)
    //                 .withRotationalRate(speeds.omegaRadiansPerSecond)); // Drive counterclockwise with negative X (left)
    // }

    /* 
     *
     * Odemetry Methods 
     * 
     */


    /**
     * 
     * @return The estimation of the robot's pose
     */
    @AutoLogOutput(key = "Drive/EstimatedPose")
    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
       // return pose;
    }

    public SwerveModuleState[] getModuleStates() {
        return drivetrain.getState().ModuleStates;
    }

    public Rotation2d getRobotRotation() {
        return getRobotPose().getRotation();
    }

    public Rotation2d getGyroRotation() {
        return drivetrain.getPigeon2().getRotation2d();
    }
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return drivetrain.getKinematics().toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getVelocity(boolean fieldRelative) {
        if(fieldRelative) {
            return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getGyroRotation());
        }
        else {
            return getRobotRelativeSpeeds();
        }
    }

    public void resetOrientation() {
        boolean flip = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
          if (flip) {
            drivetrain.resetRotation(Rotation2d.k180deg);
          } else {
            drivetrain.resetRotation(Rotation2d.kZero);
          }
    }

    /** Resets odometry to the given pose.
     * @param pose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d pose)
    {
        resetPose(pose);
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
        poseEstimator.resetPose(pose);
    }

    // public ChassisSpeeds getRobotSpeeds()
    //     {
    //         SwerveDriveKinematics m_kinematics = this.getKinematics();
    //         SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = this.getModules();
    //         SwerveModuleState[] currentStates = new SwerveModuleState[4];
    //         for (int i = 0; i < 4; i++)
    //         {
    //             SwerveModuleState phoenixModuleState = modules[i].getCurrentState();
    //             currentStates[i] = phoenixModuleState;
    //         }
    //         return m_kinematics.toChassisSpeeds(currentStates);
    //     }
}
