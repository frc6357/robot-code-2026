package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
// import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;

public class GeneratedTelemetry {
	// private final double MaxSpeed;
	private final boolean isReal;

	/**
	 * Construct a telemetry object, with the specified max speed of the robot
	 *
	 * @param maxSpeed Maximum speed in meters per second
	 * @param isReal Whether the robot is real or simulated, for any simulation-specific telemetry
	 */
	public GeneratedTelemetry(double maxSpeed, boolean isReal) {
		// MaxSpeed = maxSpeed;
		this.isReal = isReal;
	}

	/* Mechanisms to represent the swerve module states */
	// private final LoggedMechanism2d[] m_moduleMechanisms = new LoggedMechanism2d[] {
	// 	new LoggedMechanism2d(1, 1), new LoggedMechanism2d(1, 1), new LoggedMechanism2d(1, 1), new LoggedMechanism2d(1, 1),
	// };
	/* A direction and length changing ligament for speed representation */
	// private final LoggedMechanismLigament2d[] m_moduleSpeeds = new LoggedMechanismLigament2d[] {
	// 	m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
	// 	m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
	// 	m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
	// 	m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
	// };
	/* A direction changing and length constant ligament for module direction */
	// private final LoggedMechanismLigament2d[] m_moduleDirections = new LoggedMechanismLigament2d[] {
	// 	m_moduleMechanisms[0]
	// 			.getRoot("RootDirection", 0.5, 0.5)
	// 			.append(new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
	// 	m_moduleMechanisms[1]
	// 			.getRoot("RootDirection", 0.5, 0.5)
	// 			.append(new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
	// 	m_moduleMechanisms[2]
	// 			.getRoot("RootDirection", 0.5, 0.5)
	// 			.append(new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
	// 	m_moduleMechanisms[3]
	// 			.getRoot("RootDirection", 0.5, 0.5)
	// 			.append(new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
	// };

	private final double[] m_moduleStatesArray = new double[8];
	private final double[] m_moduleTargetsArray = new double[8];

	/** Accept the swerve drive state and telemeterize it to SmartDashboard and AdvantageKit */
	public void telemeterize(SwerveDriveState state) {
		/* Telemeterize the swerve drive state */
		Logger.recordOutput("Drive/DrivetrainPose", state.Pose);
		Logger.recordOutput("Drive/Speeds", state.Speeds);
		Logger.recordOutput("Drive/ModuleStates", state.ModuleStates);
		Logger.recordOutput("Drive/ModuleTargets", state.ModuleTargets);
		Logger.recordOutput("Drive/ModulePositions", state.ModulePositions);
		Logger.recordOutput("Drive/Timestamp", state.Timestamp);
		Logger.recordOutput("Drive/OdometryFrequency", 1.0 / state.OdometryPeriod);
		for (int i = 0; i < 4; ++i) {
			m_moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
			m_moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
			m_moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
			m_moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
		}

		Logger.recordOutput("Drive/OdometryPeriod", state.OdometryPeriod);

		if(isReal){
			return;
		}
		/* Telemeterize the module states to a Mechanism2d */
		// for (int i = 0; i < 4; ++i) {
		// 	m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
		// 	m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
		// 	m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

		// 	Logger.recordOutput("Drive/Module " + i, m_moduleMechanisms[i]);
		// }
	}
}
