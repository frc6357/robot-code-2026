// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.lib.commands.PathPlannerCommands;
import frc.lib.utils.SubsystemControls;
import frc.lib.utils.filters.FilteredJoystick;
import frc.robot.Robot.RobotMode;
import frc.robot.bindings.SK26ClimbBinder;
import frc.robot.bindings.FuelHuntBinder;
import frc.robot.bindings.SK26BBLauncherBinder;
import frc.robot.bindings.SK26FeederBinder;
import frc.robot.bindings.SK26GuitarHeroBinder;
import frc.robot.bindings.SK26IndexerBinder;
import frc.robot.bindings.SK26IntakePivotBinder;
import frc.robot.bindings.SK26IntakeRollersBinder;
import frc.robot.bindings.SK26LauncherBinder;
import frc.robot.bindings.SK26LightsBinder;
import frc.robot.bindings.SK26ShootingCoordinatorBinder;
import frc.robot.bindings.SK26StateBinder;
import frc.robot.bindings.SK26TurretBinder;
import frc.robot.bindings.SKSwerveBinder;
import frc.robot.bindings.SKTargetPointsBinder;
import frc.robot.bindings.SKVisionBinder;
import frc.robot.subsystems.climb.SK26Climb;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.feeder.SK26Feeder;
import frc.robot.subsystems.indexer.SK26Indexer;
import frc.robot.subsystems.intake.SK26IntakePivot;
import frc.robot.subsystems.intake.SK26IntakeRollers;
import frc.robot.subsystems.launcher.mechanisms.BangBangLauncher;
import frc.robot.subsystems.launcher.mechanisms.SK26DualLauncher;
import frc.robot.subsystems.launcher.mechanisms.SK26Launcher;
import frc.robot.subsystems.launcher.moveandshoot.ShootingCoordinator;
import frc.robot.subsystems.lights.SK26Lights;
import frc.robot.subsystems.turret.SK26Turret;
import frc.robot.subsystems.turret.SK26TurretSim;
import frc.robot.subsystems.vision.SKVision;
import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.vision.VisionConfig;

import static frc.robot.Ports.DriverPorts.kDriver;
import static frc.robot.Ports.OperatorPorts.kOperator;
import static frc.robot.StateHandler.MacroState;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // "MaxSpeed"


    // private void configurePhoenixTelemetry() {

    //     m_swerve.get().registerTelemetry(logger::telemeterize);
    // }

  // The robot's subsystems and commands are defined here...

  public Optional<SKSwerve> m_swerveContainer = Optional.empty();
  public Optional<SKVision> m_visionContainer = Optional.empty();
  public Optional<SK26Climb> m_climbComtainer = Optional.empty();
  public Optional<SK26Turret> m_turretContainer = Optional.empty();
  public Optional<BangBangLauncher> m_BBLauncherContainer = Optional.empty();
  public Optional<SK26Launcher> m_StandardLauncherContainer = Optional.empty();
  public Optional<SK26DualLauncher> m_DualLauncherContainer = Optional.empty();
  public Optional<StateHandler> m_stateHandlerContainer = Optional.empty();
  public Optional<SK26Lights> m_lightsContainer = Optional.empty();
  public Optional<SK26IntakePivot> m_intakePivotContainer = Optional.empty();
  public Optional<SK26IntakeRollers> m_intakeRollersContainer = Optional.empty();
  public Optional<SK26Indexer> m_indexerContainer = Optional.empty();
  public Optional<FuelDetection> m_fuelDetectionContainer = Optional.empty();

  public Optional<ShootingCoordinator> m_shootingCoordinator = Optional.empty();
  public Optional<SK26Feeder> m_feederContainer = Optional.empty();
  
  public static SK26Turret m_turretInstance;
  public static BangBangLauncher m_BBlauncherInstance;
  public static SK26Launcher m_standardLauncherInstance;
  public static SK26DualLauncher m_dualLauncherInstance;
  public static SK26Lights m_lightsInstance;
  public static SKSwerve m_swerveInstance;
  public static SKVision m_visionInstance;
  public static SK26Climb m_climbInstance;
  public static SK26IntakePivot m_intakePivotInstance;
  public static SK26IntakeRollers m_intakeRollersInstance;
  public static SK26Indexer m_indexerInstance;
  public static SK26Feeder m_feederInstance;
  public static FuelDetection m_fuelDetectionInstance;
  public static StateHandler m_stateHandlerInstance;
  public static ShootingCoordinator m_shootingCoordinatorInstance;


  public static Field2d m_field = new Field2d();

  // The list containing all the command binding classes
  public List<CommandBinder> buttonBinders = new ArrayList<CommandBinder>();
  LoggedDashboardChooser<Command> autoCommandSelector;

  // Creates a chooser for the operator to update on which alliance won the autonomous
  // period, thus determining which alliance will have the active shift first and every shift afterward.
  public LoggedDashboardChooser<Alliance> phaseTimerChooser;
  public Alliance winningAutoAlliance;

  // ── Phase Timer: 2026 REBUILT Alliance Shift Tracking ──────────────────────
  // Teleop is 2:20 (140s). DriverStation.getMatchTime() counts DOWN from 140.0.
  // elapsed = TELEOP_DURATION - getMatchTime() maps it back to an elapsed value.
  // TRANSITION SHIFT: elapsed  0s – 10s  (both HUBs active)
  // SHIFT 1:          elapsed 10s – 35s  (25s, auto-LOSER active first)
  // SHIFT 2:          elapsed 35s – 60s  (25s)
  // SHIFT 3:          elapsed 60s – 85s  (25s)
  // SHIFT 4:          elapsed 85s – 110s (25s)
  // END GAME:         elapsed 110s – 140s (both HUBs active)
  private static final double TRANSITION_SHIFT_END = 10.0;
  private static final double SHIFT_1_END          = 35.0;
  private static final double SHIFT_2_END          = 60.0;
  private static final double SHIFT_3_END          = 85.0;
  private static final double SHIFT_4_END          = 110.0;
  private static final double TELEOP_DURATION      = 140.0;

  // Hex color strings published as the shift label so dashboards can drive text/background color
  private static final String COLOR_RED    = "#FF1111"; // vibrant red  — RED alliance active
  private static final String COLOR_BLUE   = "#1166FF"; // vibrant blue — BLUE alliance active
  private static final String COLOR_WHITE  = "#FFFFFF"; // both HUBs active (TRANSITION / END GAME)

  // NetworkTables publishers for the "Shift Timer" dashboard table
  private StringPublisher shiftLabelPublisher;
  private DoublePublisher shiftTimeRemainingPublisher;

  // Tracks the seconds remaining in the current shift period; written by pollPhaseTimer()
  // and read by the shiftEndingSoon trigger every 20 ms loop cycle.
  private static double shiftSecondsRemaining = Double.MAX_VALUE;

  private static final double SHIFT_WARNING_THRESHOLD = 5.0;
  private static final double SHIFT_NOTICE_THRESHOLD = 10.0;

  /**
   * Active (true) during the last 5 seconds of every shift period — TRANSITION,
   * SHIFT 1-4, and END GAME. Backed by {@link #shiftSecondsRemaining} which is
   * updated by {@link #pollPhaseTimer()} before the command scheduler runs.
   */
  public static final Trigger shiftEndingSoon =
      new Trigger(() -> DriverStation.isTeleop()
                     && shiftSecondsRemaining <= SHIFT_WARNING_THRESHOLD
                     && shiftSecondsRemaining > 0.0);

  public static final Trigger shiftNotice = 
      new Trigger(() -> DriverStation.isTeleop()
                     && shiftSecondsRemaining <= SHIFT_NOTICE_THRESHOLD
                     && shiftSecondsRemaining > SHIFT_NOTICE_THRESHOLD - 1.5);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    // Creates all subsystems that are on the robot
    configureSubsystems();

    // sets up autos needed for pathplanner
    configurePathPlannerCommands();

    // Configure the trigger bindings
    configureButtonBindings();
  
    if(m_swerveContainer.isPresent()) {
        autoCommandSelector = new LoggedDashboardChooser<>("Select an Auto", AutoBuilder.buildAutoChooser());
    }
    //set delete old files = true in build.gradle to prevent sotrage of unused orphans
  
    /* 2026 REBUILT Phase Timing Utility */
    configurePhaseTimer();
}

  /**
     * Will create all the optional subsystems using the json file in the deploy directory
     */
    private void configureSubsystems()
    {
        File deployDirectory = Filesystem.getDeployDirectory();

        ObjectMapper mapper = new ObjectMapper();
        JsonFactory factory = new JsonFactory();

        try
        {
            // Looking for the Subsystems.json file in the deploy directory
            JsonParser parser =
                    factory.createParser(new File(deployDirectory, Konstants.SUBSYSTEMFILE));
            SubsystemControls subsystems = mapper.readValue(parser, SubsystemControls.class);

            // The state handler should always be present
            m_stateHandlerContainer = Optional.of(new StateHandler());
            m_stateHandlerInstance = m_stateHandlerContainer.get();

            if(Robot.isSimulation() && Robot.Mode == RobotMode.CONTROLLED) {
                if(subsystems.isSwervePresent()) {
                    m_swerveContainer = Optional.of(new SKSwerve());
                    m_swerveInstance = m_swerveContainer.get();
                }
                if(subsystems.isTurretPresent()) {
                    m_turretContainer = Optional.of(new SK26TurretSim());
                    m_turretInstance = m_turretContainer.get();
                }
                if(subsystems.isVisionPresent()) {
                    m_visionContainer = Optional.of(new SKVision(m_swerveContainer, m_turretContainer));
                    m_visionInstance = m_visionContainer.get();
                }
                if(subsystems.isIntakePivotPresent()) {
                    m_intakePivotContainer = Optional.of(new SK26IntakePivot());
                    m_intakePivotInstance = m_intakePivotContainer.get();
                }
                if(subsystems.isIntakeRollersPresent()) {
                    m_intakeRollersContainer = Optional.of(new SK26IntakeRollers());
                    m_intakeRollersInstance = m_intakeRollersContainer.get();
                }
                if(subsystems.isBangBangLauncherPresent()) {
                    m_BBLauncherContainer = Optional.of(new BangBangLauncher());
                    m_BBlauncherInstance = m_BBLauncherContainer.get();
                }
                if(subsystems.isLauncherPresent()) {
                    m_StandardLauncherContainer = Optional.of(new SK26Launcher());
                    m_standardLauncherInstance = m_StandardLauncherContainer.get();
                }
                if(subsystems.isDualLauncherPresent()) {
                    m_DualLauncherContainer = Optional.of(new SK26DualLauncher());
                    m_dualLauncherInstance = m_DualLauncherContainer.get();
                }
                if(subsystems.isLightsPresent()) {
                    m_lightsContainer = Optional.of(new SK26Lights());
                    m_lightsInstance = m_lightsContainer.get();
                }
                if(subsystems.isIndexerPresent()) {
                    m_indexerContainer = Optional.of(new SK26Indexer());
                    m_indexerInstance = m_indexerContainer.get();
                }
                if(subsystems.isFeederPresent()) {
                    m_feederContainer = Optional.of(new SK26Feeder());
                    m_feederInstance = m_feederContainer.get();
                }
                if(subsystems.isFuelDetectionPresent()) {
                    m_fuelDetectionContainer = Optional.of(new FuelDetection(VisionConfig.INTAKE_CONFIG, m_swerveContainer));
                    m_fuelDetectionInstance = m_fuelDetectionContainer.get();
                }
            }
            else {
                if(subsystems.isSwervePresent()) {
                    m_swerveContainer = Optional.of(new SKSwerve());
                    m_swerveInstance = m_swerveContainer.get(); // Returns new SKSwerve
                }
                // if(subsystems.isVisionPresent()) {
                //     m_visionContainer = Optional.of(new SKVision(m_swerveContainer));
                //     m_visionInstance = m_visionContainer.get();
                // }
                if(subsystems.isTurretPresent()) {
                    m_turretContainer = Optional.of(new SK26Turret());
                    m_turretInstance = m_turretContainer.get();
                }
                if(subsystems.isVisionPresent()) {
                    m_visionContainer = Optional.of(new SKVision(m_swerveContainer, m_turretContainer));
                    m_visionInstance = m_visionContainer.get();
                }
                if(subsystems.isClimbPresent()) {
                    m_climbComtainer = Optional.of(new SK26Climb());
                    m_climbInstance = m_climbComtainer.get();
                }
                if(subsystems.isBangBangLauncherPresent()) {
                    m_BBLauncherContainer = Optional.of(new BangBangLauncher());
                    m_BBlauncherInstance = m_BBLauncherContainer.get();
                }
                if(subsystems.isLauncherPresent()) {
                    m_StandardLauncherContainer = Optional.of(new SK26Launcher());
                    m_standardLauncherInstance = m_StandardLauncherContainer.get();
                }
                if(subsystems.isDualLauncherPresent()) {
                    m_DualLauncherContainer = Optional.of(new SK26DualLauncher());
                    m_dualLauncherInstance = m_DualLauncherContainer.get();
                }
                if(subsystems.isLightsPresent()) {
                    m_lightsContainer = Optional.of(new SK26Lights());
                    m_lightsInstance = m_lightsContainer.get();
                }
                if(subsystems.isIntakePivotPresent()) {
                    m_intakePivotContainer = Optional.of(new SK26IntakePivot());
                    m_intakePivotInstance = m_intakePivotContainer.get();
                }
                if(subsystems.isIntakeRollersPresent()) {
                    m_intakeRollersContainer = Optional.of(new SK26IntakeRollers());
                    m_intakeRollersInstance = m_intakeRollersContainer.get();
                }
                if(subsystems.isIndexerPresent()) {
                    m_indexerContainer = Optional.of(new SK26Indexer());
                    m_indexerInstance = m_indexerContainer.get();
                }
                if(subsystems.isFeederPresent()) {
                    m_feederContainer = Optional.of(new SK26Feeder());
                    m_feederInstance = m_feederContainer.get();
                }
                if(subsystems.isFuelDetectionPresent()) {
                    m_fuelDetectionContainer = Optional.of(new FuelDetection(VisionConfig.INTAKE_CONFIG, m_swerveContainer));
                    m_fuelDetectionInstance = m_fuelDetectionContainer.get();
                }
            }

            if(subsystems.isDualLauncherPresent() && subsystems.isTurretPresent() && subsystems.isSwervePresent()) 
            {
                // If both launcher and turret are present, create the shooting coordinator
                m_shootingCoordinator = Optional.of(new ShootingCoordinator(m_DualLauncherContainer.get(), m_turretContainer.get(), m_swerveContainer.get()));
                m_shootingCoordinatorInstance = m_shootingCoordinator.get();
            }

            // Give StateHandler a reference to the launcher for state readiness checking
            m_stateHandlerContainer.ifPresent(sh -> sh.setLauncherSubsystem(m_DualLauncherContainer));
            // Give StateHandler a reference to the turret for state readiness checking
            m_stateHandlerContainer.ifPresent(sh -> sh.setTurretSubsystem(m_turretContainer));
            // Give StateHandler a reference to the intake for state readiness checking
            m_stateHandlerContainer.ifPresent(sh -> sh.setIntakeSubsystem(m_intakePivotContainer));
            // Give StateHandler a reference to the drive for zone-based triggers
            m_stateHandlerContainer.ifPresent(sh -> sh.setDriveSubsystem(m_swerveContainer));
        }
        catch (IOException e)
        {
            DriverStation.reportError("Failure to read Subsystem Control File!", e.getStackTrace());
        }
    }

  /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link FilteredJoystick}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings()
    {
        buttonBinders.add(new SK26StateBinder(m_stateHandlerContainer));
        buttonBinders.add(new SKSwerveBinder(m_swerveContainer));
        buttonBinders.add(new SK26ClimbBinder(m_climbComtainer, m_stateHandlerContainer));
        buttonBinders.add(new SK26LauncherBinder(m_StandardLauncherContainer));
        buttonBinders.add(new SK26TurretBinder(m_turretContainer, m_swerveContainer));
        buttonBinders.add(new SKTargetPointsBinder());
        buttonBinders.add(new SK26BBLauncherBinder(m_BBLauncherContainer));
        // buttonBinders.add(new SK26DualLauncherBinder(m_DualLauncherContainer, m_swerveContainer));
        buttonBinders.add(new SKVisionBinder(m_visionContainer, m_swerveContainer));
        buttonBinders.add(new SK26LightsBinder(m_lightsContainer));
        buttonBinders.add(new SK26IntakePivotBinder(m_intakePivotContainer));
        buttonBinders.add(new SK26IntakeRollersBinder(m_intakeRollersContainer, m_swerveContainer));
        buttonBinders.add(new SK26IndexerBinder(m_indexerContainer));
        buttonBinders.add(new SK26ShootingCoordinatorBinder(m_shootingCoordinator));
        buttonBinders.add(new SK26FeederBinder(m_feederContainer));
        buttonBinders.add(new FuelHuntBinder(m_swerveContainer, m_fuelDetectionContainer));
        buttonBinders.add(new SK26GuitarHeroBinder(m_intakePivotContainer, m_intakeRollersContainer));
        // Traversing through all the binding classes to actually bind the buttons
        for (CommandBinder subsystemGroup : buttonBinders)
        {
            subsystemGroup.bindButtons();
        }
    }

    private void configurePathPlannerCommands()
    {
        NamedCommands.registerCommands(PathPlannerCommands.getAvailableCommands());
    }

    private void configurePhaseTimer() {
        phaseTimerChooser = new LoggedDashboardChooser<Alliance>("PhaseTimerChooser");
        phaseTimerChooser.addDefaultOption("Blue", Alliance.Blue);
        phaseTimerChooser.addOption("Red", Alliance.Red);
        phaseTimerChooser.onChange((allianceToWinAuto) -> updatePhaseTimer(allianceToWinAuto));

        // Initialise to the default so winningAutoAlliance is never null at runtime
        winningAutoAlliance = Alliance.Blue;

        // Publish to the "Shift Timer" NetworkTable — visible in Shuffleboard / AdvantageScope
        NetworkTable shiftTable = NetworkTableInstance.getDefault().getTable("Shift Timer");
        shiftLabelPublisher        = shiftTable.getStringTopic("Active Shift").publish();
        shiftTimeRemainingPublisher = shiftTable.getDoubleTopic("Seconds Remaining").publish();

        // Show neutral values while the robot is still disabled
        shiftLabelPublisher.set(COLOR_WHITE);
        shiftTimeRemainingPublisher.set(0.0);

        shiftEndingSoon.onTrue(Commands.runOnce(() -> kDriver.setRumble(RumbleType.kBothRumble, 0.85))
        .alongWith(Commands.runOnce(() -> kOperator.setRumble(RumbleType.kBothRumble, 0.85))));
        shiftEndingSoon.onFalse(Commands.runOnce(() -> kDriver.setRumble(RumbleType.kBothRumble, 0.0))
        .alongWith(Commands.runOnce(() -> kOperator.setRumble(RumbleType.kBothRumble, 0.0))));
    }

    private void updatePhaseTimer(Alliance allianceToWinAuto) {
        winningAutoAlliance = allianceToWinAuto;
    }

    /**
     * Returns the Alliance whose HUB is ACTIVE during SHIFT 1.
     * Per the game manual: the auto-winning alliance has their HUB set to INACTIVE
     * for SHIFT 1, so the AUTO LOSER scores first.
     */
    private Alliance getShift1ActiveAlliance() {
        return (winningAutoAlliance == Alliance.Red) ? Alliance.Blue : Alliance.Red;
    }

    /**
     * Returns the Alliance with the ACTIVE HUB for a given shift number (1-4).
     * Odd-numbered shifts go to the auto-losing alliance; even-numbered shifts
     * go to the auto-winning alliance (they alternate each shift).
     *
     * @param shiftNumber 1 through 4
     * @return the Alliance whose HUB is active during that shift
     */
    private Alliance getActiveAllianceForShift(int shiftNumber) {
        Alliance shift1Active = getShift1ActiveAlliance();
        // Odd shifts → auto-loser active, Even shifts → auto-winner active
        return (shiftNumber % 2 != 0) ? shift1Active
                                       : (shift1Active == Alliance.Red ? Alliance.Blue : Alliance.Red);
    }

    /**
     * Called from {@link Robot#robotPeriodic()} BEFORE the command scheduler runs.
     * Computes the current REBUILT shift period based on elapsed teleop time and
     * publishes the active shift label and countdown to the "Shift Timer" NetworkTable.
     */
    public void pollPhaseTimer() {
        if (!DriverStation.isTeleop()) {
            shiftSecondsRemaining = Double.MAX_VALUE;
            shiftLabelPublisher.set(COLOR_WHITE);
            shiftTimeRemainingPublisher.set(0.0);
            return;
        }

        // getMatchTime() counts DOWN from 140.0 during teleop.
        // Converting to elapsed time makes the shift boundaries easy to reason about.
        // Returns -1.0 if the FMS hasn't provided a time yet; treat that as the very start.
        double matchTime = DriverStation.getMatchTime();
        double elapsed   = (matchTime < 0.0) ? 0.0 : TELEOP_DURATION - matchTime;

        String label;
        double remaining;

        if (elapsed < TRANSITION_SHIFT_END) {
            // TRANSITION SHIFT — both HUBs active, no alliance advantage yet
            label     = COLOR_WHITE;
            remaining = TRANSITION_SHIFT_END - elapsed;

        } else if (elapsed < SHIFT_1_END) {
            Alliance active = getActiveAllianceForShift(1);
            label     = (active == Alliance.Red) ? COLOR_RED : COLOR_BLUE;
            remaining = SHIFT_1_END - elapsed;

        } else if (elapsed < SHIFT_2_END) {
            Alliance active = getActiveAllianceForShift(2);
            label     = (active == Alliance.Red) ? COLOR_RED : COLOR_BLUE;
            remaining = SHIFT_2_END - elapsed;

        } else if (elapsed < SHIFT_3_END) {
            Alliance active = getActiveAllianceForShift(3);
            label     = (active == Alliance.Red) ? COLOR_RED : COLOR_BLUE;
            remaining = SHIFT_3_END - elapsed;

        } else if (elapsed < SHIFT_4_END) {
            Alliance active = getActiveAllianceForShift(4);
            label     = (active == Alliance.Red) ? COLOR_RED : COLOR_BLUE;
            remaining = SHIFT_4_END - elapsed;

        } else if (elapsed < TELEOP_DURATION) {
            // END GAME — both HUBs active again
            label     = COLOR_WHITE;
            remaining = TELEOP_DURATION - elapsed;

        } else {
            label     = COLOR_WHITE;
            remaining = 0.0;
        }

        shiftLabelPublisher.set(label);
        shiftSecondsRemaining = Math.max(0.0, remaining);
        shiftTimeRemainingPublisher.set(shiftSecondsRemaining);
    }    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * <p>
     * This method loads the auto when it is called, however, it is recommended
     * to first load your paths/autos when code starts, then return the
     * pre-loaded auto/path.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return autoCommandSelector.get().withName(autoCommandSelector.getSendableChooser().getSelected());
    }

    public void disabledInit() {
        // Throttle Limelights to reduce CPU load while disabled
        m_visionContainer.ifPresent(vision -> vision.onDisabled());
    }

    public void autonomousInit() {
        if(m_stateHandlerContainer.isPresent()) {
            m_stateHandlerContainer.get().setCurrentState(MacroState.IDLE);
        }
        // Remove Limelight throttling for full performance during auto
        m_visionContainer.ifPresent(vision -> vision.onEnabled());
    }

    public void teleopInit() {
        if(m_stateHandlerContainer.isPresent()) {
            m_stateHandlerContainer.get().setCurrentState(MacroState.IDLE);
        }
        // Remove Limelight throttling for full performance during teleop
        m_visionContainer.ifPresent(vision -> vision.onEnabled());
    }

    public void testPeriodic()
    {
      // Kept this as an example of what should go here.

        // if(m_coral.isPresent())
        // {
        //     m_coral.get().testPeriodic();
        // }
    }
    public void testInit()
    {
    }
}
