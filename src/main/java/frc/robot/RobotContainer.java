// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
//import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.utils.SubsystemControls;
import frc.lib.utils.filters.FilteredJoystick;
import frc.robot.bindings.CommandBinder;
import frc.robot.bindings.SK26BBLauncherBinder;
import frc.robot.bindings.SK26TurretBinder;
import frc.robot.bindings.SK26LauncherBinder;
import frc.robot.bindings.SK26StateBinder;
import frc.robot.bindings.SK26IndexerBinder;
import frc.robot.bindings.SKSwerveBinder;
import frc.robot.bindings.SKTargetPointsBinder;
import frc.robot.bindings.SKVisionBinder;
import frc.robot.bindings.SK26LightsBinder;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.indexer.SK26Indexer;
import frc.robot.subsystems.launcher.BangBangLauncher;
import frc.robot.subsystems.launcher.SK26Launcher;
import frc.robot.bindings.PickupBinder;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.pickupOB.SK26PickupOB;
import frc.robot.subsystems.turret.SK26Turret;
import frc.robot.subsystems.vision.SKVision;
import frc.robot.subsystems.lights.SK26Lights;


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
  public Optional<SK26Turret> m_turretContainer = Optional.empty();
  public Optional<BangBangLauncher> m_BBLauncherContainer = Optional.empty();
  public Optional<SK26Launcher> m_StandardLauncherContainer = Optional.empty();
  public Optional<StateHandler> m_stateHandlerContainer = Optional.empty();
  public Optional<SK26Lights> m_lightsContainer = Optional.empty();
  public Optional<SK26PickupOB> m_pickupContainer = Optional.empty();
  public Optional<SK26Indexer> m_indexerContainer = Optional.empty();

  
  public static SK26Turret m_turretInstance;
  public static BangBangLauncher m_BBlauncherInstance;
  public static SK26Launcher m_standardLauncherInstance;
  public static SK26Lights m_lightsInstance;
  public static SKSwerve m_swerveInstance;
  public static SKVision m_visionInstance;
  public static SK26PickupOB m_pickupInstance;
  public static SK26Indexer m_indexerInstance;

  public static Field2d m_field = new Field2d();

  // The list containing all the command binding classes
  public List<CommandBinder> buttonBinders = new ArrayList<CommandBinder>();
  SendableChooser<Command> autoCommandSelector;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    // Creates all subsystems that are on the robot
    configureSubsystems();

    // sets up autos needed for pathplanner
    configurePathPlannerCommands();

    // Configure the trigger bindings
    configureButtonBindings();
  
    autoCommandSelector = AutoBuilder.buildAutoChooser("TrenchTaxi");
    //set delete old files = true in build.gradle to prevent sotrage of unused orphans
    SmartDashboard.putData("Select an Auto", autoCommandSelector);
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

            if(subsystems.isSwervePresent()) {
                m_swerveContainer = Optional.of(new SKSwerve());
                m_swerveInstance = m_swerveContainer.get(); // Returns new SKSwerve
            }
            if(subsystems.isVisionPresent()) {
                m_visionContainer = Optional.of(new SKVision(m_swerveContainer));
                m_visionInstance = m_visionContainer.get();
            }
            if(subsystems.isTurretPresent()) {
                m_turretContainer = Optional.of(new SK26Turret());
                m_turretInstance = m_turretContainer.get();
            }
            if(subsystems.isBangBangLauncherPresent()) {
                m_BBLauncherContainer = Optional.of(new BangBangLauncher());
                m_BBlauncherInstance = m_BBLauncherContainer.get();
            }
            if(subsystems.isLauncherPresent()) {
                m_StandardLauncherContainer = Optional.of(new SK26Launcher());
                m_standardLauncherInstance = m_StandardLauncherContainer.get();
            }
            if(subsystems.isLightsPresent()) {
                m_lightsContainer = Optional.of(new SK26Lights());
                m_lightsInstance = m_lightsContainer.get();
            }
            if(subsystems.isPickupPresent()) {
                m_pickupContainer = Optional.of(new SK26PickupOB());
                m_pickupInstance = m_pickupContainer.get();
            }
            if(subsystems.isIndexerPresent()) {
                m_indexerContainer = Optional.of(new SK26Indexer());
                m_indexerInstance = m_indexerContainer.get(); // Returns new SKSwerve
            }
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
        buttonBinders.add(new SK26LauncherBinder(m_StandardLauncherContainer));
        buttonBinders.add(new SK26TurretBinder(m_turretContainer, m_swerveContainer));
        buttonBinders.add(new SKTargetPointsBinder());
        buttonBinders.add(new SK26BBLauncherBinder(m_BBLauncherContainer));
        buttonBinders.add(new SKVisionBinder(m_visionContainer, m_swerveContainer));
        buttonBinders.add(new SK26LightsBinder(m_lightsContainer));
        buttonBinders.add(new PickupBinder(m_pickupContainer));
        buttonBinders.add(new SK26IndexerBinder(m_indexerContainer));
        // Traversing through all the binding classes to actually bind the buttons
        for (CommandBinder subsystemGroup : buttonBinders)
        {
            subsystemGroup.bindButtons();
        }

    }

    public void configurePathPlannerCommands()
    {
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
        return autoCommandSelector.getSelected();
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

    public void matchInit()
    {
    }

    public void teleopPeriodic()
    {
        
    }

    public void teleopInit(){
    }

    public void autonomousInit()
    {
    }

}
