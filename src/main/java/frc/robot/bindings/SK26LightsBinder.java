package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kAbutton;
import static frc.robot.Ports.OperatorPorts.kBbutton;
import static frc.robot.Ports.OperatorPorts.kYbutton;
import static frc.robot.Ports.OperatorPorts.kXbutton;
import static frc.robot.Ports.OperatorPorts.k_Start;
import static frc.robot.Ports.OperatorPorts.k_LeftTrigger;
import static frc.robot.Ports.OperatorPorts.k_LeftBumperTrigger;
import static frc.robot.Ports.OperatorPorts.k_RightBumperTrigger;
import static frc.robot.Ports.DriverPorts.kDriverGameButton;
import static frc.robot.Ports.DriverPorts.kDriverBbutton;
import static frc.robot.Ports.DriverPorts.kDriverXbutton;
import static frc.robot.Ports.DriverPorts.kDriverYbutton;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.lights.LightMode;
import frc.robot.subsystems.lights.SK26Lights;

public class SK26LightsBinder implements CommandBinder {

    private final Optional<SK26Lights> lightsSubsystem;

    public SK26LightsBinder(Optional<SK26Lights> lightsSubsystem) {
        this.lightsSubsystem = lightsSubsystem;
    }

    @Override
    public void bindButtons() {
        if (lightsSubsystem.isEmpty()) {
            return;
        }
        SK26Lights lights = lightsSubsystem.get();
        
        // Non-game mode light controls (bumpers, triggers, start)
        k_LeftBumperTrigger.button.onTrue(lights.setSolidWhite());
        k_RightBumperTrigger.button.onTrue(lights.setBreathingSKBlue());
        k_Start.button.onTrue(lights.activatePartyMode());
        k_LeftTrigger.button.onTrue(lights.setSKBlueGradient());
        
        // ==================== GAME MODE BUTTONS ====================
        // When Game Controller Mode is ON:
        //   - For Simon Says: A=Green, B=Red, X=Blue, Y=Yellow (both controllers)
        //   - For Color Knockout: Driver=P1, Operator=P2 (A=Green, B=Red, X=Blue, Y=Yellow)
        //   - For other games: A button interacts (Stop Light, Rhythm, Tug of War)
        // When Game Controller Mode is OFF:
        //   - X=Off, B=Blue, Y=Alliance Gradient, A=Red
        
        // Driver A button (Green - P1 in Knockout, Simon, or game action)
        kDriverGameButton.button.onTrue(
            Commands.either(
                Commands.either(
                    lights.simonGreenButton(),       // Simon Says: Green
                    Commands.either(
                        lights.knockoutP1GreenButton(),  // Color Knockout: P1 Green
                        lights.gameButtonPressed(),      // Other games: game action
                        () -> lights.getCurrentMode() == LightMode.COLOR_KNOCKOUT
                    ),
                    () -> lights.getCurrentMode() == LightMode.SIMON_SAYS
                ),
                lights.setSolidRed(),                // Game mode OFF: Red
                lights::isGameModeEnabled
            ).ignoringDisable(true)
        );
        
        // Driver B button (Red - P1 in Knockout/Simon)
        kDriverBbutton.button.onTrue(
            Commands.either(
                Commands.either(
                    lights.simonRedButton(),         // Simon Says: Red
                    Commands.either(
                        lights.knockoutP1RedButton(),    // Color Knockout: P1 Red
                        Commands.none(),                 // Other games: nothing
                        () -> lights.getCurrentMode() == LightMode.COLOR_KNOCKOUT
                    ),
                    () -> lights.getCurrentMode() == LightMode.SIMON_SAYS
                ),
                Commands.none(),                     // Game mode OFF: do nothing (used by vision)
                lights::isGameModeEnabled
            ).ignoringDisable(true)
        );
        
        // Driver X button (Blue - P1 in Knockout/Simon)
        kDriverXbutton.button.onTrue(
            Commands.either(
                Commands.either(
                    lights.simonBlueButton(),        // Simon Says: Blue
                    Commands.either(
                        lights.knockoutP1BlueButton(),   // Color Knockout: P1 Blue
                        Commands.none(),                 // Other games: nothing
                        () -> lights.getCurrentMode() == LightMode.COLOR_KNOCKOUT
                    ),
                    () -> lights.getCurrentMode() == LightMode.SIMON_SAYS
                ),
                Commands.none(),                     // Game mode OFF: do nothing
                lights::isGameModeEnabled
            ).ignoringDisable(true)
        );
        
        // Driver Y button (Yellow - P1 in Knockout/Simon)
        kDriverYbutton.button.onTrue(
            Commands.either(
                Commands.either(
                    lights.simonYellowButton(),      // Simon Says: Yellow
                    Commands.either(
                        lights.knockoutP1YellowButton(), // Color Knockout: P1 Yellow
                        Commands.none(),                 // Other games: nothing
                        () -> lights.getCurrentMode() == LightMode.COLOR_KNOCKOUT
                    ),
                    () -> lights.getCurrentMode() == LightMode.SIMON_SAYS
                ),
                Commands.none(),                     // Game mode OFF: do nothing (used by vision)
                lights::isGameModeEnabled
            ).ignoringDisable(true)
        );
        
        // Operator A button (Green - P2 in Knockout, Simon, or game action)
        kAbutton.button.onTrue(
            Commands.either(
                Commands.either(
                    lights.simonGreenButton(),       // Simon Says: Green
                    Commands.either(
                        lights.knockoutP2GreenButton(),  // Color Knockout: P2 Green
                        lights.gameButtonPressedAlt(),   // Other games: game action (pulls right in Tug)
                        () -> lights.getCurrentMode() == LightMode.COLOR_KNOCKOUT
                    ),
                    () -> lights.getCurrentMode() == LightMode.SIMON_SAYS
                ),
                lights.setSolidRed(),                // Game mode OFF: Red
                lights::isGameModeEnabled
            ).ignoringDisable(true)
        );
        
        // Operator B button (Red - P2 in Knockout/Simon, Blue light when off)
        kBbutton.button.onTrue(
            Commands.either(
                Commands.either(
                    lights.simonRedButton(),         // Simon Says: Red
                    Commands.either(
                        lights.knockoutP2RedButton(),    // Color Knockout: P2 Red
                        Commands.none(),                 // Other games: nothing
                        () -> lights.getCurrentMode() == LightMode.COLOR_KNOCKOUT
                    ),
                    () -> lights.getCurrentMode() == LightMode.SIMON_SAYS
                ),
                lights.setSolidBlue(),               // Game mode OFF: Blue light
                lights::isGameModeEnabled
            ).ignoringDisable(true)
        );
        
        // Operator X button (Blue - P2 in Knockout/Simon, Off when off)
        kXbutton.button.onTrue(
            Commands.either(
                Commands.either(
                    lights.simonBlueButton(),        // Simon Says: Blue
                    Commands.either(
                        lights.knockoutP2BlueButton(),   // Color Knockout: P2 Blue
                        Commands.none(),                 // Other games: nothing
                        () -> lights.getCurrentMode() == LightMode.COLOR_KNOCKOUT
                    ),
                    () -> lights.getCurrentMode() == LightMode.SIMON_SAYS
                ),
                lights.setOff(),                     // Game mode OFF: Lights off
                lights::isGameModeEnabled
            ).ignoringDisable(true)
        );
        
        // Operator Y button (Yellow - P2 in Knockout/Simon, Alliance Gradient when off)
        kYbutton.button.onTrue(
            Commands.either(
                Commands.either(
                    lights.simonYellowButton(),      // Simon Says: Yellow
                    Commands.either(
                        lights.knockoutP2YellowButton(), // Color Knockout: P2 Yellow
                        Commands.none(),                 // Other games: nothing
                        () -> lights.getCurrentMode() == LightMode.COLOR_KNOCKOUT
                    ),
                    () -> lights.getCurrentMode() == LightMode.SIMON_SAYS
                ),
                lights.setAllianceGradient(),        // Game mode OFF: Alliance Gradient
                lights::isGameModeEnabled
            ).ignoringDisable(true)
        );
    }
}