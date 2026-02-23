package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kAbutton;
import static frc.robot.Ports.OperatorPorts.kBbutton;
import static frc.robot.Ports.OperatorPorts.kYbutton;
import static frc.robot.Ports.OperatorPorts.kXbutton;
import static frc.robot.Ports.OperatorPorts.kLeftDpad;
import static frc.robot.Ports.OperatorPorts.kRightDpad;
import static frc.robot.Ports.OperatorPorts.kBackbutton;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
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

        StateHandler.whenCurrentStateReady(MacroState.SHUTTLING)
            .or(StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SHUTTLING))
        .onTrue(
            lights.setMode(LightMode.STROBE_WHITE)
        );
        
        StateHandler.whenCurrentStateReady(MacroState.SCORING)
            .or(StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SCORING))
        .onTrue(
            lights.setMode(LightMode.STROBE_SKBLUE)
        );

        // ==================== BASIC LIGHT CONTROLS (Operator D-pad) ====================
        // These two buttons always work regardless of game mode.
        kLeftDpad.button.onTrue(lights.setMode(LightMode.OFF));           // D-pad Left = Lights Off
        kRightDpad.button.onTrue(lights.setMode(LightMode.SKBLUE_GRADIENT)); // D-pad Right = SK Gradient

        // Serious light effects (Alliance Gradient, Solid colors, etc.) are controlled
        // via the "Lights/Light Effect" dropdown on SmartDashboard.

        // ==================== GAME MODE TOGGLE (Operator Back) ====================
        // Back button toggles game controller mode on/off.
        // When game mode is ON, Operator ABXY become game interaction buttons.
        // When game mode is OFF, Operator ABXY are free for other subsystems (turret, etc).
        kBackbutton.button.onTrue(lights.toggleGameModeCommand());

        // ==================== GAME MODE BUTTONS (Operator ABXY) ====================
        // These buttons ONLY do something when Game Controller Mode is ON.
        // When Game Controller Mode is OFF, they do nothing for lights,
        // leaving them free for turret/launcher bindings.
        //
        // When ON:
        //   - Simon Says: A=Green, B=Red, X=Blue, Y=Yellow
        //   - Color Knockout: A=Green, B=Red, X=Blue, Y=Yellow (P2)
        //   - Other games: A = game action

        // Operator A button (Green / game action)
        kAbutton.button.onTrue(
            Commands.either(
                Commands.either(
                    lights.simonGreenButton(),              // Simon Says: Green
                    Commands.either(
                        lights.knockoutP2GreenButton(),     // Color Knockout: P2 Green
                        lights.gameButtonPressedAlt(),      // Other games: game action
                        () -> lights.getCurrentMode() == LightMode.COLOR_KNOCKOUT
                    ),
                    () -> lights.getCurrentMode() == LightMode.SIMON_SAYS
                ),
                Commands.none(),                            // Game mode OFF: do nothing
                lights::isGameModeEnabled
            ).ignoringDisable(true)
        );

        // Operator B button (Red)
        kBbutton.button.onTrue(
            Commands.either(
                Commands.either(
                    lights.simonRedButton(),                // Simon Says: Red
                    Commands.either(
                        lights.knockoutP2RedButton(),       // Color Knockout: P2 Red
                        Commands.none(),                    // Other games: nothing
                        () -> lights.getCurrentMode() == LightMode.COLOR_KNOCKOUT
                    ),
                    () -> lights.getCurrentMode() == LightMode.SIMON_SAYS
                ),
                Commands.none(),                            // Game mode OFF: do nothing
                lights::isGameModeEnabled
            ).ignoringDisable(true)
        );

        // Operator X button (Blue)
        kXbutton.button.onTrue(
            Commands.either(
                Commands.either(
                    lights.simonBlueButton(),               // Simon Says: Blue
                    Commands.either(
                        lights.knockoutP2BlueButton(),      // Color Knockout: P2 Blue
                        Commands.none(),                    // Other games: nothing
                        () -> lights.getCurrentMode() == LightMode.COLOR_KNOCKOUT
                    ),
                    () -> lights.getCurrentMode() == LightMode.SIMON_SAYS
                ),
                Commands.none(),                            // Game mode OFF: do nothing
                lights::isGameModeEnabled
            ).ignoringDisable(true)
        );

        // Operator Y button (Yellow)
        kYbutton.button.onTrue(
            Commands.either(
                Commands.either(
                    lights.simonYellowButton(),             // Simon Says: Yellow
                    Commands.either(
                        lights.knockoutP2YellowButton(),    // Color Knockout: P2 Yellow
                        Commands.none(),                    // Other games: nothing
                        () -> lights.getCurrentMode() == LightMode.COLOR_KNOCKOUT
                    ),
                    () -> lights.getCurrentMode() == LightMode.SIMON_SAYS
                ),
                Commands.none(),                            // Game mode OFF: do nothing
                lights::isGameModeEnabled
            ).ignoringDisable(true)
        );
    }
}