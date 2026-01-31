package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kAbutton;
import static frc.robot.Ports.OperatorPorts.kBbutton;
import static frc.robot.Ports.OperatorPorts.kYbutton;
import static frc.robot.Ports.OperatorPorts.kXbutton;
import static frc.robot.Ports.OperatorPorts.k_Start;
import static frc.robot.Ports.OperatorPorts.k_RightBumperTrigger;
import static frc.robot.Ports.OperatorPorts.k_LeftBumperTrigger;
import static frc.robot.Ports.OperatorPorts.k_LeftTrigger;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SK26Lights;

public class SK26LightsBinder implements CommandBinder {

    private final Optional<SK26Lights> lightsSubsystem;
    private final Trigger PartyMode;
    private final Trigger breathingBlueTrigger;
    private final Trigger leftBumperTrigger;
    private final Trigger offTrigger;
    private final Trigger SKGradientWaveTrigger;
    
    // State triggers
    private final Trigger dsConnected;
    private final Trigger dsDisconnected;
    private final Trigger robotDisabled;
    private final Trigger robotEnabled;

    public SK26LightsBinder(Optional<SK26Lights> lightsSubsystem) {
        this.lightsSubsystem = lightsSubsystem;
        this.PartyMode = k_Start.button;
        this.breathingBlueTrigger = k_RightBumperTrigger.button;
        this.leftBumperTrigger = k_LeftBumperTrigger.button;
        this.offTrigger = kXbutton.button;
        this.SKGradientWaveTrigger = k_LeftTrigger.button;
        
        // State triggers
        this.dsConnected = new Trigger(DriverStation::isDSAttached);
        this.dsDisconnected = new Trigger(() -> !DriverStation.isDSAttached());
        this.robotDisabled = new Trigger(DriverStation::isDisabled);
        this.robotEnabled = new Trigger(DriverStation::isEnabled);
    }

    @Override
    public void bindButtons() {
        if (lightsSubsystem.isEmpty()) {
            return;
        }
        SK26Lights lights = lightsSubsystem.get();
        
        // Button controls
        offTrigger.onTrue(lights.setOff().ignoringDisable(true));
        leftBumperTrigger.onTrue(lights.setSolidWhite().ignoringDisable(true));
        PartyMode.onTrue(lights.setRainbow().ignoringDisable(true));
        SKGradientWaveTrigger.onTrue(lights.setSKBlueGradient().ignoringDisable(true));

        kAbutton.button.onTrue(lights.setSolidRed().ignoringDisable(true));
        kBbutton.button.onTrue(lights.setSolidBlue().ignoringDisable(true));
        kYbutton.button.onTrue(lights.setSolidGreen().ignoringDisable(true));

        // State-based LED changes (fires on transition)
        // DS disconnected → Breathing Blue
        dsDisconnected.onTrue(lights.setBreathingSKBlue().ignoringDisable(true));
        
        // DS connected AND disabled → SK Gradient
        dsConnected.and(robotDisabled).onTrue(lights.setSKBlueGradient().ignoringDisable(true));
        
        // Robot enabled → Breathing Blue
        robotEnabled.onTrue(lights.setBreathingSKBlue().ignoringDisable(true));
    }
}