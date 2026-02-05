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

/**
 * Button bindings for the SK26Lights subsystem.
 * 
 * <h2>Operator Controller Bindings:</h2>
 * <ul>
 *   <li><b>Start</b> - Rainbow (Party Mode)</li>
 *   <li><b>Back</b> - Game State Aware Mode (automatic)</li>
 *   <li><b>Left Bumper</b> - Solid White</li>
 *   <li><b>Right Bumper</b> - Breathing SKBlue</li>
 *   <li><b>X</b> - LEDs Off</li>
 *   <li><b>Y</b> - Alliance Color (solid)</li>
 *   <li><b>A</b> - Alliance Gradient (scrolling)</li>
 *   <li><b>B</b> - Breathing Alliance</li>
 * </ul>
 * 
 * <h2>Automatic State Triggers:</h2>
 * <ul>
 *   <li>DS Disconnected → Breathing SKBlue</li>
 *   <li>DS Connected + Disabled → SKBlue Gradient</li>
 *   <li>Autonomous → Rainbow</li>
 *   <li>Teleop → Alliance Gradient</li>
 *   <li>Endgame (last 20s) → Solid Alliance Color</li>
 *   <li>Test Mode → Solid Yellow</li>
 * </ul>
 */
public class SK26LightsBinder implements CommandBinder {

    private final Optional<SK26Lights> lightsSubsystem;
    
    // Manual control triggers (operator controller)
    private final Trigger partyModeTrigger;          // Start - Rainbow
    private final Trigger gameStateAwareTrigger;     // Back - Auto mode
    private final Trigger solidWhiteTrigger;         // Left Bumper
    private final Trigger breathingSKBlueTrigger;    // Right Bumper
    private final Trigger offTrigger;                // X - Off
    private final Trigger allianceColorTrigger;      // Y - Alliance solid
    private final Trigger allianceGradientTrigger;   // A - Alliance gradient
    private final Trigger breathingAllianceTrigger;  // B - Alliance breathing
    
    // State triggers for automatic transitions
    private final Trigger dsConnected;
    private final Trigger dsDisconnected;
    private final Trigger robotDisabled;
    private final Trigger autoEnabled;
    private final Trigger teleopEnabled;
    private final Trigger testEnabled;
    private final Trigger endgameTrigger;

    public SK26LightsBinder(Optional<SK26Lights> lightsSubsystem) {
        this.lightsSubsystem = lightsSubsystem;
        
        // Manual control buttons
        this.partyModeTrigger = k_Start.button;
        this.gameStateAwareTrigger = k_LeftTrigger.button;  // Back button
        this.solidWhiteTrigger = k_LeftBumperTrigger.button;
        this.breathingSKBlueTrigger = k_RightBumperTrigger.button;
        this.offTrigger = kXbutton.button;
        this.allianceColorTrigger = kYbutton.button;
        this.allianceGradientTrigger = kAbutton.button;
        this.breathingAllianceTrigger = kBbutton.button;
        
        // State triggers
        this.dsConnected = new Trigger(DriverStation::isDSAttached);
        this.dsDisconnected = new Trigger(() -> !DriverStation.isDSAttached());
        this.robotDisabled = new Trigger(DriverStation::isDisabled);
        this.autoEnabled = new Trigger(DriverStation::isAutonomousEnabled);
        this.teleopEnabled = new Trigger(DriverStation::isTeleopEnabled);
        this.testEnabled = new Trigger(DriverStation::isTestEnabled);
        
        // Endgame trigger: teleop enabled AND match time between 0 and 20 seconds
        this.endgameTrigger = new Trigger(() -> {
            double matchTime = DriverStation.getMatchTime();
            return DriverStation.isTeleopEnabled() && matchTime >= 0 && matchTime <= 20.0;
        });
    }

    @Override
    public void bindButtons() {
        if (lightsSubsystem.isEmpty()) {
            return;
        }
        SK26Lights lights = lightsSubsystem.get();
        
        // ========== MANUAL CONTROL BUTTONS ==========
        // These allow the operator to override the automatic patterns
        
        // Off - X button
        offTrigger.onTrue(lights.setOff().ignoringDisable(true));
        
        // Solid White - Left Bumper
        solidWhiteTrigger.onTrue(lights.setSolidWhite().ignoringDisable(true));
        
        // Party Mode (Rainbow) - Start
        partyModeTrigger.onTrue(lights.setRainbow().ignoringDisable(true));
        
        // Breathing SKBlue - Right Bumper
        breathingSKBlueTrigger.onTrue(lights.setBreathingSKBlue().ignoringDisable(true));
        
        // Game State Aware Mode - Back button
        // This enables the automatic mode that changes based on match state
        gameStateAwareTrigger.onTrue(lights.setGameStateAware().ignoringDisable(true));
        
        // ========== ALLIANCE-AWARE BUTTONS ==========
        
        // Alliance Color (solid) - Y button
        allianceColorTrigger.onTrue(lights.setAllianceColor().ignoringDisable(true));
        
        // Alliance Gradient (scrolling) - A button
        allianceGradientTrigger.onTrue(lights.setAllianceGradient().ignoringDisable(true));
        
        // Breathing Alliance - B button
        breathingAllianceTrigger.onTrue(lights.setBreathingAlliance().ignoringDisable(true));
        
        // ========== AUTOMATIC STATE TRANSITIONS ==========
        // These fire on state changes to provide visual feedback
        // Note: These only trigger when NOT in manual override mode
        
        // DS disconnected → Breathing SKBlue (waiting for connection)
        dsDisconnected.onTrue(lights.setBreathingSKBlue().ignoringDisable(true));
        
        // DS connected AND disabled → SK Gradient (ready/waiting)
        dsConnected.and(robotDisabled).onTrue(lights.setSKBlueGradient().ignoringDisable(true));
        
        // Autonomous → Rainbow (indicates auto mode active)
        autoEnabled.onTrue(lights.setRainbow().ignoringDisable(true));
        
        // Teleop → Alliance Gradient (team colors during match)
        teleopEnabled.onTrue(lights.setAllianceGradient().ignoringDisable(true));
        
        // Endgame (last 20 seconds) → Blink white over alliance color
        endgameTrigger.onTrue(lights.setEndgameAlert().ignoringDisable(true));
        
        // Test mode → Yellow (clearly indicates test mode)
        testEnabled.onTrue(lights.setSolidYellow().ignoringDisable(true));
    }
}