package frc.lib.utils;

import static frc.lib.utils.SKTrigger.INPUT_TYPE.AXIS;
import static frc.lib.utils.SKTrigger.INPUT_TYPE.BUTTON;
import static frc.lib.utils.SKTrigger.INPUT_TYPE.POV;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftTrigger;
import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftX;
import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftY;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightTrigger;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightX;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightY;
import static edu.wpi.first.wpilibj.XboxController.Button.kA;
import static edu.wpi.first.wpilibj.XboxController.Button.kB;
import static edu.wpi.first.wpilibj.XboxController.Button.kBack;
import static edu.wpi.first.wpilibj.XboxController.Button.kLeftBumper;
import static edu.wpi.first.wpilibj.XboxController.Button.kLeftStick;
import static edu.wpi.first.wpilibj.XboxController.Button.kRightBumper;
import static edu.wpi.first.wpilibj.XboxController.Button.kRightStick;
import static edu.wpi.first.wpilibj.XboxController.Button.kStart;
import static edu.wpi.first.wpilibj.XboxController.Button.kX;
import static edu.wpi.first.wpilibj.XboxController.Button.kY;

import edu.wpi.first.wpilibj.GenericHID;
import frc.lib.utils.filters.FilteredAxis;

/**
 * A Guitar Hero controller that maps identically to an Xbox controller.
 * Provides the same full set of inputs (ABXY / frets, D-pad / strum bar,
 * triggers, bumpers, sticks) as the Driver and Operator port definitions.
 *
 * <p>Guitar Hero fret-to-Xbox mapping:
 * <ul>
 *   <li>Green fret  → A button</li>
 *   <li>Red fret    → B button</li>
 *   <li>Yellow fret → Y button</li>
 *   <li>Blue fret   → X button</li>
 *   <li>Orange fret → Left Bumper</li>
 *   <li>Strum Up    → D-pad Up (POV 0°)</li>
 *   <li>Strum Down  → D-pad Down (POV 180°)</li>
 * </ul>
 */
public class GuitarHeroController {

    // Underlying HID
    public final GenericHID hid;

    // Filtered axes
    public final FilteredAxis kLeftStickY;
    public final FilteredAxis kLeftStickX;
    public final FilteredAxis kRightStickX;
    public final FilteredAxis kRightStickY;

    // ABXY (frets)
    public final SKTrigger kAbutton;
    public final SKTrigger kBbutton;
    public final SKTrigger kXbutton;
    public final SKTrigger kYbutton;

    // D-pad (strum bar + sides)
    public final SKTrigger kUpDpad;
    public final SKTrigger kRightDpad;
    public final SKTrigger kDownDpad;
    public final SKTrigger kLeftDpad;

    // Bumpers
    public final SKTrigger kRBbutton;
    public final SKTrigger kLBbutton;

    // Triggers
    public final SKTrigger kLTrigger;
    public final SKTrigger kRTrigger;

    // Menu buttons
    public final SKTrigger kStartbutton;
    public final SKTrigger kBackbutton;

    // Stick buttons
    public final SKTrigger kLSbutton;
    public final SKTrigger kRSbutton;

    /**
     * Creates a GuitarHeroController on the given USB port.
     * All inputs are mapped identically to an Xbox controller.
     *
     * @param port The USB port index (e.g. 2 for the third controller)
     */
    public GuitarHeroController(int port) {
        hid = new GenericHID(port);

        // Axes
        kLeftStickY  = new FilteredAxis(() -> hid.getRawAxis(kLeftY.value));
        kLeftStickX  = new FilteredAxis(() -> hid.getRawAxis(kLeftX.value));
        kRightStickX = new FilteredAxis(() -> hid.getRawAxis(kRightX.value));
        kRightStickY = new FilteredAxis(() -> hid.getRawAxis(kRightY.value));

        // ABXY
        kAbutton = new SKTrigger(hid, kA.value, BUTTON);
        kBbutton = new SKTrigger(hid, kB.value, BUTTON);
        kXbutton = new SKTrigger(hid, kX.value, BUTTON);
        kYbutton = new SKTrigger(hid, kY.value, BUTTON);

        // D-pad
        kUpDpad    = new SKTrigger(hid, 0, POV);
        kRightDpad = new SKTrigger(hid, 90, POV);
        kDownDpad  = new SKTrigger(hid, 180, POV);
        kLeftDpad  = new SKTrigger(hid, 270, POV);

        // Bumpers
        kRBbutton = new SKTrigger(hid, kRightBumper.value, BUTTON);
        kLBbutton = new SKTrigger(hid, kLeftBumper.value, BUTTON);

        // Triggers
        kLTrigger = new SKTrigger(hid, kLeftTrigger.value, AXIS);
        kRTrigger = new SKTrigger(hid, kRightTrigger.value, AXIS);

        // Menu
        kStartbutton = new SKTrigger(hid, kStart.value, BUTTON);
        kBackbutton  = new SKTrigger(hid, kBack.value, BUTTON);

        // Stick buttons
        kLSbutton = new SKTrigger(hid, kLeftStick.value, BUTTON);
        kRSbutton = new SKTrigger(hid, kRightStick.value, BUTTON);
    }
}
