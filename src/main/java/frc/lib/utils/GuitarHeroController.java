package frc.lib.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Wrapper for a Guitar Hero controller connected via USB adapter.
 * Provides named Trigger accessors for each fret button and the strum bar.
 *
 * <p>Typical USB-HID mappings for a Guitar Hero controller:
 * <ul>
 *   <li>Green fret  — Button 2</li>
 *   <li>Red fret    — Button 1</li>
 *   <li>Yellow fret — Button 4</li>
 *   <li>Blue fret   — Button 3</li>
 *   <li>Orange fret — Button 5</li>
 *   <li>Strum Up    — POV 0°</li>
 *   <li>Strum Down  — POV 180°</li>
 * </ul>
 */
public class GuitarHeroController {

    private final GenericHID hid;

    // Fret buttons
    private final Trigger greenFret;
    private final Trigger redFret;
    private final Trigger yellowFret;
    private final Trigger blueFret;
    private final Trigger orangeFret;

    // Strum bar (POV hat)
    private final Trigger strumUp;
    private final Trigger strumDown;

    /**
     * Creates a GuitarHeroController on the given HID port.
     * @param port The USB port index (e.g. 2 for the third controller)
     */
    public GuitarHeroController(int port) {
        hid = new GenericHID(port);

        greenFret  = new JoystickButton(hid, 2);
        redFret    = new JoystickButton(hid, 1);
        yellowFret = new JoystickButton(hid, 4);
        blueFret   = new JoystickButton(hid, 3);
        orangeFret = new JoystickButton(hid, 5);

        // Strum bar maps to POV/D-pad: up = 0°, down = 180°
        // Use direct POV value check for reliability
        strumUp   = new Trigger(() -> hid.getPOV() == 0);
        strumDown = new Trigger(() -> hid.getPOV() == 180);
    }

    /** Green fret button trigger. */
    public Trigger greenFret()  { return greenFret; }

    /** Red fret button trigger. */
    public Trigger redFret()    { return redFret; }

    /** Yellow fret button trigger. */
    public Trigger yellowFret() { return yellowFret; }

    /** Blue fret button trigger. */
    public Trigger blueFret()   { return blueFret; }

    /** Orange fret button trigger. */
    public Trigger orangeFret() { return orangeFret; }

    /** Strum bar pushed upward trigger. */
    public Trigger strumUp()    { return strumUp; }

    /** Strum bar pushed downward trigger. */
    public Trigger strumDown()  { return strumDown; }

    /** Returns the underlying GenericHID. */
    public GenericHID getHID()  { return hid; }
}
