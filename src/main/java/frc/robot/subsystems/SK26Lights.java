package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.SubsystemControls;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Konstants.LightsConstants.kLightsPWMHeader;
import static frc.robot.Konstants.LightsConstants.kLEDBufferLength;

import static frc.robot.Konstants.LightsConstants.kSKBlue;

public class SK26Lights extends SubsystemBase {

    // LED strip density (LEDs per meter)
    private static final Distance kLedSpacing = Meters.of(1 / kLEDBufferLength);

    // Hardware objects
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    // Pre-defined patterns
    private final LEDPattern m_off = LEDPattern.solid(Color.kBlack);
    private final LEDPattern m_white = LEDPattern.solid(Color.kWhite);
    private final LEDPattern m_solidGreen = LEDPattern.solid(Color.kGreen);
    private final LEDPattern m_solidRed = LEDPattern.solid(Color.kRed);
    private final LEDPattern m_solidBlue = LEDPattern.solid(Color.kBlue);
    private final LEDPattern m_solidYellow = LEDPattern.solid(Color.kYellow);
    private final LEDPattern m_solidOrange = LEDPattern.solid(Color.kOrange);

    private final LEDPattern m_SKBlue = LEDPattern.solid(kSKBlue);
    
    // Rainbow pattern
    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    private final LEDPattern m_scrollingRainbow = 
        m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);
    

    // Blinking patterns for alerts
    private final LEDPattern m_blinkingRed = m_solidRed.blink(Seconds.of(0.25));
    private final LEDPattern m_blinkingYellow = m_solidYellow.blink(Seconds.of(0.5));
    private final LEDPattern m_blinkingGreen = m_solidGreen.blink(Seconds.of(0.25));

    // Breathing patterns
    private final LEDPattern m_breathingSKBlue = m_SKBlue.breathe(Seconds.of(2));

    public String LEDStatus = "Off";

    public SK26Lights() {
        m_led = new AddressableLED(kLightsPWMHeader);
        m_buffer = new AddressableLEDBuffer(kLEDBufferLength);
        m_led.setLength(kLEDBufferLength);
        m_led.start();

        // Set default command to turn LEDs off
        setDefaultCommand(runPattern(m_off).withName("LEDs Off"));
    }

    /**
     * Creates a command that runs a pattern on the LED strip.
     *
     * @param pattern the LED pattern to run
     * @return a command that applies the pattern
     */
    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(m_buffer));
    }

    // ==================== Pattern Commands ====================

    /** Command to turn LEDs off */
    public Command setOff() {
        LEDStatus = "Off";
        return runPattern(m_off).withName("LEDs Off");
    }

    public Command setSolidWhite() {
        LEDStatus = "White";
        return runPattern(m_white).withName("LEDs White");
    }

    /** Command to set LEDs to solid green (e.g., ready/aligned) */
    public Command setSolidGreen() {
        LEDStatus = "Green";
        return runPattern(m_solidGreen).withName("Solid Green");
    }

    /** Command to set LEDs to solid red (e.g., error/not ready) */
    public Command setSolidRed() {
        LEDStatus = "Red";
        return runPattern(m_solidRed).withName("Solid Red");
    }

    /** Command to set LEDs to solid blue */
    public Command setSolidBlue() {
        LEDStatus = "Blue";
        return runPattern(m_solidBlue).withName("Solid Blue");
    }

    /** Command to set LEDs to solid yellow (e.g., warning) */
    public Command setSolidYellow() {
        LEDStatus = "Yellow";
        return runPattern(m_solidYellow).withName("Solid Yellow");
    }

    /** Command to set LEDs to solid orange (e.g., game piece detected) */
    public Command setSolidOrange() {
        LEDStatus = "Orange";
        return runPattern(m_solidOrange).withName("Solid Orange");
    }

    /** Command to display scrolling rainbow */
    public Command setRainbow() {
        LEDStatus = "Rainbow";
        return runPattern(m_scrollingRainbow).withName("Rainbow");
    }

    /** Command to display blinking red (e.g., critical alert) */
    public Command setBlinkingRed() {
        LEDStatus = "Blinking Red";
        return runPattern(m_blinkingRed).withName("Blinking Red");
    }

    /** Command to display blinking yellow (e.g., warning) */
    public Command setBlinkingYellow() {
        LEDStatus = "Blinking Yellow";
        return runPattern(m_blinkingYellow).withName("Blinking Yellow");
    }

    /** Command to display blinking green (e.g., action complete) */
    public Command setBlinkingGreen() {
        LEDStatus = "Blinking Green";
        return runPattern(m_blinkingGreen).withName("Blinking Green");
    }

    /** Command to display breathing blue (e.g., idle/standby) */
    public Command setBreathingSKBlue() {
        LEDStatus = "Breathing SKBlue";
        return runPattern(m_breathingSKBlue).withName("Breathing Blue");
    }

    @Override 
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty(
            "LED Status",
            () -> LEDStatus,
            null);
    }

    @Override
    public void periodic() {
        m_led.setData(m_buffer);
        SmartDashboard.putData("Lights", this);
    }

}
