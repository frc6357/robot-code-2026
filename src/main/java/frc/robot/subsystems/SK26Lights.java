package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Konstants.LightsConstants.kLEDBufferLength;
import static frc.robot.Konstants.LightsConstants.kLightsPWMHeader;
import static frc.robot.Konstants.LightsConstants.kSKBlue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26Lights extends SubsystemBase {

    private static final Distance kLedSpacing = Meters.of(1.0 / kLEDBufferLength);

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    private final LEDPattern m_off = LEDPattern.solid(Color.kBlack);
    private final LEDPattern m_white = LEDPattern.solid(Color.kWhite);
    private final LEDPattern m_solidGreen = LEDPattern.solid(Color.kGreen);
    private final LEDPattern m_solidRed = LEDPattern.solid(Color.kRed);
    private final LEDPattern m_solidBlue = LEDPattern.solid(Color.kBlue);
    private final LEDPattern m_solidYellow = LEDPattern.solid(Color.kYellow);
    private final LEDPattern m_solidOrange = LEDPattern.solid(Color.kOrange);

    private final LEDPattern m_SKBlue = LEDPattern.solid(kSKBlue);

    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    private final LEDPattern m_scrollingRainbow =
        m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);

    private final LEDPattern m_blinkingRed = m_solidRed.blink(Seconds.of(0.25));
    private final LEDPattern m_blinkingYellow = m_solidYellow.blink(Seconds.of(0.5));
    private final LEDPattern m_blinkingGreen = m_solidGreen.blink(Seconds.of(0.25));

    private final LEDPattern m_breathingSKBlue = m_SKBlue.breathe(Seconds.of(2));

    private String ledStatus = "Breathing SKBlue";

    public SK26Lights() {
        m_led = new AddressableLED(kLightsPWMHeader);
        m_buffer = new AddressableLEDBuffer(kLEDBufferLength);
        m_led.setLength(kLEDBufferLength);
        m_led.start();

        SmartDashboard.putData("Lights", this);

        setDefaultCommand(runPattern(m_breathingSKBlue, "Breathing SKBlue").withName("Breathing SKBlue"));
    }

    private Command runPattern(LEDPattern pattern, String status) {
        return runOnce(() -> ledStatus = status).andThen(run(() -> pattern.applyTo(m_buffer)));
    }

    public Command setOff() {
        return runPattern(m_off, "Off").withName("LEDs Off");
    }

    public Command setSolidWhite() {
        return runPattern(m_white, "White").withName("LEDs White");
    }

    public Command setSolidGreen() {
        return runPattern(m_solidGreen, "Green").withName("Solid Green");
    }

    public Command setSolidRed() {
        return runPattern(m_solidRed, "Red").withName("Solid Red");
    }

    public Command setSolidBlue() {
        return runPattern(m_solidBlue, "Blue").withName("Solid Blue");
    }

    public Command setSolidYellow() {
        return runPattern(m_solidYellow, "Yellow").withName("Solid Yellow");
    }

    public Command setSolidOrange() {
        return runPattern(m_solidOrange, "Orange").withName("Solid Orange");
    }

    public Command setRainbow() {
        return runPattern(m_scrollingRainbow, "Rainbow").withName("Rainbow");
    }

    public Command setBlinkingRed() {
        return runPattern(m_blinkingRed, "Blinking Red").withName("Blinking Red");
    }

    public Command setBlinkingYellow() {
        return runPattern(m_blinkingYellow, "Blinking Yellow").withName("Blinking Yellow");
    }

    public Command setBlinkingGreen() {
        return runPattern(m_blinkingGreen, "Blinking Green").withName("Blinking Green");
    }

    public Command setBreathingSKBlue() {
        return runPattern(m_breathingSKBlue, "Breathing SKBlue").withName("Breathing SKBlue");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("LED Status", () -> ledStatus, null);
    }

    @Override
    public void periodic() {
        m_led.setData(m_buffer);
    }
}