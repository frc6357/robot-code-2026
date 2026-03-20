package frc.robot.subsystems.lights;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Konstants.LightsConstants.kDefaultStrobeSeconds;
import static frc.robot.Konstants.LightsConstants.kLEDBufferLength;
import static frc.robot.Konstants.LightsConstants.kSKBlue;
import static frc.robot.Konstants.LightsConstants.kSKCream;
import static frc.robot.Konstants.LightsConstants.kSKTeal;
import static frc.robot.Konstants.LightsConstants.kSKDarkBlue;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Contains all the WPILib LED patterns used by the lights subsystem.
 */
public class LightPatterns {
    
    private static final Distance kLedSpacing = Meters.of(1.0 / kLEDBufferLength);
    
    // Solid colors
    public final LEDPattern off = LEDPattern.solid(Color.kBlack);
    public final LEDPattern white = LEDPattern.solid(Color.kWhite);
    public final LEDPattern green = LEDPattern.solid(Color.kGreen);
    public final LEDPattern red = LEDPattern.solid(Color.kRed);
    public final LEDPattern blue = LEDPattern.solid(Color.kBlue);
    public final LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    public final LEDPattern orange = LEDPattern.solid(Color.kOrange);
    public final LEDPattern purple = LEDPattern.solid(Color.kPurple);
    public final LEDPattern skBlue = LEDPattern.solid(kSKBlue);
    
    // Rainbow patterns
    public final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    public final LEDPattern scrollingRainbow = 
        rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);
    
    // Strobing effects
    public final LEDPattern whiteStrobe = white.blink(kDefaultStrobeSeconds);
    public final LEDPattern greenStrobe = green.blink(kDefaultStrobeSeconds);
    public final LEDPattern redStrobe = red.blink(kDefaultStrobeSeconds);
    public final LEDPattern blueStrobe = blue.blink(kDefaultStrobeSeconds);
    public final LEDPattern yellowStrobe = yellow.blink(kDefaultStrobeSeconds);
    public final LEDPattern orangeStrobe = orange.blink(kDefaultStrobeSeconds);
    public final LEDPattern purpleStrobe = purple.blink(kDefaultStrobeSeconds);
    public final LEDPattern skBlueStrobe = skBlue.blink(kDefaultStrobeSeconds);

    // Dual color patterns
    //   "Dual solid" = half-and-half using LEDPattern.steps() at the 50% mark
    //   "Dual strobe" = blink one color, with the other as the fallback via overlayOn()
    public final LEDPattern dualSolidWhiteGreen =
        LEDPattern.steps(Map.of(0.0, Color.kWhite, 0.5, Color.kGreen));
    public final LEDPattern dualSolidWhiteYellow =
        LEDPattern.steps(Map.of(0.0, Color.kWhite, 0.5, Color.kYellow));
    public final LEDPattern dualStrobeWhiteGreen =
        LEDPattern.solid(Color.kWhite).blink(kDefaultStrobeSeconds)
            .overlayOn(LEDPattern.solid(Color.kGreen));
    public final LEDPattern dualStrobeWhiteYellow =
        LEDPattern.solid(Color.kWhite).blink(kDefaultStrobeSeconds)
            .overlayOn(LEDPattern.solid(Color.kYellow));
    
    // SK Blue gradient
    public final LEDPattern skBlueGradient = LEDPattern.gradient(
        LEDPattern.GradientType.kContinuous,
        kSKCream,
        kSKTeal,
        kSKBlue,
        kSKDarkBlue
    ).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);
    
    // Alliance gradients
    public final LEDPattern redGradient = LEDPattern.gradient(
        LEDPattern.GradientType.kContinuous,
        Color.kRed,
        Color.kDarkRed,
        Color.kOrangeRed,
        Color.kRed
    ).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);
    
    public final LEDPattern blueGradient = LEDPattern.gradient(
        LEDPattern.GradientType.kContinuous,
        Color.kBlue,
        Color.kDarkBlue,
        Color.kRoyalBlue,
        Color.kBlue
    ).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);
}
