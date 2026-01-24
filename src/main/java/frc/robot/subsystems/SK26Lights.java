package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
//import com.ctre.phoenix.led.CANdle.LEDStripType;
//import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
//import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.StrobeAnimation;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Konstants.LightsConstants.kNumLedOnBot;
import static frc.robot.Konstants.LightsConstants.kSKBlue1;
import static frc.robot.Konstants.LightsConstants.kSKBlue2;
import static frc.robot.Konstants.LightsConstants.kSKBlue3;
import static frc.robot.Konstants.LightsConstants.kSKBlue4;
import static frc.robot.Konstants.LightsConstants.kWaveSpeedCyclesPerSecond;
import static frc.robot.Konstants.LightsConstants.waveColorCycleSec;
import static frc.robot.Konstants.LightsConstants.waveSpatialCycles;
import static frc.robot.Ports.LightsPorts.kCANdle;
import static frc.robot.Konstants.LightsConstants.kColorRed;
import static frc.robot.Konstants.LightsConstants.kLightsOffBrightness;
import static frc.robot.Konstants.LightsConstants.kLightsOnBrightness;
import static frc.robot.Konstants.LightsConstants.kColorBlue;
import static frc.robot.Konstants.LightsConstants.kColorWhite;
import static frc.robot.Ports.LightsPorts.canBus;

public class SK26Lights extends SubsystemBase {

    /**
     * Enum representing the different light effects with their priorities.
     * Lower priority values are MORE important and will override higher values.
     */
    public enum LightEffect {
        OFF(100),           // Lowest priority - default/idle state
        SK_BLUE_WAVE(80),   // Idle animation
        SOLID_BLUE(70),     // Alliance color
        SOLID_RED(70),      // Alliance color
        SOLID_WHITE(60),    // Auto mode indicator
        RAINBOW(50),        // Celebration/special
        STROBE_RED(20),     // Defense mode - high priority
        STROBE_BLUE(20);    // Defense mode - high priority

        private final int priority;

        LightEffect(int priority) {
            this.priority = priority;
        }

        public int getPriority() {
            return priority;
        }
    }
    /*
     * White for auto
     * 
     * Alliance based colors for teleop
     * 
     * Pulsing red for defense
     * 
     * Launcher xpring Konstant Blue Pulse depending on percentege of
     * target speeeeeeeeeed. 
     * 
     */

    private final CANdle candle;
    private final CANdleConfiguration configs;

    // Effect management variables
    private LightEffect requestedEffect = LightEffect.OFF;
    private LightEffect activeEffect = LightEffect.OFF;

    StrobeAnimation strobe;
    SolidColor solidColor;

    private static final int[][] SK_BLUES = new int[][] { kSKBlue1, kSKBlue2, kSKBlue3, kSKBlue4 };

    public SK26Lights() {
        // Configure CANdle
        configs = new CANdleConfiguration();
        configs.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        configs.LED.BrightnessScalar = kLightsOnBrightness;
        configs.LED.StripType = StripTypeValue.RGB;
        candle = new CANdle(kCANdle.ID, canBus);
        candle.getConfigurator().apply(configs);
    }

    // ==================== Effect Request Methods ====================
    
    /**
     * Request to enable the SK Blue Wave effect.
     */
    public void requestSKBlueWave() {
        requestedEffect = LightEffect.SK_BLUE_WAVE;
    }

    /**
     * Request to turn off the lights.
     */
    public void requestOff() {
        requestedEffect = LightEffect.OFF;
    }

    /**
     * Request to set the LEDs to solid red.
     */
    public void requestLEDRed() {
        requestedEffect = LightEffect.SOLID_RED;
    }

    /**
     * Request to set the LEDs to solid blue.
     */
    public void requestLEDBlue() {
        requestedEffect = LightEffect.SOLID_BLUE;
    }

    /**
     * Request to set the LEDs to solid white.
     */
    public void requestLEDWhite() {
        requestedEffect = LightEffect.SOLID_WHITE;
    }

    /**
     * Request to set the LEDs to rainbow animation.
     */
    public void requestRainbow() {
        requestedEffect = LightEffect.RAINBOW;
    }

    /**
     * Request to set the LEDs to strobe red (defense mode).
     */
    public void requestStrobeRed() {
        requestedEffect = LightEffect.STROBE_RED;
    }

    /**
     * Request to set the LEDs to strobe blue (defense mode).
     */
    public void requestStrobeBlue() {
        requestedEffect = LightEffect.STROBE_BLUE;
    }

    /**
     * Get the currently requested effect.
     */
    public LightEffect getRequestedEffect() {
        return requestedEffect;
    }

    /**
     * Get the currently active effect.
     */
    public LightEffect getActiveEffect() {
        return activeEffect;
    }

    // ==================== Legacy Methods (for backward compatibility) ====================

    // public void enableSKBlueWave() {
    //     requestSKBlueWave();
    // }

    // public void disableSKBlueWave() {
    //     requestOff();
    // }

    private static int lerpInt(int a, int b, double t) {
        t = Math.max(0.0, Math.min(1.0, t));
        return (int) Math.round(a + (b - a) * t);
    }

    private static int[] blendRGB(int[] c0, int[] c1, double t) {
        return new int[] {
            lerpInt(c0[0], c1[0], t),
            lerpInt(c0[1], c1[1], t),
            lerpInt(c0[2], c1[2], t)
        };
    }

    private static int[] skBlueGradient(double u01) {
        u01 = u01 - Math.floor(u01);
        double scaled = u01 * SK_BLUES.length;
        int i0 = (int) Math.floor(scaled) % SK_BLUES.length;
        int i1 = (i0 + 1) % SK_BLUES.length;
        double t = scaled - Math.floor(scaled);
        return blendRGB(SK_BLUES[i0], SK_BLUES[i1], t);
    }

    private void runSKBlueWave() {
        configs.LED.BrightnessScalar = kLightsOnBrightness;
        candle.getConfigurator().apply(configs);

        double now = Timer.getFPGATimestamp();

        double colorPhase = (now / waveColorCycleSec);
        double travelPhase = now * kWaveSpeedCyclesPerSecond;

        int n = kNumLedOnBot;
        for (int i = 0; i < n; i++) {
            double x = (double) i / Math.max(1.0, (double) (n - 1));

            double u = colorPhase + (x * waveSpatialCycles) + travelPhase;

            int[] base = skBlueGradient(u);

            double wave = 0.5 + 0.5 * Math.sin(2.0 * Math.PI * (travelPhase + x * waveSpatialCycles));
            double bright = 0.35 + 0.65 * wave;

            int r = (int) Math.round(base[0] * bright);
            int g = (int) Math.round(base[1] * bright);
            int b = (int) Math.round(base[2] * bright);

            r = Math.max(0, Math.min(255, r));
            g = Math.max(0, Math.min(255, g));
            b = Math.max(0, Math.min(255, b));

            solidColor = new SolidColor(i, i);
            solidColor.Color = new RGBWColor(r, g, b, 0);
            candle.setControl(solidColor);
        }
    }

    public void setLEDRed() {
        setSolidColor(kColorRed, 0, kNumLedOnBot - 1);
    }

    public void setLEDBlue() {
        setSolidColor(kColorBlue, 0, kNumLedOnBot - 1);
    }

    public void setLEDWhite() {
        setSolidColor(kColorWhite, 0, kNumLedOnBot - 1);
    }

    public void setRainbowAnimation() {
        configs.LED.BrightnessScalar = kLightsOnBrightness;
        candle.getConfigurator().apply(configs);
        candle.setControl(new com.ctre.phoenix6.controls.RainbowAnimation(0, kNumLedOnBot - 1));
    }

    public void setSolidColor(int[] colorRGB, int startIndex, int endIndex) {
        configs.LED.BrightnessScalar = kLightsOnBrightness;
        candle.getConfigurator().apply(configs);
        solidColor = new SolidColor(startIndex, endIndex);
        solidColor.Color = new RGBWColor(colorRGB[0], colorRGB[1], colorRGB[2], 0);
        candle.setControl(solidColor);
    }

    public void setStrobeAnimation(int[] colorRGB, int startIndex, int endIndex, int intervalMs) {
        configs.LED.BrightnessScalar = kLightsOnBrightness;
        candle.getConfigurator().apply(configs);
        strobe = new StrobeAnimation(startIndex, endIndex);
        strobe.FrameRate = intervalMs;
        strobe.Color = new RGBWColor(colorRGB[0], colorRGB[1], colorRGB[2], 0);
        candle.setControl(strobe);
    }

    public void turnOffLights() {
        configs.LED.BrightnessScalar = kLightsOffBrightness;
        candle.getConfigurator().apply(configs);
        solidColor = new SolidColor(0, kNumLedOnBot - 1);
        solidColor.Color = new RGBWColor(0, 0, 0, 0);
        candle.setControl(solidColor);
    }

    // ==================== Effect Execution Methods ====================

    /**
     * Applies the given effect to the LEDs.
     * This method contains the logic to actually run each effect type.
     */
    private void applyEffect(LightEffect effect) {
        switch (effect) {
            case OFF:
                turnOffLights();
                break;
            case SK_BLUE_WAVE:
                runSKBlueWave();
                break;
            case SOLID_RED:
                setLEDRed();
                break;
            case SOLID_BLUE:
                setLEDBlue();
                break;
            case SOLID_WHITE:
                setLEDWhite();
                break;
            case RAINBOW:
                setRainbowAnimation();
                break;
            case STROBE_RED:
                setStrobeAnimation(kColorRed, 0, kNumLedOnBot - 1, 100);
                break;
            case STROBE_BLUE:
                setStrobeAnimation(kColorBlue, 0, kNumLedOnBot - 1, 100);
                break;
            default:
                turnOffLights();
                break;
        }
    }

    /**
     * Processes effect requests and applies the appropriate effect based on priority.
     * Lower priority values are MORE important and will override higher values.
     */
    private void processEffectRequests() {
        // Determine which effect should be active based on priority
        // (lower priority value = higher importance)
        LightEffect effectToApply = requestedEffect;

        // Only apply if the effect has changed or if it's a continuous animation
        boolean needsUpdate = (effectToApply != activeEffect) || 
                              (effectToApply == LightEffect.SK_BLUE_WAVE); // SK Blue Wave needs continuous updates

        if (needsUpdate) {
            applyEffect(effectToApply);
            activeEffect = effectToApply;
        }
    }

    @Override 
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty(
            "Requested Effect",
            () -> requestedEffect.name(),
            null);
        builder.addStringProperty(
            "Active Effect",
            () -> activeEffect.name(),
            null);
    }

    @Override
    public void periodic() {
        // Process effect requests and apply the appropriate effect
        processEffectRequests();

        SmartDashboard.putData("Lights", this);
    }

    
}