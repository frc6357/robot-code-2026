package frc.robot.subsystems;

import static frc.robot.Konstants.LightsConstants.kColorBlue;
import static frc.robot.Konstants.LightsConstants.kColorBrown;
import static frc.robot.Konstants.LightsConstants.kColorGreen;
import static frc.robot.Konstants.LightsConstants.kColorRed;
import static frc.robot.Konstants.LightsConstants.kColorWhite;
import static frc.robot.Konstants.LightsConstants.kLightsOffBrightness;
import static frc.robot.Konstants.LightsConstants.kLightsOnBrightness;
import static frc.robot.Konstants.LightsConstants.kNumLedOnBot;
import static frc.robot.Konstants.LightsConstants.kSKBlue1;
import static frc.robot.Konstants.LightsConstants.kSKBlue2;
import static frc.robot.Konstants.LightsConstants.kSKBlue3;
import static frc.robot.Konstants.LightsConstants.kSKBlue4;
import static frc.robot.Konstants.LightsConstants.kWaveColorCycleSec;
import static frc.robot.Konstants.LightsConstants.kWaveSpatialCycles;
import static frc.robot.Konstants.LightsConstants.kWaveSpeedCyclesPerSecond;
import static frc.robot.Ports.LightsPorts.canBus;
import static frc.robot.Ports.LightsPorts.kCANdle;

import java.util.Comparator;
import java.util.PriorityQueue;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26Lights extends SubsystemBase {

    // -----------------------------
    // LED color correction
    // -----------------------------
    // LED strips rarely match commanded RGB perfectly (channel imbalance + non-linear brightness).
    // These values are meant to be tuned on the robot until white looks neutral.
    private static final String kCalKeyRoot = "Lights/Cal/";
    private static final String kCalGammaKey = kCalKeyRoot + "Gamma";
    private static final String kCalRScaleKey = kCalKeyRoot + "RScale";
    private static final String kCalGScaleKey = kCalKeyRoot + "GScale";
    private static final String kCalBScaleKey = kCalKeyRoot + "BScale";

    // Defaults (used to seed SmartDashboard entries)
    private static final double kDefaultLedGamma = 2.2;
    private static final double kDefaultLedRScale = 1.00;
    private static final double kDefaultLedGScale = 0.78;
    private static final double kDefaultLedBScale = 0.92;

    private static int clamp255(int v) {
        return Math.max(0, Math.min(255, v));
    }

    /** Applies per-channel scaling in linear space, then gamma correction (0..255). */
    private static int correctChannel(int value0to255, double scale, double gamma) {
        double x = Math.max(0.0, Math.min(1.0, value0to255 / 255.0));
        x = Math.max(0.0, Math.min(1.0, x * scale));
        x = Math.pow(x, gamma);
        return clamp255((int) Math.round(x * 255.0));
    }

    private static RGBWColor correctedRGB(int[] rgb) {
        double gamma = SmartDashboard.getNumber(kCalGammaKey, kDefaultLedGamma);
        double rScale = SmartDashboard.getNumber(kCalRScaleKey, kDefaultLedRScale);
        double gScale = SmartDashboard.getNumber(kCalGScaleKey, kDefaultLedGScale);
        double bScale = SmartDashboard.getNumber(kCalBScaleKey, kDefaultLedBScale);

        int rr = correctChannel(rgb[0], rScale, gamma);
        int gg = correctChannel(rgb[1], gScale, gamma);
        int bb = correctChannel(rgb[2], bScale, gamma);
        return new RGBWColor(rr, gg, bb, 0);
    }

    /**
     * Available light effects.
     * Lower priority values win (override higher values).
     */
    public static enum LightEffect {
        OFF(100, false),           // Default/idle
        SK_BLUE_WAVE(70, true),    // Custom wave (updated every loop)
        SOLID_BLUE(50, false),     // Temporary overlay
        SOLID_RED(50, false),      // Temporary overlay
        SOLID_GREEN(50, false),    // Temporary overlay
        SOLID_WHITE(70, false),    // Base effect
        RAINBOW(70, false),        // Base effect (CANdle animation)
        STROBE_RED(50, false),     // Temporary overlay (CANdle animation)
        STROBE_BLUE(50, false),    // Temporary overlay (CANdle animation)
        BROWNOUT(0, false);        // Critical warning

        private final int priority;
        private final boolean isCustomWave;

        LightEffect(int priority, boolean isCustomWave) {
            this.priority = priority;
            this.isCustomWave = isCustomWave;
        }

        public int getPriority() {
            return priority;
        }

        public boolean isCustomWave() {
            return isCustomWave;
        }
    }

    /** One queued effect request and its expiration time. */
    private static class EffectRequest {
        final LightEffect effect;
        final double expirationTime;

        EffectRequest(LightEffect effect, double expirationTime) {
            this.effect = effect;
            this.expirationTime = expirationTime;
        }

        boolean isExpired() {
            return Timer.getFPGATimestamp() >= expirationTime;
        }

        boolean isInfinite() {
            return expirationTime == Double.MAX_VALUE;
        }
    }

    // Sort order: timed requests first, then by priority (lower wins).
    private static final Comparator<EffectRequest> EFFECT_COMPARATOR = 
        Comparator.comparing((EffectRequest req) -> req.isInfinite())
                  .thenComparingInt(req -> req.effect.getPriority());

    private static final int[][] SK_BLUES = new int[][] { kSKBlue1, kSKBlue2, kSKBlue3, kSKBlue4 };

    private static int lerpInt(int startValue, int endValue, double blendFactor) {
        blendFactor = Math.max(0.0, Math.min(1.0, blendFactor));
        return (int) Math.round(startValue + (endValue - startValue) * blendFactor);
    }

    private static int[] blendRGB(int[] color0, int[] color1, double blendFactor) {
        return new int[] {
            lerpInt(color0[0], color1[0], blendFactor),
            lerpInt(color0[1], color1[1], blendFactor),
            lerpInt(color0[2], color1[2], blendFactor)
        };
    }
    
    private static int[] skBlueGradient(double normalizedPhase) {
        normalizedPhase = normalizedPhase - Math.floor(normalizedPhase);
        double scaledPhase = normalizedPhase * SK_BLUES.length;
        int colorIndex0 = (int) Math.floor(scaledPhase) % SK_BLUES.length;
        int colorIndex1 = (colorIndex0 + 1) % SK_BLUES.length;
        double colorBlendFactor = scaledPhase - Math.floor(scaledPhase);
        return blendRGB(SK_BLUES[colorIndex0], SK_BLUES[colorIndex1], colorBlendFactor);
    }

    private final CANdle candle;
    private final CANdleConfiguration configs;
    private final PriorityQueue<EffectRequest> effectQueue = new PriorityQueue<>(EFFECT_COMPARATOR);
    private LightEffect activeEffect = LightEffect.OFF;
    private double currentBrightness = kLightsOnBrightness;

    private StrobeAnimation strobe;
    private SolidColor solidColor;

    public SK26Lights() {
        // Seed calibration entries so they show up immediately on the dashboard.
        SmartDashboard.setDefaultNumber(kCalGammaKey, kDefaultLedGamma);
        SmartDashboard.setDefaultNumber(kCalRScaleKey, kDefaultLedRScale);
        SmartDashboard.setDefaultNumber(kCalGScaleKey, kDefaultLedGScale);
        SmartDashboard.setDefaultNumber(kCalBScaleKey, kDefaultLedBScale);

        configs = new CANdleConfiguration();
        configs.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        configs.LED.BrightnessScalar = kLightsOnBrightness;
        configs.LED.StripType = StripTypeValue.GRB;
        candle = new CANdle(kCANdle.ID, canBus);
        candle.getConfigurator().apply(configs);
        
        // Start with lights off.
        solidColor = new SolidColor(0, kNumLedOnBot - 1);
        solidColor.Color = new RGBWColor(0, 0, 0, 0);
        candle.setControl(solidColor);
    }

    private void setBrightnessIfChanged(double brightness) {
        if (currentBrightness != brightness) {
            currentBrightness = brightness;
            configs.LED.BrightnessScalar = brightness;
            candle.getConfigurator().apply(configs);
        }
    }

    /** Stops any running CANdle animation by forcing a solid black output. */
    private void stopHardwareAnimation() {
        SolidColor stop = new SolidColor(0, kNumLedOnBot - 1);
        stop.Color = new RGBWColor(0, 0, 0, 0);
        candle.setControl(stop);
    }

    /** True if this effect runs as a CANdle-side animation (doesn't need periodic updates). */
    private boolean isHardwareAnimation(LightEffect effect) {
        return effect == LightEffect.RAINBOW || 
               effect == LightEffect.STROBE_RED || 
               effect == LightEffect.STROBE_BLUE || 
               effect == LightEffect.BROWNOUT;
    }

    /** Requests an effect for a duration (seconds). Use POSITIVE_INFINITY for a base effect. */
    public void requestEffect(LightEffect effect, double timeoutSeconds) {
        double expirationTime = (timeoutSeconds == Double.POSITIVE_INFINITY) 
            ? Double.MAX_VALUE 
            : Timer.getFPGATimestamp() + timeoutSeconds;
        
    // Replace any existing request for this effect.
        effectQueue.removeIf(req -> req.effect == effect);
        
        effectQueue.add(new EffectRequest(effect, expirationTime));
    }
    
    /** Requests an infinite (base) effect. */
    public void requestEffect(LightEffect effect) {
        requestEffect(effect, Double.POSITIVE_INFINITY);
    }

    /** Sets the base effect (clears any previous base effect). */
    public void setBaseEffect(LightEffect effect) {
        // Remove previous base effects.
        effectQueue.removeIf(EffectRequest::isInfinite);
        
        // Add the new base effect.
        requestEffect(effect, Double.POSITIVE_INFINITY);
    }

    /** Adds a temporary overlay effect for a duration (seconds). */
    public void addTemporaryEffect(LightEffect effect, double durationSeconds) {
        requestEffect(effect, durationSeconds);
    }

    public void cancelEffect(LightEffect effect) {
        effectQueue.removeIf(req -> req.effect == effect);
    }

    public void clearAllEffects() {
        effectQueue.clear();
    }

    public void requestSKBlueWave() {
        setBaseEffect(LightEffect.SK_BLUE_WAVE);
    }

    public void requestOff() {
        setBaseEffect(LightEffect.OFF);
    }

    public void forceOff() {
        clearAllEffects();
        stopHardwareAnimation();
        activeEffect = LightEffect.OFF;
        setBrightnessIfChanged(kLightsOffBrightness);
        solidColor = new SolidColor(0, kNumLedOnBot - 1);
        solidColor.Color = new RGBWColor(0, 0, 0, 0);
        candle.setControl(solidColor);
    }

    public void requestLEDRed(double timeoutSeconds) {
        addTemporaryEffect(LightEffect.SOLID_RED, timeoutSeconds);
    }

    public void requestLEDBlue(double timeoutSeconds) {
        addTemporaryEffect(LightEffect.SOLID_BLUE, timeoutSeconds);
    }

    public void requestLEDGreen(double timeoutSeconds) {
        addTemporaryEffect(LightEffect.SOLID_GREEN, timeoutSeconds);
    }

    public void requestLEDWhite() {
        setBaseEffect(LightEffect.SOLID_WHITE);
    }

    public void requestRainbow() {
        setBaseEffect(LightEffect.RAINBOW);
    }

    public void requestStrobeRed(double timeoutSeconds) {
        addTemporaryEffect(LightEffect.STROBE_RED, timeoutSeconds);
    }

    public void requestStrobeBlue(double timeoutSeconds) {
        addTemporaryEffect(LightEffect.STROBE_BLUE, timeoutSeconds);
    }

    public void requestBrownout() {
        addTemporaryEffect(LightEffect.BROWNOUT, Double.POSITIVE_INFINITY);
    }

    public void cancelBrownout() {
        cancelEffect(LightEffect.BROWNOUT);
    }

    public LightEffect getHighestPriorityEffect() {
        effectQueue.removeIf(EffectRequest::isExpired);
        EffectRequest top = effectQueue.peek();
        return (top != null) ? top.effect : LightEffect.OFF;
    }

    public LightEffect getActiveEffect() {
        return activeEffect;
    }

    private void turnOffLights() {
        setBrightnessIfChanged(kLightsOffBrightness);
        solidColor = new SolidColor(0, kNumLedOnBot - 1);
        solidColor.Color = new RGBWColor(0, 0, 0, 0);
        candle.setControl(solidColor);
    }

    private void setLEDRed() {
        setSolidColor(kColorRed, 0, kNumLedOnBot - 1);
    }

    private void setLEDBlue() {
        setSolidColor(kColorBlue, 0, kNumLedOnBot - 1);
    }

    private void setLEDWhite() {
        setSolidColor(kColorWhite, 0, kNumLedOnBot - 1);
    }

    private void setLEDGreen() {
        setSolidColor(kColorGreen, 0, kNumLedOnBot - 1);
    }

    private void setRainbowAnimation() {
        setBrightnessIfChanged(kLightsOnBrightness);
        candle.setControl(new com.ctre.phoenix6.controls.RainbowAnimation(0, kNumLedOnBot - 1));
    }

    private void setSolidColor(int[] colorRGB, int startIndex, int endIndex) {
        setBrightnessIfChanged(kLightsOnBrightness);
        solidColor = new SolidColor(startIndex, endIndex);
        solidColor.Color = correctedRGB(colorRGB);
        candle.setControl(solidColor);
    }

    private void setStrobeAnimation(int[] colorRGB, int startIndex, int endIndex, int intervalMs) {
        setBrightnessIfChanged(kLightsOnBrightness);
        strobe = new StrobeAnimation(startIndex, endIndex);
        strobe.FrameRate = intervalMs;
        strobe.Color = correctedRGB(colorRGB);
        candle.setControl(strobe);
    }

    private void runSKBlueWave() {
        setBrightnessIfChanged(kLightsOnBrightness);

        double now = Timer.getFPGATimestamp();
        double colorPhase = (now / kWaveColorCycleSec);
        double travelPhase = now * kWaveSpeedCyclesPerSecond;

        for (int i = 0; i < kNumLedOnBot; i++) {
            double normalizedLEDPosition = (double) i / Math.max(1.0, (double) (kNumLedOnBot - 1));
            double wavePhase = colorPhase + (normalizedLEDPosition * kWaveSpatialCycles) + travelPhase;
            int[] base = skBlueGradient(wavePhase);

            solidColor = new SolidColor(i, i);
            solidColor.Color = correctedRGB(base);
            candle.setControl(solidColor);
        }
    }

    @Override 
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty(
            "Highest Priority Effect",
            () -> getHighestPriorityEffect().name(),
            null);
        builder.addStringProperty(
            "Active Effect",
            () -> activeEffect.name(),
            null);
        builder.addIntegerProperty(
            "Queue Size",
            () -> effectQueue.size(),
            null);
    }

    @Override
    public void periodic() {
        processEffectRequests();
        SmartDashboard.putData("Lights", this);
    }

    /** Applies the requested effect. */
    private void applyEffect(LightEffect effect, boolean isNewEffect) {
        // If switching away from a CANdle animation, stop it first.
        if (isNewEffect && isHardwareAnimation(activeEffect)) {
            stopHardwareAnimation();
        }

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
            case SOLID_GREEN:
                setLEDGreen();
                break;
            case SOLID_WHITE:
                setLEDWhite();
                break;
            case RAINBOW:
                if (isNewEffect) {
                    setRainbowAnimation();
                }
                break;
            case STROBE_RED:
                if (isNewEffect) {
                    setStrobeAnimation(kColorRed, 0, kNumLedOnBot - 1, 100);
                }
                break;
            case STROBE_BLUE:
                if (isNewEffect) {
                    setStrobeAnimation(kColorBlue, 0, kNumLedOnBot - 1, 100);
                }
                break;
            case BROWNOUT:
                if (isNewEffect) {
                    setStrobeAnimation(kColorBrown, 0, kNumLedOnBot - 1, 500);
                }
                break;
            default:
                turnOffLights();
                break;
        }
    }

    /**
     * Processes the request queue and applies the current highest-priority effect.
     * Expired requests are removed automatically.
     */
    private void processEffectRequests() {
        // Remove expired requests.
        effectQueue.removeIf(EffectRequest::isExpired);

    // Pick the highest-priority request, or OFF if there isn't one.
        EffectRequest topRequest = effectQueue.peek();
        LightEffect effectToApply = (topRequest != null) ? topRequest.effect : LightEffect.OFF;

    // Detect effect transitions.
        boolean isNewEffect = (effectToApply != activeEffect);

        // Apply on change, or continuously for custom waves.
        if (isNewEffect || effectToApply.isCustomWave()) {
            applyEffect(effectToApply, isNewEffect);
            activeEffect = effectToApply;
        }
    }
}