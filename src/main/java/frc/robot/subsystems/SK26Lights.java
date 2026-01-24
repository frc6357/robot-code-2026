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

    /**
     * Enum representing the different light effects with their priorities.
     * Lower priority values are MORE important and will override higher values.
     */
    public static enum LightEffect {
        OFF(100, false),           // Lowest priority - default/idle state
        SK_BLUE_WAVE(70, true),    // Custom wave - needs continuous updates
        SOLID_BLUE(50, false),     // Temporary overlay - higher priority than base
        SOLID_RED(50, false),      // Temporary overlay
        SOLID_GREEN(50, false),    // Temporary overlay
        SOLID_WHITE(70, false),    // Base effect
        RAINBOW(70, false),        // Base effect (hardware animation)
        STROBE_RED(50, false),     // Temporary overlay (hardware animation)
        STROBE_BLUE(50, false),    // Temporary overlay (hardware animation)
        BROWNOUT(0, false);        // Critical warning - highest priority

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

    /**
     * Represents an effect request with an associated expiration time.
     */
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

    // Sort by: timed effects first (they have priority), then by priority value (lower = more important)
    private static final Comparator<EffectRequest> EFFECT_COMPARATOR = 
        Comparator.comparing((EffectRequest req) -> req.isInfinite())  // false (timed) before true (infinite)
                  .thenComparingInt(req -> req.effect.getPriority());  // then by priority (lower = better)

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
        configs = new CANdleConfiguration();
        configs.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        configs.LED.BrightnessScalar = kLightsOnBrightness;
        configs.LED.StripType = StripTypeValue.GRB;
        candle = new CANdle(kCANdle.ID, canBus);
        candle.getConfigurator().apply(configs);
        
        // Initialize with OFF state
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

    /**
     * Stops any running hardware animation by setting a solid black color.
     */
    private void stopHardwareAnimation() {
        SolidColor stop = new SolidColor(0, kNumLedOnBot - 1);
        stop.Color = new RGBWColor(0, 0, 0, 0);
        candle.setControl(stop);
    }

    /**
     * Checks if an effect is a hardware animation that runs autonomously on the CANdle.
     */
    private boolean isHardwareAnimation(LightEffect effect) {
        return effect == LightEffect.RAINBOW || 
               effect == LightEffect.STROBE_RED || 
               effect == LightEffect.STROBE_BLUE || 
               effect == LightEffect.BROWNOUT;
    }

    /**
     * Request a temporary effect that will overlay the base effect for a duration.
     */
    public void requestEffect(LightEffect effect, double timeoutSeconds) {
        double expirationTime = (timeoutSeconds == Double.POSITIVE_INFINITY) 
            ? Double.MAX_VALUE 
            : Timer.getFPGATimestamp() + timeoutSeconds;
        
        // Remove any existing request for this same effect
        effectQueue.removeIf(req -> req.effect == effect);
        
        effectQueue.add(new EffectRequest(effect, expirationTime));
    }
    
    /**
     * Request an infinite (base) effect.
     */
    public void requestEffect(LightEffect effect) {
        requestEffect(effect, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the base effect, clearing any previous base effects.
     * Temporary effects will still overlay this.
     */
    public void setBaseEffect(LightEffect effect) {
        // Remove all infinite effects (previous base effects)
        effectQueue.removeIf(EffectRequest::isInfinite);
        
        // Add the new base effect
        requestEffect(effect, Double.POSITIVE_INFINITY);
    }

    /**
     * Adds a temporary overlay effect with a duration.
     * When it expires, the base effect will show again.
     */
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
        solidColor.Color = new RGBWColor(colorRGB[0], colorRGB[1], colorRGB[2], 0);
        candle.setControl(solidColor);
    }

    private void setStrobeAnimation(int[] colorRGB, int startIndex, int endIndex, int intervalMs) {
        setBrightnessIfChanged(kLightsOnBrightness);
        strobe = new StrobeAnimation(startIndex, endIndex);
        strobe.FrameRate = intervalMs;
        strobe.Color = new RGBWColor(colorRGB[0], colorRGB[1], colorRGB[2], 0);
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
            solidColor.Color = new RGBWColor(base[0], base[1], base[2], 0);
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

    /**
     * Applies the given effect to the LEDs.
     */
    private void applyEffect(LightEffect effect, boolean isNewEffect) {
        // If switching from a hardware animation to something else, stop it first
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
     * Processes effect requests from the queue and applies the highest priority non-expired effect.
     * Expired effects are automatically removed, allowing base effects to show through.
     */
    private void processEffectRequests() {
        // Remove all expired effects from the queue
        effectQueue.removeIf(EffectRequest::isExpired);

        // Get the highest priority effect from the queue (or default to OFF if empty)
        EffectRequest topRequest = effectQueue.peek();
        LightEffect effectToApply = (topRequest != null) ? topRequest.effect : LightEffect.OFF;

        // Check if this is a new effect
        boolean isNewEffect = (effectToApply != activeEffect);

        // Apply if: new effect, OR it's a custom wave that needs continuous updates
        if (isNewEffect || effectToApply.isCustomWave()) {
            applyEffect(effectToApply, isNewEffect);
            activeEffect = effectToApply;
        }
    }
}