package frc.robot.subsystems;

import static frc.robot.Konstants.LightsConstants.kColorBlue;
import static frc.robot.Konstants.LightsConstants.kColorBrown;
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
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.ModulateVBatOut;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
//import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26Lights extends SubsystemBase {

    /**
     * Enum representing the different light effects with their priorities.
     * Lower priority values are MORE important and will override higher values.
     */
    public static enum LightEffect {
        OFF(100, false),           // Lowest priority - default/idle state
        SK_BLUE_WAVE(70, true),    // Idle animation
        SOLID_BLUE(70, false),     // Alliance color
        SOLID_RED(70, false),      // Alliance color
        RAINBOW(70, false),        // Celebration/special
        STROBE_RED(70, false),     // Defense mode - high priority
        STROBE_BLUE(70, false),    // Defense mode - high priority
        SOLID_WHITE(70, false),    // Auto mode indicator
        BROWNOUT(0, false);        // Critical warning for brownout - highest priority

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
     * Used in the priority queue to manage timed effects.
     */
    private static class EffectRequest {
        final LightEffect effect;
        final double expirationTime; // FPGA timestamp when this effect expires (Double.MAX_VALUE = never expires)

        EffectRequest(LightEffect effect, double expirationTime) {
            this.effect = effect;
            this.expirationTime = expirationTime;
        }

        /**
         * Checks if this effect request has expired.
         * @return true if the current time is past the expiration time
         */
        boolean isExpired() {
            return Timer.getFPGATimestamp() >= expirationTime;
        }

        /**
         * Checks if this effect request is infinite (never expires).
         * @return true if the effect has no timeout
         */
        boolean isInfinite() {
            return expirationTime == Double.MAX_VALUE;
        }
    }

    /** 
     * Comparator for EffectRequest that prioritizes by:
     * 1. Timed effects ALWAYS come before infinite effects
     * 2. Within timed or infinite groups, lower priority value = more important
     */
    private static final Comparator<EffectRequest> EFFECT_COMPARATOR = 
        Comparator.comparing((EffectRequest req) -> req.isInfinite()) // false (timed) comes before true (infinite)
                  .thenComparingInt(req -> req.effect.getPriority());

    private static final int[][] SK_BLUES = new int[][] { kSKBlue1, kSKBlue2, kSKBlue3, kSKBlue4 };
    /**
     * Linear interpolation between two integer values.
     * @param startValue the starting value
     * @param endValue the ending value
     * @param blendFactor interpolation factor in range [0, 1] where 0 = startValue and 1 = endValue
     * @return interpolated value
     */
    private static int lerpInt(int startValue, int endValue, double blendFactor) {
        // Clamp blend factor to [0, 1] range to ensure valid interpolation
        blendFactor = Math.max(0.0, Math.min(1.0, blendFactor));
        // Calculate interpolated value: (1-t)*start + t*end
        return (int) Math.round(startValue + (endValue - startValue) * blendFactor);
    }

    /**
     * Blends two RGB colors together using linear interpolation.
     * @param color0 the first RGB color [R, G, B]
     * @param color1 the second RGB color [R, G, B]
     * @param blendFactor interpolation factor where 0 = color0 and 1 = color1
     * @return blended RGB color [R, G, B]
     */
    private static int[] blendRGB(int[] color0, int[] color1, double blendFactor) {
        // Interpolate each color channel independently
        return new int[] {
            lerpInt(color0[0], color1[0], blendFactor),  // Red channel
            lerpInt(color0[1], color1[1], blendFactor),  // Green channel
            lerpInt(color0[2], color1[2], blendFactor)   // Blue channel
        };
    }
    
    /**
     * Generates a color from the SK blue gradient by cycling through all SK blue shades.
     * @param normalizedPhase a value representing position in the color cycle (wrapped to [0, 1])
     * @return RGB color array sampled from the SK blue gradient
     */
    private static int[] skBlueGradient(double normalizedPhase) {
        // Wrap phase to [0, 1) to create cyclic behavior
        normalizedPhase = normalizedPhase - Math.floor(normalizedPhase);
        
        // Scale phase to span across all 4 SK blue colors
        double scaledPhase = normalizedPhase * SK_BLUES.length;
        
        // Find the two adjacent color indices to blend between
        int colorIndex0 = (int) Math.floor(scaledPhase) % SK_BLUES.length;
        int colorIndex1 = (colorIndex0 + 1) % SK_BLUES.length;
        
        // Calculate blend factor between the two colors
        double colorBlendFactor = scaledPhase - Math.floor(scaledPhase);
        
        // Return the blended color
        return blendRGB(SK_BLUES[colorIndex0], SK_BLUES[colorIndex1], colorBlendFactor);
    }

    private final CANdle candle;
    private final CANdleConfiguration configs;

    // Queue of effect requests sorted by priority (lowest priority value = most important = first)
    private final PriorityQueue<EffectRequest> effectQueue = new PriorityQueue<>(EFFECT_COMPARATOR);

    // The currently active effect being displayed
    private LightEffect activeEffect = LightEffect.OFF;

    // Track current brightness to avoid redundant config applications
    private double currentBrightness = kLightsOnBrightness;

    StrobeAnimation strobe;

    SolidColor solidColor;

    public SK26Lights() {
        // Configure CANdle
        configs = new CANdleConfiguration();
        configs.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        configs.LED.BrightnessScalar = kLightsOnBrightness;
        configs.LED.StripType = StripTypeValue.RGB;
        candle = new CANdle(kCANdle.ID, canBus);
        candle.getConfigurator().apply(configs);
        applyEffect(activeEffect);
    }

    /**
     * Sets the brightness only if it has changed, to avoid redundant CAN config messages.
     * @param brightness the desired brightness scalar
     */
    private void setBrightnessIfChanged(double brightness) {
        if (currentBrightness != brightness) {
            currentBrightness = brightness;
            configs.LED.BrightnessScalar = brightness;
            candle.getConfigurator().apply(configs);
        }
    }

    /**
     * Request an effect with a specified timeout duration.
     * 
     * Behavior:
     * - Timed effects queue "in front of" infinite effects of equal priority (infinite resumes when timed expires)
     * - Infinite effects replace other infinite effects of the same type
     * - Infinite effects replace ALL other infinite effects (there can only be one "background" effect)
     * - Timed effects queue in front of infinite effects
     * - Higher priority timed effects (lower value) take precedence over lower priority timed effects
     * 
     * @param effect the effect to request
     * @param timeoutSeconds how long the effect should last (use Double.POSITIVE_INFINITY for no timeout)
     */
    public void requestEffect(LightEffect effect, double timeoutSeconds) {
        boolean isNewEffectInfinite = (timeoutSeconds == Double.POSITIVE_INFINITY);
        double expirationTime = isNewEffectInfinite 
            ? Double.MAX_VALUE 
            : Timer.getFPGATimestamp() + timeoutSeconds;
        
        if (isNewEffectInfinite) {
            // Infinite effect: remove ALL existing infinite requests (only one background effect allowed)
            effectQueue.removeIf(req -> req.isInfinite());
        } else {
            // Timed effect: remove any existing timed requests for the same effect
            // (keep infinite effects so they resume after this timed one expires)
            effectQueue.removeIf(req -> req.effect == effect && !req.isInfinite());
        }
        
        // Add the new request to the queue
        effectQueue.add(new EffectRequest(effect, expirationTime));
    }
    
    /**
     * Request an effect that never expires (persistent until manually removed or overridden).
     * @param effect the effect to request
     */
    public void requestEffect(LightEffect effect) {
        requestEffect(effect, Double.POSITIVE_INFINITY);
    }

    /**
     * Cancel a specific effect, removing it from the queue.
     * @param effect the effect to cancel
     */
    public void cancelEffect(LightEffect effect) {
        effectQueue.removeIf(req -> req.effect == effect);
    }

    /**
     * Clear all effect requests from the queue.
     */
    public void clearAllEffects() {
        effectQueue.clear();
    }

    /**
     * Request to enable the SK Blue Wave effect.
     */
    public void requestSKBlueWave() {
        requestEffect(LightEffect.SK_BLUE_WAVE);
    }

    /**
     * Request to enable the SK Blue Wave effect with a timeout.
     * @param timeoutSeconds how long the effect should last
     */
    public void requestSKBlueWave(double timeoutSeconds) {
        requestEffect(LightEffect.SK_BLUE_WAVE, timeoutSeconds);
    }

    /**
     * Request to turn off the lights.
     */
    public void requestOff() {
        requestEffect(LightEffect.OFF);
    }

    /**
     * Request to turn off the lights with a timeout.
     * @param timeoutSeconds how long the effect should last
     */
    public void requestOff(double timeoutSeconds) {
        requestEffect(LightEffect.OFF, timeoutSeconds);
    }

    /**
     * Forcefully turn off the lights immediately, clearing all queued effects.
     */
    public void forceOff() {
        clearAllEffects();
        activeEffect = LightEffect.OFF;
        turnOffLights();
    }

    @Override
    public Command idle() {
        // TODO Auto-generated method stub
        return super.idle();
    }

    /**
     * Request to set the LEDs to solid red.
     */
    public void requestLEDRed() {
        requestEffect(LightEffect.SOLID_RED);
    }

    /**
     * Request to set the LEDs to solid red with a timeout.
     * @param timeoutSeconds how long the effect should last
     */
    public void requestLEDRed(double timeoutSeconds) {
        requestEffect(LightEffect.SOLID_RED, timeoutSeconds);
    }

    /**
     * Request to set the LEDs to solid blue.
     */
    public void requestLEDBlue() {
        requestEffect(LightEffect.SOLID_BLUE);
    }

    /**
     * Request to set the LEDs to solid blue with a timeout.
     * @param timeoutSeconds how long the effect should last
     */
    public void requestLEDBlue(double timeoutSeconds) {
        requestEffect(LightEffect.SOLID_BLUE, timeoutSeconds);
    }

    /**
     * Request to set the LEDs to solid white.
     */
    public void requestLEDWhite() {
        requestEffect(LightEffect.SOLID_WHITE);
    }

    /**
     * Request to set the LEDs to solid white with a timeout.
     * @param timeoutSeconds how long the effect should last
     */
    public void requestLEDWhite(double timeoutSeconds) {
        requestEffect(LightEffect.SOLID_WHITE, timeoutSeconds);
    }

    /**
     * Request to set the LEDs to rainbow animation.
     */
    public void requestRainbow() {
        requestEffect(LightEffect.RAINBOW);
    }

    /**
     * Request to set the LEDs to rainbow animation with a timeout.
     * @param timeoutSeconds how long the effect should last
     */
    public void requestRainbow(double timeoutSeconds) {
        requestEffect(LightEffect.RAINBOW, timeoutSeconds);
    }

    /**
     * Request to set the LEDs to strobe red (defense mode).
     */
    public void requestStrobeRed() {
        requestEffect(LightEffect.STROBE_RED);
    }

    /**
     * Request to set the LEDs to strobe red with a timeout.
     * @param timeoutSeconds how long the effect should last
     */
    public void requestStrobeRed(double timeoutSeconds) {
        requestEffect(LightEffect.STROBE_RED, timeoutSeconds);
    }

    /**
     * Request to set the LEDs to strobe blue (defense mode).
     */
    public void requestStrobeBlue() {
        requestEffect(LightEffect.STROBE_BLUE);
    }

    /**
     * Request to set the LEDs to strobe blue with a timeout.
     * @param timeoutSeconds how long the effect should last
     */
    public void requestStrobeBlue(double timeoutSeconds) {
        requestEffect(LightEffect.STROBE_BLUE, timeoutSeconds);
    }

    /**
     * Request to set the LEDs to brownout effect (critical warning).
     */
    public void requestBrownout() {
        requestEffect(LightEffect.BROWNOUT);
    }

    /**
     * Request to set the LEDs to brownout effect with a timeout.
     * @param timeoutSeconds how long the effect should last
     */
    public void requestBrownout(double timeoutSeconds) {
        requestEffect(LightEffect.BROWNOUT, timeoutSeconds);
    }

    /**
     * Get the highest priority effect currently in the queue (or OFF if queue is empty).
     * @return the highest priority queued effect
     */
    public LightEffect getHighestPriorityEffect() {
        // Remove expired effects before checking
        effectQueue.removeIf(EffectRequest::isExpired);
        
        EffectRequest top = effectQueue.peek();
        return (top != null) ? top.effect : LightEffect.OFF;
    }

    /**
     * Get the currently active effect.
     */
    public LightEffect getActiveEffect() {
        return activeEffect;
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
        setBrightnessIfChanged(kLightsOnBrightness);
        candle.setControl(new com.ctre.phoenix6.controls.RainbowAnimation(0, kNumLedOnBot - 1));
    }

    public void setSolidColor(int[] colorRGB, int startIndex, int endIndex) {
        setBrightnessIfChanged(kLightsOnBrightness);
        solidColor = new SolidColor(startIndex, endIndex);
        solidColor.Color = new RGBWColor(colorRGB[0], colorRGB[1], colorRGB[2], 0);
        candle.setControl(solidColor);
    }

    public void setStrobeAnimation(int[] colorRGB, int startIndex, int endIndex, int intervalMs) {
        setBrightnessIfChanged(kLightsOnBrightness);
        strobe = new StrobeAnimation(startIndex, endIndex);
        strobe.FrameRate = intervalMs;
        strobe.Color = new RGBWColor(colorRGB[0], colorRGB[1], colorRGB[2], 0);
        candle.setControl(strobe);
    }

    public void turnOffLights() {
        setBrightnessIfChanged(kLightsOffBrightness);
        solidColor = new SolidColor(0, kNumLedOnBot - 1);
        solidColor.Color = new RGBWColor(0, 0, 0, 0);
        candle.setControl(solidColor);
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

    // ==================== Effect Execution Methods ====================

    @Override
    public void periodic() {
        // Process effect requests and apply the appropriate effect
        processEffectRequests();

        SmartDashboard.putData("Lights", this);
    }

    private void runSKBlueWave() {
        setBrightnessIfChanged(kLightsOnBrightness);

        double now = Timer.getFPGATimestamp();

        // Controls which color in the SK blue gradient is displayed (cycles through all shades every kWaveColorCycleSec)
        double colorPhase = (now / kWaveColorCycleSec);
        
        // Controls how far the wave has traveled along the LED strip (animates the wave motion)
        double travelPhase = now * kWaveSpeedCyclesPerSecond;

        for (int i = 0; i < kNumLedOnBot; i++) {
            // Normalize LED index to [0, 1] range for spatial wave calculations
            double normalizedLEDPosition = (double) i / Math.max(1.0, (double) (kNumLedOnBot - 1));

            // Combine color cycle, spatial position, and wave travel to get this LED's position in the wave
            double wavePhase = colorPhase + (normalizedLEDPosition * kWaveSpatialCycles) + travelPhase;

            int[] base = skBlueGradient(wavePhase);

            solidColor = new SolidColor(i, i);
            solidColor.Color = new RGBWColor(base[0], base[1], base[2], 0);
            candle.setControl(solidColor);
        }
    }

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
            case BROWNOUT:
                setStrobeAnimation(kColorBrown, 0, kNumLedOnBot - 1, 500);
                break;
            default:
                turnOffLights();
                break;
        }
    }

    /**
     * Processes effect requests from the queue and applies the highest priority non-expired effect.
     * Expired effects are automatically removed from the queue.
     */
    private void processEffectRequests() {
        // Remove all expired effects from the queue
        effectQueue.removeIf(EffectRequest::isExpired);

        // Get the highest priority effect from the queue (or default to OFF if empty)
        EffectRequest topRequest = effectQueue.peek();
        LightEffect effectToApply = (topRequest != null) ? topRequest.effect : LightEffect.OFF;

        // Only apply if the effect has changed or if it's a continuous animation that needs updating
        boolean needsUpdate = (effectToApply != activeEffect) || effectToApply.isCustomWave();

        if (needsUpdate) {
            applyEffect(effectToApply);
            activeEffect = effectToApply;
        }
    }

    
}