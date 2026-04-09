package frc.robot.subsystems.lights;

import static frc.robot.Konstants.LightsConstants.kLEDBufferLength;
import static frc.robot.Konstants.LightsConstants.kLightsPWMHeader;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LED lights subsystem for controlling addressable LEDs.
 * Supports automatic game-state based lighting and 40+ fun effects.
 */
public class SK26Lights extends SubsystemBase {

    // Color correction defaults
    private static final double DEFAULT_GAMMA = 2.8;
    private static final double DEFAULT_RED_CORRECTION = 1.0;
    private static final double DEFAULT_GREEN_CORRECTION = 1.0;
    private static final double DEFAULT_BLUE_CORRECTION = 0.85;
    private static final double DEFAULT_BRIGHTNESS = 1.0;

    // Color correction values
    private double gamma = DEFAULT_GAMMA;
    private double redCorrection = DEFAULT_RED_CORRECTION;
    private double greenCorrection = DEFAULT_GREEN_CORRECTION;
    private double blueCorrection = DEFAULT_BLUE_CORRECTION;
    private double brightness = DEFAULT_BRIGHTNESS;

    // Precomputed gamma LUTs (256 entries per channel) — rebuilt when calibration changes
    private int[] gammaLutR = new int[256];
    private int[] gammaLutG = new int[256];
    private int[] gammaLutB = new int[256];
    private int calibrationReadCounter = 0;
    private static final int CALIBRATION_READ_INTERVAL = 50; // read SmartDashboard every N cycles

    // Hardware
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    // Components
    private final LightPatterns patterns;
    private final LightEffects effects;

    // State
    private String ledStatus = "Auto";
    private LightMode currentMode = LightMode.AUTO;
    private boolean autoLightsEnabled = true;

    // Calibration
    private boolean runCalibrationTest = false;
    private int calibrationStep = 0;
    private int calibrationTimer = 0;
    private static final int CALIBRATION_STEP_DURATION = 150;

    // Unified effect chooser — one dropdown for everything
    private final SendableChooser<LightMode> effectChooser = new SendableChooser<>();
    private LightMode lastChooserSelection = LightMode.AUTO;
    
    // Game controller mode - when enabled, A buttons interact with games instead of lights
    private boolean gameModeEnabled = false;

    public SK26Lights() {
        // Initialize hardware
        m_led = new AddressableLED(kLightsPWMHeader);
        m_buffer = new AddressableLEDBuffer(kLEDBufferLength);
        m_led.setLength(kLEDBufferLength);
        m_led.start();

        // Build initial gamma LUTs
        rebuildGammaLuts();

        // Initialize components
        patterns = new LightPatterns();
        effects = new LightEffects(patterns);

        // Setup SmartDashboard inputs
        SmartDashboard.putNumber("Lights/Gamma", DEFAULT_GAMMA);
        SmartDashboard.putNumber("Lights/Red Correction", DEFAULT_RED_CORRECTION);
        SmartDashboard.putNumber("Lights/Green Correction", DEFAULT_GREEN_CORRECTION);
        SmartDashboard.putNumber("Lights/Blue Correction", DEFAULT_BLUE_CORRECTION);
        SmartDashboard.putNumber("Lights/Brightness", DEFAULT_BRIGHTNESS);

        // Setup unified effect chooser
        setupEffectChooser();
    }

    /**
     * Single unified effect chooser on SmartDashboard.
     * AUTO (default) = follow FMS game state + StateHandler macro state (binder controls).
     * Any other selection = override with that specific effect.
     */
    private void setupEffectChooser() {
        // Default: follow state handler / FMS
        effectChooser.setDefaultOption("🤖 Auto (Follow State)", LightMode.AUTO);

        // Solid colors
        effectChooser.addOption("Off", LightMode.OFF);
        effectChooser.addOption("Solid White", LightMode.SOLID_WHITE);
        effectChooser.addOption("Solid Green", LightMode.SOLID_GREEN);
        effectChooser.addOption("Solid Red", LightMode.SOLID_RED);
        effectChooser.addOption("Solid Blue", LightMode.SOLID_BLUE);
        effectChooser.addOption("Solid Yellow", LightMode.SOLID_YELLOW);
        effectChooser.addOption("Solid Orange", LightMode.SOLID_ORANGE);

        // Team / Alliance
        effectChooser.addOption("Alliance Gradient", LightMode.ALLIANCE_GRADIENT);
        effectChooser.addOption("Alliance Solid", LightMode.ALLIANCE_SOLID);
        effectChooser.addOption("Alliance Flash White", LightMode.ALLIANCE_FLASH_WHITE);
        effectChooser.addOption("SK Breathing", LightMode.BREATHING_SKBLUE);
        effectChooser.addOption("SK Gradient", LightMode.SKBLUE_GRADIENT);
        effectChooser.addOption("Rainbow", LightMode.RAINBOW);

        // Classic effects
        effectChooser.addOption("Fire", LightMode.FIRE);
        effectChooser.addOption("Police", LightMode.POLICE);
        effectChooser.addOption("Sparkle", LightMode.SPARKLE);
        effectChooser.addOption("Color Chase", LightMode.COLOR_CHASE);
        effectChooser.addOption("Meteor", LightMode.METEOR);
        effectChooser.addOption("Theater Chase", LightMode.THEATER_CHASE);
        effectChooser.addOption("Lava Lamp", LightMode.LAVA_LAMP);
        effectChooser.addOption("Ocean Wave", LightMode.OCEAN_WAVE);
        effectChooser.addOption("Twinkle Stars", LightMode.TWINKLE_STARS);
        effectChooser.addOption("Candy Cane", LightMode.CANDY_CANE);
        effectChooser.addOption("Plasma", LightMode.PLASMA);
        effectChooser.addOption("Knight Rider", LightMode.KNIGHT_RIDER);
        effectChooser.addOption("Heartbeat", LightMode.HEARTBEAT);
        effectChooser.addOption("Lightning", LightMode.LIGHTNING);
        effectChooser.addOption("Confetti", LightMode.CONFETTI);
        effectChooser.addOption("Comet Trail", LightMode.COMET_TRAIL);

        // Awesome effects
        effectChooser.addOption("Aurora Borealis", LightMode.AURORA_BOREALIS);
        effectChooser.addOption("Galaxy Swirl", LightMode.GALAXY_SWIRL);
        effectChooser.addOption("Neon Pulse", LightMode.NEON_PULSE);
        effectChooser.addOption("Matrix Rain", LightMode.MATRIX_RAIN);
        effectChooser.addOption("Fireworks", LightMode.FIREWORKS);
        effectChooser.addOption("Breathing Rainbow", LightMode.BREATHING_RAINBOW);
        effectChooser.addOption("Wave Collision", LightMode.WAVE_COLLISION);
        effectChooser.addOption("Disco Ball", LightMode.DISCO_BALL);
        effectChooser.addOption("Cyberpunk", LightMode.CYBERPUNK);
        effectChooser.addOption("Snake Game", LightMode.SNAKE_GAME);
        effectChooser.addOption("Ripple", LightMode.RIPPLE);
        effectChooser.addOption("Gradient Bounce", LightMode.GRADIENT_BOUNCE);
        effectChooser.addOption("Electric Sparks", LightMode.ELECTRIC_SPARKS);
        effectChooser.addOption("Sunset", LightMode.SUNSET);
        effectChooser.addOption("Northern Lights", LightMode.NORTHERN_LIGHTS);
        effectChooser.addOption("Pacman", LightMode.PACMAN);
        effectChooser.addOption("Sound Wave", LightMode.SOUND_WAVE);
        effectChooser.addOption("DNA Helix", LightMode.DNA_HELIX);
        effectChooser.addOption("Portal", LightMode.PORTAL);
        effectChooser.addOption("Hypnotic Spiral", LightMode.HYPNOTIC_SPIRAL);
        effectChooser.addOption("Pixel Rain", LightMode.PIXEL_RAIN);
        effectChooser.addOption("Fireflies", LightMode.FIREFLIES);
        effectChooser.addOption("Nyan Cat", LightMode.NYAN_CAT);
        effectChooser.addOption("Racing Stripes", LightMode.RACING_STRIPES);
        effectChooser.addOption("Drip", LightMode.DRIP);
        effectChooser.addOption("🦎 Godzilla Charging", LightMode.GODZILLA_CHARGING);

        // Interactive games
        effectChooser.addOption("🎮 Tug of War", LightMode.TUG_OF_WAR);
        effectChooser.addOption("🎮 Simon Says", LightMode.SIMON_SAYS);

        SmartDashboard.putData("Lights/Effect", effectChooser);
        SmartDashboard.putBoolean("Lights/Game Controller Mode", false);
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        // Only read calibration from SmartDashboard periodically
        if (++calibrationReadCounter >= CALIBRATION_READ_INTERVAL) {
            calibrationReadCounter = 0;
            updateCalibrationValues();
        }
        
        gameModeEnabled = SmartDashboard.getBoolean("Lights/Game Controller Mode", false);
        
        // Read the unified effect chooser
        LightMode chosen = effectChooser.getSelected();
        if (chosen == null) chosen = LightMode.AUTO;

        if (chosen != lastChooserSelection) {
            lastChooserSelection = chosen;
            if (chosen == LightMode.AUTO) {
                // Hand control back to the binder (state handler / FMS triggers)
                autoLightsEnabled = true;
                ledStatus = "Auto";
            } else {
                // Manual override from the chooser
                autoLightsEnabled = false;
                currentMode = chosen;
                ledStatus = "Manual: " + chosen.toString();
                if (chosen == LightMode.BREATHING_SKBLUE) {
                    effects.resetBreathePhase();
                }
            }
        }

        if (runCalibrationTest) {
            runCalibrationSequence();
        } else {
            applyCurrentMode();
        }

        applyColorCorrection();
        m_led.setData(m_buffer);

        logOutputs();
    }

    // ==================== MODE APPLICATION ====================

    private void applyCurrentMode() {
        switch (currentMode) {
            // Auto — binder controls mode via setMode(); if nothing has been set yet, breathe SK blue
            case AUTO: effects.applyBreathingSKBlue(m_buffer); break;

            // Basic modes
            case OFF: patterns.off.applyTo(m_buffer); break;
            case SOLID_WHITE: patterns.white.applyTo(m_buffer); break;
            case SOLID_GREEN: patterns.green.applyTo(m_buffer); break;
            case SOLID_RED: patterns.red.applyTo(m_buffer); break;
            case SOLID_BLUE: patterns.blue.applyTo(m_buffer); break;
            case SOLID_YELLOW: patterns.yellow.applyTo(m_buffer); break;
            case SOLID_ORANGE: patterns.orange.applyTo(m_buffer); break;
            case RAINBOW: patterns.scrollingRainbow.applyTo(m_buffer); break;
            case SKBLUE_GRADIENT: patterns.skBlueGradient.applyTo(m_buffer); break;

            // Strobe modes
            case STROBE_WHITE: patterns.whiteStrobe.applyTo(m_buffer); break;
            case STROBE_GREEN: patterns.greenStrobe.applyTo(m_buffer); break;
            case STROBE_RED: patterns.redStrobe.applyTo(m_buffer); break;
            case STROBE_BLUE: patterns.blueStrobe.applyTo(m_buffer); break;
            case STROBE_YELLOW: patterns.yellowStrobe.applyTo(m_buffer); break;
            case STROBE_ORANGE: patterns.orangeStrobe.applyTo(m_buffer); break;
            case STROBE_PURPLE: patterns.purpleStrobe.applyTo(m_buffer); break;
            case STROBE_SKBLUE: patterns.skBlueStrobe.applyTo(m_buffer); break;

            // Dual color modes
            case DUAL_SOLID_WHITE_GREEN: patterns.dualSolidWhiteGreen.applyTo(m_buffer); break;
            case DUAL_SOLID_WHITE_YELLOW: patterns.dualSolidWhiteYellow.applyTo(m_buffer); break;
            case DUAL_STROBE_WHITE_GREEN: patterns.dualStrobeWhiteGreen.applyTo(m_buffer); break;
            case DUAL_STROBE_WHITE_YELLOW: patterns.dualStrobeWhiteYellow.applyTo(m_buffer); break;
            
            // Team/Alliance modes
            case BREATHING_SKBLUE: effects.applyBreathingSKBlue(m_buffer); break;
            case ALLIANCE_GRADIENT: effects.applyAllianceGradient(m_buffer); break;
            case ALLIANCE_SOLID: effects.applyAllianceSolid(m_buffer); break;
            case ALLIANCE_FLASH_WHITE: effects.applyAllianceFlashWhite(m_buffer); break;
            
            // Classic effects
            case FIRE: effects.applyFire(m_buffer); break;
            case POLICE: effects.applyPolice(m_buffer); break;
            case SPARKLE: effects.applySparkle(m_buffer); break;
            case COLOR_CHASE: effects.applyColorChase(m_buffer); break;
            case METEOR: effects.applyMeteor(m_buffer); break;
            case THEATER_CHASE: effects.applyTheaterChase(m_buffer); break;
            case LAVA_LAMP: effects.applyLavaLamp(m_buffer); break;
            case OCEAN_WAVE: effects.applyOceanWave(m_buffer); break;
            case TWINKLE_STARS: effects.applyTwinkleStars(m_buffer); break;
            case CANDY_CANE: effects.applyCandyCane(m_buffer); break;
            case PLASMA: effects.applyPlasma(m_buffer); break;
            case KNIGHT_RIDER: effects.applyKnightRider(m_buffer); break;
            case HEARTBEAT: effects.applyHeartbeat(m_buffer); break;
            case LIGHTNING: effects.applyLightning(m_buffer); break;
            case CONFETTI: effects.applyConfetti(m_buffer); break;
            case COMET_TRAIL: effects.applyCometTrail(m_buffer); break;
            
            // Awesome effects
            case AURORA_BOREALIS: effects.applyAuroraBorealis(m_buffer); break;
            case GALAXY_SWIRL: effects.applyGalaxySwirl(m_buffer); break;
            case NEON_PULSE: effects.applyNeonPulse(m_buffer); break;
            case MATRIX_RAIN: effects.applyMatrixRain(m_buffer); break;
            case FIREWORKS: effects.applyFireworks(m_buffer); break;
            case BREATHING_RAINBOW: effects.applyBreathingRainbow(m_buffer); break;
            case WAVE_COLLISION: effects.applyWaveCollision(m_buffer); break;
            case DISCO_BALL: effects.applyDiscoBall(m_buffer); break;
            case CYBERPUNK: effects.applyCyberpunk(m_buffer); break;
            case SNAKE_GAME: effects.applySnakeGame(m_buffer); break;
            case RIPPLE: effects.applyRipple(m_buffer); break;
            case GRADIENT_BOUNCE: effects.applyGradientBounce(m_buffer); break;
            case ELECTRIC_SPARKS: effects.applyElectricSparks(m_buffer); break;
            case SUNSET: effects.applySunset(m_buffer); break;
            case NORTHERN_LIGHTS: effects.applyNorthernLights(m_buffer); break;
            case PACMAN: effects.applyPacman(m_buffer); break;
            case SOUND_WAVE: effects.applySoundWave(m_buffer); break;
            case DNA_HELIX: effects.applyDNAHelix(m_buffer); break;
            case PORTAL: effects.applyPortal(m_buffer); break;
            case HYPNOTIC_SPIRAL: effects.applyHypnoticSpiral(m_buffer); break;
            case PIXEL_RAIN: effects.applyPixelRain(m_buffer); break;
            case FIREFLIES: effects.applyFireflies(m_buffer); break;
            case NYAN_CAT: effects.applyNyanCat(m_buffer); break;
            case RACING_STRIPES: effects.applyRacingStripes(m_buffer); break;
            case DRIP: effects.applyDrip(m_buffer); break;
            case GODZILLA_CHARGING: effects.applyGodzillaCharging(m_buffer); break;
            
            // Interactive games
            case TUG_OF_WAR: effects.applyTugOfWar(m_buffer); break;
            case SIMON_SAYS: effects.applySimonSays(m_buffer); break;
        }
    }

    // ==================== COLOR CORRECTION ====================

    private void updateCalibrationValues() {
        double newGamma = SmartDashboard.getNumber("Lights/Gamma", DEFAULT_GAMMA);
        double newRed = SmartDashboard.getNumber("Lights/Red Correction", DEFAULT_RED_CORRECTION);
        double newGreen = SmartDashboard.getNumber("Lights/Green Correction", DEFAULT_GREEN_CORRECTION);
        double newBlue = SmartDashboard.getNumber("Lights/Blue Correction", DEFAULT_BLUE_CORRECTION);
        double newBrightness = SmartDashboard.getNumber("Lights/Brightness", DEFAULT_BRIGHTNESS);

        // Only rebuild LUTs if values actually changed
        if (newGamma != gamma || newRed != redCorrection || newGreen != greenCorrection
                || newBlue != blueCorrection || newBrightness != brightness) {
            gamma = newGamma;
            redCorrection = newRed;
            greenCorrection = newGreen;
            blueCorrection = newBlue;
            brightness = newBrightness;
            rebuildGammaLuts();
        }
    }

    /**
     * Precompute 256-entry lookup tables for each RGB channel.
     * Bakes in gamma, per-channel correction, and brightness so
     * applyColorCorrection becomes a simple table lookup.
     */
    private void rebuildGammaLuts() {
        for (int i = 0; i < 256; i++) {
            double norm = i / 255.0;
            double gammaCorrected = Math.pow(norm, gamma);

            int rOut = (int) Math.min(255, Math.max(0, gammaCorrected * redCorrection * brightness * 255));
            int gOut = (int) Math.min(255, Math.max(0, gammaCorrected * greenCorrection * brightness * 255));
            int bOut = (int) Math.min(255, Math.max(0, gammaCorrected * blueCorrection * brightness * 255));

            gammaLutR[i] = rOut;
            gammaLutG[i] = gOut;
            gammaLutB[i] = bOut;
        }
    }

    /**
     * Apply color correction using precomputed LUTs — O(1) per pixel, no Math.pow.
     */
    private void applyColorCorrection() {
        for (int i = 0; i < m_buffer.getLength(); i++) {
            int r = m_buffer.getRed(i);
            int g = m_buffer.getGreen(i);
            int b = m_buffer.getBlue(i);

            if (r == 0 && g == 0 && b == 0) {
                continue;
            }

            m_buffer.setRGB(i, gammaLutR[r], gammaLutG[g], gammaLutB[b]);
        }
    }

    // ==================== CALIBRATION ====================

    private void runCalibrationSequence() {
        calibrationTimer++;

        if (calibrationTimer >= CALIBRATION_STEP_DURATION) {
            calibrationTimer = 0;
            calibrationStep++;
            if (calibrationStep > 5) {
                calibrationStep = 0;
            }
        }

        LEDPattern pattern;
        String colorName;

        switch (calibrationStep) {
            case 0: pattern = patterns.white; colorName = "White"; break;
            case 1: pattern = patterns.red; colorName = "Red"; break;
            case 2: pattern = patterns.green; colorName = "Green"; break;
            case 3: pattern = patterns.blue; colorName = "Blue"; break;
            case 4: pattern = patterns.skBlue; colorName = "SKBlue"; break;
            case 5: pattern = patterns.skBlueGradient; colorName = "SKBlue Gradient"; break;
            default: pattern = patterns.white; colorName = "White"; break;
        }

        pattern.applyTo(m_buffer);
        ledStatus = "Calibration: " + colorName;
    }

    // ==================== COMMANDS ====================

    public Command setMode(LightMode mode) {
        return setMode(mode, mode.toString()); 
    }
    public void setModeImmediate(LightMode mode) {
        currentMode = mode;
        ledStatus = mode.toString();
        if (mode == LightMode.BREATHING_SKBLUE) {
            effects.resetBreathePhase();
        }
    }

    public Command setMode(LightMode mode, String status) {
        return runOnce(() -> {
            currentMode = mode;
            ledStatus = status;
            if (mode == LightMode.BREATHING_SKBLUE) {
                effects.resetBreathePhase();
            }
        }).ignoringDisable(true).withName(status);
    }

    public Command enableAutoLights() {
        return runOnce(() -> {
            autoLightsEnabled = true;
        }).ignoringDisable(true).withName("Enable Auto Lights");
    }

    public Command disableAutoLights() {
        return runOnce(() -> {
            autoLightsEnabled = false;
        }).ignoringDisable(true).withName("Disable Auto Lights");
    }

    public Command activatePartyMode() {
        return runOnce(() -> {
            autoLightsEnabled = true;
        }).ignoringDisable(true).withName("Activate Party Mode");
    }

    public void resetMatchState() {
    }

    // ==================== GAME BUTTON ====================

    /**
     * Toggles game controller mode on/off.
     * When ON, Operator ABXY buttons interact with light games.
     * When OFF, Operator ABXY buttons are free for other subsystems.
     */
    public Command toggleGameModeCommand() {
        return runOnce(() -> {
            gameModeEnabled = !gameModeEnabled;
            SmartDashboard.putBoolean("Lights/Game Controller Mode", gameModeEnabled);
        }).ignoringDisable(true).withName("Toggle Game Mode");
    }

    /**
     * Universal game button - handles Tug of War pull toward driver side (left).
     * Bound to: Driver A Button (when game mode enabled)
     */
    public Command gameButtonPressed() {
        return runOnce(() -> {
            if (currentMode == LightMode.TUG_OF_WAR) {
                effects.tugOfWarPullLeft();
            }
        }).ignoringDisable(true).withName("Game Button (Driver)");
    }

    /**
     * Operator game button - handles Tug of War pull toward operator side (right).
     * Bound to: Operator A Button (when game mode enabled)
     */
    public Command gameButtonPressedAlt() {
        return runOnce(() -> {
            if (currentMode == LightMode.TUG_OF_WAR) {
                effects.tugOfWarPullRight();
            }
        }).ignoringDisable(true).withName("Game Button (Operator)");
    }

    /**
     * Reset the current game
     */
    public Command resetCurrentGame() {
        return runOnce(() -> {
            switch (currentMode) {
                case TUG_OF_WAR:
                    effects.resetTugOfWar();
                    break;
                case SIMON_SAYS:
                    effects.resetSimonSays();
                    break;
                default:
                    break;
            }
        }).ignoringDisable(true).withName("Reset Game");
    }

    // ==================== SIMON SAYS BUTTONS ====================
    
    /**
     * Simon Says - Red button (B button on controller)
     * Maps to section 0 (Red)
     */
    public Command simonRedButton() {
        return runOnce(() -> {
            if (currentMode == LightMode.SIMON_SAYS) {
                effects.simonSaysInput(0); // Red = 0
            }
        }).ignoringDisable(true).withName("Simon Red (B)");
    }
    
    /**
     * Simon Says - Green button (A button on controller)
     * Maps to section 1 (Green)
     */
    public Command simonGreenButton() {
        return runOnce(() -> {
            if (currentMode == LightMode.SIMON_SAYS) {
                effects.simonSaysInput(1); // Green = 1
            }
        }).ignoringDisable(true).withName("Simon Green (A)");
    }
    
    /**
     * Simon Says - Blue button (X button on controller)
     * Maps to section 2 (Blue)
     */
    public Command simonBlueButton() {
        return runOnce(() -> {
            if (currentMode == LightMode.SIMON_SAYS) {
                effects.simonSaysInput(2); // Blue = 2
            }
        }).ignoringDisable(true).withName("Simon Blue (X)");
    }
    
    /**
     * Simon Says - Yellow button (Y button on controller)
     * Maps to section 3 (Yellow)
     */
    public Command simonYellowButton() {
        return runOnce(() -> {
            if (currentMode == LightMode.SIMON_SAYS) {
                effects.simonSaysInput(3); // Yellow = 3
            }
        }).ignoringDisable(true).withName("Simon Yellow (Y)");
    }
    
    public Command startCalibrationTest() {
        return runOnce(() -> {
            runCalibrationTest = true;
            calibrationStep = 0;
            calibrationTimer = 0;
        }).ignoringDisable(true).withName("Start Calibration Test");
    }

    public Command stopCalibrationTest() {
        return runOnce(() -> {
            runCalibrationTest = false;
        }).ignoringDisable(true).withName("Stop Calibration Test");
    }

    // ==================== GETTERS / SETTERS ====================

    public boolean isAutoLightsEnabled() { return autoLightsEnabled; }
    public LightMode getCurrentMode() { return currentMode; }
    public boolean isGameModeEnabled() { return gameModeEnabled; }

    /** Allows external callers (e.g. SK26LightsBinder) to stamp the status string. */
    public void setLedStatus(String status) { ledStatus = status; }

    // ==================== LOGGING ====================

    private void logOutputs() {
        Logger.recordOutput("Lights/Status", ledStatus);
        Logger.recordOutput("Lights/Gamma", gamma);
        Logger.recordOutput("Lights/Red Correction", redCorrection);
        Logger.recordOutput("Lights/Green Correction", greenCorrection);
        Logger.recordOutput("Lights/Blue Correction", blueCorrection);
        Logger.recordOutput("Lights/Brightness", brightness);
    }
}
