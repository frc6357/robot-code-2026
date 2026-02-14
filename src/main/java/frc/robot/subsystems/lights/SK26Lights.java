package frc.robot.subsystems.lights;

import static frc.robot.Konstants.LightsConstants.kLEDBufferLength;
import static frc.robot.Konstants.LightsConstants.kLightsPWMHeader;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
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

    // Hardware
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    private final AddressableLEDBuffer m_baseBuffer;

    // Components
    private final LightPatterns patterns;
    private final LightEffects effects;

    // State
    private String ledStatus = "Breathing SKBlue";
    private LightMode currentMode = LightMode.BREATHING_SKBLUE;
    private GameState currentGameState = GameState.PRE_MATCH_NO_FMS;
    private GameState previousGameState = GameState.PRE_MATCH_NO_FMS;
    private boolean autoLightsEnabled = true;
    private boolean partyModeActive = false;
    private boolean wasInAuto = false;

    // Calibration
    private boolean runCalibrationTest = false;
    private int calibrationStep = 0;
    private int calibrationTimer = 0;
    private static final int CALIBRATION_STEP_DURATION = 150;

    // Game timing
    private static final double ENDGAME_START_TIME = 10.0;
    private static final double ENDGAME_FLASH_DURATION = 3.0;

    // Fun mode
    private final SendableChooser<LightMode> funEffectChooser = new SendableChooser<>();
    private boolean funModeEnabled = false;
    
    // Game controller mode - when enabled, A buttons interact with games instead of lights
    private boolean gameModeEnabled = false;

    public SK26Lights() {
        // Initialize hardware
        m_led = new AddressableLED(kLightsPWMHeader);
        m_buffer = new AddressableLEDBuffer(kLEDBufferLength);
        m_baseBuffer = new AddressableLEDBuffer(kLEDBufferLength);
        m_led.setLength(kLEDBufferLength);
        m_led.start();

        // Initialize components
        patterns = new LightPatterns();
        effects = new LightEffects(patterns);

        // Setup SmartDashboard
        SmartDashboard.putData("Lights", this);
        SmartDashboard.putNumber("Lights/Gamma", DEFAULT_GAMMA);
        SmartDashboard.putNumber("Lights/Red Correction", DEFAULT_RED_CORRECTION);
        SmartDashboard.putNumber("Lights/Green Correction", DEFAULT_GREEN_CORRECTION);
        SmartDashboard.putNumber("Lights/Blue Correction", DEFAULT_BLUE_CORRECTION);
        SmartDashboard.putNumber("Lights/Brightness", DEFAULT_BRIGHTNESS);

        // Setup fun effect chooser
        setupFunEffectChooser();
    }

    private void setupFunEffectChooser() {
        funEffectChooser.setDefaultOption("Rainbow", LightMode.RAINBOW);
        
        // Classic effects
        funEffectChooser.addOption("Fire", LightMode.FIRE);
        funEffectChooser.addOption("Police", LightMode.POLICE);
        funEffectChooser.addOption("Sparkle", LightMode.SPARKLE);
        funEffectChooser.addOption("Color Chase", LightMode.COLOR_CHASE);
        funEffectChooser.addOption("Meteor", LightMode.METEOR);
        funEffectChooser.addOption("Theater Chase", LightMode.THEATER_CHASE);
        funEffectChooser.addOption("Lava Lamp", LightMode.LAVA_LAMP);
        funEffectChooser.addOption("Ocean Wave", LightMode.OCEAN_WAVE);
        funEffectChooser.addOption("Twinkle Stars", LightMode.TWINKLE_STARS);
        funEffectChooser.addOption("Candy Cane", LightMode.CANDY_CANE);
        funEffectChooser.addOption("Plasma", LightMode.PLASMA);
        funEffectChooser.addOption("Knight Rider", LightMode.KNIGHT_RIDER);
        funEffectChooser.addOption("Heartbeat", LightMode.HEARTBEAT);
        funEffectChooser.addOption("Lightning", LightMode.LIGHTNING);
        funEffectChooser.addOption("Confetti", LightMode.CONFETTI);
        funEffectChooser.addOption("Comet Trail", LightMode.COMET_TRAIL);
        
        // Awesome effects
        funEffectChooser.addOption("Aurora Borealis", LightMode.AURORA_BOREALIS);
        funEffectChooser.addOption("Galaxy Swirl", LightMode.GALAXY_SWIRL);
        funEffectChooser.addOption("Neon Pulse", LightMode.NEON_PULSE);
        funEffectChooser.addOption("Matrix Rain", LightMode.MATRIX_RAIN);
        funEffectChooser.addOption("Fireworks", LightMode.FIREWORKS);
        funEffectChooser.addOption("Breathing Rainbow", LightMode.BREATHING_RAINBOW);
        funEffectChooser.addOption("Wave Collision", LightMode.WAVE_COLLISION);
        funEffectChooser.addOption("Disco Ball", LightMode.DISCO_BALL);
        funEffectChooser.addOption("Cyberpunk", LightMode.CYBERPUNK);
        funEffectChooser.addOption("Snake Game", LightMode.SNAKE_GAME);
        funEffectChooser.addOption("Ripple", LightMode.RIPPLE);
        funEffectChooser.addOption("Gradient Bounce", LightMode.GRADIENT_BOUNCE);
        funEffectChooser.addOption("Electric Sparks", LightMode.ELECTRIC_SPARKS);
        funEffectChooser.addOption("Sunset", LightMode.SUNSET);
        funEffectChooser.addOption("Northern Lights", LightMode.NORTHERN_LIGHTS);
        funEffectChooser.addOption("Pacman", LightMode.PACMAN);
        funEffectChooser.addOption("Sound Wave", LightMode.SOUND_WAVE);
        funEffectChooser.addOption("DNA Helix", LightMode.DNA_HELIX);
        funEffectChooser.addOption("Portal", LightMode.PORTAL);
        funEffectChooser.addOption("Hypnotic Spiral", LightMode.HYPNOTIC_SPIRAL);
        funEffectChooser.addOption("Pixel Rain", LightMode.PIXEL_RAIN);
        funEffectChooser.addOption("Fireflies", LightMode.FIREFLIES);
        funEffectChooser.addOption("Nyan Cat", LightMode.NYAN_CAT);
        funEffectChooser.addOption("Racing Stripes", LightMode.RACING_STRIPES);
        funEffectChooser.addOption("Drip", LightMode.DRIP);
        funEffectChooser.addOption("🦎 Godzilla Charging", LightMode.GODZILLA_CHARGING);
        
        // Interactive games (use Game Button to interact!)
        funEffectChooser.addOption("🎮 Stop The Light", LightMode.STOP_THE_LIGHT);
        funEffectChooser.addOption("🎮 Tug of War", LightMode.TUG_OF_WAR);
        funEffectChooser.addOption("🎮 Rhythm Game", LightMode.RHYTHM_GAME);
        funEffectChooser.addOption("🎮 Simon Says", LightMode.SIMON_SAYS);
        funEffectChooser.addOption("🎮 Color Knockout (1v1)", LightMode.COLOR_KNOCKOUT);
        
        // SK effects
        funEffectChooser.addOption("SK Gradient", LightMode.SKBLUE_GRADIENT);
        funEffectChooser.addOption("SK Breathing", LightMode.BREATHING_SKBLUE);
        
        SmartDashboard.putData("Lights/Fun Effects", funEffectChooser);
        SmartDashboard.putBoolean("Lights/Fun Mode", false);
        SmartDashboard.putBoolean("Lights/Game Controller Mode", false);
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        updateCalibrationValues();
        
        funModeEnabled = SmartDashboard.getBoolean("Lights/Fun Mode", false);
        gameModeEnabled = SmartDashboard.getBoolean("Lights/Game Controller Mode", false);
        
        if (funModeEnabled) {
            autoLightsEnabled = false;
            LightMode selectedEffect = funEffectChooser.getSelected();
            if (selectedEffect != null) {
                currentMode = selectedEffect;
                ledStatus = "Fun Mode: " + selectedEffect.toString();
            }
        } else if (autoLightsEnabled) {
            updateAutoLights();
        }

        if (runCalibrationTest) {
            runCalibrationSequence();
        } else {
            applyCurrentMode();
            copyBaseToMain();
        }

        applyColorCorrection();
        m_led.setData(m_buffer);
    }

    // ==================== MODE APPLICATION ====================

    private void applyCurrentMode() {
        switch (currentMode) {
            // Basic modes
            case OFF: patterns.off.applyTo(m_baseBuffer); break;
            case SOLID_WHITE: patterns.white.applyTo(m_baseBuffer); break;
            case SOLID_GREEN: patterns.green.applyTo(m_baseBuffer); break;
            case SOLID_RED: patterns.red.applyTo(m_baseBuffer); break;
            case SOLID_BLUE: patterns.blue.applyTo(m_baseBuffer); break;
            case SOLID_YELLOW: patterns.yellow.applyTo(m_baseBuffer); break;
            case SOLID_ORANGE: patterns.orange.applyTo(m_baseBuffer); break;
            case RAINBOW: patterns.scrollingRainbow.applyTo(m_baseBuffer); break;
            case SKBLUE_GRADIENT: patterns.skBlueGradient.applyTo(m_baseBuffer); break;
            
            // Team/Alliance modes
            case BREATHING_SKBLUE: effects.applyBreathingSKBlue(m_baseBuffer); break;
            case ALLIANCE_GRADIENT: effects.applyAllianceGradient(m_baseBuffer); break;
            case ALLIANCE_SOLID: effects.applyAllianceSolid(m_baseBuffer); break;
            case ALLIANCE_FLASH_WHITE: effects.applyAllianceFlashWhite(m_baseBuffer); break;
            
            // Classic effects
            case FIRE: effects.applyFire(m_baseBuffer); break;
            case POLICE: effects.applyPolice(m_baseBuffer); break;
            case SPARKLE: effects.applySparkle(m_baseBuffer); break;
            case COLOR_CHASE: effects.applyColorChase(m_baseBuffer); break;
            case METEOR: effects.applyMeteor(m_baseBuffer); break;
            case THEATER_CHASE: effects.applyTheaterChase(m_baseBuffer); break;
            case LAVA_LAMP: effects.applyLavaLamp(m_baseBuffer); break;
            case OCEAN_WAVE: effects.applyOceanWave(m_baseBuffer); break;
            case TWINKLE_STARS: effects.applyTwinkleStars(m_baseBuffer); break;
            case CANDY_CANE: effects.applyCandyCane(m_baseBuffer); break;
            case PLASMA: effects.applyPlasma(m_baseBuffer); break;
            case KNIGHT_RIDER: effects.applyKnightRider(m_baseBuffer); break;
            case HEARTBEAT: effects.applyHeartbeat(m_baseBuffer); break;
            case LIGHTNING: effects.applyLightning(m_baseBuffer); break;
            case CONFETTI: effects.applyConfetti(m_baseBuffer); break;
            case COMET_TRAIL: effects.applyCometTrail(m_baseBuffer); break;
            
            // Awesome effects
            case AURORA_BOREALIS: effects.applyAuroraBorealis(m_baseBuffer); break;
            case GALAXY_SWIRL: effects.applyGalaxySwirl(m_baseBuffer); break;
            case NEON_PULSE: effects.applyNeonPulse(m_baseBuffer); break;
            case MATRIX_RAIN: effects.applyMatrixRain(m_baseBuffer); break;
            case FIREWORKS: effects.applyFireworks(m_baseBuffer); break;
            case BREATHING_RAINBOW: effects.applyBreathingRainbow(m_baseBuffer); break;
            case WAVE_COLLISION: effects.applyWaveCollision(m_baseBuffer); break;
            case DISCO_BALL: effects.applyDiscoBall(m_baseBuffer); break;
            case CYBERPUNK: effects.applyCyberpunk(m_baseBuffer); break;
            case SNAKE_GAME: effects.applySnakeGame(m_baseBuffer); break;
            case RIPPLE: effects.applyRipple(m_baseBuffer); break;
            case GRADIENT_BOUNCE: effects.applyGradientBounce(m_baseBuffer); break;
            case ELECTRIC_SPARKS: effects.applyElectricSparks(m_baseBuffer); break;
            case SUNSET: effects.applySunset(m_baseBuffer); break;
            case NORTHERN_LIGHTS: effects.applyNorthernLights(m_baseBuffer); break;
            case PACMAN: effects.applyPacman(m_baseBuffer); break;
            case SOUND_WAVE: effects.applySoundWave(m_baseBuffer); break;
            case DNA_HELIX: effects.applyDNAHelix(m_baseBuffer); break;
            case PORTAL: effects.applyPortal(m_baseBuffer); break;
            case HYPNOTIC_SPIRAL: effects.applyHypnoticSpiral(m_baseBuffer); break;
            case PIXEL_RAIN: effects.applyPixelRain(m_baseBuffer); break;
            case FIREFLIES: effects.applyFireflies(m_baseBuffer); break;
            case NYAN_CAT: effects.applyNyanCat(m_baseBuffer); break;
            case RACING_STRIPES: effects.applyRacingStripes(m_baseBuffer); break;
            case DRIP: effects.applyDrip(m_baseBuffer); break;
            case GODZILLA_CHARGING: effects.applyGodzillaCharging(m_baseBuffer); break;
            
            // Interactive games
            case STOP_THE_LIGHT: effects.applyStopTheLight(m_baseBuffer); break;
            case TUG_OF_WAR: effects.applyTugOfWar(m_baseBuffer); break;
            case RHYTHM_GAME: effects.applyRhythmGame(m_baseBuffer); break;
            case SIMON_SAYS: effects.applySimonSays(m_baseBuffer); break;
            case COLOR_KNOCKOUT: effects.applyColorKnockout(m_baseBuffer); break;
        }
    }

    // ==================== AUTO LIGHTS ====================

    private GameState determineGameState() {
        boolean fmsAttached = DriverStation.isFMSAttached();
        boolean dsAttached = DriverStation.isDSAttached();
        boolean auto = DriverStation.isAutonomousEnabled();
        boolean teleop = DriverStation.isTeleopEnabled();
        double matchTime = DriverStation.getMatchTime();

        if (auto) {
            return GameState.AUTO;
        }

        if (teleop) {
            if (matchTime > 0 && matchTime <= ENDGAME_START_TIME) {
                if (matchTime > ENDGAME_START_TIME - ENDGAME_FLASH_DURATION) {
                    return GameState.ENDGAME_FLASH;
                }
                return GameState.ENDGAME_SOLID;
            }
            return GameState.TELEOP;
        }

        if (previousGameState == GameState.ENDGAME_SOLID || 
            previousGameState == GameState.ENDGAME_FLASH ||
            previousGameState == GameState.POST_MATCH) {
            return GameState.POST_MATCH;
        }

        if (fmsAttached || dsAttached) {
            return GameState.PRE_MATCH_FMS;
        }
        
        return GameState.PRE_MATCH_NO_FMS;
    }

    private void updateAutoLights() {
        if (!autoLightsEnabled) {
            return;
        }

        previousGameState = currentGameState;
        GameState rawState = determineGameState();

        if (rawState == GameState.AUTO) {
            wasInAuto = true;
            currentGameState = GameState.AUTO;
        }
        else if (wasInAuto && rawState != GameState.TELEOP) {
            currentGameState = GameState.AUTO_TO_TELEOP;
        }
        else if (rawState == GameState.TELEOP) {
            wasInAuto = false;
            currentGameState = GameState.TELEOP;
        }
        else {
            if (rawState == GameState.PRE_MATCH_NO_FMS || rawState == GameState.PRE_MATCH_FMS) {
                wasInAuto = false;
            }
            currentGameState = rawState;
        }

        if (partyModeActive && currentGameState == GameState.AUTO) {
            currentMode = LightMode.RAINBOW;
            ledStatus = "Party Mode";
            return;
        }

        switch (currentGameState) {
            case PRE_MATCH_NO_FMS:
                currentMode = LightMode.BREATHING_SKBLUE;
                ledStatus = "Pre-Match (No DS)";
                break;
            case PRE_MATCH_FMS:
                currentMode = LightMode.SKBLUE_GRADIENT;
                ledStatus = "Pre-Match (DS Connected)";
                break;
            case AUTO:
                if (!partyModeActive) {
                    partyModeActive = true;
                }
                currentMode = LightMode.RAINBOW;
                ledStatus = "Auto (Party)";
                break;
            case AUTO_TO_TELEOP:
                currentMode = LightMode.ALLIANCE_SOLID;
                ledStatus = "Auto→Teleop (Alliance Solid)";
                break;
            case TELEOP:
                currentMode = LightMode.ALLIANCE_GRADIENT;
                ledStatus = "Teleop (Alliance)";
                partyModeActive = false;
                break;
            case ENDGAME_FLASH:
                currentMode = LightMode.ALLIANCE_FLASH_WHITE;
                ledStatus = "Endgame (Flash)";
                break;
            case ENDGAME_SOLID:
                currentMode = LightMode.ALLIANCE_SOLID;
                ledStatus = "Endgame (Solid)";
                break;
            case POST_MATCH:
                currentMode = LightMode.ALLIANCE_SOLID;
                ledStatus = "Post-Match (Alliance)";
                break;
        }
    }

    // ==================== COLOR CORRECTION ====================

    private void updateCalibrationValues() {
        gamma = SmartDashboard.getNumber("Lights/Gamma", DEFAULT_GAMMA);
        redCorrection = SmartDashboard.getNumber("Lights/Red Correction", DEFAULT_RED_CORRECTION);
        greenCorrection = SmartDashboard.getNumber("Lights/Green Correction", DEFAULT_GREEN_CORRECTION);
        blueCorrection = SmartDashboard.getNumber("Lights/Blue Correction", DEFAULT_BLUE_CORRECTION);
        brightness = SmartDashboard.getNumber("Lights/Brightness", DEFAULT_BRIGHTNESS);
    }

    private void applyColorCorrection() {
        for (int i = 0; i < m_buffer.getLength(); i++) {
            int r = m_buffer.getRed(i);
            int g = m_buffer.getGreen(i);
            int b = m_buffer.getBlue(i);

            if (r == 0 && g == 0 && b == 0) {
                continue;
            }

            double rNorm = r / 255.0;
            double gNorm = g / 255.0;
            double bNorm = b / 255.0;

            rNorm = Math.pow(rNorm, gamma) * redCorrection;
            gNorm = Math.pow(gNorm, gamma) * greenCorrection;
            bNorm = Math.pow(bNorm, gamma) * blueCorrection;

            double maxAfterBrightness = Math.max(rNorm, Math.max(gNorm, bNorm)) * brightness;
            
            if (maxAfterBrightness > 1.0) {
                double scale = brightness / maxAfterBrightness;
                rNorm *= scale;
                gNorm *= scale;
                bNorm *= scale;
            } else {
                rNorm *= brightness;
                gNorm *= brightness;
                bNorm *= brightness;
            }

            rNorm = Math.min(1.0, Math.max(0.0, rNorm));
            gNorm = Math.min(1.0, Math.max(0.0, gNorm));
            bNorm = Math.min(1.0, Math.max(0.0, bNorm));

            m_buffer.setRGB(i, (int)(rNorm * 255), (int)(gNorm * 255), (int)(bNorm * 255));
        }
    }

    private void copyBaseToMain() {
        for (int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, m_baseBuffer.getRed(i), m_baseBuffer.getGreen(i), m_baseBuffer.getBlue(i));
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

    public void setModeImmediate(LightMode mode) {
        autoLightsEnabled = false;
        currentMode = mode;
        ledStatus = mode.toString();
        if (mode == LightMode.BREATHING_SKBLUE) {
            effects.resetBreathePhase();
        }
    }

    public Command setMode(LightMode mode, String status) {
        return runOnce(() -> {
            autoLightsEnabled = false;
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
            partyModeActive = false;
        }).ignoringDisable(true).withName("Enable Auto Lights");
    }

    public Command disableAutoLights() {
        return runOnce(() -> {
            autoLightsEnabled = false;
        }).ignoringDisable(true).withName("Disable Auto Lights");
    }

    public Command activatePartyMode() {
        return runOnce(() -> {
            partyModeActive = true;
            autoLightsEnabled = true;
        }).ignoringDisable(true).withName("Activate Party Mode");
    }

    public void resetMatchState() {
        wasInAuto = false;
        partyModeActive = false;
        currentGameState = GameState.PRE_MATCH_NO_FMS;
        previousGameState = GameState.PRE_MATCH_NO_FMS;
    }

    // ==================== GAME BUTTON ====================

    /**
     * Universal game button - handles interaction for all interactive light games.
     * - Stop The Light: Stops the bouncing light
     * - Tug of War: Pulls toward driver side (left)
     * - Rhythm Game: Hits the note
     * - Simon Says: Handled separately with colored buttons
     * 
     * Bound to: Driver A Button
     */
    public Command gameButtonPressed() {
        return runOnce(() -> {
            switch (currentMode) {
                case STOP_THE_LIGHT:
                    effects.stopTheLight(m_baseBuffer.getLength());
                    break;
                case TUG_OF_WAR:
                    effects.tugOfWarPullLeft(); // Driver pulls left
                    break;
                case RHYTHM_GAME:
                    effects.rhythmGameHit(m_baseBuffer.getLength());
                    break;
                default:
                    // Not a game mode, do nothing
                    break;
            }
        }).ignoringDisable(true).withName("Game Button (Driver)");
    }

    /**
     * Operator game button - handles interaction for all interactive light games.
     * - Stop The Light: Stops the bouncing light (same as driver)
     * - Tug of War: Pulls toward operator side (right)
     * - Rhythm Game: Hits the note (same as driver)
     * - Simon Says: Handled separately with colored buttons
     * 
     * Bound to: Operator Right Stick Button
     */
    public Command gameButtonPressedAlt() {
        return runOnce(() -> {
            switch (currentMode) {
                case STOP_THE_LIGHT:
                    effects.stopTheLight(m_baseBuffer.getLength()); // Same as driver
                    break;
                case TUG_OF_WAR:
                    effects.tugOfWarPullRight(); // Operator pulls right
                    break;
                case RHYTHM_GAME:
                    effects.rhythmGameHit(m_baseBuffer.getLength()); // Same as driver
                    break;
                default:
                    // Not a game mode, do nothing
                    break;
            }
        }).ignoringDisable(true).withName("Game Button (Operator)");
    }

    /**
     * Reset the current game
     */
    public Command resetCurrentGame() {
        return runOnce(() -> {
            switch (currentMode) {
                case STOP_THE_LIGHT:
                    effects.resetStopTheLight();
                    break;
                case TUG_OF_WAR:
                    effects.resetTugOfWar();
                    break;
                case RHYTHM_GAME:
                    effects.resetRhythmGame();
                    break;
                case SIMON_SAYS:
                    effects.resetSimonSays();
                    break;
                case COLOR_KNOCKOUT:
                    effects.resetColorKnockout();
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
    
    // ==================== COLOR KNOCKOUT COMMANDS ====================
    
    /**
     * Color Knockout - Player 1 (Driver) Red button (B button)
     */
    public Command knockoutP1RedButton() {
        return runOnce(() -> {
            if (currentMode == LightMode.COLOR_KNOCKOUT) {
                effects.knockoutP1Press(0); // Red = 0
            }
        }).ignoringDisable(true).withName("Knockout P1 Red (B)");
    }
    
    /**
     * Color Knockout - Player 1 (Driver) Green button (A button)
     */
    public Command knockoutP1GreenButton() {
        return runOnce(() -> {
            if (currentMode == LightMode.COLOR_KNOCKOUT) {
                effects.knockoutP1Press(1); // Green = 1
            }
        }).ignoringDisable(true).withName("Knockout P1 Green (A)");
    }
    
    /**
     * Color Knockout - Player 1 (Driver) Blue button (X button)
     */
    public Command knockoutP1BlueButton() {
        return runOnce(() -> {
            if (currentMode == LightMode.COLOR_KNOCKOUT) {
                effects.knockoutP1Press(2); // Blue = 2
            }
        }).ignoringDisable(true).withName("Knockout P1 Blue (X)");
    }
    
    /**
     * Color Knockout - Player 1 (Driver) Yellow button (Y button)
     */
    public Command knockoutP1YellowButton() {
        return runOnce(() -> {
            if (currentMode == LightMode.COLOR_KNOCKOUT) {
                effects.knockoutP1Press(3); // Yellow = 3
            }
        }).ignoringDisable(true).withName("Knockout P1 Yellow (Y)");
    }
    
    /**
     * Color Knockout - Player 2 (Operator) Red button (B button)
     */
    public Command knockoutP2RedButton() {
        return runOnce(() -> {
            if (currentMode == LightMode.COLOR_KNOCKOUT) {
                effects.knockoutP2Press(0); // Red = 0
            }
        }).ignoringDisable(true).withName("Knockout P2 Red (B)");
    }
    
    /**
     * Color Knockout - Player 2 (Operator) Green button (A button)
     */
    public Command knockoutP2GreenButton() {
        return runOnce(() -> {
            if (currentMode == LightMode.COLOR_KNOCKOUT) {
                effects.knockoutP2Press(1); // Green = 1
            }
        }).ignoringDisable(true).withName("Knockout P2 Green (A)");
    }
    
    /**
     * Color Knockout - Player 2 (Operator) Blue button (X button)
     */
    public Command knockoutP2BlueButton() {
        return runOnce(() -> {
            if (currentMode == LightMode.COLOR_KNOCKOUT) {
                effects.knockoutP2Press(2); // Blue = 2
            }
        }).ignoringDisable(true).withName("Knockout P2 Blue (X)");
    }
    
    /**
     * Color Knockout - Player 2 (Operator) Yellow button (Y button)
     */
    public Command knockoutP2YellowButton() {
        return runOnce(() -> {
            if (currentMode == LightMode.COLOR_KNOCKOUT) {
                effects.knockoutP2Press(3); // Yellow = 3
            }
        }).ignoringDisable(true).withName("Knockout P2 Yellow (Y)");
    }

    // Quick command methods
    public Command setOff() { return setMode(LightMode.OFF, "Off"); }
    public Command setSolidWhite() { return setMode(LightMode.SOLID_WHITE, "White"); }
    public Command setSolidGreen() { return setMode(LightMode.SOLID_GREEN, "Green"); }
    public Command setSolidRed() { return setMode(LightMode.SOLID_RED, "Red"); }
    public Command setSolidBlue() { return setMode(LightMode.SOLID_BLUE, "Blue"); }
    public Command setSolidYellow() { return setMode(LightMode.SOLID_YELLOW, "Yellow"); }
    public Command setSolidOrange() { return setMode(LightMode.SOLID_ORANGE, "Orange"); }
    public Command setRainbow() { return setMode(LightMode.RAINBOW, "Rainbow"); }
    public Command setBreathingSKBlue() { return setMode(LightMode.BREATHING_SKBLUE, "Breathing SKBlue"); }
    public Command setSKBlueGradient() { return setMode(LightMode.SKBLUE_GRADIENT, "SKBlue Gradient"); }
    public Command setAllianceGradient() { return setMode(LightMode.ALLIANCE_GRADIENT, "Alliance Gradient"); }

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

    // ==================== GETTERS ====================

    public boolean isAutoLightsEnabled() { return autoLightsEnabled; }
    public GameState getCurrentGameState() { return currentGameState; }
    public LightMode getCurrentMode() { return currentMode; }
    public boolean isGameModeEnabled() { return gameModeEnabled; }

    // ==================== SENDABLE ====================

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Status", () -> ledStatus, null);
        builder.addStringProperty("Game State", () -> currentGameState.toString(), null);
        builder.addDoubleProperty("Gamma", () -> gamma, null);
        builder.addDoubleProperty("Red Correction", () -> redCorrection, null);
        builder.addDoubleProperty("Green Correction", () -> greenCorrection, null);
        builder.addDoubleProperty("Blue Correction", () -> blueCorrection, null);
        builder.addDoubleProperty("Brightness", () -> brightness, null);
    }
}
