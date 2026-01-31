package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Konstants.LightsConstants.kLEDBufferLength;
import static frc.robot.Konstants.LightsConstants.kLightsPWMHeader;

import static frc.robot.Konstants.LightsConstants.kSKBlue;
import static frc.robot.Konstants.LightsConstants.kSKCream;
import static frc.robot.Konstants.LightsConstants.kSKTeal;
import static frc.robot.Konstants.LightsConstants.kSKDarkBlue;

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

    private static final double DEFAULT_GAMMA = 2.8;
    private static final double DEFAULT_RED_CORRECTION = 1.0;
    private static final double DEFAULT_GREEN_CORRECTION = 1.0;
    private static final double DEFAULT_BLUE_CORRECTION = 0.85;
    private static final double DEFAULT_BRIGHTNESS = 1.0;

    private double gamma = DEFAULT_GAMMA;
    private double redCorrection = DEFAULT_RED_CORRECTION;
    private double greenCorrection = DEFAULT_GREEN_CORRECTION;
    private double blueCorrection = DEFAULT_BLUE_CORRECTION;
    private double brightness = DEFAULT_BRIGHTNESS;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    private final AddressableLEDBuffer m_baseBuffer;

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

    private final LEDPattern m_SKBlueGradient = LEDPattern.gradient(
        LEDPattern.GradientType.kContinuous,
        kSKCream,
        kSKTeal,
        kSKBlue,
        kSKDarkBlue
    ).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);

    private String ledStatus = "Breathing SKBlue";
    private boolean runCalibrationTest = false;
    private int calibrationStep = 0;
    private int calibrationTimer = 0;
    private static final int CALIBRATION_STEP_DURATION = 150;

    private double breathePhase = 0.0;
    private static final double BREATHE_SPEED = 0.05;

    private enum BaseMode {
        OFF,
        SOLID_WHITE,
        SOLID_GREEN,
        SOLID_RED,
        SOLID_BLUE,
        SOLID_YELLOW,
        SOLID_ORANGE,
        RAINBOW,
        BREATHING_SKBLUE,
        SKBLUE_GRADIENT
    }

    private BaseMode currentBaseMode = BaseMode.BREATHING_SKBLUE;

    public SK26Lights() {
        m_led = new AddressableLED(kLightsPWMHeader);
        m_buffer = new AddressableLEDBuffer(kLEDBufferLength);
        m_baseBuffer = new AddressableLEDBuffer(kLEDBufferLength);
        m_led.setLength(kLEDBufferLength);
        m_led.start();

        SmartDashboard.putData("Lights", this);

        SmartDashboard.putNumber("Lights/Gamma", DEFAULT_GAMMA);
        SmartDashboard.putNumber("Lights/Red Correction", DEFAULT_RED_CORRECTION);
        SmartDashboard.putNumber("Lights/Green Correction", DEFAULT_GREEN_CORRECTION);
        SmartDashboard.putNumber("Lights/Blue Correction", DEFAULT_BLUE_CORRECTION);
        SmartDashboard.putNumber("Lights/Brightness", DEFAULT_BRIGHTNESS);
        // SmartDashboard.putBoolean("Lights/Run Calibration", false);
        // SmartDashboard.putString("Lights/Calibration Color", "None");
    }

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

            // Apply gamma correction first
            rNorm = Math.pow(rNorm, gamma) * redCorrection;
            gNorm = Math.pow(gNorm, gamma) * greenCorrection;
            bNorm = Math.pow(bNorm, gamma) * blueCorrection;

            // Apply brightness by scaling proportionally to preserve gradients
            // Find the max channel value after brightness is applied
            double maxAfterBrightness = Math.max(rNorm, Math.max(gNorm, bNorm)) * brightness;
            
            if (maxAfterBrightness > 1.0) {
                // Scale so the brightest channel hits 1.0, preserving ratios
                double scale = brightness / maxAfterBrightness;
                rNorm *= scale;
                gNorm *= scale;
                bNorm *= scale;
            } else {
                // No clipping needed, just apply brightness directly
                rNorm *= brightness;
                gNorm *= brightness;
                bNorm *= brightness;
            }

            // Clamp to valid range (safety)
            rNorm = Math.min(1.0, Math.max(0.0, rNorm));
            gNorm = Math.min(1.0, Math.max(0.0, gNorm));
            bNorm = Math.min(1.0, Math.max(0.0, bNorm));

            m_buffer.setRGB(i, (int)(rNorm * 255), (int)(gNorm * 255), (int)(bNorm * 255));
        }
    }

    private void applyBreathingSKBlue() {
        breathePhase += BREATHE_SPEED;
        if (breathePhase > Math.PI * 2) {
            breathePhase -= Math.PI * 2;
        }

        // Sine wave from 0 to 1
        double breatheAmount = (Math.sin(breathePhase) + 1.0) / 2.0;
        // Range from minBrightness to 1.0
        double minBrightness = 0.1;
        double breatheBrightness = minBrightness + (breatheAmount * (1.0 - minBrightness));

        int r = (int)(kSKBlue.red * 255 * breatheBrightness);
        int g = (int)(kSKBlue.green * 255 * breatheBrightness);
        int b = (int)(kSKBlue.blue * 255 * breatheBrightness);

        for (int i = 0; i < m_baseBuffer.getLength(); i++) {
            m_baseBuffer.setRGB(i, r, g, b);
        }
    }

    private void applyBaseMode() {
        switch (currentBaseMode) {
            case OFF:
                m_off.applyTo(m_baseBuffer);
                break;
            case SOLID_WHITE:
                m_white.applyTo(m_baseBuffer);
                break;
            case SOLID_GREEN:
                m_solidGreen.applyTo(m_baseBuffer);
                break;
            case SOLID_RED:
                m_solidRed.applyTo(m_baseBuffer);
                break;
            case SOLID_BLUE:
                m_solidBlue.applyTo(m_baseBuffer);
                break;
            case SOLID_YELLOW:
                m_solidYellow.applyTo(m_baseBuffer);
                break;
            case SOLID_ORANGE:
                m_solidOrange.applyTo(m_baseBuffer);
                break;
            case RAINBOW:
                m_scrollingRainbow.applyTo(m_baseBuffer);
                break;
            case BREATHING_SKBLUE:
                applyBreathingSKBlue();
                break;
            case SKBLUE_GRADIENT:
                m_SKBlueGradient.applyTo(m_baseBuffer);
                break;
        }
    }

    private void copyBaseToMain() {
        for (int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, m_baseBuffer.getRed(i), m_baseBuffer.getGreen(i), m_baseBuffer.getBlue(i));
        }
    }

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
            case 0:
                pattern = m_white;
                colorName = "White";
                break;
            case 1:
                pattern = m_solidRed;
                colorName = "Red";
                break;
            case 2:
                pattern = m_solidGreen;
                colorName = "Green";
                break;
            case 3:
                pattern = m_solidBlue;
                colorName = "Blue";
                break;
            case 4:
                pattern = m_SKBlue;
                colorName = "SKBlue";
                break;
            case 5:
                pattern = m_SKBlueGradient;
                colorName = "SKBlue Gradient";
                break;
            default:
                pattern = m_white;
                colorName = "White";
                break;
        }

        pattern.applyTo(m_buffer);
        // SmartDashboard.putString("Lights/Calibration Color", colorName);
        ledStatus = "Calibration: " + colorName;
    }

    /**
     * Sets the base LED mode directly (no command).
     * Use this for immediate mode changes.
     * 
     * @param mode The base mode to set
     */
    public void setModeImmediate(BaseMode mode) {
        currentBaseMode = mode;
        ledStatus = mode.toString();
        if (mode == BaseMode.BREATHING_SKBLUE) {
            breathePhase = 0.0;
        }
    }

    /**
     * Sets the base LED mode via command.
     * 
     * @param mode The base mode to set
     * @param status The status string to display
     * @return Command that sets the mode (does not require subsystem)
     */
    public Command setMode(BaseMode mode, String status) {
        return runOnce(() -> {
            currentBaseMode = mode;
            ledStatus = status;
            if (mode == BaseMode.BREATHING_SKBLUE) {
                breathePhase = 0.0;
            }
        }).ignoringDisable(true).withName(status);
    }

    /** Turn LEDs off */
    public Command setOff() {
        return setMode(BaseMode.OFF, "Off");
    }

    /** Set LEDs to solid white */
    public Command setSolidWhite() {
        return setMode(BaseMode.SOLID_WHITE, "White");
    }

    /** Set LEDs to solid green */
    public Command setSolidGreen() {
        return setMode(BaseMode.SOLID_GREEN, "Green");
    }

    /** Set LEDs to solid red */
    public Command setSolidRed() {
        return setMode(BaseMode.SOLID_RED, "Red");
    }

    /** Set LEDs to solid blue */
    public Command setSolidBlue() {
        return setMode(BaseMode.SOLID_BLUE, "Blue");
    }

    /** Set LEDs to solid yellow */
    public Command setSolidYellow() {
        return setMode(BaseMode.SOLID_YELLOW, "Yellow");
    }

    /** Set LEDs to solid orange */
    public Command setSolidOrange() {
        return setMode(BaseMode.SOLID_ORANGE, "Orange");
    }

    /** Set LEDs to scrolling rainbow pattern */
    public Command setRainbow() {
        return setMode(BaseMode.RAINBOW, "Rainbow");
    }

    /** Set LEDs to breathing SK blue animation */
    public Command setBreathingSKBlue() {
        return setMode(BaseMode.BREATHING_SKBLUE, "Breathing SKBlue");
    }

    /** Set LEDs to scrolling SK brand gradient */
    public Command setSKBlueGradient() {
        return setMode(BaseMode.SKBLUE_GRADIENT, "SKBlue Gradient");
    }

    /** Start the color calibration test sequence */
    public Command startCalibrationTest() {
        return runOnce(() -> {
            runCalibrationTest = true;
            calibrationStep = 0;
            calibrationTimer = 0;
        }).ignoringDisable(true).withName("Start Calibration Test");
    }

    /** Stop the color calibration test */
    public Command stopCalibrationTest() {
        return runOnce(() -> {
            runCalibrationTest = false;
            // SmartDashboard.putBoolean("Lights/Run Calibration", false);
            // SmartDashboard.putString("Lights/Calibration Color", "None");
        }).ignoringDisable(true).withName("Stop Calibration Test");
    }

    /** Get the current base mode */
    public BaseMode getCurrentBaseMode() {
        return currentBaseMode;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Status", () -> ledStatus, null);
        builder.addDoubleProperty("Gamma", () -> gamma, null);
        builder.addDoubleProperty("Red Correction", () -> redCorrection, null);
        builder.addDoubleProperty("Green Correction", () -> greenCorrection, null);
        builder.addDoubleProperty("Blue Correction", () -> blueCorrection, null);
        builder.addDoubleProperty("Brightness", () -> brightness, null);
    }

    @Override
    public void periodic() {
        updateCalibrationValues();

        // boolean shouldRunCalibration = SmartDashboard.getBoolean("Lights/Run Calibration", false);
        // if (shouldRunCalibration && !runCalibrationTest) {
        //     runCalibrationTest = true;
        //     calibrationStep = 0;
        //     calibrationTimer = 0;
        // } else if (!shouldRunCalibration && runCalibrationTest) {
        //     runCalibrationTest = false;
        //     SmartDashboard.putString("Lights/Calibration Color", "None");
        // }

        if (runCalibrationTest) {
            runCalibrationSequence();
        } else {
            applyBaseMode();
            copyBaseToMain();
        }

        applyColorCorrection();
        m_led.setData(m_buffer);
    }
}