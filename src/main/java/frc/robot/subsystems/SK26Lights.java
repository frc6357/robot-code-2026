package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
//import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
//import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.StrobeAnimation;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Konstants.LightsConstants.kLightsPWMHeader;
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

public class SK26Lights extends SubsystemBase{
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

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private final CANdle candle;
    private final CANdleConfiguration configs;

    public boolean skBlueWaveEnabled = false;

    StrobeAnimation strobe;
    SolidColor solidColor;

    private static final int[][] SK_BLUES = new int[][] { kSKBlue1, kSKBlue2, kSKBlue3, kSKBlue4 };

    public SK26Lights() {
        led = new AddressableLED(kLightsPWMHeader);
        ledBuffer = new AddressableLEDBuffer(kNumLedOnBot);
        led.setLength(ledBuffer.getLength());

        // Configure CANdle
        configs = new CANdleConfiguration();
        configs.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        configs.LED.BrightnessScalar = kLightsOnBrightness;
        configs.LED.StripType = StripTypeValue.RGB;
        candle = new CANdle(kCANdle.ID, canBus);
        candle.getConfigurator().apply(configs);
    }

    /*public void setAllLEDs(int[] colorRGB) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, colorRGB[0], colorRGB[1], colorRGB[2]);
        }
        led.setData(ledBuffer);
        led.start();
    }*/

    public void enableSKBlueWave() {
        skBlueWaveEnabled = true;
        configs.LED.BrightnessScalar = kLightsOnBrightness;
    }

    public void disableSKBlueWave() {
        skBlueWaveEnabled = false;
        configs.LED.BrightnessScalar = kLightsOffBrightness;
    }

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
        double now = Timer.getFPGATimestamp();

        double colorPhase = (now / waveColorCycleSec);

        double travelPhase = now * kWaveSpeedCyclesPerSecond;

        int n = ledBuffer.getLength();
        for (int i = 0; i < n; i++) {
            double x = (double) i / Math.max(1, n - 1);

            double u = colorPhase + (x * waveSpatialCycles) + travelPhase;

            int[] rgb = skBlueGradient(u);

            double pulse = 0.65 + 0.35 * Math.sin(2.0 * Math.PI * (travelPhase + x));
            int r = (int) Math.round(rgb[0] * pulse);
            int g = (int) Math.round(rgb[1] * pulse);
            int b = (int) Math.round(rgb[2] * pulse);

            ledBuffer.setRGB(i, r, g, b);
        }

        led.setData(ledBuffer);
    }

    public void setLEDRed() {
        setSolidColor(kColorRed, 0, ledBuffer.getLength() - 1);
    }

    public void setLEDBlue() {
        setSolidColor(kColorBlue, 0, ledBuffer.getLength() - 1);
    }

    public void setLEDWhite() {
        setSolidColor(kColorWhite, 0, ledBuffer.getLength() - 1);
    }

    public void setRainbowAnimation() {
        configs.LED.BrightnessScalar = kLightsOnBrightness;
        candle.setControl(new com.ctre.phoenix6.controls.RainbowAnimation(0, ledBuffer.getLength() - 1));
    }

    public void setSolidColor(int[] colorRGB, int startIndex, int endIndex) {
        configs.LED.BrightnessScalar = kLightsOnBrightness;
        solidColor = new SolidColor(startIndex, endIndex);
        solidColor.Color = new RGBWColor(colorRGB[0], colorRGB[1], colorRGB[2], 0);
        candle.setControl(solidColor);
    }

    // public void setStrobeAnimation(int[] colorRGB, int startIndex, int endIndex, int intervalMs) {
    //     configs.LED.BrightnessScalar = kLightsOnBrightness;
    //     strobe = new StrobeAnimation(startIndex, endIndex);
    //     strobe.FrameRate = intervalMs;
    //     strobe.Color = new RGBWColor(colorRGB[0], colorRGB[1], colorRGB[2], 0);
    //     candle.setControl(strobe);
    // }

    public void turnOffLights() {
        configs.LED.BrightnessScalar = kLightsOffBrightness;
        solidColor = new SolidColor(0, ledBuffer.getLength() - 1);
        solidColor.Color = new RGBWColor(0, 0, 0, 0);
        candle.setControl(solidColor);
    }

    public void update_LED_SKBlue() {
        if (skBlueWaveEnabled) {
            runSKBlueWave();
        }
    }

    @Override 
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty(
            "SK Blue Wave Enabled", 
            () -> skBlueWaveEnabled, 
            null);
    }

    @Override
    public void periodic() {

        update_LED_SKBlue();

        SmartDashboard.putData("Lights", this);

    }


}

