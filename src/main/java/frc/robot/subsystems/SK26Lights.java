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

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import static frc.robot.Konstants.LightsConstants.kLightsPWMHeader;
import static frc.robot.Konstants.LightsConstants.kNumLedOnBot;
import static frc.robot.Ports.LightsPorts.kCANdle;
import static frc.robot.Konstants.LightsConstants.kLightsBrightnessScalarOff;
import static frc.robot.Konstants.LightsConstants.kLightsBrightnessScalarOn;

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

    StrobeAnimation strobe;
    SolidColor solidColor;


    public SK26Lights(CANBus canBus) {
        led = new AddressableLED(kLightsPWMHeader);
        ledBuffer = new AddressableLEDBuffer(kNumLedOnBot);
        led.setLength(ledBuffer.getLength());

        // Configure CANdle
        configs = new CANdleConfiguration();
        configs.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        configs.LED.BrightnessScalar = kLightsBrightnessScalarOn;
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

    public void setSolidColor(int[] colorRGB, int startIndex, int endIndex) {
        configs.LED.BrightnessScalar = kLightsBrightnessScalarOn;
        solidColor = new SolidColor(startIndex, endIndex);
        solidColor.Color = new RGBWColor(colorRGB[0], colorRGB[1], colorRGB[2], 0);
        candle.setControl(solidColor);
    }

    public void setStrobeAnimation(int[] colorRGB, int startIndex, int endIndex, int intervalMs) {
        configs.LED.BrightnessScalar = kLightsBrightnessScalarOn;
        strobe = new StrobeAnimation(startIndex, endIndex);
        strobe.FrameRate = intervalMs;
        strobe.Color = new RGBWColor(colorRGB[0], colorRGB[1], colorRGB[2], 0);
        candle.setControl(strobe);
    }

    public void turnOffLights() {
        configs.LED.BrightnessScalar = kLightsBrightnessScalarOff;
        solidColor = new SolidColor(0, ledBuffer.getLength() - 1);
        solidColor.Color = new RGBWColor(0, 0, 0, 0);
        candle.setControl(solidColor);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("LED Length", ledBuffer.getLength());
    }
}
