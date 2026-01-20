package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import static frc.robot.Konstants.LightsConstants.kLightsPWMHeader;
import static frc.robot.Konstants.LightsConstants.kLEDBufferLength;

public class SK26Lights {
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

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    public SK26Lights(){
        
        led = new AddressableLED(kLightsPWMHeader);

        ledBuffer = new AddressableLEDBuffer(kLEDBufferLength);
        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
    
    }

    public void turnOnLights(int r, int g, int b){

        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        led.setData(ledBuffer);
        led.start();
    }

}
