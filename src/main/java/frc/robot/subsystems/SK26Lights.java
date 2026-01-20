package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Konstants.LightsConstants.kLightsPWMHeader;
import static frc.robot.Konstants.LightsConstants.kLEDBufferLength;

public class SK26Lights extends SubsystemBase {
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
    private int[] rgb;

    public SK26Lights(){
        
        led = new AddressableLED(kLightsPWMHeader);

        ledBuffer = new AddressableLEDBuffer(kLEDBufferLength);
        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
    
    }

    public void turnOnLights(int r, int g, int b){

        rgb[0] = r;
        rgb[1] = g;
        rgb[2] = b;

        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        led.setData(ledBuffer);
        led.start();
    }

    public void turnOffLights(){
        led.stop();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putString("Lights RGB", rgb[0] + ", " + rgb[1] + ", " + rgb[2]);
    }

}
