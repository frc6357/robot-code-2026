package frc.robot.subsystems.lights;

import static frc.robot.Konstants.LightsConstants.kSKBlue;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Contains all LED effect implementations.
 * Effects modify an AddressableLEDBuffer directly.
 */
public class LightEffects {
    
    private final Random random = new Random();
    private final LightPatterns patterns;
    
    // Effect state variables
    private double breathePhase = 0.0;
    private int flashCounter = 0;
    private int chasePosition = 0;
    private int meteorPosition = 0;
    private int theaterChaseOffset = 0;
    private int effectTimer = 0;
    
    private int knightRiderPosition = 0;
    private boolean knightRiderForward = true;
    private int lightningTimer = 0;
    private int lightningType = 0;
    private int lightningBoltPos = 0;
    private double[] lightningFade;
    private boolean lightningFlash = false;
    private int cometPosition = 0;
    private double plasmaPhase = 0.0;
    private double heartbeatPhase = 0.0;
    private double oceanPhase = 0.0;
    private double lavaPhase = 0.0;
    private int[] twinkleBuffer;
    
    // New effect variables
    private double auroraPhase = 0.0;
    private double galaxyPhase = 0.0;
    private double neonPhase = 0.0;
    private int[] matrixDrops;
    private int[] matrixSpeeds;
    private double[] fireworksParticles;
    private int[] fireworksColors;
    private double rainbowBreathPhase = 0.0;
    private double discoPhase = 0.0;
    private double cyberPhase = 0.0;
    private int snakePos = 0;
    private int snakeLength = 8;
    private boolean snakeForward = true;
    private double[] rippleWaves;
    private int rippleCenter = -1;
    private double gradientBouncePos = 0.0;
    private double gradientBounceDir = 1.0;
    private double[] sparkPositions;
    private double[] sparkVelocities;
    private double sunsetPhase = 0.0;
    private double northernPhase = 0.0;
    private int pacmanPos = 0;
    private int[] pacmanGhosts;
    private double soundWavePhase = 0.0;
    private double dnaPhase = 0.0;
    private double portalPhase = 0.0;
    private double spiralPhase = 0.0;
    private int[] pixelDrops;
    private int[] pixelColors;
    private double[] fireflyPositions;
    private double[] fireflyBrightness;
    private int nyanPos = 0;
    private int[] racingPositions;
    private int[] dripPositions;
    private int[] dripLengths;
    
    private static final double BREATHE_SPEED = 0.05;
    private static final int FLASH_PERIOD = 10;
    
    public LightEffects(LightPatterns patterns) {
        this.patterns = patterns;
    }
    
    public void resetBreathePhase() {
        breathePhase = 0.0;
    }
    
    // ==================== BASIC EFFECTS ====================
    
    public void applyBreathingSKBlue(AddressableLEDBuffer buffer) {
        breathePhase += BREATHE_SPEED;
        if (breathePhase > Math.PI * 2) {
            breathePhase -= Math.PI * 2;
        }

        double breatheAmount = (Math.sin(breathePhase) + 1.0) / 2.0;
        double minBrightness = 0.1;
        double breatheBrightness = minBrightness + (breatheAmount * (1.0 - minBrightness));

        int r = (int)(kSKBlue.red * 255 * breatheBrightness);
        int g = (int)(kSKBlue.green * 255 * breatheBrightness);
        int b = (int)(kSKBlue.blue * 255 * breatheBrightness);

        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
    }
    
    public void applyAllianceGradient(AddressableLEDBuffer buffer) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red) {
            patterns.redGradient.applyTo(buffer);
        } else {
            patterns.blueGradient.applyTo(buffer);
        }
    }

    public void applyAllianceSolid(AddressableLEDBuffer buffer) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red) {
            patterns.red.applyTo(buffer);
        } else {
            patterns.blue.applyTo(buffer);
        }
    }

    public void applyAllianceFlashWhite(AddressableLEDBuffer buffer) {
        flashCounter++;
        if (flashCounter >= FLASH_PERIOD * 2) {
            flashCounter = 0;
        }
        
        if (flashCounter < FLASH_PERIOD) {
            patterns.white.applyTo(buffer);
        } else {
            applyAllianceSolid(buffer);
        }
    }
    
    // ==================== CLASSIC EFFECTS ====================

    public void applyFire(AddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) {
            int flicker = random.nextInt(80);
            int r = Math.min(255, 200 + random.nextInt(55));
            int g = Math.max(0, 60 - flicker);
            int b = 0;
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyPolice(AddressableLEDBuffer buffer) {
        effectTimer++;
        if (effectTimer >= 60) {
            effectTimer = 0;
        }
        
        int half = buffer.getLength() / 2;
        boolean phase = (effectTimer / 5) % 2 == 0;
        
        for (int i = 0; i < buffer.getLength(); i++) {
            if (i < half) {
                if (phase) {
                    buffer.setRGB(i, 255, 0, 0);
                } else {
                    buffer.setRGB(i, 0, 0, 50);
                }
            } else {
                if (phase) {
                    buffer.setRGB(i, 0, 0, 50);
                } else {
                    buffer.setRGB(i, 0, 0, 255);
                }
            }
        }
    }

    public void applySparkle(AddressableLEDBuffer buffer) {
        patterns.skBlueGradient.applyTo(buffer);
        
        int sparkleCount = Math.max(1, buffer.getLength() / 10);
        for (int s = 0; s < sparkleCount; s++) {
            if (random.nextInt(100) < 30) {
                int pos = random.nextInt(buffer.getLength());
                buffer.setRGB(pos, 255, 255, 255);
            }
        }
    }

    public void applyColorChase(AddressableLEDBuffer buffer) {
        chasePosition++;
        if (chasePosition >= buffer.getLength()) {
            chasePosition = 0;
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 30);
        }
        
        int chaseLength = 5;
        for (int i = 0; i < chaseLength; i++) {
            int pos = (chasePosition + i) % buffer.getLength();
            int brightness = 255 - (i * 50);
            buffer.setRGB(pos, 0, brightness, brightness);
        }
    }

    public void applyMeteor(AddressableLEDBuffer buffer) {
        meteorPosition++;
        if (meteorPosition >= buffer.getLength() + 15) {
            meteorPosition = 0;
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            int r = Math.max(0, buffer.getRed(i) - 20);
            int g = Math.max(0, buffer.getGreen(i) - 20);
            int b = Math.max(0, buffer.getBlue(i) - 20);
            buffer.setRGB(i, r, g, b);
        }
        
        int meteorLength = 8;
        for (int i = 0; i < meteorLength; i++) {
            int pos = meteorPosition - i;
            if (pos >= 0 && pos < buffer.getLength()) {
                int brightness = 255 - (i * 30);
                buffer.setRGB(pos, brightness, brightness, brightness);
            }
        }
    }

    public void applyTheaterChase(AddressableLEDBuffer buffer) {
        theaterChaseOffset++;
        if (theaterChaseOffset >= 3) {
            theaterChaseOffset = 0;
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            if ((i + theaterChaseOffset) % 3 == 0) {
                double hue = (double)(i + effectTimer) / buffer.getLength();
                int r = (int)(Math.sin(hue * Math.PI * 2) * 127 + 128);
                int g = (int)(Math.sin(hue * Math.PI * 2 + 2.094) * 127 + 128);
                int b = (int)(Math.sin(hue * Math.PI * 2 + 4.188) * 127 + 128);
                buffer.setRGB(i, r, g, b);
            } else {
                buffer.setRGB(i, 0, 0, 0);
            }
        }
        effectTimer++;
    }

    public void applyLavaLamp(AddressableLEDBuffer buffer) {
        lavaPhase += 0.02;
        for (int i = 0; i < buffer.getLength(); i++) {
            double pos = (double) i / buffer.getLength();
            double wave1 = Math.sin(pos * 4 + lavaPhase);
            double wave2 = Math.sin(pos * 6 - lavaPhase * 0.7);
            double wave3 = Math.sin(pos * 2 + lavaPhase * 1.3);
            double combined = (wave1 + wave2 + wave3) / 3.0;
            combined = (combined + 1.0) / 2.0;
            
            int r = (int)(255 * combined);
            int g = (int)(50 * combined * combined);
            int b = (int)(100 * (1.0 - combined));
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyOceanWave(AddressableLEDBuffer buffer) {
        oceanPhase += 0.03;
        for (int i = 0; i < buffer.getLength(); i++) {
            double pos = (double) i / buffer.getLength();
            double wave1 = Math.sin(pos * 8 + oceanPhase) * 0.5 + 0.5;
            double wave2 = Math.sin(pos * 12 - oceanPhase * 1.5) * 0.3 + 0.5;
            double wave3 = Math.sin(pos * 4 + oceanPhase * 0.5) * 0.2 + 0.5;
            double combined = (wave1 + wave2 + wave3) / 3.0;
            
            int r = 0;
            int g = (int)(100 * combined + 50);
            int b = (int)(200 * combined + 55);
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyTwinkleStars(AddressableLEDBuffer buffer) {
        if (twinkleBuffer == null || twinkleBuffer.length != buffer.getLength()) {
            twinkleBuffer = new int[buffer.getLength()];
            for (int i = 0; i < twinkleBuffer.length; i++) {
                twinkleBuffer[i] = random.nextInt(255);
            }
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            if (random.nextInt(100) < 5) {
                twinkleBuffer[i] = 255;
            } else {
                twinkleBuffer[i] = Math.max(0, twinkleBuffer[i] - 5);
            }
            
            int brightness = twinkleBuffer[i];
            int r = brightness;
            int g = brightness;
            int b = (int)(brightness * 0.8 + 50);
            buffer.setRGB(i, r, g, Math.min(255, b));
        }
    }

    public void applyCandyCane(AddressableLEDBuffer buffer) {
        effectTimer++;
        int stripeWidth = 4;
        int offset = (effectTimer / 3) % (stripeWidth * 2);
        
        for (int i = 0; i < buffer.getLength(); i++) {
            int pos = (i + offset) % (stripeWidth * 2);
            if (pos < stripeWidth) {
                buffer.setRGB(i, 255, 0, 0);
            } else {
                buffer.setRGB(i, 255, 255, 255);
            }
        }
    }

    public void applyPlasma(AddressableLEDBuffer buffer) {
        plasmaPhase += 0.015;
        for (int i = 0; i < buffer.getLength(); i++) {
            double x = (double) i / buffer.getLength();
            double v1 = Math.sin(x * 6 + plasmaPhase);
            double v2 = Math.sin(x * 4 - plasmaPhase * 0.5);
            double v3 = Math.sin(x * 8 + plasmaPhase * 0.3);
            double v = (v1 + v2 + v3) / 3.0;
            
            int r = (int)((Math.sin(v * Math.PI) + 1) * 127);
            int g = (int)((Math.sin(v * Math.PI + 2.094) + 1) * 127);
            int b = (int)((Math.sin(v * Math.PI + 4.188) + 1) * 127);
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyKnightRider(AddressableLEDBuffer buffer) {
        if (knightRiderForward) {
            knightRiderPosition++;
            if (knightRiderPosition >= buffer.getLength() - 1) {
                knightRiderForward = false;
            }
        } else {
            knightRiderPosition--;
            if (knightRiderPosition <= 0) {
                knightRiderForward = true;
            }
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            int distance = Math.abs(i - knightRiderPosition);
            int brightness = Math.max(0, 255 - distance * 40);
            buffer.setRGB(i, brightness, 0, 0);
        }
    }

    public void applyHeartbeat(AddressableLEDBuffer buffer) {
        heartbeatPhase += 0.04;
        double cycle = heartbeatPhase % 4.0;
        double beat = 0;
        
        if (cycle < 0.2) {
            beat = Math.sin(cycle * Math.PI / 0.2) * 0.6;
        }
        else if (cycle >= 0.3 && cycle < 0.6) {
            beat = Math.sin((cycle - 0.3) * Math.PI / 0.3);
        }
        else if (cycle >= 0.6 && cycle < 1.5) {
            double fadeProgress = (cycle - 0.6) / 0.9;
            beat = Math.max(0, (1.0 - fadeProgress) * 0.3);
        }
        else {
            beat = 0;
        }
        
        beat = Math.max(0, Math.min(1, beat));
        
        int r = (int)(255 * beat);
        int g = (int)(20 * beat);
        int b = (int)(60 * beat);
        
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyLightning(AddressableLEDBuffer buffer) {
        if (lightningFade == null || lightningFade.length != buffer.getLength()) {
            lightningFade = new double[buffer.getLength()];
        }
        
        lightningTimer++;
        
        for (int i = 0; i < lightningFade.length; i++) {
            lightningFade[i] *= 0.85;
        }
        
        if (!lightningFlash && random.nextInt(100) < 3) {
            lightningFlash = true;
            lightningTimer = 0;
            lightningType = random.nextInt(3);
            lightningBoltPos = random.nextInt(buffer.getLength());
        }
        
        if (lightningFlash) {
            switch (lightningType) {
                case 0:
                    if (lightningTimer < 2) {
                        for (int i = 0; i < lightningFade.length; i++) {
                            lightningFade[i] = 1.0;
                        }
                    } else if (lightningTimer > 5) {
                        lightningFlash = false;
                    }
                    break;
                    
                case 1:
                    if (lightningTimer < 15) {
                        int spread = lightningTimer * 2;
                        for (int i = -spread; i <= spread; i++) {
                            int pos = lightningBoltPos + i;
                            if (pos >= 0 && pos < lightningFade.length) {
                                double intensity = 1.0 - (Math.abs(i) / (double)(spread + 1)) * 0.5;
                                if (random.nextInt(100) < 80) {
                                    lightningFade[pos] = Math.max(lightningFade[pos], intensity);
                                }
                            }
                        }
                        if (lightningTimer > 5 && random.nextInt(100) < 30) {
                            int flickerPos = random.nextInt(buffer.getLength());
                            lightningFade[flickerPos] = 0.7;
                        }
                    } else if (lightningTimer > 25) {
                        lightningFlash = false;
                    }
                    break;
                    
                case 2:
                    if (lightningTimer < 3) {
                        for (int i = 0; i < lightningFade.length; i++) {
                            lightningFade[i] = 1.0;
                        }
                    } else if (lightningTimer == 6 || lightningTimer == 8) {
                        for (int i = 0; i < lightningFade.length; i++) {
                            if (random.nextInt(100) < 60) {
                                lightningFade[i] = 0.8;
                            }
                        }
                    } else if (lightningTimer > 12) {
                        lightningFlash = false;
                    }
                    break;
            }
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            int baseR = 8;
            int baseG = 5;
            int baseB = 20;
            
            double flash = lightningFade[i];
            int r = (int)(baseR + (255 - baseR) * flash);
            int g = (int)(baseG + (255 - baseG) * flash);
            int b = (int)(baseB + (255 - baseB) * flash);
            
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyConfetti(AddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) {
            int r = Math.max(0, buffer.getRed(i) - 15);
            int g = Math.max(0, buffer.getGreen(i) - 15);
            int b = Math.max(0, buffer.getBlue(i) - 15);
            buffer.setRGB(i, r, g, b);
        }
        
        if (random.nextInt(100) < 40) {
            int pos = random.nextInt(buffer.getLength());
            int colorChoice = random.nextInt(6);
            switch (colorChoice) {
                case 0: buffer.setRGB(pos, 255, 0, 0); break;
                case 1: buffer.setRGB(pos, 0, 255, 0); break;
                case 2: buffer.setRGB(pos, 0, 0, 255); break;
                case 3: buffer.setRGB(pos, 255, 255, 0); break;
                case 4: buffer.setRGB(pos, 255, 0, 255); break;
                case 5: buffer.setRGB(pos, 0, 255, 255); break;
            }
        }
    }

    public void applyCometTrail(AddressableLEDBuffer buffer) {
        cometPosition++;
        if (cometPosition >= buffer.getLength() * 2) {
            cometPosition = 0;
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            int r = Math.max(0, (int)(buffer.getRed(i) * 0.9));
            int g = Math.max(0, (int)(buffer.getGreen(i) * 0.85));
            int b = Math.max(0, (int)(buffer.getBlue(i) * 0.8));
            buffer.setRGB(i, r, g, b);
        }
        
        int headPos = cometPosition % buffer.getLength();
        boolean forward = (cometPosition / buffer.getLength()) % 2 == 0;
        if (!forward) {
            headPos = buffer.getLength() - 1 - headPos;
        }
        
        if (headPos >= 0 && headPos < buffer.getLength()) {
            buffer.setRGB(headPos, 255, 200, 100);
        }
    }

    // ==================== AWESOME EFFECTS ====================

    public void applyAuroraBorealis(AddressableLEDBuffer buffer) {
        auroraPhase += 0.02;
        for (int i = 0; i < buffer.getLength(); i++) {
            double pos = (double) i / buffer.getLength();
            double wave1 = Math.sin(pos * 3 + auroraPhase) * 0.5 + 0.5;
            double wave2 = Math.sin(pos * 5 - auroraPhase * 0.7) * 0.3 + 0.5;
            double wave3 = Math.sin(pos * 2 + auroraPhase * 1.2) * 0.4 + 0.5;
            
            double blend = (wave1 + wave2) / 2.0;
            int r = (int)(100 * wave3 + 50 * blend);
            int g = (int)(255 * wave1 * 0.8);
            int b = (int)(200 * wave2 + 100 * wave3);
            
            buffer.setRGB(i, Math.min(255, r), Math.min(255, g), Math.min(255, b));
        }
    }

    public void applyGalaxySwirl(AddressableLEDBuffer buffer) {
        galaxyPhase += 0.025;
        for (int i = 0; i < buffer.getLength(); i++) {
            double pos = (double) i / buffer.getLength();
            double angle = pos * Math.PI * 4 + galaxyPhase;
            double spiral = Math.sin(angle) * 0.5 + 0.5;
            double sparkle = random.nextInt(100) < 5 ? 1.0 : 0.0;
            
            int r = (int)(80 * spiral + 175 * sparkle);
            int g = (int)(30 * spiral + 175 * sparkle);
            int b = (int)(180 * spiral + 75 * sparkle);
            
            buffer.setRGB(i, Math.min(255, r), Math.min(255, g), Math.min(255, b));
        }
    }

    public void applyNeonPulse(AddressableLEDBuffer buffer) {
        neonPhase += 0.08;
        double pulse = (Math.sin(neonPhase) + 1) / 2.0;
        int colorCycle = ((int)(neonPhase / 2)) % 4;
        
        for (int i = 0; i < buffer.getLength(); i++) {
            double localPulse = pulse * (0.7 + 0.3 * Math.sin((double)i / 3 + neonPhase));
            int r = 0, g = 0, b = 0;
            
            switch (colorCycle) {
                case 0: r = (int)(255 * localPulse); g = 0; b = (int)(255 * localPulse); break;
                case 1: r = 0; g = (int)(255 * localPulse); b = (int)(255 * localPulse); break;
                case 2: r = (int)(255 * localPulse); g = (int)(255 * localPulse); b = 0; break;
                case 3: r = 0; g = (int)(255 * localPulse); b = 0; break;
            }
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyMatrixRain(AddressableLEDBuffer buffer) {
        if (matrixDrops == null || matrixDrops.length != buffer.getLength()) {
            matrixDrops = new int[buffer.getLength()];
            matrixSpeeds = new int[buffer.getLength()];
            for (int i = 0; i < matrixDrops.length; i++) {
                matrixDrops[i] = random.nextInt(255);
                matrixSpeeds[i] = random.nextInt(15) + 5;
            }
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            matrixDrops[i] -= matrixSpeeds[i];
            if (matrixDrops[i] < 0) {
                matrixDrops[i] = 255;
                matrixSpeeds[i] = random.nextInt(15) + 5;
            }
            
            int brightness = Math.max(0, matrixDrops[i]);
            buffer.setRGB(i, 0, brightness, brightness / 4);
        }
    }

    public void applyFireworks(AddressableLEDBuffer buffer) {
        if (fireworksParticles == null) {
            fireworksParticles = new double[buffer.getLength()];
            fireworksColors = new int[buffer.getLength()];
        }
        
        for (int i = 0; i < fireworksParticles.length; i++) {
            fireworksParticles[i] *= 0.92;
        }
        
        if (random.nextInt(100) < 8) {
            int center = random.nextInt(buffer.getLength());
            int color = random.nextInt(6);
            int radius = random.nextInt(10) + 5;
            
            for (int i = -radius; i <= radius; i++) {
                int pos = center + i;
                if (pos >= 0 && pos < buffer.getLength()) {
                    double intensity = 1.0 - Math.abs(i) / (double)radius;
                    fireworksParticles[pos] = Math.max(fireworksParticles[pos], intensity);
                    fireworksColors[pos] = color;
                }
            }
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            double p = fireworksParticles[i];
            int r = 0, g = 0, b = 0;
            switch (fireworksColors[i]) {
                case 0: r = (int)(255 * p); break;
                case 1: g = (int)(255 * p); break;
                case 2: b = (int)(255 * p); break;
                case 3: r = (int)(255 * p); g = (int)(255 * p); break;
                case 4: r = (int)(255 * p); b = (int)(255 * p); break;
                case 5: g = (int)(255 * p); b = (int)(255 * p); break;
            }
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyBreathingRainbow(AddressableLEDBuffer buffer) {
        rainbowBreathPhase += 0.03;
        double breath = (Math.sin(rainbowBreathPhase) + 1) / 2.0;
        breath = 0.3 + breath * 0.7;
        
        for (int i = 0; i < buffer.getLength(); i++) {
            double hue = ((double)i / buffer.getLength() + rainbowBreathPhase * 0.1) % 1.0;
            int r = (int)(255 * breath * (Math.sin(hue * Math.PI * 2) * 0.5 + 0.5));
            int g = (int)(255 * breath * (Math.sin(hue * Math.PI * 2 + 2.094) * 0.5 + 0.5));
            int b = (int)(255 * breath * (Math.sin(hue * Math.PI * 2 + 4.188) * 0.5 + 0.5));
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyWaveCollision(AddressableLEDBuffer buffer) {
        effectTimer++;
        int len = buffer.getLength();
        
        for (int i = 0; i < len; i++) {
            double leftWave = Math.sin((double)(i + effectTimer) / 5.0) * 0.5 + 0.5;
            double rightWave = Math.sin((double)(len - i + effectTimer) / 5.0) * 0.5 + 0.5;
            
            double collision = leftWave * rightWave;
            double centerBoost = 1.0 + Math.exp(-Math.pow((i - len/2.0) / (len/4.0), 2)) * collision;
            
            int r = (int)(255 * leftWave * centerBoost);
            int g = (int)(100 * collision * centerBoost);
            int b = (int)(255 * rightWave * centerBoost);
            
            buffer.setRGB(i, Math.min(255, r), Math.min(255, g), Math.min(255, b));
        }
    }

    public void applyDiscoBall(AddressableLEDBuffer buffer) {
        discoPhase += 0.15;
        
        for (int i = 0; i < buffer.getLength(); i++) {
            double spotlight = Math.sin(discoPhase + i * 0.3) * 0.5 + 0.5;
            spotlight = Math.pow(spotlight, 3);
            
            int colorIndex = ((int)(discoPhase / 0.5) + i / 3) % 6;
            int r = 0, g = 0, b = 0;
            
            switch (colorIndex) {
                case 0: r = 255; break;
                case 1: r = 255; g = 255; break;
                case 2: g = 255; break;
                case 3: g = 255; b = 255; break;
                case 4: b = 255; break;
                case 5: r = 255; b = 255; break;
            }
            
            buffer.setRGB(i, (int)(r * spotlight), (int)(g * spotlight), (int)(b * spotlight));
        }
    }

    public void applyCyberpunk(AddressableLEDBuffer buffer) {
        cyberPhase += 0.04;
        
        for (int i = 0; i < buffer.getLength(); i++) {
            double pos = (double) i / buffer.getLength();
            
            double glitch = random.nextInt(100) < 3 ? random.nextDouble() : 0;
            double neon1 = Math.sin(pos * 10 + cyberPhase) * 0.5 + 0.5;
            double neon2 = Math.sin(pos * 15 - cyberPhase * 1.5) * 0.5 + 0.5;
            
            int r = (int)(255 * (neon1 * 0.8 + glitch));
            int g = (int)(50 * neon2);
            int b = (int)(255 * (neon2 * 0.9 + glitch * 0.5));
            
            buffer.setRGB(i, Math.min(255, r), Math.min(255, g), Math.min(255, b));
        }
    }

    public void applySnakeGame(AddressableLEDBuffer buffer) {
        effectTimer++;
        if (effectTimer % 3 == 0) {
            if (snakeForward) {
                snakePos++;
                if (snakePos >= buffer.getLength() - snakeLength) {
                    snakeForward = false;
                }
            } else {
                snakePos--;
                if (snakePos <= 0) {
                    snakeForward = true;
                    snakeLength = Math.min(snakeLength + 1, 20);
                }
            }
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 20, 0);
        }
        
        for (int i = 0; i < snakeLength; i++) {
            int pos = snakePos + i;
            if (pos >= 0 && pos < buffer.getLength()) {
                int brightness = 255 - (i * 10);
                buffer.setRGB(pos, 0, brightness, 0);
            }
        }
        
        if (random.nextInt(100) < 5) {
            int foodPos = random.nextInt(buffer.getLength());
            buffer.setRGB(foodPos, 255, 0, 0);
        }
    }

    public void applyRipple(AddressableLEDBuffer buffer) {
        if (rippleWaves == null || rippleWaves.length != buffer.getLength()) {
            rippleWaves = new double[buffer.getLength()];
        }
        
        for (int i = 0; i < rippleWaves.length; i++) {
            rippleWaves[i] *= 0.95;
        }
        
        if (random.nextInt(100) < 5) {
            rippleCenter = random.nextInt(buffer.getLength());
        }
        
        if (rippleCenter >= 0) {
            effectTimer++;
            int radius = effectTimer % 30;
            
            for (int offset = -1; offset <= 1; offset += 2) {
                int pos = rippleCenter + radius * offset;
                if (pos >= 0 && pos < rippleWaves.length) {
                    rippleWaves[pos] = 1.0;
                }
            }
            
            if (radius >= 30) {
                rippleCenter = -1;
                effectTimer = 0;
            }
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            int b = (int)(255 * rippleWaves[i]);
            int g = (int)(150 * rippleWaves[i]);
            buffer.setRGB(i, 0, g, b);
        }
    }

    public void applyGradientBounce(AddressableLEDBuffer buffer) {
        gradientBouncePos += 0.02 * gradientBounceDir;
        if (gradientBouncePos >= 1.0 || gradientBouncePos <= 0.0) {
            gradientBounceDir *= -1;
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            double pos = (double) i / buffer.getLength();
            double dist = Math.abs(pos - gradientBouncePos);
            double intensity = Math.max(0, 1.0 - dist * 3);
            
            double hue = pos + gradientBouncePos;
            int r = (int)(255 * intensity * (Math.sin(hue * Math.PI * 2) * 0.5 + 0.5));
            int g = (int)(255 * intensity * (Math.sin(hue * Math.PI * 2 + 2.094) * 0.5 + 0.5));
            int b = (int)(255 * intensity * (Math.sin(hue * Math.PI * 2 + 4.188) * 0.5 + 0.5));
            
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyElectricSparks(AddressableLEDBuffer buffer) {
        if (sparkPositions == null) {
            sparkPositions = new double[10];
            sparkVelocities = new double[10];
            for (int i = 0; i < 10; i++) {
                sparkPositions[i] = random.nextDouble() * buffer.getLength();
                sparkVelocities[i] = (random.nextDouble() - 0.5) * 4;
            }
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            int r = Math.max(0, buffer.getRed(i) - 30);
            int g = Math.max(0, buffer.getGreen(i) - 30);
            int b = Math.max(0, buffer.getBlue(i) - 30);
            buffer.setRGB(i, r, g, b);
        }
        
        for (int s = 0; s < sparkPositions.length; s++) {
            sparkPositions[s] += sparkVelocities[s];
            sparkVelocities[s] += (random.nextDouble() - 0.5) * 0.5;
            
            if (sparkPositions[s] < 0 || sparkPositions[s] >= buffer.getLength()) {
                sparkPositions[s] = random.nextDouble() * buffer.getLength();
                sparkVelocities[s] = (random.nextDouble() - 0.5) * 4;
            }
            
            int pos = (int) sparkPositions[s];
            if (pos >= 0 && pos < buffer.getLength()) {
                buffer.setRGB(pos, 200, 200, 255);
                if (pos > 0) buffer.setRGB(pos - 1, 100, 100, 200);
                if (pos < buffer.getLength() - 1) buffer.setRGB(pos + 1, 100, 100, 200);
            }
        }
    }

    public void applySunset(AddressableLEDBuffer buffer) {
        sunsetPhase += 0.01;
        
        for (int i = 0; i < buffer.getLength(); i++) {
            double pos = (double) i / buffer.getLength();
            double wave = Math.sin(pos * Math.PI + sunsetPhase * 0.5) * 0.3;
            
            double blend = pos + wave;
            int r, g, b;
            
            if (blend < 0.33) {
                double t = blend / 0.33;
                r = (int)(180 + 75 * t);
                g = (int)(50 * t);
                b = (int)(100 * (1 - t));
            } else if (blend < 0.66) {
                double t = (blend - 0.33) / 0.33;
                r = 255;
                g = (int)(50 + 150 * t);
                b = 0;
            } else {
                double t = (blend - 0.66) / 0.34;
                r = (int)(255 * (1 - t * 0.3));
                g = (int)(200 * (1 - t));
                b = (int)(150 * t);
            }
            
            buffer.setRGB(i, Math.min(255, Math.max(0, r)), Math.min(255, Math.max(0, g)), Math.min(255, Math.max(0, b)));
        }
    }

    public void applyNorthernLights(AddressableLEDBuffer buffer) {
        northernPhase += 0.015;
        
        for (int i = 0; i < buffer.getLength(); i++) {
            double pos = (double) i / buffer.getLength();
            
            double curtain1 = Math.sin(pos * 5 + northernPhase) * 0.5 + 0.5;
            double curtain2 = Math.sin(pos * 7 - northernPhase * 0.8) * 0.3 + 0.5;
            double curtain3 = Math.sin(pos * 3 + northernPhase * 1.2) * 0.4 + 0.5;
            double shimmer = random.nextInt(100) < 10 ? 0.2 : 0;
            
            int r = (int)((80 * curtain3 + 40 * shimmer));
            int g = (int)(200 * curtain1 + 55 * curtain2);
            int b = (int)(150 * curtain2 + 50 * curtain1);
            
            buffer.setRGB(i, Math.min(255, r), Math.min(255, g), Math.min(255, b));
        }
    }

    public void applyPacman(AddressableLEDBuffer buffer) {
        effectTimer++;
        if (effectTimer % 2 == 0) {
            pacmanPos++;
            if (pacmanPos >= buffer.getLength() + 10) {
                pacmanPos = -10;
            }
        }
        
        if (pacmanGhosts == null) {
            pacmanGhosts = new int[4];
            for (int i = 0; i < 4; i++) {
                pacmanGhosts[i] = random.nextInt(buffer.getLength());
            }
        }
        
        for (int i = 0; i < pacmanGhosts.length; i++) {
            if (random.nextInt(100) < 30) {
                pacmanGhosts[i] += random.nextInt(3) - 1;
                pacmanGhosts[i] = Math.max(0, Math.min(buffer.getLength() - 1, pacmanGhosts[i]));
            }
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            if (i % 4 == 0 && i > pacmanPos) {
                buffer.setRGB(i, 255, 255, 200);
            } else {
                buffer.setRGB(i, 0, 0, 30);
            }
        }
        
        if (pacmanPos >= 0 && pacmanPos < buffer.getLength()) {
            buffer.setRGB(pacmanPos, 255, 255, 0);
            if (pacmanPos > 0) buffer.setRGB(pacmanPos - 1, 200, 200, 0);
        }
        
        int[][] ghostColors = {{255, 0, 0}, {255, 180, 255}, {0, 255, 255}, {255, 180, 0}};
        for (int i = 0; i < pacmanGhosts.length; i++) {
            int pos = pacmanGhosts[i];
            if (pos >= 0 && pos < buffer.getLength()) {
                buffer.setRGB(pos, ghostColors[i][0], ghostColors[i][1], ghostColors[i][2]);
            }
        }
    }

    public void applySoundWave(AddressableLEDBuffer buffer) {
        soundWavePhase += 0.1;
        int center = buffer.getLength() / 2;
        
        for (int i = 0; i < buffer.getLength(); i++) {
            double dist = Math.abs(i - center);
            double wave = Math.sin(soundWavePhase - dist * 0.3) * 0.5 + 0.5;
            double envelope = Math.exp(-dist * 0.05);
            double intensity = wave * envelope;
            
            int r = (int)(255 * intensity * (1 - envelope * 0.5));
            int g = (int)(255 * intensity * envelope);
            int b = (int)(150 * intensity);
            
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyDNAHelix(AddressableLEDBuffer buffer) {
        dnaPhase += 0.05;
        
        for (int i = 0; i < buffer.getLength(); i++) {
            double pos = (double) i / buffer.getLength() * Math.PI * 4;
            
            double helix1 = Math.sin(pos + dnaPhase);
            double helix2 = Math.sin(pos + dnaPhase + Math.PI);
            
            double bright1 = (helix1 + 1) / 2.0;
            double bright2 = (helix2 + 1) / 2.0;
            
            int r = (int)(255 * bright1 * 0.8);
            int g = (int)(100 * Math.max(bright1, bright2));
            int b = (int)(255 * bright2 * 0.8);
            
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyPortal(AddressableLEDBuffer buffer) {
        portalPhase += 0.08;
        int center = buffer.getLength() / 2;
        
        for (int i = 0; i < buffer.getLength(); i++) {
            double dist = Math.abs(i - center);
            double normalDist = dist / (buffer.getLength() / 2.0);
            
            double swirl = Math.sin(normalDist * 10 - portalPhase * 2) * 0.5 + 0.5;
            double glow = Math.exp(-normalDist * 2);
            
            int r = (int)(255 * swirl * glow);
            int g = (int)(100 * swirl * glow);
            int b = (int)(255 * (1 - swirl) * glow);
            
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyHypnoticSpiral(AddressableLEDBuffer buffer) {
        spiralPhase += 0.06;
        
        for (int i = 0; i < buffer.getLength(); i++) {
            double pos = (double) i / buffer.getLength();
            double spiral = Math.sin(pos * 20 - spiralPhase) * 0.5 + 0.5;
            
            double tint = Math.sin(spiralPhase * 0.3) * 0.5 + 0.5;
            int r = (int)(255 * spiral * tint);
            int g = (int)(255 * spiral * (1 - tint * 0.5));
            int b = (int)(255 * spiral * (1 - tint));
            
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyPixelRain(AddressableLEDBuffer buffer) {
        if (pixelDrops == null || pixelDrops.length != buffer.getLength()) {
            pixelDrops = new int[buffer.getLength()];
            pixelColors = new int[buffer.getLength()];
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            pixelDrops[i] = Math.max(0, pixelDrops[i] - 8);
        }
        
        if (random.nextInt(100) < 20) {
            int pos = random.nextInt(buffer.getLength());
            pixelDrops[pos] = 255;
            pixelColors[pos] = random.nextInt(6);
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            int bright = pixelDrops[i];
            int r = 0, g = 0, b = 0;
            switch (pixelColors[i]) {
                case 0: r = bright; break;
                case 1: g = bright; break;
                case 2: b = bright; break;
                case 3: r = bright; g = bright; break;
                case 4: r = bright; b = bright; break;
                case 5: g = bright; b = bright; break;
            }
            buffer.setRGB(i, r, g, b);
        }
    }

    public void applyFireflies(AddressableLEDBuffer buffer) {
        if (fireflyPositions == null) {
            fireflyPositions = new double[15];
            fireflyBrightness = new double[15];
            for (int i = 0; i < 15; i++) {
                fireflyPositions[i] = random.nextDouble() * buffer.getLength();
                fireflyBrightness[i] = random.nextDouble();
            }
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 5, 0);
        }
        
        for (int f = 0; f < fireflyPositions.length; f++) {
            fireflyPositions[f] += (random.nextDouble() - 0.5) * 0.5;
            if (fireflyPositions[f] < 0) fireflyPositions[f] = buffer.getLength() - 1;
            if (fireflyPositions[f] >= buffer.getLength()) fireflyPositions[f] = 0;
            
            fireflyBrightness[f] += (random.nextDouble() - 0.5) * 0.1;
            fireflyBrightness[f] = Math.max(0, Math.min(1, fireflyBrightness[f]));
            
            int pos = (int) fireflyPositions[f];
            int glow = (int)(255 * fireflyBrightness[f]);
            int r = (int)(glow * 0.9);
            int g = glow;
            int b = (int)(glow * 0.3);
            
            if (pos >= 0 && pos < buffer.getLength()) {
                buffer.setRGB(pos, r, g, b);
            }
        }
    }

    public void applyNyanCat(AddressableLEDBuffer buffer) {
        effectTimer++;
        nyanPos++;
        if (nyanPos >= buffer.getLength() + 15) {
            nyanPos = -10;
        }
        
        int[] rainbowColors = {
            0xFF0000, 0xFF7F00, 0xFFFF00, 0x00FF00, 0x0000FF, 0x4B0082
        };
        
        for (int i = 0; i < buffer.getLength(); i++) {
            if (i < nyanPos && i > nyanPos - 20) {
                int colorIndex = (i + effectTimer / 2) % rainbowColors.length;
                int color = rainbowColors[colorIndex];
                buffer.setRGB(i, (color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);
            } else {
                if (random.nextInt(100) < 2) {
                    buffer.setRGB(i, 255, 255, 255);
                } else {
                    buffer.setRGB(i, 0, 0, 40);
                }
            }
        }
        
        if (nyanPos >= 0 && nyanPos < buffer.getLength()) {
            buffer.setRGB(nyanPos, 255, 150, 200);
        }
    }

    public void applyRacingStripes(AddressableLEDBuffer buffer) {
        if (racingPositions == null) {
            racingPositions = new int[5];
            for (int i = 0; i < 5; i++) {
                racingPositions[i] = i * 10;
            }
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 20, 20, 20);
        }
        
        int[][] colors = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255}};
        for (int r = 0; r < racingPositions.length; r++) {
            racingPositions[r] += (r + 1);
            if (racingPositions[r] >= buffer.getLength() + 5) {
                racingPositions[r] = -5;
            }
            
            for (int j = 0; j < 4; j++) {
                int pos = racingPositions[r] + j;
                if (pos >= 0 && pos < buffer.getLength()) {
                    int fade = 255 - (j * 50);
                    buffer.setRGB(pos, 
                        colors[r][0] * fade / 255,
                        colors[r][1] * fade / 255,
                        colors[r][2] * fade / 255);
                }
            }
        }
    }

    public void applyDrip(AddressableLEDBuffer buffer) {
        if (dripPositions == null) {
            dripPositions = new int[8];
            dripLengths = new int[8];
            for (int i = 0; i < 8; i++) {
                dripPositions[i] = random.nextInt(buffer.getLength());
                dripLengths[i] = 0;
            }
        }
        
        for (int i = 0; i < buffer.getLength(); i++) {
            int r = Math.max(0, buffer.getRed(i) - 10);
            int g = Math.max(0, buffer.getGreen(i) - 10);
            int b = Math.max(0, buffer.getBlue(i) - 10);
            buffer.setRGB(i, r, g, b);
        }
        
        for (int d = 0; d < dripPositions.length; d++) {
            dripLengths[d]++;
            
            if (dripLengths[d] > 15) {
                dripPositions[d] = random.nextInt(buffer.getLength());
                dripLengths[d] = 0;
            }
            
            for (int j = 0; j < dripLengths[d]; j++) {
                int pos = dripPositions[d] + j;
                if (pos >= 0 && pos < buffer.getLength()) {
                    int bright = 255 - (j * 15);
                    buffer.setRGB(pos, 0, bright / 2, bright);
                }
            }
        }
    }
}
