# Adding New Light Effects | SK26 Lights Guide

This guide explains how to add new LED effects to the SK26 robot, either through SmartDashboard or controller button bindings.

---

## Table of Contents

1. [Quick Overview](#quick-overview)
2. [Method 1: SmartDashboard Fun Mode](#method-1-smartdashboard-fun-mode)
3. [Method 2: Controller Button Bindings](#method-2-controller-button-bindings)
4. [Creating Custom Effects](#creating-custom-effects)
5. [File Reference](#file-reference)
6. [Example: Complete Walkthrough](#example-complete-walkthrough)

---

## Quick Overview

The lights system has **4 main files** you need to know:

| File | Purpose |
|------|---------|
| `LightMode.java` | Enum of all available effect names |
| `LightPatterns.java` | Simple WPILib LED patterns (solid colors, gradients) |
| `LightEffects.java` | Complex animated effects (fire, plasma, etc.) |
| `SK26Lights.java` | Main subsystem - connects modes to patterns/effects |
| `SK26LightsBinder.java` | Controller button bindings |

---

## Method 1: SmartDashboard Fun Mode

### Using Existing Effects

1. Open **SmartDashboard** or **Elastic Dashboard**
2. Find `Lights/Fun Mode` and set it to `true`
3. Select an effect from the `Lights/Fun Effects` dropdown
4. The effect will apply immediately!

### Adding a New Effect to the SmartDashboard Dropdown

**Step 1:** Add to `LightMode.java`

```java
public enum LightMode {
    // ...existing modes...
    
    // Add your new mode
    MY_COOL_EFFECT
}
```

**Step 2:** Add to the chooser in `SK26Lights.java`

Find the `setupFunEffectChooser()` method and add:

```java
private void setupFunEffectChooser() {
    // ...existing options...
    
    funEffectChooser.addOption("My Cool Effect", LightMode.MY_COOL_EFFECT);
}
```

**Step 3:** Create the effect implementation

See [Creating Custom Effects](#creating-custom-effects) section below.

**Step 4:** Add to the switch statement in `SK26Lights.java`

Find `applyCurrentMode()` and add your case:

```java
private void applyCurrentMode() {
    switch (currentMode) {
        // ...existing cases...
        
        case MY_COOL_EFFECT: 
            effects.applyMyCoolEffect(m_baseBuffer); 
            break;
    }
}
```

---

## Method 2: Controller Button Bindings

### Current Button Layout (Operator Controller)

| Button | Effect | Command |
|--------|--------|---------|
| X | Off | `setOff()` |
| A | Solid Red | `setSolidRed()` |
| B | Solid Blue | `setSolidBlue()` |
| Y | Alliance Gradient | `setAllianceGradient()` |
| Left Bumper | Solid White | `setSolidWhite()` |
| Right Bumper | Breathing SK Blue | `setBreathingSKBlue()` |
| Start | Party Mode (Rainbow) | `activatePartyMode()` |
| Left Trigger | SK Blue Gradient | `setSKBlueGradient()` |

### Adding a New Button Binding

**Step 1:** Create a command method in `SK26Lights.java`

```java
// Quick way - uses existing setMode helper
public Command setMyCoolEffect() { 
    return setMode(LightMode.MY_COOL_EFFECT, "My Cool Effect"); 
}

// Or more control - manual way
public Command setMyCoolEffect() {
    return runOnce(() -> {
        autoLightsEnabled = false;  // Disable auto-switching
        currentMode = LightMode.MY_COOL_EFFECT;
        ledStatus = "My Cool Effect";
    }).ignoringDisable(true).withName("My Cool Effect");
}
```

**Step 2:** Bind to a button in `SK26LightsBinder.java`

```java
package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.*;  // Import button definitions

public class SK26LightsBinder implements CommandBinder {

    @Override
    public void bindButtons() {
        if (lightsSubsystem.isEmpty()) {
            return;
        }
        SK26Lights lights = lightsSubsystem.get();
        
        // Existing bindings...
        kXbutton.button.onTrue(lights.setOff());
        kAbutton.button.onTrue(lights.setSolidRed());
        // ...
        
        // ADD YOUR NEW BINDING HERE
        // Example: Back button triggers your effect
        k_Back.button.onTrue(lights.setMyCoolEffect());
    }
}
```

### Available Buttons (from OperatorPorts)

Check `frc/robot/Ports.java` for available buttons:

| Port Constant | Description |
|---------------|-------------|
| `kAbutton` | A button |
| `kBbutton` | B button |
| `kXbutton` | X button |
| `kYbutton` | Y button |
| `k_Start` | Start button |
| `k_Back` | Back/Select button |
| `k_LeftBumperTrigger` | Left bumper |
| `k_RightBumperTrigger` | Right bumper |
| `k_LeftTrigger` | Left trigger (as button) |
| `k_RightTrigger` | Right trigger (as button) |
| `k_LeftStickButton` | Left stick click |
| `k_RightStickButton` | Right stick click |

---

## Creating Custom Effects

### Option A: Simple Pattern (Solid Color / Gradient)

Add to `LightPatterns.java`:

```java
public class LightPatterns {
    // ...existing patterns...
    
    // Solid color
    public final LEDPattern myColor = LEDPattern.solid(Color.kPurple);
    
    // Scrolling gradient
    public final LEDPattern myGradient = LEDPattern.gradient(
        LEDPattern.GradientType.kContinuous,
        Color.kPurple,
        Color.kBlue,
        Color.kCyan
    ).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);
    
    // Blinking pattern
    public final LEDPattern myBlink = LEDPattern.solid(Color.kGreen)
        .blink(Seconds.of(0.5));
}
```

Then in `SK26Lights.java`:

```java
case MY_COOL_EFFECT: 
    patterns.myGradient.applyTo(m_baseBuffer); 
    break;
```

### Option B: Complex Animation

Add to `LightEffects.java`:

```java
public class LightEffects {
    // Add state variables at the top
    private double myCoolPhase = 0.0;
    
    // Add the effect method
    public void applyMyCoolEffect(AddressableLEDBuffer buffer) {
        // Update animation state
        myCoolPhase += 0.05;
        if (myCoolPhase > Math.PI * 2) {
            myCoolPhase -= Math.PI * 2;
        }
        
        // Apply to each LED
        for (int i = 0; i < buffer.getLength(); i++) {
            double position = (double) i / buffer.getLength();
            
            // Create a wave pattern
            double wave = Math.sin(position * 4 + myCoolPhase);
            wave = (wave + 1.0) / 2.0;  // Normalize to 0-1
            
            // Calculate RGB values
            int r = (int)(255 * wave);
            int g = (int)(100 * (1.0 - wave));
            int b = (int)(150);
            
            buffer.setRGB(i, r, g, b);
        }
    }
}
```

### Animation Techniques

**Sine Wave (smooth oscillation):**
```java
double wave = Math.sin(phase);  // Returns -1 to 1
wave = (wave + 1.0) / 2.0;      // Convert to 0 to 1
```

**Random Sparkle:**
```java
if (random.nextInt(100) < 10) {  // 10% chance
    buffer.setRGB(i, 255, 255, 255);  // White sparkle
}
```

**Chasing/Moving:**
```java
position++;
if (position >= buffer.getLength()) position = 0;
```

**Fading Trail:**
```java
int r = Math.max(0, buffer.getRed(i) - 20);  // Fade by 20
```

**Color from position:**
```java
double hue = (double) i / buffer.getLength();  // 0 to 1 across strip
// Use hue to pick color
```

---

## File Reference

### LightMode.java

All effect names must be listed here as enum values:

```java
public enum LightMode {
    // Basic
    OFF, SOLID_WHITE, SOLID_GREEN, SOLID_RED, SOLID_BLUE, ...
    
    // Team modes
    RAINBOW, BREATHING_SKBLUE, SKBLUE_GRADIENT, ALLIANCE_GRADIENT, ...
    
    // Classic effects
    FIRE, POLICE, SPARKLE, COLOR_CHASE, METEOR, ...
    
    // Your new effects
    MY_COOL_EFFECT
}
```

### LightPatterns.java

Simple WPILib patterns - colors and gradients:

```java
public class LightPatterns {
    private static final Distance kLedSpacing = Meters.of(1.0 / kLEDBufferLength);
    
    public final LEDPattern off = LEDPattern.solid(Color.kBlack);
    public final LEDPattern red = LEDPattern.solid(Color.kRed);
    public final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    // etc.
}
```

### LightEffects.java

Complex animations with state variables:

```java
public class LightEffects {
    private double breathePhase = 0.0;
    private int effectTimer = 0;
    
    public void applyBreathingSKBlue(AddressableLEDBuffer buffer) { ... }
    public void applyFire(AddressableLEDBuffer buffer) { ... }
    // etc.
}
```

### SK26Lights.java - Key Sections

**setupFunEffectChooser()** - Adds effects to SmartDashboard dropdown  
**applyCurrentMode()** - Switch statement connecting modes to implementations  
**Command methods** - `setOff()`, `setSolidRed()`, etc.

### SK26LightsBinder.java

Button bindings:

```java
public void bindButtons() {
    kXbutton.button.onTrue(lights.setOff());
    kAbutton.button.onTrue(lights.setSolidRed());
    // Add new bindings here
}
```

---

## Example: Complete Walkthrough

Let's add a "Pulse Purple" effect that pulses between purple and black.

### Step 1: Add to LightMode.java

```java
public enum LightMode {
    // ...existing...
    PULSE_PURPLE
}
```

### Step 2: Add the effect in LightEffects.java

```java
// At top with other state variables
private double pulsePurplePhase = 0.0;

// In the class
public void applyPulsePurple(AddressableLEDBuffer buffer) {
    pulsePurplePhase += 0.08;  // Speed
    if (pulsePurplePhase > Math.PI * 2) {
        pulsePurplePhase -= Math.PI * 2;
    }
    
    // Pulse brightness from 0 to 1
    double brightness = (Math.sin(pulsePurplePhase) + 1.0) / 2.0;
    
    int r = (int)(128 * brightness);  // Purple = red + blue
    int g = 0;
    int b = (int)(255 * brightness);
    
    for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, r, g, b);
    }
}
```

### Step 3: Connect in SK26Lights.java

In `applyCurrentMode()`:
```java
case PULSE_PURPLE: 
    effects.applyPulsePurple(m_baseBuffer); 
    break;
```

In `setupFunEffectChooser()`:
```java
funEffectChooser.addOption("Pulse Purple", LightMode.PULSE_PURPLE);
```

Create command method:
```java
public Command setPulsePurple() { 
    return setMode(LightMode.PULSE_PURPLE, "Pulse Purple"); 
}
```

### Step 4: (Optional) Bind to a button

In `SK26LightsBinder.java`:
```java
k_Back.button.onTrue(lights.setPulsePurple());
```

### Step 5: Build and test!

```bash
./gradlew build
```

---

## Tips

- **Speed:** Smaller phase increment = slower animation
- **Brightness:** Multiply RGB values by 0.0-1.0 to dim
- **Smoothness:** Use `Math.sin()` for smooth transitions
- **Randomness:** Use `random.nextInt()` for sparkles/flickers
- **Colors:** Check `edu.wpi.first.wpilibj.util.Color` for preset colors
- **Testing:** Use SmartDashboard Fun Mode to test before binding to buttons
- **`.ignoringDisable(true)`:** Always add this so lights work when robot is disabled!

---

## Existing Effects Reference

### Basic Modes
`OFF`, `SOLID_WHITE`, `SOLID_GREEN`, `SOLID_RED`, `SOLID_BLUE`, `SOLID_YELLOW`, `SOLID_ORANGE`

### Team/Alliance Modes
`RAINBOW`, `BREATHING_SKBLUE`, `SKBLUE_GRADIENT`, `ALLIANCE_GRADIENT`, `ALLIANCE_SOLID`, `ALLIANCE_FLASH_WHITE`

### Classic Effects
`FIRE`, `POLICE`, `SPARKLE`, `COLOR_CHASE`, `METEOR`, `THEATER_CHASE`, `LAVA_LAMP`, `OCEAN_WAVE`, `TWINKLE_STARS`, `CANDY_CANE`, `PLASMA`, `KNIGHT_RIDER`, `HEARTBEAT`, `LIGHTNING`, `CONFETTI`, `COMET_TRAIL`

### Awesome Effects
`AURORA_BOREALIS`, `GALAXY_SWIRL`, `NEON_PULSE`, `MATRIX_RAIN`, `FIREWORKS`, `BREATHING_RAINBOW`, `WAVE_COLLISION`, `DISCO_BALL`, `CYBERPUNK`, `SNAKE_GAME`, `RIPPLE`, `GRADIENT_BOUNCE`, `ELECTRIC_SPARKS`, `SUNSET`, `NORTHERN_LIGHTS`, `PACMAN`, `SOUND_WAVE`, `DNA_HELIX`, `PORTAL`, `HYPNOTIC_SPIRAL`, `PIXEL_RAIN`, `FIREFLIES`, `NYAN_CAT`, `RACING_STRIPES`, `DRIP`

---

## Summary Checklist

Adding a new effect:

- [ ] Add enum value to `LightMode.java`
- [ ] Create pattern in `LightPatterns.java` OR animation in `LightEffects.java`
- [ ] Add case to `applyCurrentMode()` in `SK26Lights.java`
- [ ] Add to `setupFunEffectChooser()` for SmartDashboard access
- [ ] Create command method (e.g., `setMyEffect()`) in `SK26Lights.java`
- [ ] (Optional) Bind to button in `SK26LightsBinder.java`
- [ ] Build and test!
