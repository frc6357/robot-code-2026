# Addressable LEDs | 6357 Spring Konstant! 

<img src="images/sk-logo.png" alt="Spring Konstant Logo" width="200">

The SK26Lights subsystem controls a WS2812B addressable LED strip. It handles animations, color correction, and automatic state changes based on robot status.

---

## Quick Reference

### Manual Control Buttons (Operator Controller)

| Mode | Command | Button | Description |
|------|---------|--------|-------------|
| Off | `setOff()` | X | All LEDs off |
| White | `setSolidWhite()` | Left Bumper | Solid white |
| Rainbow | `setRainbow()` | Start | Scrolling rainbow (Party Mode) |
| Breathing SKBlue | `setBreathingSKBlue()` | Right Bumper | Breathing team color |
| Game State Aware | `setGameStateAware()` | Back | Automatic mode based on match state |
| Alliance Color | `setAllianceColor()` | Y | Solid red/blue based on alliance |
| Alliance Gradient | `setAllianceGradient()` | A | Scrolling gradient in alliance colors |
| Breathing Alliance | `setBreathingAlliance()` | B | Breathing effect in alliance color |

### Additional Modes (Available via Code)

| Mode | Command | Description |
|------|---------|-------------|
| Red | `setSolidRed()` | Solid red |
| Blue | `setSolidBlue()` | Solid blue |
| Green | `setSolidGreen()` | Solid green |
| Yellow | `setSolidYellow()` | Solid yellow |
| Orange | `setSolidOrange()` | Solid orange |
| SK Gradient | `setSKBlueGradient()` | Scrolling SK brand colors |

![Controller Button Layout](images/controller-layout.png)

---

## Game States & Alliance Detection

The lights subsystem automatically detects:
- **Alliance** (Red/Blue) from the Driver Station
- **Game State** (Disconnected, Disabled, Auto, Teleop, Endgame, Test)
- **Match Time** for endgame detection (last 20 seconds)

### Game State Aware Mode

When `setGameStateAware()` is active (Back button), the LEDs automatically change based on the current match state:

| Game State | LED Pattern | Purpose |
|------------|-------------|---------|
| **Disconnected** | Breathing SKBlue | Shows robot is powered but not connected to DS |
| **Disabled** (connected) | SKBlue Gradient | Ready and waiting, connected to DS |
| **Autonomous** | Rainbow Scroll | Indicates autonomous mode is active |
| **Teleop** | Alliance Gradient | Team colors during main match play |
| **Endgame** (last 20s) | Solid Alliance Color | Alerts drivers that endgame has started! |
| **Test** | Solid Yellow | Clearly indicates test mode |

### Automatic State Transitions

Even when NOT in Game State Aware mode, these transitions fire automatically to provide visual feedback:

| State Change | LED Pattern | Why |
|--------------|-------------|-----|
| DS Disconnected | Breathing SKBlue | Shows robot is on but not connected |
| DS Connected + Disabled | SKBlue Gradient | Shows connection is good, waiting to enable |
| Autonomous Starts | Rainbow | Visual indicator auto mode began |
| Teleop Starts | Alliance Gradient | Team colors for main match |
| Test Mode Starts | Solid Yellow | Clearly indicates test mode |

You can override these at any time with a button press. The automatic state only triggers on the transition (when the state changes).

### Alliance-Aware Colors

| Alliance | Solid Color | Gradient Colors |
|----------|-------------|-----------------|
| Red | Red | Red → Dark Red → Orange Red |
| Blue | Blue | Blue → Dark Blue → Royal Blue |
| Unknown | SKBlue | SKBlue Gradient (team colors) |

<p align="center">
  <img src="images/led-states.gif" alt="LED State Animations" width="400">
</p>

---

## SmartDashboard Settings

All settings are under `Lights/` in SmartDashboard or Elastic.

| Setting | Default | Range | What it does |
|---------|---------|-------|--------------|
| Gamma | 2.8 | 1.0 - 4.0 | Brightness curve. Lower = brighter midtones, higher = more contrast |
| Red Correction | 1.0 | 0.0 - 2.0 | Multiplier for red channel |
| Green Correction | 1.0 | 0.0 - 2.0 | Multiplier for green channel |
| Blue Correction | 0.85 | 0.0 - 2.0 | Multiplier for blue channel |
| Brightness | 1.0 | 0.0 - 3.0 | Overall brightness multiplier |

![SmartDashboard Lights Panel](images/smartdashboard-lights.png)

### How Brightness Works

The brightness setting multiplies all color values. Setting it above 1.0 makes everything brighter, but the code is smart about it.

When brightness would cause colors to clip (go above 255), instead of just cutting off the values, it scales everything proportionally. This keeps gradients looking smooth instead of turning into solid blobs of color.

Example at brightness 2.0:
- A color at (100, 150, 200) would normally become (200, 300, 400)
- Since 400 > 255, we scale everything so the max hits 255
- Result: (128, 191, 255) - same color ratios, no clipping

### How Gamma Works

Gamma correction adjusts how brightness is distributed across the range.

- Gamma 1.0 = linear (what you put in is what you get)
- Gamma 2.8 = darker midtones, better contrast (default)
- Gamma 3.5 = even darker midtones, very punchy

LEDs don't display brightness linearly, so gamma correction makes colors look more natural.

<img src="images/gamma-curves.png" alt="Game curves" width="300">

### How Color Correction Works

Each color channel has a multiplier. The defaults are calibrated for our LED strips:

- Red: 1.0 (no change)
- Green: 1.0 (no change)  
- Blue: 0.85 (slightly reduced because our strips run a bit blue)

---

## SK Brand Colors

These are defined in `Konstants.java` under `LightsConstants`.

| Name | RGB | Hex | Used In |
|------|-----|-----|---------|
| SK Cream | (233, 235, 229) | #E9EBE5 | Gradient highlight |
| SK Teal | (104, 185, 196) | #68B9C4 | Gradient |
| SK Blue | (81, 171, 185) | #51ABB9 | Breathing, Gradient |
| SK Dark Blue | (0, 118, 133) | #007685 | Gradient shadow |

The gradient scrolls through these colors in order: Cream → Teal → Blue → Dark Blue → repeat.

<img scrc="images/sk-colors.png" alt="Game curves" width="300">

---

## Hardware Setup

| Property | Value |
|----------|-------|
| PWM Port | 9 |
| LED Count | ~60 (will change) |
| LED Type | WS2812B (addressable RGB) |
| Data Direction | Check the arrows on your strip |

The LED strip needs:
- 5V power (from VRM or dedicated 5V supply)
- Ground (shared with roboRIO)
- Data line to PWM port 9

<img src="images/led-wiring.png" alt="VRM to supply 5V to strip" width="300">

Make sure the data direction arrow on the strip points away from the roboRIO.

<img src="images/led-direction.png" alt="LED Input" width="300">

---

## Code Structure

### Files

| File | Purpose |
|------|---------|
| `SK26Lights.java` | Main subsystem, handles all LED logic |
| `SK26LightsBinder.java` | Button bindings and automatic state triggers |
| `Konstants.java` | LED count, PWM port, SK brand colors |

### Public API Methods

#### LED Mode Commands
| Method | Returns | Description |
|--------|---------|-------------|
| `setOff()` | Command | Turn LEDs off |
| `setSolidWhite()` | Command | Solid white |
| `setSolidRed()` | Command | Solid red |
| `setSolidBlue()` | Command | Solid blue |
| `setSolidGreen()` | Command | Solid green |
| `setSolidYellow()` | Command | Solid yellow |
| `setSolidOrange()` | Command | Solid orange |
| `setRainbow()` | Command | Scrolling rainbow |
| `setBreathingSKBlue()` | Command | Breathing SK team color |
| `setSKBlueGradient()` | Command | Scrolling SK brand gradient |
| `setAllianceColor()` | Command | Solid alliance color |
| `setAllianceGradient()` | Command | Scrolling alliance gradient |
| `setBreathingAlliance()` | Command | Breathing alliance color |
| `setGameStateAware()` | Command | Auto-change based on match state |

#### State Query Methods
| Method | Returns | Description |
|--------|---------|-------------|
| `getAlliance()` | `Optional<Alliance>` | Current alliance (Red/Blue) or empty |
| `getAllianceColor()` | `Color` | Alliance color (SKBlue if unknown) |
| `getGameState()` | `GameState` | Current game state enum |
| `getMatchTime()` | `double` | Match time in seconds (-1 if not in match) |
| `isAllianceKnown()` | `boolean` | True if alliance has been received |
| `isDSConnected()` | `boolean` | True if connected to DriverStation |
| `isEnabled()` | `boolean` | True if robot is enabled |
| `isEndgame()` | `boolean` | True if in last 20 seconds of teleop |
| `gameStateChanged()` | `boolean` | True if state changed this cycle |

### BaseMode Enum

All available modes are defined in the `BaseMode` enum inside `SK26Lights.java`:

```java
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
    SKBLUE_GRADIENT,
    ALLIANCE_COLOR,        // Solid alliance color (red or blue)
    ALLIANCE_GRADIENT,     // Scrolling gradient in alliance colors
    BREATHING_ALLIANCE,    // Breathing effect in alliance color
    GAME_STATE_AWARE       // Automatically changes based on game state
}
```

### GameState Enum

Game states are tracked with the `GameState` enum:

```java
public enum GameState {
    DISCONNECTED,   // Not connected to DriverStation
    DISABLED,       // Connected but disabled
    AUTONOMOUS,     // Autonomous period
    TELEOP,         // Teleop (not endgame)
    ENDGAME,        // Last 20 seconds of teleop
    TEST            // Test mode
}
```

### How Updates Work

Every robot loop (50 times per second):

1. `updateCalibrationValues()` - reads SmartDashboard settings
2. `applyBaseMode()` - writes the current animation to `m_baseBuffer`
3. `copyBaseToMain()` - copies base buffer to main buffer
4. `applyColorCorrection()` - applies gamma, color correction, and brightness
5. `m_led.setData(m_buffer)` - sends the final data to the LEDs

The two-buffer system exists so we could add overlays in the future (like alerts blinking on top of the base animation).

---

## Adding New Modes

### Step 1: Add to BaseMode enum

```java
private enum BaseMode {
    // ...existing modes...
    MY_NEW_MODE
}
```

### Step 2: Create the pattern

For a solid color:
```java
private final LEDPattern m_myColor = LEDPattern.solid(Color.kPurple);
```

For a gradient:
```java
private final LEDPattern m_myGradient = LEDPattern.gradient(
    LEDPattern.GradientType.kContinuous,
    Color.kRed,
    Color.kBlue
).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);
```

For a custom animation, create a method like `applyBreathingSKBlue()`.

### Step 3: Add to applyBaseMode()

```java
private void applyBaseMode() {
    switch (currentBaseMode) {
        // ...existing cases...
        case MY_NEW_MODE:
            m_myColor.applyTo(m_baseBuffer);
            break;
    }
}
```

### Step 4: Create the public command

```java
public Command setMyNewMode() {
    return setMode(BaseMode.MY_NEW_MODE, "My New Mode");
}
```

### Step 5: Bind to a button (optional)

In `SK26LightsBinder.java`:
```java
someButton.onTrue(lights.setMyNewMode().ignoringDisable(true));
```

---

## Tuning Guide

### White Looks Wrong

| Problem | Fix |
|---------|-----|
| Purple/blue tint | Lower Blue Correction (try 0.7) |
| Yellow/green tint | Lower Green Correction (try 0.8) |
| Pink tint | Lower Red Correction (try 0.8) |

### Brightness Issues

| Problem | Fix |
|---------|-----|
| Too dark overall | Raise Brightness (try 1.5 or 2.0) |
| Too bright/washed out | Lower Brightness or raise Gamma |
| Midtones look wrong | Adjust Gamma (2.2 = brighter mids, 3.0 = darker mids) |

### Gradients Look Bad

| Problem | Fix |
|---------|-----|
| Colors bleeding together | Raise Gamma |
| Sharp transitions | Lower Gamma |
| Gradient looks solid at high brightness | This is fixed in code, but check that brightness isn't crazy high |

---

## Breathing Animation Details

The breathing effect uses a sine wave to smoothly fade the SK Blue color.

- Speed: 0.05 radians per loop (about 2.5 seconds per cycle)
- Minimum brightness: 10%
- Maximum brightness: 100%

The formula:
```java
// breatheAmount goes from 0 to 1 based on sine wave
double breatheAmount = (Math.sin(breathePhase) + 1.0) / 2.0;
// Map to 10% - 100% range
double breatheBrightness = 0.1 + (breatheAmount * 0.9);
```

---

## Notes

- All LED commands use `.ignoringDisable(true)` so they work even when the robot is disabled
- Color correction is applied after the animation, so animations don't need to worry about it

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| LEDs not turning on | Wrong PWM port | Check `kLightsPWMHeader` in Konstants.java |
| LEDs flickering | Power issue | Use a dedicated 5V supply, not the roboRIO |
| Only some LEDs work | Bad connection | Check data line, might have a dead LED |
| Colors look wrong | Calibration | Adjust correction values in SmartDashboard |
| Animations not changing | Command not scheduled | Make sure `.ignoringDisable(true)` is on the command |
| Stuck on one mode | Trigger issue | Check the binder for conflicting triggers |

---

*Documentation for FRC Team 6357 - Spring Konstant*