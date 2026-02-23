# Turret Subsystem | 6357 Spring Konstant

<img src="images/sk-logo.png" alt="Spring Konstant Logo" width="200">

The turret subsystem (`SK26Turret`) controls a rotating turret mechanism using a TalonFX motor with a CANcoder absolute encoder for position feedback. It supports manual joystick control, button-press preset angles, and automatic target point tracking that integrates with the swerve drive's field pose.

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Hardware](#hardware)
- [Position Feedback](#position-feedback)
- [PID Control](#pid-control)
- [Wrapping Behavior](#wrapping-behavior)
- [Commands](#commands)
- [State Machine Integration](#state-machine-integration)
- [Controller Bindings](#controller-bindings)
- [Telemetry](#telemetry)
- [Constants & Tuning](#constants--tuning)
- [Key Classes & Files](#key-classes--files)
- [Troubleshooting](#troubleshooting)

---

## Overview

The turret is a rotating mechanism that can aim independently of the robot's chassis. It:

1. Uses a **CANcoder absolute encoder** as the sole position feedback source  
2. Runs a **WPILib PID controller** in the periodic loop (not on the motor controller)  
3. Supports **angle wrapping** — when commanded past ±170°, it wraps to the opposite side  
4. Can **track field points** automatically, accounting for robot position and heading  
5. Integrates with the **StateHandler** to automatically aim based on game state  

<p align="center">
  <img src="images/turret-overview.png" alt="Turret mechanism overview" width="400">
</p>

---

## Architecture

```
SK26Turret (SubsystemBase)
├── TalonFX turretMotor        ← Drives the turret rotation
├── CANcoder turretEncoder     ← Absolute position feedback (2:1 gear ratio)
├── PIDController              ← WPILib PID (runs in periodic)
├── VoltageOut control         ← Motor output mode
└── SKPreferences              ← Runtime-tunable PID gains
```

### Control Loop

```
periodic() {
    1. Read current angle from CANcoder
    2. Calculate PID output: pidController.calculate(currentAngle)
    3. Clamp output to ±kMaxTurretOutput
    4. Invert if needed (kTurretMotorInverted)
    5. Apply voltage to motor
    6. Update wrapping status
    7. Publish telemetry
}
```

---

## Hardware

| Component | Type | CAN ID | CAN Bus |
|-----------|------|--------|---------|
| Turret Motor | TalonFX | 55 | roboRIO |
| Turret Encoder | CANcoder | 57 | roboRIO |

### Physical Specs

| Property | Value |
|----------|-------|
| Encoder Gear Ratio | 2:1 (2 encoder rotations = 1 turret rotation) |
| Turret Range | -170° to +170° (340° total) |
| Motor Neutral Mode | Coast |
| Motor Inverted | Yes |
| Encoder Inverted | No |
| Encoder Offset | -0.3828125 rotations |

<p align="center">
  <img src="images/turret-gear-ratio.png" alt="2:1 gear ratio diagram between CANcoder and turret" width="350">
</p>

---

## Position Feedback

The CANcoder provides **absolute position** so the turret always knows its angle, even after power cycling. The conversion from encoder to turret angle:

$$\text{turretDegrees} = \text{encoderRotations} \times \frac{360°}{\text{gearRatio}}$$

With a 2:1 gear ratio:

$$\text{turretDegrees} = \text{encoderRotations} \times 180°$$

The encoder's magnet offset (`kTurretEncoderOffset = -0.3828125`) is applied in the CANcoder configuration so the raw reading already accounts for mechanical mounting.

---

## PID Control

The turret uses a **WPILib PIDController** running on the roboRIO (not onboard the TalonFX). This gives flexibility for custom features like wrapping and integration with SKPreferences.

### Default PID Gains

| Parameter | Value | SKPreference Key |
|-----------|-------|-----------------|
| kP | 0.07 | `TurretPID/p` |
| kI | 0.02 | `TurretPID/i` |
| kD | 0.005 | `TurretPID/d` |
| I-Zone | 8° | Hardcoded |
| Tolerance | 0.5° | Hardcoded |

> **Runtime Tuning**: PID gains are attached to `SKPreferences` and can be changed on the fly from SmartDashboard. Changes take effect immediately.

### Output Limits

| Parameter | Value |
|-----------|-------|
| Max Output | ±2.25 V |
| Control Mode | VoltageOut |

The PID output is clamped to `kMaxTurretOutput` (2.25V) for safety, then inverted if `kTurretMotorInverted` is true.

<p align="center">
  <img src="images/turret-pid-response.png" alt="PID step response graph" width="450">
</p>

---

## Wrapping Behavior

The turret has a 340° range (-170° to +170°). When a command requests an angle beyond these limits, the turret **wraps around** to the opposite side instead of hitting a hard stop.

### How Wrapping Works

1. `setAngleDegreesWrapped(angleDeg)` is called with an angle outside [-170°, +170°]  
2. The angle is shifted by the full range (340°) until it falls within bounds  
3. The `wrapping` flag is set to `true`  
4. During wrapping, **joystick commands are ignored** to prevent interference  
5. Once the target is back within bounds, `wrapping` resets to `false`  

### Example

```
Request: 185° → Wraps to: 185° - 340° = -155°
Request: -180° → Wraps to: -180° + 340° = 160°
```

<p align="center">
  <img src="images/turret-wrapping.gif" alt="Turret wrapping animation" width="400">
</p>

---

## Commands

### TurretJoystickCommand (Default Command)

Manual control using the operator's right joystick.

| Property | Value |
|----------|-------|
| Input | Right Stick X (Operator) |
| Speed | 360°/s at full deflection |
| Deadband | 0.15 |
| Wrapping | Uses `setAngleDegreesWrapped()` |

The command integrates joystick input over time:

```
newAngle = currentTarget + (joystickInput × kManualTurretSpeed × dt)
```

A slew rate limiter (1.75× manual speed) smooths the joystick input.

> **Wrapping Guard**: If the turret is currently wrapping, new joystick inputs are ignored until the wrap completes.

### TurretTemporaryButtonCommand

Holds the turret at a specific preset angle while a button is held.

| Button | Angle | Condition |
|--------|-------|-----------|
| Operator B | 90° | Only when state is IDLE |
| Operator Y | 0° (forward) | Only when state is IDLE |

When the button is released, the turret returns to joystick control.

### TurretTrackPointCommand

Continuously calculates the field-relative angle to a target point and converts it to a robot-relative turret angle.

#### Angle Calculation

```
1. Get field-relative angle to target using atan2
2. Subtract robot heading to get turret angle
3. Apply to turret via setAngleDegrees()
```

$$\theta_{\text{turret}} = \theta_{\text{field→target}} - \theta_{\text{robot heading}}$$

#### Tracking Targets

| State | Target Point | Command Name |
|-------|-------------|--------------|
| IDLE + A button toggle | Operator Controlled point | TurretManualTrackHubCommand |
| SCORING | Alliance Hub (auto-detected) | TurretTrackHubCommand |
| STEADY_STREAM_SCORING | Alliance Hub | TurretTrackHubCommand |
| SHUTTLING | Operator Controlled point | TurretTrackShuttleCommand |
| STEADY_STREAM_SHUTTLING | Operator Controlled point | TurretTrackShuttleCommand |

<p align="center">
  <img src="images/turret-tracking-diagram.png" alt="Turret tracking a field point with geometry shown" width="500">
</p>

---

## State Machine Integration

The turret behavior changes based on the robot's **MacroState** (managed by `StateHandler`):

| MacroState | Turret Behavior |
|------------|----------------|
| IDLE | Manual joystick control + preset buttons available |
| SCORING | Auto-track alliance hub |
| STEADY_STREAM_SCORING | Auto-track alliance hub |
| SHUTTLING | Auto-track operator-defined shuttle point |
| STEADY_STREAM_SHUTTLING | Auto-track operator-defined shuttle point |
| INTAKING | Manual joystick (default) |
| CLIMBING | Manual joystick (default) |

---

## Controller Bindings

### Operator Controller (Xbox - Port 1)

| Input | Action | Condition |
|-------|--------|-----------|
| Right Stick X | Manual turret rotation (360°/s) | Default command, only when IDLE |
| A Button | Toggle auto-track operator point | Only when IDLE |
| B Button (hold) | Snap to 90° | Only when IDLE |
| Y Button (hold) | Snap to 0° (forward) | Only when IDLE |

<p align="center">
  <img src="images/turret-controller-layout.png" alt="Operator controller turret bindings" width="450">
</p>

---

## Telemetry

### SmartDashboard

| Key | Type | Description |
|-----|------|-------------|
| `Turret` | Subsystem data | Full Sendable |
| `Turret/PIDController` | PIDController | Live PID data & tuning |
| `Turret/Turret Angle (deg)` | double | Current angle from CANcoder |
| `Turret/Turret Target (deg)` | double | Current target setpoint |
| `Turret/Turret At Target` | boolean | Within 0.5° tolerance |
| `Turret/Turret Error (deg)` | double | Setpoint - actual |
| `Turret/Turret Motor DutyCycle Output` | double | Motor duty cycle (0-1) |
| `Turret/Turret Motor Voltage Output` | double | Motor voltage (V) |
| `Turret/Turret Wrapping` | boolean | Currently wrapping around |

### TurretTrackPoint SmartDashboard

| Key | Type | Description |
|-----|------|-------------|
| `TurretTrack/FieldAngleToTarget` | double | Field-relative angle to target |
| `TurretTrack/RobotHeading` | double | Current robot heading |
| `TurretTrack/DesiredTurretAngle` | double | Calculated turret angle |
| `TurretTrack/DistanceToTarget` | double | Distance to target (m) |

---

## Constants & Tuning

### All Turret Constants (`Konstants.TurretConstants`)

| Constant | Value | Description |
|----------|-------|-------------|
| `kTurretMinPosition` | -170° | Soft limit (lower bound) |
| `kTurretMaxPosition` | +170° | Soft limit (upper bound) |
| `kTurretAngleTolerance` | 0.5° | PID "at setpoint" tolerance |
| `kTurretEncoderOffset` | -0.3828125 rot | CANcoder magnet offset |
| `kTurretEncoderInverted` | false | Encoder direction |
| `kEncoderGearRatio` | 2.0 | Encoder-to-turret ratio |
| `kTurretMotorInverted` | true | Motor direction |
| `kTurretP` | 0.07 | PID proportional |
| `kTurretI` | 0.02 | PID integral |
| `kTurretD` | 0.005 | PID derivative |
| `kMaxTurretOutput` | 2.25 V | Output safety clamp |
| `kManualTurretSpeed` | 360°/s | Max joystick speed |
| `kTurretJoystickDeadband` | 0.15 | Joystick deadband |

### Preset Positions (`TurretPosition` enum)

| Position | Angle |
|----------|-------|
| `kTurretLeftPosition` | 90° |
| `kTurretZeroPosition` | 0° |

---

## Key Classes & Files

| File | Purpose |
|------|---------|
| `subsystems/SK26Turret.java` | Main turret subsystem — PID, wrapping, CANcoder |
| `commands/TurretJoystickCommand.java` | Manual joystick control with wrapping |
| `commands/TurretTemporaryButtonCommand.java` | Preset angle while button held |
| `commands/TurretTrackPointCommand.java` | Automatic field-point tracking |
| `bindings/SK26TurretBinder.java` | Controller bindings + state-based command scheduling |
| `Konstants.java` → `TurretConstants` | All turret-related constants |
| `Ports.java` → `LauncherPorts` | CAN IDs for turret motor and encoder |

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Turret oscillates around target | PID gains too aggressive | Lower kP via SmartDashboard `TurretPID/p` |
| Turret doesn't move | Motor inverted wrong, or output too low | Check `kTurretMotorInverted`, increase `kMaxTurretOutput` |
| Turret angle reads wrong | Encoder offset incorrect | Re-calibrate `kTurretEncoderOffset` |
| Turret wraps unexpectedly | Manual speed too high | Reduce `kManualTurretSpeed` |
| Turret doesn't respond to joystick | Not in IDLE state | Check `StateHandler` — only IDLE allows manual control |
| Target tracking jitters | Robot pose noisy | Improve vision integration or increase PID kD |
| Turret gets stuck during wrap | Wrapping flag not clearing | Check that target ends up within [-170°, +170°] range |
| PID changes not taking effect | Wrong SKPreferences key | Keys are `TurretPID/p`, `TurretPID/i`, `TurretPID/d` |
| "Turret At Target" never goes true | Tolerance too tight or steady-state error | Increase `kTurretAngleTolerance` or adjust kI |
