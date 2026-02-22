# Launcher Subsystem (Standard PID) | 6357 Spring Konstant

<img src="images/sk-logo.png" alt="Spring Konstant Logo" width="200">

The standard launcher subsystem (`SK26Launcher`) controls a dual-motor flywheel shooter using CTRE Phoenix 6 velocity closed-loop control. It features a leader-follower motor configuration with configurable PID slots and supports exit velocity or direct RPS targeting.

> **Note**: This is the **standard PID-based launcher**. The team also has a [Bang-Bang Launcher](BangBang-Launcher.md) that uses a different control strategy. Only one launcher subsystem is enabled at a time via `Subsystems.json`.

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Hardware](#hardware)
- [Control Modes](#control-modes)
- [PID Configuration](#pid-configuration)
- [Commands](#commands)
- [Unjamming](#unjamming)
- [Telemetry](#telemetry)
- [Constants & Tuning](#constants--tuning)
- [Key Classes & Files](#key-classes--files)
- [Troubleshooting](#troubleshooting)

---

## Overview

The standard launcher uses **CTRE VelocityDutyCycle** closed-loop control to spin a flywheel to a precise RPM. The system:

1. Accepts a target in either **exit velocity (m/s)** or **motor RPS**  
2. Converts exit velocity to RPS using wheel radius  
3. Runs velocity PID on the TalonFX motor controller  
4. Reports when the flywheel is within tolerance of the target speed  
5. Supports **coasting** (low idle speed) for quick re-launching  

<p align="center">
  <img src="images/launcher-standard-overview.png" alt="Standard launcher system overview" width="400">
</p>

---

## Architecture

```
SK26Launcher (SubsystemBase)
├── TalonFX launchermotor           ← Primary motor (runs velocity PID)
├── TalonFX launchermotorFollower   ← Follows primary (opposed direction)
├── VelocityDutyCycle control       ← Closed-loop velocity output
├── Slot0Configs                    ← Primary PID slot
└── Slot1Configs                    ← Secondary PID slot (alternate tuning)
```

### Leader-Follower Configuration

The follower motor is configured with `MotorAlignmentValue.Opposed`, meaning it spins in the **opposite direction** from the leader. This is typical for flywheel launchers where the two wheels need to spin inward together.

```
Leader Motor ──→ [Flywheel]
                    ↑ Game Piece ↑
Follower Motor ←── [Flywheel]
```

---

## Hardware

| Component | Type | CAN ID | CAN Bus |
|-----------|------|--------|---------|
| Launcher Motor (Leader) | TalonFX | 50 | roboRIO |
| Launcher Motor (Follower) | TalonFX | 51 | roboRIO |

### Physical Specs

| Property | Value |
|----------|-------|
| Wheel Radius | 0.0508 m (2 inches) |
| Neutral Mode | Brake |
| Follower Alignment | Opposed (spins opposite direction) |

<p align="center">
  <img src="images/launcher-motor-diagram.png" alt="Leader-follower motor wiring diagram" width="350">
</p>

---

## Control Modes

### 1. Exit Velocity Mode

Converts a desired exit velocity (m/s) to motor RPS using the wheel circumference:

$$\text{motorRPS} = \frac{\text{exitVelocity}}{2\pi \times \text{wheelRadius}}$$

Example: 5 m/s → ~15.7 RPS

### 2. Direct RPS Mode

Directly sets the motor target in rotations per second.

### 3. Coast Mode

Runs the flywheel at a very low speed (`kCoastLauncherRPS = 0.25 RPS`) so it can spin up to full speed quickly when needed. This is activated when a launch command ends.

### 4. Stop Mode

Sets the motor to 0 and the target to 0.

### Speed Check

The `isLauncherAtSpeed()` method reports `true` when:

$$|\text{actualRPS} - \text{targetRPS}| < \text{kShooterTolerance}$$

Where `kShooterTolerance = 0.5 RPS`.

---

## PID Configuration

The TalonFX has two PID slots configured:

### Slot 0 (Primary)

| Parameter | Value |
|-----------|-------|
| kP | 1.8 |
| kI | 0 |
| kD | 0 |
| kV | 0.093 |
| kA | 0 |
| kS | 0.25 |

### Slot 1 (Secondary)

| Parameter | Value |
|-----------|-------|
| kP | 0.5 |
| kI | 0 |
| kD | 0 |
| kV | 0.093 |
| kA | 0 |
| kS | 0.25 |

> **Feedforward**: Both slots share the same feedforward values (kV, kA, kS). The `kV` (velocity feedforward) is the most important — it provides the base voltage needed to maintain a given speed. The `kS` (static feedforward) overcomes friction.

---

## Commands

### RunLauncherWithRPSCommand

Continuously runs the launcher at a specified RPS. When the command ends (button released), the launcher **coasts** instead of stopping.

```
execute() → runLauncherRPS(targetRPS)
end()     → coastLauncher()  // Quick re-launch capability
```

### RunLauncherWithVelCommand

Same behavior but takes a target exit velocity in m/s instead of RPS.

```
execute() → runLauncherExitVel(targetVelocity)
end()     → coastLauncher()
```

### LauncherUnJamCommandGroup

See [Unjamming](#unjamming) below.

---

## Unjamming

The `LauncherUnJamCommandGroup` is a sequential command group that alternates the motor direction to dislodge stuck game pieces:

```
Loop (repeating):
1. Reverse at -kUnJamLauncherRPS for 0.25s
2. Stop for 0.25s
3. Forward at +kUnJamLauncherRPS for 0.25s
4. Stop for 0.25s
```

The loop repeats as long as the unjam button is held. When released, `stopLauncher()` is called via `finallyDo()`.

### Unjam Constants

| Constant | Value |
|----------|-------|
| `kUnJamLauncherRunTime` | 0.25 s |
| `kUnJamLauncherPauseTime` | 0.25 s |
| `kUnJamLauncherRPS` | 4.0 RPS (1/runTime) |

<p align="center">
  <img src="images/launcher-unjam-sequence.png" alt="Unjam timing diagram showing reverse-stop-forward-stop pattern" width="500">
</p>

---

## Telemetry

### SmartDashboard

| Key | Type | Description |
|-----|------|-------------|
| `Static Launcher` | Subsystem data | Full Sendable |
| `Static Launcher/Launcher Status` | String | Current status message |
| `Static Launcher/Is at launcher speed` | boolean | Within tolerance of target |
| `Static Launcher/Launcher Velocity` | double | Current exit velocity (m/s) |
| `Static Launcher/Target RPS` | double | Current target RPS |

### Status Messages

| Message | Meaning |
|---------|---------|
| "Stopped" | Motor at 0 |
| "Launching" | Actively spinning to target |
| "Waiting to Shoot" | Coasting at low speed |
| "Unjamming" | Running unjam sequence |

---

## Constants & Tuning

### All Launcher Constants (`Konstants.LauncherConstants`)

| Constant | Value | Description |
|----------|-------|-------------|
| `kWheelRadius` | 0.0508 m | Flywheel radius for velocity conversion |
| `kShooterTolerance` | 0.5 RPS | "At speed" tolerance |
| `kTargetlaunchVelocity` | 5 m/s | Default launch target |
| `kTargetMotorRPS` | 15.665 RPS | Equivalent of default velocity |
| `kCoastLauncherRPS` | 0.25 RPS | Idle coast speed |
| `kStopLauncher` | 0 | Stop value |
| `kUnJamLauncherRunTime` | 0.25 s | Unjam rotation duration |
| `kUnJamLauncherPauseTime` | 0.25 s | Unjam pause duration |
| `kLauncherV` | 0.093 | Velocity feedforward |
| `kLauncherA` | 0 | Acceleration feedforward |
| `kLauncherS` | 0.25 | Static feedforward |

---

## Key Classes & Files

| File | Purpose |
|------|---------|
| `subsystems/SK26Launcher.java` | Main launcher subsystem — velocity PID, coast, unjam |
| `commands/RunLauncherWithRPSCommand.java` | Run launcher at specific RPS |
| `commands/RunLauncherWithVelCommand.java` | Run launcher at specific exit velocity |
| `commands/commandGroups/LauncherUnJamCommandGroup.java` | Alternating unjam sequence |
| `bindings/SK26LauncherBinder.java` | Controller bindings for launcher |
| `Konstants.java` → `LauncherConstants` | All launcher-related constants |
| `Ports.java` → `LauncherPorts` | CAN IDs |

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Flywheel doesn't spin | Motor CAN ID wrong or motor not powered | Check CAN IDs (50, 51), check power |
| Flywheel oscillates around target | kP too high | Reduce Slot0 kP |
| Flywheel takes too long to reach speed | kV too low | Increase `kLauncherV` |
| "Is at launcher speed" always false | Tolerance too tight or speed never converges | Increase `kShooterTolerance` |
| Follower spins same direction as leader | Wrong alignment value | Verify `MotorAlignmentValue.Opposed` |
| Game piece not launching far enough | Target velocity too low | Increase `kTargetlaunchVelocity` |
| Unjam doesn't work | Game piece really stuck | Try manually reversing, check mechanical binding |
| Motor browns out | Drawing too much current | Add current limits to TalonFX config |
