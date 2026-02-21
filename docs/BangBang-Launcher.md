# Bang-Bang Launcher Subsystem | 6357 Spring Konstant

<img src="images/sk-logo.png" alt="Spring Konstant Logo" width="200">

The Bang-Bang launcher subsystem (`BangBangLauncher`) controls a dual-motor flywheel using a hybrid **bang-bang + duty cycle** control strategy instead of traditional PID. It dynamically switches between Torque Current FOC and Duty Cycle modes based on how far the flywheel is from its target velocity, achieving fast spinup with precise steady-state control.

> **Note**: This is the **bang-bang launcher**. The team also has a [Standard PID Launcher](Launcher.md) that uses CTRE velocity closed-loop. Only one launcher subsystem is enabled at a time via `Subsystems.json`.

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Hardware](#hardware)
- [Control Strategy](#control-strategy)
- [Control Mode Switching](#control-mode-switching)
- [Debouncing](#debouncing)
- [Commands](#commands)
- [State Machine Integration](#state-machine-integration)
- [Runtime Tuning](#runtime-tuning)
- [Telemetry](#telemetry)
- [Key Classes & Files](#key-classes--files)
- [Troubleshooting](#troubleshooting)

---

## Overview

Unlike the standard launcher which uses onboard TalonFX PID, the bang-bang launcher:

1. Uses **Duty Cycle feedforward** as the primary control mode  
2. Switches to **Torque Current FOC** bang-bang when close (but not too close) to the target  
3. Uses **debouncers** to prevent rapid mode switching  
4. Has a separate "at goal" debouncer to ensure the flywheel is truly at speed  
5. All key parameters are **runtime-tunable** via SKPreferences  

This approach provides very fast spinup (full duty cycle FF) with tight steady-state control (torque current bang-bang).

<p align="center">
  <img src="images/bangbang-overview-diagram.png" alt="Bang-bang control strategy overview" width="550">
</p>

---

## Architecture

```
BangBangLauncher (SubsystemBase)
├── TalonFX mainMotor              ← Primary motor (CW positive, Coast)
├── TalonFX followingMotor         ← Follows primary (CCW positive, Coast)
├── DutyCycleOut control           ← Feedforward-based duty cycle mode
├── TorqueCurrentFOC control       ← Bang-bang torque current mode
├── CoastOut control               ← Idle coast mode
├── Debouncer torqueCurrentDebouncer ← Prevents rapid mode switching
├── Debouncer atGoalDebouncer      ← Ensures stable "at goal" signal
└── SKPreferences                  ← Runtime-tunable parameters
```

---

## Hardware

| Component | Type | CAN ID | CAN Bus | Direction |
|-----------|------|--------|---------|-----------|
| Main Motor | TalonFX | 50 | roboRIO | Clockwise Positive |
| Following Motor | TalonFX | 51 | roboRIO | CounterClockwise Positive |

### Configuration

| Property | Main Motor | Following Motor |
|----------|-----------|-----------------|
| Neutral Mode | Coast | Coast |
| Inverted | CW Positive | CCW Positive |
| Velocity Filter Time Constant | 0.01 s | — |
| Torque Neutral Deadband | 4 A | — |
| Control | Various (see below) | Follower (opposed) |

> **Follower**: The following motor uses `Follower` control with `MotorAlignmentValue.Opposed`, spinning in the opposite direction.

---

## Control Strategy

The launcher has three control modes that it switches between dynamically:

### 1. DUTY_CYCLE_BANG_BANG (Primary)

Used when the flywheel is **far from** or **very close to** the target velocity.

$$\text{output} = \text{clamp}(\text{targetRPS} \times \text{dutyCycleFF}, -0.75, 0.75)$$

- FOC is **enabled** for better efficiency  
- The `dutyCycleFF` preference acts as a simple linear feedforward  
- Default FF: 0.01489 %/rps  
- Output clamped to ±75% to prevent brownouts  

### 2. TORQUE_CURRENT_BANG_BANG (Close Range)

Used when the flywheel is **close but not at** the target velocity.

$$\text{output} = \text{sign}(\text{targetRPS} - \text{actualRPS}) \times \text{torqueCurrentOutput}$$

- Applies full torque in the direction needed to reach the target  
- Default torque output: 54 A  
- Pure bang-bang: either full positive or full negative torque  
- Very aggressive control that brings the speed to exactly the setpoint  

### 3. COAST

Used when the launcher is **stopped**. The motor is allowed to spin down naturally.

<p align="center">
  <img src="images/bangbang-control-modes.png" alt="Graph showing duty cycle vs torque current zones" width="500">
</p>

---

## Control Mode Switching

The mode selection happens in `runVelocity()` every loop:

```
1. Calculate error: |actualRPS - targetRPS|
2. Check: inTolerance? (error ≤ torqueCurrentControlTolerance)
3. Check: tooFar? (error > 16 × torqueCurrentControlTolerance)
4. Debounce: runTorqueCurrent = debounce(!inTolerance && !tooFar)
5. If runTorqueCurrent → TORQUE_CURRENT_BANG_BANG
   Else → DUTY_CYCLE_BANG_BANG
```

### Zone Diagram

```
Error:  |←── tooFar ──→|←── Torque Current ──→|←── In Tolerance ──→|
        |               |                       |                    |
        |  DUTY_CYCLE   |  TORQUE_CURRENT_BB    |   DUTY_CYCLE       |
        |               |                       |                    |
        16x tolerance   tolerance               0                   
```

### Why Both Zones Use Duty Cycle

- **Too far**: Duty cycle FF provides the fastest spinup since it's proportional to target speed  
- **In tolerance**: Duty cycle FF maintains the speed smoothly without the aggressiveness of torque current  
- **Close but not there**: Torque current bang-bang provides the precise kick needed to reach the target  

---

## Debouncing

Two debouncers prevent noisy signals:

### 1. Torque Current Mode Debouncer

| Parameter | Default | SKPreference Key |
|-----------|---------|-----------------|
| Debounce time | 0.025 s | `TorqueCurrentControl Debounce (sec)` |
| Type | Falling edge | — |

Prevents the system from rapidly switching between duty cycle and torque current modes when the velocity is near a threshold boundary.

### 2. At-Goal Debouncer

| Parameter | Default | SKPreference Key |
|-----------|---------|-----------------|
| Debounce time | 0.2 s | `AtGoalVelocity Debounce (sec)` |
| Type | Falling edge | — |

The flywheel must be within tolerance for 200ms before `atGoal` reports `true`. This prevents false "ready to fire" signals from brief velocity dips.

---

## Commands

### `runFixedSpeedCommand(Supplier<AngularVelocity>)`

Runs the flywheel at the supplied velocity while the command is active. Stops when the command ends.

```java
// Usage in binder:
launcher.runFixedSpeedCommand(() -> RotationsPerSecond.of(12.25))
```

This is a `runEnd()` command — it continuously calls `runVelocity()` during execution and `stop()` on end.

### `stopCommand()`

Instantly switches to COAST mode and resets the target velocity.

---

## State Machine Integration

The launcher responds to the robot's **MacroState**:

| State | Behavior | Trigger |
|-------|----------|---------|
| IDLE + Right Trigger | Run at half manual velocity | Operator RT held |
| SCORING | Run at half manual velocity | Automatic |
| STEADY_STREAM_SCORING | Run at half manual velocity | Automatic |
| SHUTTLING | Run at half manual velocity | Automatic |
| STEADY_STREAM_SHUTTLING | Run at half manual velocity | Automatic |
| All other states | Stopped (coast) | — |

### Manual Velocity

The manual velocity is controlled by the SKPreference `BBLauncher/ManualShootVelocity (rps)` (default: 24.5 RPS). The actual command runs at **half** this value (12.25 RPS).

---

## Runtime Tuning

All key parameters are adjustable at runtime via `SKPreferences`:

| SKPreference Key | Default | Description |
|-----------------|---------|-------------|
| `TorqueCurrent Output (A)` | 54.0 | Torque current magnitude in bang-bang mode |
| `TorqueCurrentControl Tolerance (rps)` | 0.85 | In-tolerance threshold for mode switching |
| `TorqueCurrentControl Debounce (sec)` | 0.025 | Debounce time for mode switching |
| `DutyCycle FF (%/rps)` | 0.01489 | Feedforward gain for duty cycle mode |
| `AtGoalVelocity Debounce (sec)` | 0.2 | Debounce time for "at goal" signal |
| `BBLauncher/ManualShootVelocity (rps)` | 24.5 | Manual target velocity (halved in commands) |

<p align="center">
  <img src="images/bangbang-tuning-dashboard.png" alt="SmartDashboard with BBLauncher tuning values" width="500">
</p>

---

## Telemetry

### SmartDashboard

| Key | Type | Description |
|-----|------|-------------|
| `BBLauncher` | Subsystem data | Full Sendable |
| `BBLauncher/AtGoal` | boolean | Flywheel at target (debounced) |
| `BBLauncher/Target Velocity (rps)` | double | Current target RPS (also settable!) |
| `BBLauncher/Current Velocity (rps)` | double | Actual motor velocity |
| `BBLauncher/Control Mode` | String | DUTY_CYCLE_BANG_BANG / TORQUE_CURRENT_BANG_BANG / COAST |
| `BBLauncher/Too Far` | boolean | Error too large for torque current |

> **Interactive**: The `Target Velocity` property has a setter — you can type a value directly into SmartDashboard to test the launcher without a controller.

---

## Key Classes & Files

| File | Purpose |
|------|---------|
| `subsystems/launcher/BangBangLauncher.java` | Main subsystem — control modes, debouncing, tuning |
| `bindings/SK26BBLauncherBinder.java` | Controller bindings with state machine integration |
| `Konstants.java` → `LauncherConstants` | Shared constants (wheel radius, unjam, etc.) |
| `Ports.java` → `LauncherPorts` | CAN IDs (shared with standard launcher) |

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Flywheel doesn't spin | Wrong CAN IDs or motor not powered | Check CAN IDs (50, 51), check power |
| Flywheel oscillates between modes | Debounce too short | Increase `TorqueCurrentControl Debounce` |
| "AtGoal" flickers rapidly | AtGoal debounce too short | Increase `AtGoalVelocity Debounce` |
| Flywheel spinup too slow | Duty cycle FF too low | Increase `DutyCycle FF (%/rps)` |
| Flywheel overshoots target | Torque current output too high | Reduce `TorqueCurrent Output (A)` |
| Motor trips over-current | Torque current too aggressive | Reduce `TorqueCurrent Output (A)` and check wiring |
| Follower spins same direction | Wrong motor alignment | Verify `MotorAlignmentValue.Opposed` |
| Coast mode doesn't coast | Wrong neutral mode | Verify `NeutralModeValue.Coast` in config |
| `ControlMode` stuck on COAST | `stop()` was called, target is zero | Call `runFixedSpeedCommand()` again |
| Brownout during spinup | Duty cycle clamped too high | Reduce the 0.75 clamp in periodic |
| Config error on startup | CAN bus issue | Check "Main motor config failed" error in Driver Station |

<p align="center">
  <img src="images/bangbang-velocity-graph.png" alt="Graph showing velocity response with mode transitions marked" width="550">
</p>
