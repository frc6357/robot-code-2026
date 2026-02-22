# Vision Subsystem — Complete Reference

> **Audience:** Anyone on the team — no prior knowledge of the codebase required.  
> **Last updated:** February 2026

---

## 1. What Does the Vision Subsystem Do?

The vision subsystem uses **Limelight cameras** to detect **AprilTags** on the field and produce a **robot pose estimate** (where the robot thinks it is on the field). That estimate is merged with the drivetrain's own odometry via WPILib's `SwerveDrivePoseEstimator`, giving the robot a more accurate position even when wheel odometry drifts.

In short:

| Capability | How |
|---|---|
| **Detect AprilTags** | Limelight cameras running the AprilTag pipeline |
| **Estimate robot pose** | Limelight MegaTag / MegaTag2 algorithms |
| **Fuse with odometry** | `SwerveDrivePoseEstimator.addVisionMeasurement()` |
| **Filter bad data** | Multi-layer rejection & adaptive standard deviations |
| **Support driver actions** | Button-triggered "reset pose to vision" commands |

---

## 2. Hardware Overview

- **Camera:** Limelight (currently one unit named `"limelight-front"`).
- **Mounting:** 0.3 m forward of center, 0 m right, 0.25 m up; rotated 180° yaw (faces the back of the field relative to the robot's forward).
- **Pipeline 0** is the AprilTag pipeline (used for all pose-estimation tasks).
- **IMU Mode:** Set to `EXTERNAL` on startup — Limelight 4's internal IMU is ignored in favor of the robot's gyro.
- **LEDs:** Turned **off** on startup.
- **Field Map:** Uses the 2026 Rebuilt Andymark AprilTag field layout (`AprilTagFields.k2026RebuiltAndymark`).

---

## 3. Architecture & File Map

```
src/main/java/frc/
├── lib/vision/
│   ├── Limelight.java            ← Low-level wrapper around one Limelight camera
│   ├── LimelightHelpers.java     ← Official Limelight NetworkTables helper (vendor code)
│   ├── frc2025r2.fmap            ← Field maps
│   └── frc2025r2_ANDYMARK.fmap
├── robot/
│   ├── Konstants.java            ← VisionConstants inner class (physical measurements)
│   ├── RobotContainer.java       ← Instantiates SKVision if enabled in Subsystems.json
│   ├── bindings/
│   │   └── SKVisionBinder.java   ← Maps controller buttons → vision commands
│   └── subsystems/vision/
│       ├── SKVision.java         ← The subsystem — filtering, fusion, pose estimation
│       └── VisionConfig.java     ← All tuning constants, thresholds, std-devs, command configs
src/test/java/frc/robot/subsystems/vision/
│   ├── SKVisionTest.java         ← Unit tests for SKVision logic
│   └── VisionConfigTest.java     ← Unit tests for config constants
src/main/deploy/
│   └── Subsystems.json           ← Toggle {"vision": true/false}
```

**Data Flow:**

```
Limelight Camera
      │ (NetworkTables)
      ▼
LimelightHelpers  ──►  Limelight.java  (wrapper with config & safety)
                              │
                              ▼
                        SKVision.java  (filtering + scoring)
                              │
                              ▼
                    SKSwerve.addVisionMeasurement()
                              │
                              ▼
                  SwerveDrivePoseEstimator  (WPILib)
```

---

## 4. Configuration & Constants

### 4.1 Limelight Physical Config (`Konstants` → `VisionConstants`)

Defined in `Konstants.java` inside the `VisionConstants` class:

| Constant | Value | Meaning |
|---|---|---|
| `kAprilTagFieldLayout` | `k2026RebuiltAndymark` | Which AprilTag map to use |
| `kAprilTagPipeline` | `0` | Default pipeline index |

Each camera gets its own inner class (e.g. `FrontLimelight`):

| Constant | Value | Meaning |
|---|---|---|
| `kName` | `"limelight-front"` | NetworkTables hostname — **must match** the Limelight's config page |
| `kForward` | `0.3` m | Distance forward from robot center |
| `kRight` | `0.0` m | Distance right from robot center |
| `kUp` | `0.25` m | Distance up from robot center |
| `kRoll` | `0°` | Camera roll |
| `kPitch` | `0°` | Camera pitch |
| `kYaw` | `180°` | Camera yaw (faces backward) |
| `kAttached` | `true` | Set `false` to software-disable the camera |

### 4.2 `VisionConfig.java`

This file holds **all tuning knobs** for the vision subsystem. It is organized into:

#### Default Standard Deviations

```java
VISION_STD_DEV_X     = 0.5      // meters
VISION_STD_DEV_Y     = 0.5      // meters
VISION_STD_DEV_THETA = 99999999 // effectively ignore rotation from vision
```

These are the *starting* values. They get **overwritten** every cycle by the filtering logic based on how much the system trusts the current measurement. Smaller = more trust.

#### `PoseStdDevs` Record

A `record(double xy, double theta)` that pairs an XY standard deviation with a rotational standard deviation. Pre-built constants:

| Name | XY | Theta | When Used |
|---|---|---|---|
| `STATIONARY_CLOSE` | 0.1 | 0.1 | Robot still + tag very close |
| `STRONG_MULTI` | 0.1 | 0.1 | Multiple tags, large target area |
| `MULTI_TAG` | 0.25 | 8.0 | Multiple tags, medium target area |
| `CLOSE_SINGLE` | 0.5 | 16.0 | Single tag, very close |
| `PROXIMITY` | 2.0 | 999999 | Moderate tag, close pose match |
| `STABLE` | 0.5 | 999999 | Low ambiguity, any distance |
| `RESET` | 0.001 | 0.001 | Forced pose reset — highest trust |
| `HIGH_AMBIGUITY_PENALTY` | 0 | 15.0 | Penalty applied on top of base |
| `HIGH_ROTATION_PENALTY` | 0 | 15.0 | Penalty applied on top of base |

> **Key insight:** A theta of `999999` means "ignore the rotation from vision entirely — just trust the gyro."

#### `Thresholds` Class

Hard limits that cause a measurement to be rejected or treated differently:

| Category | Constant | Value | Purpose |
|---|---|---|---|
| **Physical** | `MAX_HEIGHT` | 0.25 m | Reject if tag says bot is floating |
| | `MAX_TILT` | 5° | Reject if roll/pitch is unreasonable |
| **Ambiguity** | `MAX_AMBIGUITY` | 0.9 | Reject measurement |
| | `HIGH_AMBIGUITY` | 0.5 | Apply theta penalty |
| | `LOW_AMBIGUITY` | 0.25 | Qualify for "stable" integration |
| **Motion** | `MAX_ROTATION_SPEED` | 4π rad/s | Reject — spinning too fast |
| | `HIGH_ROTATION_SPEED` | 0.5 rad/s | Apply theta penalty |
| | `STATIONARY_SPEED` | 0.2 m/s | Qualifies for "stationary" integration |
| **Target Size** | `MIN_SIZE` | 0.025 | Reject — tag too small to trust |
| | `VERY_CLOSE_SIZE` | 0.4 | Stationary-close integration |
| | `CLOSE_SIZE` | 0.8 | Close-single integration |
| | `MODERATE_SIZE` | 0.1 | Proximity integration |
| | `SMALL_MULTI_SIZE` | 0.05 | Multi-tag integration |
| | `LARGE_MULTI_SIZE` | 0.09 | Strong multi integration |
| | `STABLE_SIZE` | 0.03 | Stable integration |
| **Pose Diff** | `CLOSE_POSE_DIFF` | 0.5 m | Close-single criterion |
| | `PROXIMITY_POSE_DIFF` | 0.3 m | Proximity criterion |
| **Scoring** | `TAG_COUNT_WEIGHT` | 100.0 | Weight for best-LL selection |

### 4.3 Enabling / Disabling Vision

In `src/main/deploy/Subsystems.json`:

```json
{
    "vision": true
}
```

Set to `true` to create the `SKVision` subsystem on boot. Set to `false` to skip it entirely (the `Optional<SKVision>` will be empty and all bindings become no-ops).

You can also disable individual cameras by setting `kAttached = false` in their constants class.

---

## 5. The Limelight Wrapper (`Limelight.java`)

Located at `src/main/java/frc/lib/vision/Limelight.java`. This is a **reusable library class** that wraps a single Limelight camera.

### 5.1 `LimelightConfig` Builder

```java
LimelightConfig config = new LimelightConfig("limelight-front")
    .withTranslation(forward, right, up)   // meters from robot center
    .withRotation(roll, pitch, yaw)        // degrees
    .withAttached(true);                   // software enable/disable
```

Uses **Lombok** `@Getter` / `@Setter` annotations to auto-generate accessors.

### 5.2 Key Methods

| Method | Returns | Description |
|---|---|---|
| `targetInView()` | `boolean` | True if the camera sees at least one valid target |
| `multipleTagsInView()` | `boolean` | True if 2+ AprilTags visible |
| `getTagCountInView()` | `double` | Number of tags visible |
| `getHorizontalOffset()` | `double` | TX — horizontal degrees to target center |
| `getVerticalOffset()` | `double` | TY — vertical degrees to target center |
| `getTargetSize()` | `double` | TA — % of camera view filled by target (0–100) |
| `getClosestTagID()` | `double` | Fiducial ID of the most-centered tag |
| `getRawPose3d()` | `Pose3d` | MegaTag1 bot pose (blue-alliance origin) |
| `getMegaPose2d()` | `Pose2d` | MegaTag2 bot pose (blue-alliance origin) |
| `getRawPoseTimestamp()` | `double` | Timestamp of the MegaTag1 pose (seconds) |
| `getMegaPoseTimestamp()` | `double` | Timestamp of the MegaTag2 pose (seconds) |
| `getCameraPoseTS3d()` | `Pose3d` | Camera pose relative to the target (target-space) |
| `getRawFiducial()` | `RawFiducial[]` | Raw data for every visible tag (ID, ambiguity, etc.) |
| `getDistanceToTagFromCamera()` | `double` | Euclidean distance (m) from camera to closest tag |
| `hasAccuratePose()` | `boolean` | True if multi-tag AND target size > 0.1 |
| `setRobotOrientation(deg)` | `void` | Feeds gyro heading to LL for MegaTag2 |
| `setLimelightPipeline(idx)` | `void` | Switch the active pipeline |
| `setLEDMode(enabled)` | `void` | Force LEDs on/off |
| `blinkLEDs()` | `void` | Force LEDs to blink |
| `setIMUMode(mode)` | `void` | `EXTERNAL` / `FUSED` / `INTERNAL` — for Limelight 4 |
| `isCameraConnected()` | `boolean` | Checks if the camera is sending data |
| `sendValidStatus(msg)` | `void` | Logs that the camera is integrating + sets integrating flag |
| `sendInvalidStatus(msg)` | `void` | Logs that the camera was rejected + clears integrating flag |

---

## 6. The Vision Subsystem (`SKVision.java`)

Located at `src/main/java/frc/robot/subsystems/vision/SKVision.java`. This is a WPILib `SubsystemBase` that orchestrates all vision processing.

### 6.1 Lifecycle (Construction & Periodic)

**Constructor** (`SKVision(Optional<SKSwerve>)`):
1. Requires an `SKSwerve` reference (throws if absent — vision cannot work without the drivetrain).
2. Calls `startupLimelights()`:
   - Sets each LL's IMU mode to `EXTERNAL`.
   - Turns off LEDs.
   - Logs "Disabled" for any unattached camera.

**`periodic()` — runs every 20 ms:**
1. Clear the visible-tag list.
2. For each pose-limelight:
   - Send the robot's current gyro heading to the Limelight (needed for MegaTag2).
   - Scan for visible AprilTags → populate `tagIDsInView`.
3. Publish telemetry (`SmartDashboard` + NetworkTables `StructArrayPublisher` of tag poses).
4. Call `estimatePose()` — the main pose-fusion logic.

### 6.2 Pose Estimation Pipeline

`estimatePose()` has two distinct paths depending on robot mode:

#### Autonomous Mode (`updatePoseAutonomous`)
- Iterates over **all** pose-limelights.
- For every camera that sees a tag, runs `updatePoseSingleCam()` which calls `addFilteredLimelightInput()`.
- Measurements are filtered and (if accepted) fed into the pose estimator immediately.

#### Teleop / Disabled Mode (`updatePoseTeleop`)
- Scores all pose-limelights using `getBestLimelight()`.
- **Only the best camera** feeds data into the pose estimator; others get a "Rejected: Not best Limelight" status.
- This drastically reduces noisy / conflicting measurements.

### 6.3 Standard Deviations & Trust Levels

WPILib's `SwerveDrivePoseEstimator` uses **standard deviations** to decide how much to trust each vision update vs. odometry.

- **Low std-devs** (e.g. `0.1`) = high trust → the pose estimate moves toward the vision measurement aggressively.
- **High std-devs** (e.g. `999999`) = almost zero trust → the measurement is basically ignored for that axis.
- **Theta std-dev of `999999`** means "do NOT adjust rotation from vision — trust the gyro."

The system **dynamically selects std-devs** every cycle based on conditions (see §6.4).

### 6.4 Integration Strategy (Decision Tree)

When a measurement passes all rejection filters, it enters `calculateIntegrationStdDevs()`. The conditions are checked in order (first match wins):

```
1. STATIONARY + VERY CLOSE TAG?
   └─ speed ≤ 0.2 m/s AND targetSize > 0.4
   └─ Use: STATIONARY_CLOSE (0.1, 0.1) — highest trust

2. MULTIPLE TAGS?
   └─ multiTags AND targetSize > 0.05
   ├─ targetSize > 0.09 → STRONG_MULTI (0.1, 0.1)
   └─ else → MULTI_TAG (0.25, 8.0)

3. CLOSE SINGLE TAG?
   └─ targetSize > 0.8 AND poseDiff < 0.5m
   └─ Use: CLOSE_SINGLE (0.5, 16.0)

4. PROXIMITY?
   └─ targetSize > 0.1 AND poseDiff < 0.3m
   └─ Use: PROXIMITY (2.0, 999999)

5. STABLE (low ambiguity)?
   └─ ambiguity < 0.25 AND targetSize ≥ 0.03
   └─ Use: STABLE (0.5, 999999)

6. NONE OF THE ABOVE?
   └─ REJECT ("catch rejection")
```

**After selecting base std-devs, penalties may be applied:**

- If `highestAmbiguity > 0.5` → theta capped at 15°.
- If `|rotationSpeed| ≥ 0.5 rad/s` → theta capped at 15°.

The final XY and theta std-devs are sent to `SwerveDrivePoseEstimator` along with the `Pose2d` and timestamp.

### 6.5 Rejection Criteria

Before even reaching the integration strategy, a measurement can be **hard-rejected** in `shouldRejectPose()`:

| Check | Threshold | Log Message |
|---|---|---|
| Tag ambiguity too high | > 0.9 | `"ambiguity rejection"` |
| Pose outside field bounds | `Field.poseOutOfField()` | `"bound rejection - pose OOB"` |
| Robot rotating too fast | ≥ 4π rad/s | `"rot speed rejection - too fast"` |
| Robot appears to be floating | Z > 0.25 m | `"height rejection - in air"` |
| Robot appears tilted | roll/pitch > 5° | `"roll/pitch rejection - tilted"` |
| Target too small to trust | ≤ 0.025 | `"size rejection - target too small"` |

There is also a basic requirement check in `validateBasicRequirements()`:
- Camera must see at least one target (`targetInView()`), otherwise it logs `"no tag found rejection"`.

### 6.6 Best Limelight Selection

`getBestLimelight()` scores each camera:

```
score = (tagCount × 100) + targetSize
```

The camera with the highest score wins. This means:
- Seeing 1 tag = 100 points; 2 tags = 200 points.
- Target size (0–100 range) acts as a tiebreaker / proximity bonus.

### 6.7 Pose Reset Methods

These are explicit one-shot actions (usually triggered by a button press, not running continuously):

| Method | When to Use | What It Does |
|---|---|---|
| `forcePoseToVision()` | **Y button** — "I trust vision right now, slam the pose" | Gets the best limelight's raw 3D pose and calls `m_swerve.resetPose()`. Skipped in simulation. |
| `resetPoseToVision()` | **B button** — "Soft reset with validation" | Gets the best limelight, validates the pose (height, tilt, OOB checks), then feeds it as a measurement with `RESET` std-devs (0.001) for maximum trust. |
| `autonResetPoseToVision()` | Called from auto commands | Looks at the last 5 stored multi-cam poses and tries to integrate the most recent valid one. Prints success/failure to console. |

### 6.8 Autonomous Pose Batching

During auto, `updatePoseMultiCam()` stores `Trio<Pose3d, Pose2d, Double>` (3D pose, MegaTag2 pose, timestamp) into `multiCamPoses`. When an auto command calls `autonResetPoseToVision()`, it scans the last 5 entries and integrates the first one that passes validation.

---

## 7. Button Bindings (`SKVisionBinder.java`)

| Button | Action | Command Name |
|---|---|---|
| **B** (Driver) | Validated pose reset — runs `resetPoseToVision()` | `"VisionResetPose"` |
| **Y** (Driver) | Force pose reset — runs `forcePoseToVision()` | `"VisionForceResetPose"` |
| **D-pad Up** | *(Planned)* Enable vision | — |
| **D-pad Down** | *(Planned)* Disable vision | — |

Both B and Y are `onTrue` — they fire once when the button is pressed.

---

## 8. Command Configs (DriveToPose / RotateToPose)

`VisionConfig` contains two inner classes that extend `MultiLimelightCommandConfig`, providing pre-configured PID and motion profile settings for vision-assisted driving:

### `DriveToPose`

| Parameter | Value |
|---|---|
| PID | P=1, I=0, D=0.001 |
| Tolerance | 0.02 m |
| Max Velocity | 55% of robot max speed |
| Max Acceleration | 2× max velocity |
| Error | 0.01 m |
| Pipeline | AprilTag (0) |
| Limelights | All pose limelights |

### `RotateToPose`

| Parameter | Value |
|---|---|
| PID | P=0.006, I=0, D=0.00015 |
| Tolerance | 1.5° |
| Max Velocity | 10% of max angular rate |
| Max Acceleration | 5× max velocity |
| Error | 1° |
| Pipeline | AprilTag (0) |
| Limelights | All pose limelights |

Get a config instance with `DriveToPose.getConfig()` or `RotateToPose.getConfig()`.

These configs are designed to be passed to commands that drive/rotate the robot toward a target pose using vision feedback.

---

## 9. Telemetry & Debugging

### SmartDashboard / Sendable

SKVision publishes itself under `"Vision"` in SmartDashboard with:

| Key | Type | Description |
|---|---|---|
| `Vision Driving` | Boolean | Whether vision is currently driving the robot |
| `ResetPoseToVision Status` | String | Last result of a pose reset attempt |
| `StdDevs/Vision Std Dev X` | Double | Current X std-dev being used |
| `StdDevs/Y` | Double | Current Y std-dev |
| `StdDevs/Theta` | Double | Current theta std-dev |
| `[LL name] Status` | String | Per-limelight status (e.g. `"Stationary close integration"`, `"ambiguity rejection"`) |

### NetworkTables

- **`VisibleTargetPoses`** — A `StructArrayPublisher<Pose3d>` of all visible AprilTag poses. Useful for visualizing in AdvantageScope or other NT viewers.

### Limelight Status Strings

Each Limelight's `logStatus` is updated in real time to indicate what happened on the most recent cycle:

| Status | Meaning |
|---|---|
| `"Disabled"` | Camera not attached |
| `"no tag found rejection"` | No targets in view |
| `"ambiguity rejection"` | Ambiguity too high (> 0.9) |
| `"bound rejection - pose OOB"` | Estimated pose is outside the field |
| `"rot speed rejection - too fast"` | Robot spinning too quickly |
| `"height rejection - in air"` | Z height too large |
| `"roll/pitch rejection - tilted"` | Robot appears tilted |
| `"size rejection - target too small"` | Target area below minimum |
| `"catch rejection: X poseDiff"` | No integration criteria met |
| `"Rejected: Not best Limelight"` | Another camera scored higher |
| `"Stationary close integration"` | Integrated with highest trust |
| `"Strong Multi integration"` | Multi-tag, large area — high trust |
| `"Multi integration"` | Multi-tag, medium area |
| `"Close integration"` | Single tag, very close |
| `"Proximity integration"` | Moderate tag, nearby pose |
| `"Stable integration"` | Low ambiguity, any distance |

---

## 10. Testing

Tests live in `src/test/java/frc/robot/subsystems/vision/`:

### `VisionConfigTest.java`
- Validates all `PoseStdDevs` preset values.
- Validates all `Thresholds` constant values and logical ordering (e.g., `LOW_AMBIGUITY < HIGH_AMBIGUITY < MAX_AMBIGUITY`).
- Regression test: documents the velocity-magnitude bug fix (must use `Math.hypot`, not addition).

### `SKVisionTest.java`
- Uses **Mockito** to mock `SKSwerve` and `Limelight`.
- Tests `findHighestAmbiguity()` — including the regression fix (was initialized to 2, now -1).
- Tests `shouldRejectPose()` for each rejection condition (ambiguity, rotation, height, tilt, size) and for valid poses.
- Tests `calculateIntegrationStdDevs()` for every integration path (stationary, multi, strong-multi, close, proximity, stable, rejection).
- Regression test for velocity-magnitude bug: `(0.5, -0.5)` m/s should NOT be stationary (old code added vx+vy, new code uses `Math.hypot`).

Run tests with:
```
./gradlew test
```

---

## 11. How to Add a New Limelight

### Step 1 — Add physical constants in `Konstants.java`

Inside `VisionConstants`, create a new inner class:

```java
public static final class RightLimelight {
    public static final String kName = "limelight-right"; // Must match LL dashboard hostname
    public static final double kForward = 0.17;  // meters forward of center
    public static final double kRight = 0.27;    // meters right of center
    public static final double kUp = 0.28;       // meters up from center
    public static final double kRoll = 0;        // degrees
    public static final double kPitch = 0;       // degrees
    public static final double kYaw = 5;         // degrees
    public static final boolean kAttached = true;
}
```

### Step 2 — Create a `LimelightConfig` in `VisionConfig.java`

```java
public static final String RIGHT_LL = RightLimelight.kName;
public static final int RIGHT_TAG_PIPELINE = kAprilTagPipeline;
public static final LimelightConfig RIGHT_CONFIG = 
    new LimelightConfig(RightLimelight.kName)
        .withTranslation(RightLimelight.kForward, RightLimelight.kRight, RightLimelight.kUp)
        .withRotation(RightLimelight.kRoll, RightLimelight.kPitch, RightLimelight.kYaw)
        .withAttached(RightLimelight.kAttached);
```

### Step 3 — Register the Limelight in `SKVision.java`

```java
// Declare the limelight
public final Limelight rightLL = new Limelight(VisionConfig.RIGHT_CONFIG); // limelight-right

// Add to the allLimelights array
public final Limelight[] allLimelights = {frontLL, rightLL};

// Add to poseLimelights (order by preference: best-view first)
public final Limelight[] poseLimelights = {frontLL, rightLL};
```

### Step 4 — Deploy & Test

1. Set `"vision": true` in `Subsystems.json`.
2. Deploy the code: `./gradlew deploy`.
3. Open SmartDashboard / AdvantageScope and check the new limelight's status string.
4. Verify the new camera contributes to pose estimation by observing the `VisibleTargetPoses` NetworkTables topic.

---

## 12. Quick-Reference Cheat Sheet

| Question | Answer |
|---|---|
| **Where do I change camera position?** | `Konstants.java` → `VisionConstants` → `FrontLimelight` |
| **Where do I tune filtering?** | `VisionConfig.java` → `Thresholds` and `PoseStdDevs` |
| **Where is the main loop?** | `SKVision.periodic()` → `estimatePose()` |
| **How do I disable vision?** | Set `"vision": false` in `Subsystems.json` |
| **How do I disable one camera?** | Set `kAttached = false` in its constants class |
| **Driver resets pose?** | Press **B** (validated) or **Y** (forced) |
| **Why was a measurement rejected?** | Check the limelight's status string in SmartDashboard |
| **How do I add a camera?** | See [§11](#11-how-to-add-a-new-limelight) |
| **Where are the tests?** | `src/test/java/frc/robot/subsystems/vision/` |
| **What is MegaTag2?** | Limelight's algorithm that uses the robot's gyro heading + tag detections for a more accurate 2D pose. Fed via `setRobotOrientation()`. |
