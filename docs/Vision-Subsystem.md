# Vision Subsystem Documentation

This document explains the complete vision subsystem used for robot localization and pose estimation using Limelight cameras and AprilTags.

## Overview

The vision subsystem enables the robot to:
- **Detect AprilTags** on the field using Limelight cameras
- **Estimate the robot's position** (pose) based on tag detection
- **Correct odometry drift** by integrating vision measurements with the swerve drive's pose estimator
- **Support autonomous and teleop operations** with different strategies for each mode

## Architecture

The vision system consists of three main components:

```
┌─────────────────────────────────────────────────────────────────┐
│                        SKVision (Subsystem)                     │
│  - Main subsystem class                                         │
│  - Manages all Limelights                                       │
│  - Handles pose estimation logic                                │
│  - Integrates with SKSwerve drivetrain                          │
└─────────────────────────────────────────────────────────────────┘
                              │
           ┌──────────────────┴──────────────────┐
           ▼                                     ▼
┌─────────────────────┐              ┌─────────────────────────────┐
│   Limelight.java    │              │     LimelightHelpers.java   │
│  (Wrapper Class)    │              │    (Official FRC Library)   │
│                     │              │                             │
│  - Config settings  │   Uses ──►   │  - NetworkTables interface  │
│  - High-level API   │              │  - JSON parsing             │
│  - Pose retrieval   │              │  - MegaTag algorithms       │
│  - LED control      │              │  - IMU settings             │
└─────────────────────┘              └─────────────────────────────┘
```

## Key Files

| File | Purpose |
|------|---------|
| `SKVision.java` | Main subsystem managing all vision processing and pose estimation |
| `VisionConfig.java` | Configuration for Limelights, standard deviations, and command configs |
| `Limelight.java` | Wrapper class for individual Limelight camera control |
| `LimelightHelpers.java` | Official Limelight library for NetworkTables communication |
| `Konstants.java` | Contains physical Limelight mounting positions and AprilTag field layout |

## How It Works

### 1. Limelight Configuration

Each Limelight camera must be configured with its physical position and orientation on the robot:

```java
public static final class FrontLimelight {
    public static final String kName = "limelight-front";  // Network hostname
    
    // Position from robot center (meters)
    public static final double kForward = 0.3;   // Forward/backward
    public static final double kRight = 0.0;     // Left/right
    public static final double kUp = 0.25;       // Height
    
    // Rotation (degrees)
    public static final double kRoll = 0;        // Side-to-side tilt
    public static final double kPitch = 0;       // Up/down angle
    public static final double kYaw = 180;       // Rotation (180° = facing backward)
}
```

This configuration is wrapped into a `LimelightConfig` object used to create `Limelight` instances.

### 2. Pose Estimation Pipeline

Every robot loop cycle (periodic), the vision subsystem:

1. **Scans for AprilTags** - Checks each Limelight for visible tags
2. **Extracts Vision Measurements** - Gets pose data, timestamps, and quality metrics
3. **Validates the Measurement** - Checks for rejection criteria
4. **Calculates Standard Deviations** - Determines how much to trust the vision data
5. **Integrates with Pose Estimator** - Adds the measurement to the swerve drivetrain's pose estimator

### 3. MegaTag vs MegaTag2

The system supports two Limelight pose algorithms:

| Feature | MegaTag (MT1) | MegaTag2 (MT2) |
|---------|---------------|----------------|
| Tag Requirement | Single or multiple tags | Single or multiple tags |
| Robot Orientation | Calculated from tags | Uses external gyro (IMU) |
| Accuracy | Good with multiple tags | More accurate, especially with single tags |
| Use Case | General pose estimation | Primary algorithm used |

**The system primarily uses MegaTag2** because it leverages the robot's gyro for more accurate heading estimation.

## Rejection Filters

Vision measurements are **rejected** (not integrated) if any of these conditions are true:

| Condition | Threshold | Reason |
|-----------|-----------|--------|
| High ambiguity | > 0.9 | Tag detection is unreliable |
| Pose out of field | Outside field boundaries | Impossible position |
| Rotating too fast | > 4π rad/s | Motion blur affects accuracy |
| Robot "in air" | Z height > 0.25m | Invalid 3D pose |
| Robot tilted | Roll/pitch > 5° | Robot isn't flat |
| Target too small | < 2.5% of frame | Too far away for accuracy |

## Standard Deviations (Trust Levels)

Standard deviations control how much the pose estimator trusts vision data:
- **Lower values** = Higher trust, faster correction
- **Higher values** = Lower trust, smoother but slower correction

The system dynamically adjusts trust based on conditions:

| Condition | XY Trust | Rotation Trust | Description |
|-----------|----------|----------------|-------------|
| Stationary + Close | 0.1m | 0.1° | Robot not moving, tag nearby |
| Strong Multi-tag | 0.1m | 0.1° | Multiple tags, large view |
| Multi-tag | 0.25m | 8° | Multiple tags visible |
| Close Single | 0.5m | 16° | Single tag, close range |
| Proximity | 2.0m | Ignored | Moderate distance |
| Stable | 0.5m | Ignored | Low ambiguity detection |
| Force Reset | 0.001m | 0.001° | Manual pose reset |

**Note:** A rotation trust of 999999° means rotation is essentially ignored (rely on gyro instead).

## Autonomous vs Teleop Behavior

### Autonomous Mode
- **All limelights** contribute pose measurements
- Poses are stored in an array for batch processing
- `autonResetPoseToVision()` can force-apply the best recent pose
- Reviews the last 5 measurements to find an acceptable one

### Teleop/Disabled Mode
- **Only the "best" limelight** contributes measurements
- Best limelight is scored by: `(tag count × 100) + target size`
- Reduces conflicting/noisy measurements from multiple cameras

## Limelight Scoring

When selecting the best Limelight (teleop mode):

```java
score = (tagCount × 100) + targetSize
```

- **Tag Count Weight**: 100 points per tag (prioritizes seeing multiple tags)
- **Target Size**: 0-100 based on how much of the frame the tag occupies

## Available Limelight Methods

### Information Retrieval
| Method | Returns | Description |
|--------|---------|-------------|
| `targetInView()` | boolean | True if any AprilTag is visible |
| `multipleTagsInView()` | boolean | True if 2+ tags are visible |
| `getTagCountInView()` | double | Number of visible tags |
| `getHorizontalOffset()` | double | TX: degrees left/right of crosshair |
| `getVerticalOffset()` | double | TY: degrees up/down of crosshair |
| `getTargetSize()` | double | Percentage of frame (0-100) |
| `getClosestTagID()` | double | ID of the most centered tag |

### Pose Retrieval
| Method | Returns | Description |
|--------|---------|-------------|
| `getRawPose3d()` | Pose3d | Robot pose using MegaTag (3D) |
| `getMegaPose2d()` | Pose2d | Robot pose using MegaTag2 (2D) |
| `getRawPoseTimestamp()` | double | When the pose was captured (seconds) |
| `getDistanceToTagFromCamera()` | double | Direct distance to tag (meters) |

### Control
| Method | Description |
|--------|-------------|
| `setLimelightPipeline(int)` | Change the processing pipeline |
| `setLEDMode(boolean)` | Turn LEDs on/off |
| `blinkLEDs()` | Flash the LEDs |
| `setRobotOrientation(double)` | Set gyro heading for MegaTag2 |
| `setIMUMode(IMUMode)` | Configure IMU usage (EXTERNAL recommended) |

## Vision Command Configurations

The system includes pre-configured settings for vision-assisted commands:

### DriveToPose
Used for autonomous driving to a specific field position:
- PID: P=1, I=0, D=0.001
- Tolerance: 2cm
- Max Speed: 55% of robot max
- Max Acceleration: 2× max speed

### RotateToPose  
Used for rotating to face a specific direction:
- PID: P=0.006, I=0, D=0.00015
- Tolerance: 1.5°
- Max Angular Speed: 10% of robot max
- Max Angular Acceleration: 5× speed

## Integration with Swerve Drive

The vision subsystem requires a reference to `SKSwerve` to:
1. **Read robot rotation** for MegaTag2 heading input
2. **Get chassis speeds** for motion-based rejection
3. **Add vision measurements** to the pose estimator
4. **Set standard deviations** for vision trust

```java
// Example integration in periodic()
ll.setRobotOrientation(m_swerve.getRobotRotation().getDegrees());
m_swerve.addVisionMeasurement(integratedPose, timestamp);
```

## Telemetry & Debugging

The subsystem publishes data to SmartDashboard/AdvantageScope:

- **Vision Driving**: Whether vision is actively updating pose
- **ResetPoseToVision Status**: Result of last forced pose reset
- **Per-Limelight Status**: Integration status for each camera
- **Standard Deviations**: Current trust values (X, Y, Theta)
- **Visible Tag Poses**: 3D poses of detected tags

## AprilTag Field Layout

The system uses the official 2026 (Reefscape/Andymark) field layout:
```java
AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)
```

This provides the exact 3D positions of all AprilTags on the competition field.

## Troubleshooting

### Vision Not Integrating
1. Check SmartDashboard for rejection reasons
2. Verify Limelight is connected (`isCameraConnected()`)
3. Confirm physical mounting matches config values
4. Ensure robot is moving slowly enough

### Inaccurate Positioning
1. Re-measure Limelight position on robot
2. Check roll/pitch/yaw values
3. Verify AprilTag field layout matches your field
4. Lower standard deviations for more aggressive correction

### No Tags Detected
1. Verify Limelight is on pipeline 0 (AprilTag)
2. Check LED brightness settings
3. Ensure camera is focused properly
4. Confirm network connectivity (hostname match)

## Quick Reference: Adding a New Limelight

1. **Define constants** in `Konstants.java`:
```java
public static final class NewLimelight {
    public static final String kName = "limelight-new";
    public static final double kForward = 0.0;
    public static final double kRight = 0.0;
    public static final double kUp = 0.0;
    public static final double kRoll = 0;
    public static final double kPitch = 0;
    public static final double kYaw = 0;
    public static final boolean kAttached = true;
}
```

2. **Create config** in `VisionConfig.java`:
```java
public static final LimelightConfig NEW_CONFIG = 
    new LimelightConfig(NewLimelight.kName)
        .withTranslation(NewLimelight.kForward, NewLimelight.kRight, NewLimelight.kUp)
        .withRotation(NewLimelight.kRoll, NewLimelight.kPitch, NewLimelight.kYaw)
        .withAttached(NewLimelight.kAttached);
```

3. **Add to SKVision.java**:
```java
public final Limelight newLL = new Limelight(VisionConfig.NEW_CONFIG);
public final Limelight[] allLimelights = {frontLL, newLL};
public final Limelight[] poseLimelights = {frontLL, newLL};
```

## Summary

The vision subsystem provides robust robot localization by:
- Using Limelight cameras to detect AprilTags
- Applying the MegaTag2 algorithm for accurate pose estimation
- Filtering out unreliable measurements
- Dynamically adjusting trust levels based on conditions
- Supporting both autonomous multi-camera and teleop single-camera modes

This enables the robot to maintain accurate field position throughout a match, even when odometry drifts due to wheel slip or collisions.
