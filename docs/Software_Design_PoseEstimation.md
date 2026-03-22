# Software Design - Pose Estimation

## System Overview
The pose estimation system implements a **vision-primary** approach using dual Limelight cameras with MegaTag1 AprilTag tracking fused with wheel odometry via WPILib's `SwerveDrivePoseEstimator`. This architecture provides robust, drift-resistant localization throughout the match with automatic initialization during the disabled period.

## Hardware Configuration

### Dual Limelight Setup
- **Front Camera** (`limelight-jhnbron`): Forward-facing at -30° pitch, 0.3m behind robot center, 0.25m above ground
- **Back Camera** (`limelight-tuhbron`): Backward-facing at -30° pitch, -0.3m behind robot center, 0.25m above ground

Both cameras run **MegaTag1** algorithm (ORB feature tracking) for stable pose estimates across frames.

### Field Coordinate System
**WPI Blue Origin**: All pose estimates use blue alliance origin (bottom-left corner = 0,0) regardless of alliance color. Alliance-specific coordinate transformations occur only in PathPlanner for path mirroring.

## Vision Processing

### MegaTag1 Algorithm
**ORB (Oriented FAST and Rotated BRIEF) feature tracking** provides frame-to-frame consistency:
- Tracks AprilTag features across multiple frames
- Reduces pose jitter compared to single-frame detection
- More stable than MegaTag2 for our application
- Handles partial tag occlusion better

### Pose Measurement Integration
Vision measurements added to `SwerveDrivePoseEstimator` every robot periodic cycle (20ms):

```java
drivetrain.addVisionMeasurement(
    poseEstimate.pose,                 // Pose2d from Limelight
    poseEstimate.timestampSeconds,     // Latency-compensated timestamp
    dynamicStandardDeviations          // Measurement uncertainty
)
```

### Dynamic Standard Deviations
Measurement trust varies based on **tag geometry quality**:

**Base Standard Deviations** (meters, meters, radians):
- **Single Tag**: [0.05, 0.05, 0.1] - Very low uncertainty, high trust
- **Multi-Tag**: [0.01, 0.01, 0.02] - Extremely low uncertainty, maximum trust

**Dynamic Scaling** based on:
1. **Tag Count**: More tags = higher confidence
2. **Tag Distance**: Closer tags = better accuracy
3. **Tag Span**: Wider tag spread = better geometry
4. **Tag Area**: Larger tags in image = closer/more reliable

**Formula**:
```java
double xyStdDev = baseXY * (1 + distance * 0.1) * (1 / Math.sqrt(tagCount));
double rotStdDev = baseRot * (1 + distance * 0.2) * (1 / Math.sqrt(Math.max(1, tagCount - 1)));
```

This scaling increases trust for close, multi-tag measurements and decreases trust for distant, single-tag detections.

### Minimal Filtering Philosophy
**Design Decision**: Trust MegaTag1 heavily, filter minimally
- **Only reject** if `tagCount == 0` (no tags visible)
- **No distance filtering** - MegaTag1 works well at all ranges
- **No pose jump filtering** - Causes lag, MegaTag1 handles this internally
- **No ambiguity filtering** - MegaTag1 resolves ambiguity via feature tracking

This aggressive trust maximizes correction of wheel odometry drift while MegaTag1's temporal consistency prevents wild jumps.

## Initial Pose Seeding

### Disabled Period Initialization
During robot disabled (pre-match, between matches), vision continuously updates pose:

```java
@Override
public void disabledPeriodic() {
    Pose2d bestVisionPose = m_robotContainer.getVisionSubsystem().getBestVisionPose();
    if (bestVisionPose != null) {
        m_robotContainer.getDrivetrain().resetPose(bestVisionPose);
    }
}
```

**Camera Selection**: Uses camera with most tags visible (no filtering on quality). This ensures robot knows its position before autonomous/tele-op starts.

### Why This Matters
- **Autonomous path following** requires accurate starting pose
- **Field-centric drive** needs correct heading
- **Target tracking** depends on knowing robot position
- **Eliminates manual pose initialization** ritual

## Odometry Fusion

### SwerveDrivePoseEstimator
WPILib's Kalman filter fuses:
1. **Wheel Odometry** (high frequency, drifts over time)
2. **Vision Measurements** (low frequency, absolute position)
3. **Gyro Rotation** (high frequency, minimal drift)

**Result**: Best of both worlds - high-frequency updates with long-term accuracy.

### Odometry Update Rate
- **Drivetrain Odometry**: 50Hz (every 20ms) - based on wheel encoder positions
- **Vision Measurements**: Variable (dependent on Limelight processing, typically 20-30Hz)
- **Pose Estimator Output**: 50Hz (synchronized with robot periodic)

### Latency Compensation
**Timestamped Measurements**: Vision measurements include latency data from Limelight:
```java
double timestampSeconds = (Timer.getFPGATimestamp() - (latency_ms / 1000.0));
```

`SwerveDrivePoseEstimator` retroactively applies vision corrections to account for capture→processing→transmission delay (typically 20-40ms).

## Telemetry and Visualization

### AdvantageKit Logging
Published every robot periodic cycle:
- **Vision/FrontCamera/Pose**: Front camera's raw pose estimate
- **Vision/BackCamera/Pose**: Back camera's raw pose estimate
- **Vision/FrontCamera/TagCount**: Number of tags seen by front camera
- **Vision/BackCamera/TagCount**: Number of tags seen by back camera
- **Vision/BestInitPose**: Pose used for disabled initialization
- **Odometry/Pose**: Final fused pose from SwerveDrivePoseEstimator

### AdvantageScope Visualization
**3D Field Widget** displays:
- Robot pose trail (odometry history)
- Individual camera pose estimates (semi-transparent)
- AprilTag positions from field layout
- Real-time pose updates

**Robot Telemetry Table**:
- Current pose (X, Y, rotation)
- Vision measurement count
- Tag counts per camera
- Measurement standard deviations

### Performance Monitoring
Throttled telemetry (500ms intervals) includes:
- Vision processing enabled/disabled state
- Camera connection status
- Measurement quality metrics
- Vision subsystem health

## Error Handling and Robustness

### No-Vision Fallback
If vision fails (cameras disconnected, no tags visible):
- System gracefully degrades to **wheel odometry only**
- Pose estimator continues updating from wheel encoders
- Warning logged but robot remains operational
- Accuracy degrades over time (typical wheel odometry drift)

### Toggle Vision Processing
Operator can disable/enable vision via dashboard or controller:
```java
operatorController.rightBumper().onTrue(Commands.runOnce(() -> visionSubsystem.toggleVision()));
```

**Use Cases**:
- Disable if vision measurements appear erroneous
- Debug odometry-only performance
- Reduce computational load if needed

### Camera Failure Handling
If one camera fails:
- Other camera continues providing measurements
- Pose estimation still works (reduced field of view)
- System automatically uses best available camera

## Configuration Tuning

### Standard Deviation Tuning
Located in `VisionConstants.java`:
```java
public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.05, 0.05, 0.1);
public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.01, 0.01, 0.02);
```

**Lower values** = More trust in vision (aggressive correction)  
**Higher values** = More trust in odometry (conservative correction)

Current values heavily favor vision due to MegaTag1 stability.

### Camera Transform Tuning
Camera positions relative to robot center in `VisionConstants.java`:
```java
public static final Transform3d FRONT_CAMERA_TO_ROBOT = new Transform3d(...);
public static final Transform3d BACK_CAMERA_TO_ROBOT = new Transform3d(...);
```

**Critical**: Accurate transforms are essential for correct pose estimation. Measure carefully!

## Design Philosophy

The pose estimation system prioritizes **reliability** and **trust in vision**:
- **Vision-primary approach** aggressively corrects odometry drift
- **Dual cameras** provide 360° field coverage and redundancy
- **MegaTag1 feature tracking** eliminates need for complex filtering
- **Automatic initialization** removes pre-match setup burden
- **Dynamic uncertainty** adapts trust to measurement quality
- **Graceful degradation** maintains operation even with vision failure
- **Comprehensive telemetry** enables rapid debugging and tuning

**Key Insight**: By trusting high-quality vision measurements heavily and letting MegaTag1 handle temporal consistency, we achieve both accuracy and stability without complex filtering logic.
