# Turret Target Tracking - Summary

## What Was Added

### New Files Created

1. **`TrackTargetCommand.java`** - Main command for automatic turret tracking
   - Continuously calculates angle from robot to target
   - Compensates for chassis rotation
   - Respects turret limits (50° to 360°)
   - Provides detailed telemetry

2. **`TURRET_TRACKING_GUIDE.md`** - Comprehensive usage guide
   - Multiple usage examples
   - Coordinate system explanation
   - Integration patterns
   - Debugging tips

### Modified Files

1. **`RobotContainer.java`** - Added commented example button bindings
   - Ready-to-use examples for tracking blue/red hubs
   - Just uncomment to enable

## Quick Start

### To Enable Turret Tracking:

1. Open `RobotContainer.java`
2. Find the "AUTOMATIC TARGET TRACKING" section (around line 283)
3. Uncomment one of the examples:

```java
// Hold operator B button to track blue alliance hub
operatorController.b().whileTrue(
    new TrackTargetCommand(
        turretSubsystem,
        drivetrain,
        () -> frc.robot.constants.FieldConstants.BLUE_HUB
    )
);
```

4. Build and deploy
5. Hold the B button on operator controller - turret will track the blue hub!

## How It Works

### The Math
1. **Get robot pose** from accurate vision-based pose estimation
2. **Calculate field angle** from robot to target
3. **Convert to robot-relative** angle (subtract robot heading)
4. **Convert to turret angle** (add 180° since turret 180° = robot forward)
5. **Command turret** to calculated angle

### Key Features
- ✅ **Chassis-Independent**: Robot can drive/rotate freely while turret tracks
- ✅ **Limit Safe**: Automatically clamps to turret limits (50°-360°)
- ✅ **Real-time**: Updates every loop cycle (50Hz)
- ✅ **Telemetry**: Publishes detailed tracking data to SmartDashboard

## Testing Steps

### 1. Static Test (Robot Not Moving)
1. Place robot at known field position
2. Enable tracking command
3. Verify turret points at target
4. Rotate robot base manually - turret should counter-rotate to maintain aim

### 2. Dynamic Test (Robot Driving)
1. Enable tracking command
2. Drive robot around the field
3. Watch telemetry: `TrackTarget/TurretAtTarget` should be mostly true
4. Turret should smoothly track target despite robot movement

### 3. Limit Test
1. Drive robot to position where target would be at ~45° (outside limits)
2. Enable tracking
3. Verify turret moves to 50° limit (not past it)
4. Check `TrackTarget/OutOfRange` is true

## Available Telemetry

Published to SmartDashboard when enabled:

| Key | Description |
|-----|-------------|
| `TrackTarget/TargetX` | Target X position (meters) |
| `TrackTarget/TargetY` | Target Y position (meters) |
| `TrackTarget/DistanceToTarget` | Distance to target (meters) |
| `TrackTarget/FieldAngleToTarget` | Field-relative angle to target |
| `TrackTarget/RobotRelativeAngle` | Robot-relative angle to target |
| `TrackTarget/TurretCommandAngle` | Commanded turret angle (0-360°) |
| `TrackTarget/TurretAtTarget` | True when turret is aimed |
| `TrackTarget/OutOfRange` | True when target outside limits |

## Advanced Usage

### Track Alliance-Specific Target
```java
import edu.wpi.first.wpilibj.DriverStation;

operatorController.y().whileTrue(
    new TrackTargetCommand(
        turretSubsystem,
        drivetrain,
        () -> {
            var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            return alliance == Alliance.Blue ? 
                FieldConstants.BLUE_HUB : 
                FieldConstants.RED_HUB;
        }
    )
);
```

### Make It Default Command
```java
// In RobotContainer constructor:
turretSubsystem.setDefaultCommand(
    new TrackTargetCommand(
        turretSubsystem,
        drivetrain,
        () -> FieldConstants.BLUE_HUB,
        false  // Disable telemetry to reduce network load
    )
);
```

### Integrate with Shooting
```java
// Track, spin up, shoot
Commands.sequence(
    new TrackTargetCommand(turretSubsystem, drivetrain, () -> FieldConstants.BLUE_HUB)
        .until(() -> turretSubsystem.atTarget()),
    Commands.runOnce(() -> flywheelSubsystem.spinUp()),
    Commands.waitUntil(() -> flywheelSubsystem.atSpeed()),
    Commands.runOnce(() -> indexerSubsystem.feed())
);
```

## Troubleshooting

### Turret Not Moving
- **Check**: Is turret at a limit? See `Turret/At Min Limit` or `Turret/At Max Limit`
- **Check**: Is target in range? See `TrackTarget/OutOfRange`
- **Check**: Is turret motor enabled? Check CAN connection

### Turret Points Wrong Direction
- **Fix**: Calibrate turret - make sure 180° is robot forward
- **Fix**: Verify robot pose is accurate (vision system working)
- **Fix**: Check field target coordinates in FieldConstants

### Turret Oscillates/Jitters
- **Fix**: Tune PID gains in TurretConstants
- **Fix**: Check for mechanical backlash
- **Fix**: Verify accurate pose estimation (reduce noise)

## Dependencies

This feature requires:
- ✅ **Accurate robot pose** - Now working with vision-based pose estimation!
- ✅ **Working turret subsystem** - Already implemented
- ✅ **Field target coordinates** - Already defined in FieldConstants
- ✅ **TurretConstants properly configured** - Already done

## Next Steps

1. **Test basic tracking** with one target
2. **Tune PID** if needed for smooth tracking
3. **Add more target options** (HPS stations, etc.)
4. **Integrate with shooting commands** for full auto-aim
5. **Add vision-based tracking** for moving targets

## Notes

- Turret limits are 50° to 360° to prevent cable wrap
- Turret at 180° points forward (robot forward direction)
- Turret at 270° points left, 90° points right
- Command runs continuously - use `.until()` or `.withTimeout()` to stop
- Telemetry can be disabled for default commands to reduce network load

