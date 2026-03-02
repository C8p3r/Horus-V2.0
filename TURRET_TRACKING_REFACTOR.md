# Turret Tracking Refactor

## Overview
The turret tracking system has been refactored to run continuously as the default command of the turret subsystem, rather than as a separate command that needs to be held. This makes it simpler to use and more robust.

## What Changed

### 1. **Tracking Logic Moved to TurretSubsystem**
- All tracking calculations now happen inside `TurretSubsystem.periodic()`
- The tracking automatically uses the turret's current robot-relative angle
- Correctly accounts for turret starting position (180° = facing forward, 0° = facing backward)

### 2. **Continuous Tracking**
- Turret now tracks continuously in the background
- No need to hold a button - just tap to select a target
- The default command ensures tracking runs every loop

### 3. **Simplified Button Bindings**
- **Driver Left Trigger**: Switch tracking to BLUE hub
- **Driver Right Trigger**: Switch tracking to RED hub  
- **Driver Y Button**: Disable tracking (turret holds current position)
- **Operator A Button**: Zero turret calibration

### 4. **TrackTargetCommand.java** (Deprecated)
The separate `TrackTargetCommand.java` file is no longer used, but has been left in place in case you want to reference the logic or use it for other purposes.

## How It Works

### Target Selection
```java
// In button bindings:
joystick.leftTrigger().onTrue(
    Commands.runOnce(() -> turretSubsystem.setTarget(FieldConstants.BLUE_HUB))
);
```

### Tracking Math
The tracking accounts for:
1. **Robot heading**: Where the chassis is facing
2. **Turret angle**: Current turret position (0-360°)
3. **Turret origin**: Turret at 180° = robot forward, 0° = robot backward

Calculation:
```
fieldAngleToTarget = atan2(target - robot)
robotRelativeAngle = fieldAngleToTarget - robotHeading
turretAngle = robotRelativeAngle + 180°
```

### Visualization
All AdvantageScope trajectories are still published:
- **TrackTarget/TargetPose**: Red marker at target location
- **TrackTarget/LineToTarget**: Green line from robot to target
- **TrackTarget/TurretAimLine**: Blue line showing turret aim (5m long)

## API Methods

### TurretSubsystem Methods
```java
// Set target to track (null to disable)
void setTarget(Translation3d target)

// Get current target
Translation3d getTarget()

// Check if tracking is enabled
boolean isTracking()
```

## Usage Example

```java
// Select blue hub target
turretSubsystem.setTarget(FieldConstants.BLUE_HUB);
// Turret now continuously tracks blue hub

// Switch to red hub
turretSubsystem.setTarget(FieldConstants.RED_HUB);
// Turret smoothly transitions to track red hub

// Stop tracking
turretSubsystem.setTarget(null);
// Turret holds current position
```

## Benefits

1. **Always Ready**: Tracking runs continuously, no delay when you need it
2. **Simpler Code**: One method call to switch targets
3. **Better Integration**: Tracking logic is where it belongs (in the subsystem)
4. **Chassis Independent**: Correctly uses turret's current angle relative to chassis
5. **More Robust**: Can't accidentally release the button and lose tracking

## Testing Checklist

- [ ] Enable robot and verify turret initializes to 180°
- [ ] Press left trigger - turret should track blue hub
- [ ] Drive around - turret should maintain aim at blue hub
- [ ] Press right trigger - turret should smoothly switch to red hub
- [ ] Press Y button - turret should stop tracking and hold position
- [ ] Verify AdvantageScope trajectories display correctly
- [ ] Check that turret respects 50°-360° limits

## Notes

- The turret now properly accounts for starting backward (opposite chassis front)
- Tracking uses the turret's **current** robot-relative angle in calculations
- All telemetry and visualization remains the same
