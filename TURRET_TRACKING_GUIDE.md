# Turret Target Tracking Guide

## Overview
The `TrackTargetCommand` enables the turret to automatically track field targets (like the speaker/hub) independently of the chassis orientation. This allows the robot to drive in any direction while the turret maintains aim at the target.

## How It Works

### Coordinate Systems
- **Field Coordinates**: Global field reference (blue alliance origin)
- **Robot Coordinates**: Relative to robot forward direction
- **Turret Coordinates**: 
  - 0° = Robot backward
  - 180° = Robot forward
  - 270° = Robot left
  - 90° = Robot right

### Tracking Math
1. Calculate field-relative angle from robot to target
2. Subtract robot heading to get robot-relative angle
3. Add 180° to convert to turret angle (since turret 180° = robot forward)
4. Clamp to turret limits (50° to 360°)

## Usage Examples

### Example 1: Track Blue Alliance Hub
```java
// In RobotContainer.java

// Add import
import frc.robot.commands.TrackTargetCommand;
import frc.robot.constants.FieldConstants;

// In configureBindings():
// Hold button 5 to track blue hub
driverController.button(5).whileTrue(
    new TrackTargetCommand(
        turretSubsystem,
        drivetrain,
        () -> FieldConstants.BLUE_HUB
    )
);
```

### Example 2: Track Alliance-Specific Hub
```java
// Track the correct hub based on alliance color
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

driverController.button(6).whileTrue(
    new TrackTargetCommand(
        turretSubsystem,
        drivetrain,
        () -> {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            return alliance == Alliance.Blue ? 
                FieldConstants.BLUE_HUB : 
                FieldConstants.RED_HUB;
        }
    )
);
```

### Example 3: Track Dynamic Target
```java
// Track a target that can change based on game state
private Translation3d currentTarget = FieldConstants.BLUE_HUB;

public void setTargetToHub() {
    currentTarget = FieldConstants.BLUE_HUB;
}

public void setTargetToHPSClose() {
    currentTarget = FieldConstants.BLUE_HPS_CLOSE_PASS;
}

// In configureBindings():
driverController.button(7).whileTrue(
    new TrackTargetCommand(
        turretSubsystem,
        drivetrain,
        () -> currentTarget
    )
);

// Switch targets with other buttons
driverController.button(8).onTrue(Commands.runOnce(() -> setTargetToHub()));
driverController.button(9).onTrue(Commands.runOnce(() -> setTargetToHPSClose()));
```

### Example 4: Default Tracking Command
```java
// Set turret to always track target unless another command is running
// In RobotContainer constructor after subsystem creation:
turretSubsystem.setDefaultCommand(
    new TrackTargetCommand(
        turretSubsystem,
        drivetrain,
        () -> FieldConstants.BLUE_HUB,
        false  // Disable telemetry to reduce network traffic
    )
);
```

### Example 5: Track with Timeout
```java
// Track for 5 seconds then stop
driverController.button(10).onTrue(
    new TrackTargetCommand(
        turretSubsystem,
        drivetrain,
        () -> FieldConstants.BLUE_HUB
    ).withTimeout(5.0)
);
```

### Example 6: Track Until Aligned
```java
// Track until turret is aimed at target
driverController.button(11).onTrue(
    new TrackTargetCommand(
        turretSubsystem,
        drivetrain,
        () -> FieldConstants.BLUE_HUB
    ).until(() -> turretSubsystem.atTarget())
);
```

## Telemetry

When telemetry is enabled (default), the command publishes to SmartDashboard:

- `TrackTarget/TargetX` - Target X position (field coordinates)
- `TrackTarget/TargetY` - Target Y position (field coordinates)
- `TrackTarget/DistanceToTarget` - Distance from robot to target (meters)
- `TrackTarget/FieldAngleToTarget` - Field-relative angle to target
- `TrackTarget/RobotRelativeAngle` - Robot-relative angle to target
- `TrackTarget/TurretCommandAngle` - Commanded turret angle
- `TrackTarget/TurretAtTarget` - Whether turret has reached target
- `TrackTarget/OutOfRange` - Whether target is outside turret limits

## Turret Limits

The turret has hardware/software limits:
- **Minimum**: 50° (prevents cable wrap)
- **Maximum**: 360° (prevents cable wrap)

If a target is outside these limits, the turret will move to the nearest limit and `OutOfRange` will be true.

## Integration with Shooting

Typical shooting sequence:
```java
// Create a command group for automatic shooting
public Command autoShootCommand() {
    return Commands.sequence(
        // 1. Track the target
        new TrackTargetCommand(turretSubsystem, drivetrain, () -> FieldConstants.BLUE_HUB)
            .until(() -> turretSubsystem.atTarget()),
        
        // 2. Spin up flywheel
        Commands.runOnce(() -> flywheelSubsystem.setTargetVelocity(4000)),
        Commands.waitUntil(() -> flywheelSubsystem.atTargetVelocity()),
        
        // 3. Fire
        Commands.runOnce(() -> indexerSubsystem.feed())
    );
}
```

## Debugging

### Visual Debugging in AdvantageScope
1. The turret pose is published as `Turret/Pose3d`
2. Add a 3D Field widget in AdvantageScope
3. Add both `Robot/Pose` and `Turret/Pose3d` to see turret orientation

### Common Issues

**Turret not moving:**
- Check that turret is not at a limit (see `Turret/At Min Limit` or `Turret/At Max Limit`)
- Verify target is within limits (see `TrackTarget/OutOfRange`)
- Check turret motor is enabled and not faulted

**Turret pointing wrong direction:**
- Verify turret calibration (180° should be robot forward)
- Check robot pose is accurate (see vision system)
- Verify field target coordinates are correct

**Turret oscillating:**
- May need to tune PID gains in `TurretConstants`
- Check for mechanical issues or backlash

## Advanced: Custom Target Calculation

You can track any point in 3D space:
```java
// Track a moving target (e.g., another robot)
Supplier<Translation3d> dynamicTarget = () -> {
    // Get target position from vision, alliance partner, etc.
    double targetX = someVisionSystem.getTargetX();
    double targetY = someVisionSystem.getTargetY();
    double targetZ = 2.0; // Height of target
    return new Translation3d(targetX, targetY, targetZ);
};

new TrackTargetCommand(turretSubsystem, drivetrain, dynamicTarget);
```

## Testing Procedure

1. **Static Test**: Place robot at known position, verify turret points at target
2. **Rotation Test**: Rotate robot, verify turret maintains aim
3. **Drive Test**: Drive robot around, verify continuous tracking
4. **Limit Test**: Drive to positions where target is outside limits, verify safe behavior
5. **Alliance Test**: Test with both blue and red alliance configurations

