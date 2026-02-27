# Telemetry Toggle System

## Overview
A master toggle to disable ALL telemetry and logging in the robot code for maximum performance.

## Quick Start

### To Disable All Telemetry:
1. Open `RobotContainer.java`
2. Find the constant at the top:
   ```java
   public static final boolean DISABLE_ALL_TELEMETRY = false;
   ```
3. Change to:
   ```java
   public static final boolean DISABLE_ALL_TELEMETRY = true;
   ```
4. Rebuild and deploy

## What Gets Disabled

When `DISABLE_ALL_TELEMETRY = true`, the following are disabled:

### ✅ Core Systems
- **AdvantageKit Logging** - No logging to USB or NetworkTables
- **SmartDashboard Updates** - No dashboard telemetry
- **Field Constants Publishing** - Network table tuning disabled

### ✅ RobotContainer Telemetry
- Controller telemetry (button states, joystick values)
- Power monitoring (battery voltage, current draw)
- CAN bus monitoring (utilization, errors)
- Drivetrain telemetry registration
- Auto chooser SmartDashboard display
- Turret calibration buttons on dashboard

### ✅ Subsystem Telemetry (Manual Updates Required)
To disable telemetry in individual subsystems, wrap SmartDashboard calls:

**Example:**
```java
@Override
public void periodic() {
    if (!TelemetryManager.isDisabled()) {
        SmartDashboard.putNumber("Flywheel/Velocity RPS", getVelocityRPS());
        SmartDashboard.putBoolean("Flywheel/At Target", atTargetVelocity());
    }
}
```

### Subsystems with Telemetry:
- `FlywheelSubsystem` - Velocity, target, temperature
- `TurretSubsystem` - Angle, limits
- `HoodSubsystem` - Angle, target
- `IndexerSubsystem` - Floor/fire velocities
- `IntakeRollerSubsystem` - Velocity
- `IntakePositionSubsystem` - Position, deployed state
- `CANdleSubsystem` - LED colors, mode
- `VisionSubsystem` - Tag count, distances, latency

## Utility Class

Use `TelemetryManager` for consistent checks:

```java
import frc.robot.util.TelemetryManager;

// Check if enabled
if (TelemetryManager.isEnabled()) {
    SmartDashboard.putNumber("Key", value);
}

// Check if disabled
if (TelemetryManager.isDisabled()) {
    return; // Skip telemetry updates
}
```

## Performance Benefits

Disabling telemetry provides:
- **Reduced CAN bus traffic** - No telemetry status signals
- **Lower CPU usage** - No SmartDashboard updates
- **Faster loop times** - Less code execution
- **No USB logging** - Eliminates file I/O overhead
- **NetworkTables bandwidth** - Reduces network traffic

## Competition Mode

**Recommendation:** 
- Development/Testing: `DISABLE_ALL_TELEMETRY = false` (telemetry ON)
- Competition Matches: `DISABLE_ALL_TELEMETRY = true` (telemetry OFF)

## Notes

- The toggle is checked at initialization, not runtime
- Requires rebuild and redeploy to take effect
- LED feedback and controller rumble still work (not affected)
- Robot functionality remains unchanged
- Auto selection still works (stored in SendableChooser)

## Current Status

**Location:** `RobotContainer.java` line ~30
**Default:** `false` (telemetry ENABLED)
**Type:** `public static final boolean`

## Future Improvements

To fully disable subsystem telemetry, update each subsystem's `periodic()` method:

```java
@Override
public void periodic() {
    // Refresh signals (always needed for control)
    signal.refresh();
    
    // Telemetry (can be disabled)
    if (TelemetryManager.isEnabled()) {
        telemetryCounter++;
        if (telemetryCounter >= TELEMETRY_UPDATE_PERIOD) {
            telemetryCounter = 0;
            SmartDashboard.putNumber("Key", value);
        }
    }
}
```

This ensures critical robot control continues while eliminating telemetry overhead.
