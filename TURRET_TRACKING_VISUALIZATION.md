# Turret Tracking Visualization in AdvantageScope

## Overview
The turret tracking system now publishes rich telemetry data that can be visualized in AdvantageScope to see exactly how the turret is tracking targets.

## What's Published

### Pose3d Arrays (Trajectories/Lines)
These appear as lines in the 3D field view:

1. **`TrackTarget/LineToTarget`** - Green line from robot to target
   - Shows the direct path from robot center to the target
   - Length represents distance to target

2. **`TrackTarget/TurretAimLine`** - Blue line showing where turret is pointing
   - 5 meters long for visibility
   - Shows actual turret aim direction
   - Should align with LineToTarget when turret is on target

### Single Pose3d
3. **`TrackTarget/TargetPose`** - Red marker at target location
   - Shows the 3D position of the target being tracked
   - Height matches actual target height (e.g., 2m for hub)

### Numeric Data (For Graphs)
All angles and distances for plotting over time:

- `TrackTarget/FieldAngleToTarget` - Field-relative angle to target (degrees)
- `TrackTarget/RobotRelativeAngle` - Robot-relative angle to target (degrees)
- `TrackTarget/TurretCommandAngle` - Commanded turret angle (0-360°)
- `TrackTarget/TurretCurrentAngle` - Actual turret angle (0-360°)
- `TrackTarget/TurretFieldHeading` - Turret's field-relative heading (degrees)
- `TrackTarget/DistanceToTarget` - Distance to target (meters)
- `TrackTarget/OutOfRange` - Whether target is outside turret limits
- `TrackTarget/AtTarget` - Whether turret has reached commanded angle

## Setting Up AdvantageScope

### Step 1: Open AdvantageScope
1. Launch AdvantageScope
2. Connect to robot or open a log file

### Step 2: Add a 3D Field Widget
1. Click **"Add Tab"** → **"3D Field"**
2. This shows a bird's-eye view of the FRC field

### Step 3: Add Objects to 3D Field

#### Add Robot Pose
1. In the left panel, expand **"NT:/RealOutputs"** (or NT: for live)
2. Find **"Robot/Pose"** or your main robot pose topic
3. **Drag and drop** onto the 3D field
4. Robot should appear as a chassis outline

#### Add Turret Pose
1. Find **"Turret/Pose3d"**
2. Drag onto the 3D field
3. This shows turret orientation independent of chassis

#### Add Target Position
1. Find **"TrackTarget/TargetPose"**
2. Drag onto the 3D field
3. Color: Red (use color picker)
4. This marks the target location

#### Add Line to Target
1. Find **"TrackTarget/LineToTarget"**
2. Drag onto the 3D field
3. Color: Green
4. This draws a line from robot to target

#### Add Turret Aim Line
1. Find **"TrackTarget/TurretAimLine"**
2. Drag onto the 3D field
3. Color: Blue or Yellow
4. This shows where turret is actually pointing

### Step 4: Add Line Charts for Angles

1. Click **"Add Tab"** → **"Line Graph"**
2. Drag these topics onto the graph:
   - `TrackTarget/TurretCommandAngle` (commanded)
   - `TrackTarget/TurretCurrentAngle` (actual)
   - `TrackTarget/FieldAngleToTarget` (ideal field angle)
3. This shows tracking performance over time

### Step 5: Add Status Indicators

1. Click **"Add Tab"** → **"Line Graph"** or use existing graph
2. Add these boolean values:
   - `TrackTarget/AtTarget` - Should be TRUE most of the time
   - `TrackTarget/OutOfRange` - Should be FALSE (unless target unreachable)

## What to Look For

### Successful Tracking
✅ **Turret Aim Line (blue) aligns with Line to Target (green)**
   - When aligned, turret is perfectly aimed at target
   
✅ **`TrackTarget/AtTarget` is TRUE**
   - Turret has reached commanded angle
   
✅ **Turret Aim Line rotates smoothly**
   - No jitter or oscillation
   
✅ **Robot can drive/rotate freely**
   - Turret maintains aim despite chassis movement

### Problem Indicators
❌ **Aim line doesn't align with target line**
   - Check turret calibration (is 180° really forward?)
   - Check robot pose accuracy
   
❌ **Aim line oscillates/jitters**
   - PID gains may need tuning
   - Check for mechanical backlash
   
❌ **`OutOfRange` is TRUE**
   - Target is outside turret's 50°-360° range
   - Robot needs to reposition
   
❌ **Large gap between Command and Current angle graphs**
   - Turret not responding fast enough
   - Check motion magic parameters

## Example Visualization Setup

```
┌─────────────────────────────────────────┐
│          3D Field View                   │
│                                          │
│    [Robot] ─────green───→ [Target]      │
│       │                       ↑          │
│       └──blue─→ (Turret Aim)             │
│                                          │
│  Legend:                                 │
│  • Green line = Direct path to target   │
│  • Blue line = Where turret points      │
│  • Red marker = Target location         │
└─────────────────────────────────────────┘

┌─────────────────────────────────────────┐
│          Angle Graph                     │
│  360° ┤     ╱─Command                   │
│       ┤    ╱                              │
│  180° ┤───●─Current (following)          │
│       ┤                                   │
│    0° └─────────────────→ Time           │
└─────────────────────────────────────────┘

┌─────────────────────────────────────────┐
│          Status                          │
│  AtTarget:    ██████████ (TRUE)         │
│  OutOfRange:  __________ (FALSE)        │
└─────────────────────────────────────────┘
```

## Recording and Playback

### To Record
1. In AdvantageScope, click **"Start Recording"**
2. Drive robot and activate turret tracking (hold L or R trigger)
3. Click **"Stop Recording"**
4. Log file saved to USB on robot

### To Playback
1. Copy log file from robot to computer
2. Open AdvantageScope → **"File"** → **"Open Log"**
3. Select your log file
4. Use timeline to scrub through the run
5. Watch turret tracking behavior in 3D field

## Advanced: Creating Custom Views

### Side-by-Side View
1. Create two tabs:
   - Left: 3D Field (top-down view)
   - Right: Line graph (angle tracking)
2. Synchronized timeline shows both at once

### Split Screen
1. Click **"+"** to add multiple widgets to one tab
2. Arrange as grid:
   - Top-left: 3D Field
   - Top-right: Angle graph
   - Bottom: Status table

### Odometry Replay
1. Open a log file
2. 3D field automatically shows robot path over time
3. See how turret maintained aim throughout the drive

## Troubleshooting

### "No data" in 3D Field
- Verify AdvantageKit is logging (check `Logger.start()` in Robot.java)
- Check NT4Publisher is active (should be by default)
- Ensure robot is connected to same network

### Lines not showing
- Check data type is `Pose3d[]` (array)
- Verify array has exactly 2 elements (start and end)
- Try refreshing the widget

### Angles look wrong
- Remember: Turret angles are 0-360° where 180° = forward
- Field angles are standard: 0° = +X axis (toward red wall)
- Robot relative angles are relative to robot forward

## Tips

1. **Use color coding**:
   - Green = Desired/target
   - Blue = Actual/current
   - Red = Errors/problems

2. **Set useful ranges** on graphs:
   - Angles: 0-360° or -180 to 180°
   - Distance: 0-10 meters (typical shooting range)

3. **Use multiple timelines**:
   - One for live viewing
   - One for reviewing specific moments

4. **Export images**:
   - Right-click 3D field → "Export as PNG"
   - Useful for documentation/debugging

5. **Compare runs**:
   - Open two log files side-by-side
   - Compare tracking performance between tuning iterations

