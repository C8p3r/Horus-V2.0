# Swerve Drive Module State Visualization

Your swerve drivetrain now publishes module states that can be visualized in Shuffleboard, Elastic, or AdvantageScope.

## Published Data

The following data is published to NetworkTables:

### Module States Array
- **Topic**: `Swerve/ModuleStates`
- **Format**: `[FL_angle_rad, FL_speed, FR_angle_rad, FR_speed, BL_angle_rad, BL_speed, BR_angle_rad, BR_speed]`
- **Use**: For widgets that display all 4 modules at once

### Individual Module Data
Each module publishes:
- `Swerve/FrontLeft/Angle` - Degrees
- `Swerve/FrontLeft/Speed` - Meters per second
- `Swerve/FrontRight/Angle` - Degrees
- `Swerve/FrontRight/Speed` - Meters per second
- `Swerve/BackLeft/Angle` - Degrees
- `Swerve/BackLeft/Speed` - Meters per second
- `Swerve/BackRight/Angle` - Degrees
- `Swerve/BackRight/Speed` - Meters per second

## Setup in Shuffleboard

### Method 1: Swerve Widget (Best for visualization)

1. Open Shuffleboard
2. Go to **View** → **Add Widget** → **Swerve**
3. Configure the widget:
   - **Source**: Select `Swerve/ModuleStates`
   - **Module Layout**: Choose your robot's configuration (square, wide, etc.)
   - **Max Speed**: Set to your max speed (e.g., 4.5 m/s)
   - **Rotation Unit**: Radians (as published)
   - **Size**: Adjust as needed

### Method 2: Individual Gauges

For each module, add:
1. **Angle Gauge**: 
   - Source: `Swerve/[ModuleName]/Angle`
   - Min: -180, Max: 180
   - Type: Compass or Dial
2. **Speed Gauge**:
   - Source: `Swerve/[ModuleName]/Speed`
   - Min: 0, Max: 5.0 (your max speed)
   - Type: Number Bar

### Method 3: Graph (for trends)

1. Add a **Graph** widget
2. Add all angle topics:
   - `Swerve/FrontLeft/Angle`
   - `Swerve/FrontRight/Angle`
   - `Swerve/BackLeft/Angle`
   - `Swerve/BackRight/Angle`

## Setup in Elastic Dashboard

### Using the Swerve Widget

1. In the Elastic dashboard, add a **Swerve** widget
2. Configure:
   - **Topic**: `Swerve/ModuleStates`
   - **Size**: Adjust the widget size to match your robot dimensions
   - **Max Speed**: 4.5 m/s (or your configured max)

### Updating elastic-swerve-layout.json

Add this to your layout file:

```json
{
  "type": "swerve",
  "params": {
    "topic": "/Swerve/ModuleStates",
    "title": "Swerve Modules",
    "maxSpeed": 4.5,
    "rotationUnit": "radians",
    "width": 4,
    "height": 4
  }
}
```

## Setup in AdvantageScope

1. Open AdvantageScope
2. Add a **Swerve States** tab
3. Configure:
   - **Left Topic**: `Swerve/ModuleStates` 
   - **Robot Type**: Swerve Drive
   - **Module Layout**: Configure your module positions
   - **Max Speed**: 4.5 m/s

Or view individual modules:
1. Add a **Line Chart** tab
2. Add fields:
   - `Swerve/FrontLeft/Angle`
   - `Swerve/FrontRight/Angle`
   - `Swerve/BackLeft/Angle`
   - `Swerve/BackRight/Angle`

## What You'll See

The visualization will show:
- **Arrow Direction**: The direction each wheel is pointed
- **Arrow Length**: The speed of each wheel (longer = faster)
- **Color**: Typically changes based on speed magnitude
- **Real-time Updates**: As you drive, the arrows will rotate and change length

## Tips

1. **During Testing**: Watch that all modules point in the correct direction when driving
2. **Zeroing Check**: When stopped, all speeds should be near zero
3. **Rotation Check**: When rotating in place, all modules should point tangent to the circle
4. **Translation Check**: When driving straight, all modules should point the same direction
5. **Crab Check**: For diagonal movement, all modules should point at the same angle

## Troubleshooting

- **No data appearing**: Make sure robot code is deployed and robot is enabled
- **Wrong orientation**: Check your module angle conventions (radians vs degrees)
- **Inverted speeds**: Check that positive speed corresponds to your expected direction
- **Flickering**: Normal if updating very fast, can reduce update rate if needed
