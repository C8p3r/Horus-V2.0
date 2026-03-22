# Software Design - Autonomous

## System Overview
The autonomous system leverages **PathPlanner 2026** for sophisticated path following and on-the-fly pathfinding with full holonomic control. The architecture supports pre-programmed autonomous routines, dynamic pathfinding to specific poses, and seamless integration with the swerve drivetrain's odometry and vision systems.

## PathPlanner Integration

### Core Architecture
The `PathPlannerSubsystem` provides a unified interface to PathPlanner's **AutoBuilder** framework:
- **Holonomic Path Following**: Full control over translation and rotation throughout paths
- **PID Control**: Dual-loop control with translation PID (5.0 kP) and rotation PID (5.0 kP)
- **Robot Configuration**: Loaded from PathPlanner GUI settings (mass, MOI, wheel properties, constraints)
- **Alliance Flipping**: Automatic path mirroring for red alliance operation

### Configuration
**Default Path Constraints**:
- Maximum Velocity: 4.0 m/s
- Maximum Acceleration: 3.0 m/s²
- Maximum Angular Velocity: π rad/s (180°/s)
- Maximum Angular Acceleration: π rad/s²

These constraints can be overridden per-path for fine-tuned control.

## Autonomous Capabilities

### 1. Pre-Made Path Following
**Workflow**: Create paths in PathPlanner GUI → Export to `deploy/pathplanner/paths/` → Execute via command

```java
Command followPath = pathPlannerSubsystem.followPath("Example Path");
```

**Features**:
- Bezier spline interpolation for smooth trajectories
- Rotation control at waypoints (holonomic advantage)
- Event markers for coordinated subsystem actions
- Real-time trajectory visualization in AdvantageScope
- Automatic alliance color flipping

### 2. On-the-Fly Pathfinding
**Dynamic path generation** to specific field poses using PathPlanner's NavGrid obstacle avoidance:

```java
Pose2d targetPose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90));
Command pathfind = pathPlannerSubsystem.pathfindToPose(targetPose);
```

**Applications**:
- Adaptive autonomous that responds to game piece positions
- Recovery from defensive disruption
- Opportunistic scoring when paths are available

**NavGrid Integration**: PathPlanner uses pre-computed navigation meshes from GUI for efficient obstacle avoidance.

### 3. Complex Autonomous Routines
**Full autonomous programs** with multiple paths and subsystem coordination:

```java
Command auto = pathPlannerSubsystem.getAutonomousCommand("Four Note Auto");
```

**Auto Routine Capabilities** (defined in PathPlanner GUI):
- Sequential path execution
- Parallel command groups (drive while intaking)
- Named commands for subsystem integration
- Conditional branching based on sensor data
- Timed waypoint stops

### 4. Hybrid Pathfinding + Following
**Pathfind to starting position, then execute pre-made path**:

```java
Command hybrid = pathPlannerSubsystem.pathfindThenFollowPath("Close Climb");
```

**Use Case**: Variable starting positions that need to navigate to a specific path entry point (e.g., climbing positions from anywhere on field).

## Odometry and Pose Management

### Pose Source Integration
PathPlanner directly interfaces with swerve drivetrain odometry:
- **Pose Supplier**: `() -> drivetrain.getState().Pose` (continuous odometry updates)
- **Pose Reset**: `(Pose2d pose) -> drivetrain.resetPose(pose)` (for path initialization)
- **Chassis Speeds**: `() -> drivetrain.getState().Speeds` (for feedforward control)

### Vision-Enhanced Localization
**SwerveDrivePoseEstimator** fuses wheel odometry with vision measurements:
- Dual Limelight cameras (front/back) with MegaTag1 AprilTag tracking
- Vision measurements integrated via `addVisionMeasurement()` with dynamic standard deviations
- Odometry drift correction during autonomous operation
- Initial pose seeding from vision during disabled period (see Pose Estimation doc)

### Robot-Relative Control
PathPlanner outputs **robot-relative ChassisSpeeds** to the drivetrain:

```java
private void driveRobotRelative(ChassisSpeeds speeds) {
    drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds()
        .withSpeeds(speeds));
}
```

This enables PathPlanner to directly command swerve modules without field-to-robot coordinate transformations.

## Autonomous Chooser

### SendableChooser Integration
Dashboard-selectable autonomous routines via SmartDashboard:

```java
autoChooser.setDefaultOption("None", Commands.none());
autoChooser.addOption("Example Auto", pathPlannerSubsystem.getAutonomousCommand("Example Auto"));
autoChooser.addOption("Drive Forward", simpleDriveForwardSequence);
```

### Auto Initialization
Autonomous command scheduled in `Robot.autonomousInit()`:
- Pose already initialized from vision during disabled period (no re-initialization)
- Auto command retrieved from chooser and scheduled
- Command runs until completion or tele-op transition

## Real-Time Telemetry

### AdvantageKit Logging
Comprehensive path following visualization:
- **Active Path Name**: Currently executing path
- **Target Pose**: PathPlanner's commanded target position
- **Current Pose**: Live robot odometry
- **Chassis Speeds**: Commanded velocities
- **Following State**: Boolean flags for pathfinding/following
- **Auto Name**: Currently executing autonomous routine name

### Visualization in AdvantageScope
3D Field widget displays:
- Robot pose trail (odometry history)
- Planned trajectory overlays
- Target waypoints
- Real-time path execution progress

## Error Handling and Robustness

### Configuration Validation
System checks for PathPlanner configuration on initialization:
- Verifies `RobotConfig` loaded from GUI settings
- Reports errors if configuration files missing
- Gracefully degrades with `Commands.none()` on failure

### Path Loading Safety
Try-catch blocks around all path operations:
- File not found → Error report + empty command
- Invalid path data → Error report + empty command
- Driver station error reporting with stack traces

### Subsystem Requirements
All PathPlanner commands explicitly require drivetrain subsystem:
- Prevents conflicts with tele-op control
- Ensures proper command interruption
- Maintains scheduler integrity

## Example Autonomous Routines

### Simple: Drive Forward
```java
Commands.sequence(
    drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
    drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
        .withVelocityY(0).withRotationalRate(0)).withTimeout(3.0),
    drivetrain.applyRequest(() -> idle)
)
```

### Complex: PathPlanner Auto
Defined in PathPlanner GUI with:
1. Starting pose and holonomic rotation
2. Bezier waypoint path (smooth curves)
3. Event markers: "Intake", "Shoot", "PrepareClimb"
4. Named commands for subsystem coordination
5. End pose and final rotation

## Design Philosophy

The autonomous system prioritizes **flexibility** and **reliability**:
- **GUI-driven path creation** enables non-programmers to design autonomous
- **On-the-fly pathfinding** provides adaptive autonomous capabilities
- **Vision-enhanced odometry** maintains accuracy throughout long autonomous periods
- **Comprehensive telemetry** enables rapid iteration and debugging
- **Graceful degradation** prevents match failures from configuration errors
- **Holonomic control** maximizes swerve drive advantages
