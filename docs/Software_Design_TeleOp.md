# Software Design - Tele-Operated

## System Overview
The tele-operated control system implements a comprehensive dual-controller interface with intelligent subsystem coordination, predictive targeting, and adaptive speed control. The architecture prioritizes intuitive driver controls while providing precision operator overrides for advanced functions.

## Control Architecture

### Dual Controller System
- **Driver Controller (Xbox 0)**: Primary robot operation, field-centric driving, targeting, and shooting
- **Operator Controller (Xbox 1)**: Subsystem calibration, manual overrides, climbing, and system diagnostics

### Drivetrain Control
**Field-Centric Swerve Drive** with adaptive speed modulation:
- **Translation**: Left stick (X/Y) with 10% deadband
- **Rotation**: Right stick (X) with 10% deadband  
- **Speed Reduction**: Automatic 75% reduction when climber deployed for precise positioning
- **Chassis Rotation Assist**: Automatic rotation compensation when turret tracking targets near mechanical limits

**Maximum Performance**: 5.12 m/s translation, 0.75 rotations/sec angular velocity

### Shooter System Control

#### Target Acquisition (Driver Triggers)
- **Left Trigger**: Auto-track BLUE alliance hub + execute smart shoot
- **Right Trigger**: Auto-track RED alliance hub + execute smart shoot
- **A Button**: Execute smart shoot on current target (allows re-shooting without re-targeting)

The system uses **predictive motion compensation** - turret tracking accounts for robot velocity with 150ms lookahead, 20% translation overcorrection, and 30% rotation overcorrection for moving shots.

#### Smart Shooting System
The smart shoot sequence integrates multiple subsystems:
1. **Target Validation**: Verifies target is set and within range
2. **Calibration System Query**: Retrieves hood angle and flywheel velocity from interpolated calibration table
3. **Turret Positioning**: Aims turret with full 360° coverage (-50° to 312°, with 0° = forward)
4. **Hood Adjustment**: Sets hood angle (14° flat for close shots, 40° steep for far shots)
5. **Flywheel Spin-Up**: Accelerates flywheel to target RPS with Motion Magic velocity control
6. **Indexer Feed**: Fires note when all subsystems at target with controller rumble feedback

#### Manual Calibration Mode
When `SHOOTER_CALIBRATION_MODE = true`:
- **Hood/Flywheel**: Follow dashboard slider values (ManualHood 14-40°, ManualFlywheel 0-80 RPS)
- **Indexers**: Follow dashboard duty cycle sliders (ManualFloorIndexer, ManualFireIndexer -1.0 to 1.0)
- **Record Button**: Saves current values as calibration point at current distance
- **Interpolation**: System builds calibration curve from recorded points using `InterpolatingDoubleTreeMap`

### Intake System
- **Right Bumper (Driver)**: Toggle intake deployment and roller power
- **Left Bumper (Driver)**: Eject (hold) - reverses rollers and indexers

State machine maintains intake position and roller state independently, allowing interrupted sequences to resume cleanly.

### Climbing System (Operator)
- **Y Button**: Deploy climber (unspool winch, lock out shooter/intake)
- **X Button**: Store climber (spool winch, actively climb)
- **Safety Interlocks**: Automatically disables shooter and intake during climbing operations

### Advanced Controls

#### Pathfinding (Driver POV)
- **POV Left**: Pathfind to "Far Climb" position
- **POV Right**: Pathfind to "Close Climb" position
- **POV Up**: Reset field-centric heading to 0° (forward)

#### System Management (Operator)
- **A Button**: Zero turret encoder at current position
- **B Button**: Manual shoot with fixed test parameters (60 RPS, 0° turret, 25° hood)
- **POV Down**: Clear current target tracking
- **Right Bumper**: Toggle vision processing on/off
- **Back/Start + X/Y**: SysId characterization routines for drivetrain tuning

## Default Command Architecture

### Continuous Subsystem Updates
All major subsystems run persistent default commands for seamless operation:

**Drivetrain**: Field-centric drive with speed modulation and chassis rotation assist  
**Turret**: Continuous tracking with predictive motion compensation  
**Hood**: Follows calibration values (manual sliders or distance-based interpolation)  
**Flywheel**: Follows calibration values with Motion Magic velocity control  
**Indexers**: Manual duty cycle control in calibration mode, stopped otherwise

### LED Feedback System
Priority-based visual status indication via CANdle RGB LEDs:
1. **Critical**: Rapid red blink (flywheel overheating)
2. **High Priority**: Rainbow patterns (climber deployed/active)
3. **Medium Priority**: Orange flash (note scored - velocity dip detection)
4. **Shooting**: Green pulse (ready), yellow breath (spinning up)
5. **Intake**: Blue pulse (active)
6. **Aiming**: Purple pulse (turret/hood at target)
7. **Idle**: Dim white breath (standby)

### Controller Rumble Feedback
Haptic feedback provides tactile confirmation:
- **Flywheel Ready**: 300ms rumble burst when flywheel reaches target velocity
- **Note Scored**: Tied to LED orange flash when velocity dip detected

## Telemetry and Performance

### Throttled Updates
Aggressive telemetry throttling prevents loop overruns:
- **Robot Periodic**: 0.2Hz (5 second intervals)
- **Vision**: 25 cycle intervals (500ms)
- **Subsystems**: 25-50 cycle intervals (500ms-1s)

### AdvantageKit Integration
Real-time logging for all subsystems:
- Pose estimates and trajectories
- Subsystem states and targets
- Command execution traces
- Performance metrics

### Dashboard Integration
SmartDashboard/Elastic provides:
- Live subsystem telemetry
- Calibration sliders and controls
- Target tracking visualization
- System health monitoring

## Configuration Toggles

**DISABLE_ALL_TELEMETRY** (false): Master switch for all logging and dashboard updates  
**SHOOTER_CALIBRATION_MODE** (true): Toggle between manual sliders and auto interpolation  
**RETRACT_INTAKE_WHILE_SHOOTING** (false): Control intake behavior during shooting  
**CLIMBING_SPEED_REDUCTION** (0.25): Speed multiplier when climber deployed

## Design Philosophy

The tele-op system prioritizes **driver efficiency** and **operator precision**:
- **Single-button targeting and shooting** eliminates multi-step sequences
- **Automatic subsystem coordination** reduces cognitive load
- **Visual and haptic feedback** provides instant status confirmation
- **Manual override capability** allows fine-tuning during competition
- **Calibration mode** enables rapid on-field parameter adjustment without redeployment
