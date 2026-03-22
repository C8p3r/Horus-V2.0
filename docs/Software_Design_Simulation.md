# Software Design - Simulation

## System Overview
The simulation framework provides comprehensive physics-based modeling of robot mechanisms and game piece dynamics using **AdvantageKit** logging and the custom **FuelSim** physics engine. This enables realistic testing of autonomous routines, shooting trajectories, and subsystem coordination without physical hardware.

## FuelSim Physics Engine

### Core Architecture
`FuelSim` is a custom 3D physics simulation specifically designed for FRC game piece dynamics:

**Key Features**:
- Full 3D projectile motion with gravity (9.81 m/s²)
- Air resistance modeling (drag coefficient 0.47, 0.025m² cross-section, 0.300kg mass)
- Collision detection (fuel-to-fuel, fuel-to-robot, fuel-to-field boundaries)
- Intake simulation with velocity-based capture zones
- Field boundary enforcement with elastic collisions
- Sub-tick physics updates for accuracy (default: 1 sub-tick per 20ms)

### Projectile Physics

**Forces Modeled**:
1. **Gravity**: Constant -9.81 m/s² in Z-axis
2. **Air Resistance**: Quadratic drag force proportional to velocity squared
   ```
   F_drag = 0.5 * ρ * C_d * A * v²
   ```
   - Air density (ρ): 1.225 kg/m³
   - Drag coefficient (C_d): 0.47 (spherical)
   - Cross-sectional area (A): 0.025 m²

3. **Impact Forces**: Elastic collisions with coefficient of restitution
   - Fuel-to-fuel: 0.6 (energy loss on collision)
   - Fuel-to-field: 0.5 (bounce damping)

### Fuel Spawning and Lifecycle

**Spawning Fuel**:
```java
FuelSim.getInstance().spawnFuel(
    new Translation3d(x, y, z),  // Starting position
    new Translation3d(vx, vy, vz) // Initial velocity vector
);
```

**Starting Fuel**: Automatic spawning in neutral zone and depots when simulation starts

**Fuel Removal**: Automatic cleanup when:
- Out of field bounds (X: 0-16.54m, Y: 0-8.21m, Z: 0-10m)
- Captured by robot intake
- Manually cleared via `clearFuel()`

### Robot Integration

**Registration**:
```java
FuelSim.getInstance().registerRobot(
    0.76,  // Width (meters)
    0.76,  // Length (meters)
    0.30,  // Bumper height (meters)
    () -> drivetrain.getState().Pose,      // Pose supplier
    () -> drivetrain.getState().Speeds     // ChassisSpeeds supplier
);
```

**Collision Detection**: Fuel that contacts robot bumpers (X/Y plane, Z < bumper height) triggers collision response with velocity transfer.

**Intake Simulation**: Fuel within intake zone (configurable radius and cone angle) automatically captured when intake active.

## Shooter Trajectory Simulation

### ShooterSimulator Utility
Physics-based trajectory calculation with air resistance for shot optimization:

**Trajectory Simulation**:
```java
TrajectoryResult result = ShooterSimulator.simulateTrajectory(
    velocity,      // m/s
    angle,         // radians
    targetX,       // horizontal distance (m)
    targetZ        // vertical distance (m)
);
```

**Returns**:
- Flight time to target
- Maximum height reached
- Entry angle at target (degrees, negative = downward)
- Hit success boolean

**Numerical Integration**: 1ms time steps (dt=0.001s) with Euler method for velocity and position updates.

### Shooting Solution Calculation

**Optimal Angle Finding**:
The system iterates through launch angles (50-76° initially, 15-75° if no solution) to find best shot:
1. Simulate trajectory at each angle
2. Check if projectile passes through target with ±10cm tolerance
3. Validate entry angle is within acceptable range (-80° to +20°)
4. Return angle with lowest required velocity (most efficient)

**ShootingCalculator Integration**:
- Calculates turret angle for target direction
- Uses calibration interpolation for hood angle and flywheel velocity
- Generates trajectory visualization (Pose3d array) for AdvantageScope
- Logs solution to AdvantageKit: `SmartShoot/Trajectory`, `SmartShoot/EntryAngle`, etc.

### Trajectory Visualization

**AdvantageScope 3D Field**:
Shooter trajectories visualized as connected pose arrays:
```java
Logger.recordOutput("SmartShoot/Trajectory", trajectoryPoints); // Pose3d[]
```

Displays:
- Projectile path from shooter to target
- Apex height marker
- Entry angle vector
- Target position indicator

## AdvantageKit Logging Framework

### Real vs Simulation Logging

**Real Robot**:
- Logs to USB stick via `WPILOGWriter` (.wpilog files)
- Publishes to NetworkTables via `NT4Publisher` (live AdvantageScope)
- Full hardware telemetry (motor currents, temperatures, CAN status)

**Simulation**:
- Publishes to NetworkTables only (no USB)
- Simulated hardware via Phoenix 6 simulation APIs
- FuelSim physics published to `FuelSim/Fuels` NetworkTables topic

### Logging Throttling
Aggressive throttling prevents loop overruns (target: 50Hz main loop):
- **Vision**: 500ms intervals
- **Subsystems**: 500ms-1s intervals  
- **High-frequency**: Odometry, motor positions (50Hz)
- **Event-based**: Command executions, state changes (immediate)

**Master Toggle**: `DISABLE_ALL_TELEMETRY` disables all logging for competition if needed.

## CTRE Phoenix 6 Simulation

### Swerve Drivetrain Simulation
CTRE's `SimSwerveDrivetrain` provides:
- Individual swerve module dynamics (drive + azimuth motors)
- Gyro simulation with drift modeling
- CAN bus simulation with realistic latency
- Battery voltage modeling under load

**Sim Loop**: Runs at faster-than-real-time rate (configurable `kSimLoopPeriod`) for PID tuning convergence.

### Motor Simulation
TalonFX motors simulated with:
- Voltage-to-torque-to-velocity dynamics
- Current draw modeling (affects battery voltage)
- Motion Magic profile following
- Position/velocity feedback with configurable noise

### Mechanism Simulation
Custom mechanism sims (turret, hood, flywheel) use Phoenix 6 APIs:
- `m_sim.setSupplyVoltage()`: Simulated battery voltage
- `m_sim.setRawRotorPosition()`: Simulated encoder position
- `m_sim.setRotorVelocity()`: Simulated velocity
- Physical dynamics (inertia, friction) modeled externally, fed into sim

## Simulation Workflow

### Setup
1. **Launch Simulation**: Run "WPILib: Simulate Robot Code" in VS Code
2. **Connect AdvantageScope**: Connect to `localhost:5810` (NT4 server)
3. **Start FuelSim**: Automatically starts in `Robot.simulationPeriodic()`
4. **Spawn Fuel**: Pre-loaded starting positions or manual spawning

### Testing Scenarios

**Autonomous Testing**:
- PathPlanner paths execute with simulated drivetrain physics
- Vision pose estimates simulated (can use fixed values or manual setting)
- Fuel intake and shooting visualized in 3D field

**Shooter Tuning**:
- Adjust calibration values on dashboard
- Fire simulated shots (spawns fuel into FuelSim)
- Observe trajectories in AdvantageScope 3D field
- Iterate on hood angles and flywheel velocities

**Intake Testing**:
- Spawn fuel on field via dashboard commands
- Drive robot near fuel (keyboard/gamepad in simulator)
- Verify intake capture zones and fuel collection

### Visualization

**AdvantageScope 3D Field Widget**:
- Robot pose and orientation (swerve module directions)
- Fuel positions (orange spheres)
- Shooter trajectories (green curves)
- AprilTag positions (field layout)
- Path planning visualization (blue lines)

**Telemetry Graphs**:
- Subsystem positions and velocities over time
- Command execution timeline
- Vision measurement acceptance rates
- Motor currents and temperatures

## Simulation Variance

### Realistic Inaccuracy Modeling
FuelSim adds variance to shooter launches for realism:

```java
// PhysicsConstants.java
public static final double SIM_VELOCITY_VARIANCE_PERCENT = 2.0;  // ±2%
public static final double SIM_ANGLE_VARIANCE_DEGREES = 0.5;      // ±0.5°
public static final double SIM_DIRECTION_VARIANCE_DEGREES = 0.3;  // ±0.3°
public static final double SIM_POSITION_VARIANCE_METERS = 0.01;   // ±1cm
```

**Applied When**:
- Launching fuel from shooter (simulates mechanical tolerance)
- Spawning fuel (slight position randomness)

**Not Applied**:
- Physics calculations (gravity, drag remain constant)
- Collision detection (deterministic)

This variance reveals sensitivity to aiming accuracy and shot consistency.

## Performance Optimization

### Sub-Tick Physics
FuelSim supports multiple physics iterations per robot loop:
```java
FuelSim.getInstance().setSubticks(5);  // 5 physics steps per 20ms
```

**Trade-off**:
- **Higher sub-ticks**: More accurate physics, slower simulation
- **Lower sub-ticks**: Faster simulation, potential tunneling artifacts

Default: 1 sub-tick (adequate for most scenarios)

### Collision Optimization
**Broad Phase**: Spatial hashing reduces O(n²) collision checks
**Narrow Phase**: Only tests objects in nearby grid cells
**Result**: Handles 100+ simultaneous game pieces without slowdown

## Design Philosophy

The simulation framework prioritizes **development velocity** and **physical realism**:
- **FuelSim integration** enables end-to-end testing of game piece handling
- **Physics-based trajectories** provide realistic shooting behavior
- **AdvantageKit logging** creates identical data flow between sim and real robot
- **3D visualization** makes debugging intuitive and fast
- **Configurable variance** reveals sensitivity to real-world inaccuracies
- **Performance optimization** maintains real-time execution even with complex scenarios

**Key Benefit**: Teams can develop and test full autonomous routines, shooting algorithms, and intake strategies entirely in simulation before robot availability.
