# Robot Controls Documentation

## Controller Layout

### Driver Controller (Port 0)

#### Drive Controls
- **Left Stick**: Translate (forward/backward, left/right)
- **Right Stick X**: Rotate

#### Special Drive Modes
- **A Button**: X-brake (lock wheels in X formation)
- **B Button**: Point wheels at joystick direction

#### Shooting
- **Right Trigger**: Shoot at 60 RPS (high speed, while held)
- **Left Trigger**: Shoot at 40 RPS (medium speed, while held)

#### Vision & Navigation
- **D-Pad Left**: Pathfind to "Far Climb"
- **D-Pad Right**: Pathfind to "Close Climb"

#### Intake Controls
- **Right Bumper**: Intake (deploy, run roller + indexer, while held)
- **Left Bumper**: Eject (reverse all intake/indexer motors, while held)

#### SysId (Characterization)
- **Back + Y**: Dynamic forward
- **Back + X**: Dynamic reverse
- **Start + Y**: Quasistatic forward
- **Start + X**: Quasistatic reverse

---

### Operator Controller (Port 1)

#### Manual Adjustments
- **Left Stick Y**: Hood angle adjustment
  - Up = increase angle
  - Down = decrease angle
  - Rate: 50°/sec with squared input curve
- **Right Stick X**: Turret angle adjustment
  - Right = clockwise
  - Left = counterclockwise
  - Rate: 180°/sec with squared input curve

#### Vision & Navigation
- **Left Bumper**: Reset field-centric heading
- **Right Bumper**: Toggle vision processing

---

## Choreographed Sequences

### Shooting Sequence
When you press and hold a trigger to shoot:

1. **Phase 1**: Flywheel spins up to target velocity
   - Floor indexer runs at LOW speed (5 RPS)
   - Keeps game piece slightly compressed

2. **Phase 2**: Wait for flywheel to reach target
   - Safety timeout: 3 seconds

3. **Phase 3**: Brief pause
   - Floor indexer stops for 50ms
   - Ensures clean feed

4. **Phase 4**: Fire indexer starts
   - Fire motor ramps to HIGH speed (60 RPS)
   - 100ms ramp-up time

5. **Phase 5**: Stable feed
   - Floor indexer runs at stable 60 RPS
   - Fire indexer runs continuously at 60 RPS
   - Both indexers feed continuously while trigger held

6. **On Release**: Cleanup
   - Fire indexer reverses briefly (-20 RPS for 150ms)
   - Clears any jammed game pieces
   - All motors stop

### Intake Sequence
When you press and hold the right bumper (driver):

1. **Deploy**: Intake extends (300ms)
2. **Run**: Intake roller (40 RPS) + Floor indexer (30 RPS)
3. **On Release**: Stop motors and retract intake

### Eject Sequence
When you press and hold the left bumper (driver):

1. **Reverse All**: Intake roller, floor indexer, and fire indexer run backwards
2. **On Release**: Stop all motors

---

## Constants Reference

### Shooting Speeds
- **High Speed**: 60 RPS (Right trigger)
- **Medium Speed**: 40 RPS (Left trigger)

### Indexer Speeds
- **Floor Low**: 5 RPS (during flywheel spinup)
- **Floor/Fire High**: 60 RPS (stable feeding during shooting)
- **Floor Intake**: 30 RPS (intake mode)
- **Fire Reverse**: -20 RPS (clearing jams)

### Intake Speeds
- **Roller**: 40 RPS
- **Floor Indexer**: 30 RPS

### Timings
- **Intake Deploy**: 300ms
- **Floor Pause**: 50ms
- **Fire Ramp**: 100ms
- **Reverse Burst**: 150ms
- **Flywheel Timeout**: 3000ms

---

## Tips

### For Drivers
- Use the triggers for quick shooting - they handle all the sequencing automatically
- The robot will wait for the flywheel to spin up before feeding
- Don't release the trigger too quickly - let the shot complete
- Right bumper for intake - it will automatically deploy and retract
- Left bumper to eject if you accidentally intake an opponent's game piece

### For Operators
- Hood and turret adjustments are continuous - small movements for precision
- Squared input curves mean slow movements are very precise, fast movements are responsive
- Left bumper resets field-centric heading if the robot gets confused
- Right bumper toggles vision processing on/off if vision is causing issues

### During Competition
- Right trigger (60 RPS) for long shots and full-court shots
- Left trigger (40 RPS) for close-range shots or amp scoring
- The auto-reverse after shooting helps prevent jams
- Driver controls intake with right bumper - operator focuses on aiming
- Use operator's vision toggle if vision is causing issues

---

## LED Feedback (CANdle)

The robot uses a CANdle LED strip with built-in SingleFadeAnimation to provide dynamic visual feedback on robot state with smooth breathing/pulsing effects. All LEDs display the same color based on priority:

### LED States (Highest to Lowest Priority)

1. **Shooting States**:
   - 🟢 **Pulsing Green** (fast): Flywheel at target speed - ready to shoot!
     - Speed: 3 Hz (3 pulses/second)
     - Brightness: 60% to 100%
   - 🟡 **Breathing Yellow** (medium): Flywheel spinning up - wait for green
     - Speed: 1.5 Hz
     - Brightness: 40% to 90%

2. **Intake Active**:
   - 🔵 **Pulsing Blue** (fast): Intake rollers running
     - Speed: 2.5 Hz
     - Brightness: 50% to 100%

3. **Aiming Ready**:
   - 🟣 **Pulsing Purple** (medium): Turret and hood at target positions - ready to fire
     - Speed: 2 Hz
     - Brightness: 70% to 100%

4. **Idle**:
   - ⚪ **Slow Breathing White**: Robot idle, no active mechanisms
     - Speed: 0.5 Hz (slow breathing)
     - Brightness: 10% to 30%

### LED Configuration
- **LED Count**: 68 total (8 onboard + 60 external)
- **Animation Engine**: CANdle built-in SingleFadeAnimation (hardware-accelerated)
- **Update Rate**: Handled by CANdle hardware automatically
- **Modulation**: Smooth breathing effect using CANdle's animation engine

### Brightness Modulation Features
- **SingleFadeAnimation**: Uses CANdle's built-in hardware animation engine
- **Hardware Accelerated**: No CPU overhead - runs on CANdle hardware
- **Smooth Transitions**: Hardware-based smooth fade in/out
- **Speed Indicators**: Faster pulsing = more urgent/active state
- **Visual Appeal**: Dynamic feedback is more engaging and easier to spot
- **Efficient**: No manual periodic updates needed

---

## Safety Features

1. **Flywheel Timeout**: If flywheel doesn't reach speed in 3 seconds, sequence continues anyway
2. **Auto-Reverse**: Prevents jams by clearing the fire indexer after each shot
3. **Intake Retract**: Automatically retracts when intake button is released
4. **Deploy Motor Disable**: Can be toggled in code for testing (saves wear)
5. **LED Priority System**: Ensures most critical state is always visible

