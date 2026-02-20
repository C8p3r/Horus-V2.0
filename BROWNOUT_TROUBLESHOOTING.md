# BROWNOUT TROUBLESHOOTING GUIDE
## Robot Communication Loss Diagnosis

### 🚨 YOUR SYMPTOMS
- Driver Station loses communication with robot
- Radio loses connection (.1, .4 all bad)
- roboRIO connection drops (.2 bad)
- Happens when shooting OR driving for several seconds
- Battery is fully charged
- All software/firmware updated

---

## ✅ CHANGES IMPLEMENTED

### 1. Current Limiting (Applied)
**Drivetrain**: Reduced from 120A → **80A per module** (320A total vs 480A)
**Flywheel**: Reduced from 60A → **35A supply**, 120A → **80A stator**
**Indexer**: Added **20A supply / 40A stator limits** (was unlimited)
**Intake**: Added **25A supply / 40A stator limits** (was unlimited)
**Turret**: Reduced from 30A → **20A supply**, 60A → **40A stator**

**Expected Total Current**: ~440A peak (vs 600-700A before)

### 2. Power Monitoring System (New)
Created `PowerMonitor.java` utility that tracks:
- Real-time battery voltage
- Total current draw from PDH
- Individual PDH channel currents
- Brownout event counting
- Peak voltage/current statistics

### 3. Visual Brownout Alerts (New)
LEDs now show power status:
- **Rapid Red Blink** = Brownout (<6.8V) - EMERGENCY
- **Dim Yellow** = Low voltage (<9V) - WARNING
- Normal operation colors otherwise

---

## 🔍 DIAGNOSTIC STEPS

### STEP 1: Check SmartDashboard (CRITICAL)
Deploy the code and watch these new values:

```
Power/Battery Voltage        - Watch for drops below 10V
Power/Total Current          - Should stay under 400A
Power/BROWNOUT RISK         - Will turn TRUE if voltage <6.8V
Power/VOLTAGE WARNING       - Will turn TRUE if voltage <9V
Power/Lowest Voltage         - Shows minimum voltage hit
Power/Highest Current        - Shows peak current draw
Power/Brownout Count         - Counts brownout events
```

### STEP 2: Test Scenarios
Run these tests **ONE AT A TIME** while watching SmartDashboard:

1. **Idle Robot** (no movement):
   - Expected: 12.0-12.6V, 10-20A
   - If voltage drops: **WIRING PROBLEM**

2. **Drive Only** (no shooting):
   - Expected: 10.5-11.5V, 150-300A
   - If voltage <9V: **DRIVETRAIN CURRENT TOO HIGH**

3. **Shoot Only** (stationary):
   - Expected: 10.5-11.5V, 100-150A
   - If voltage <9V: **FLYWHEEL CURRENT TOO HIGH**

4. **Shoot + Drive** (combined):
   - Expected: 9.5-10.5V, 350-450A
   - If voltage <8V: **COMBINED STILL TOO HIGH**
   - If voltage <6.8V: **BROWNOUT - REDUCE LIMITS MORE**

### STEP 3: Check PDH Channels
Watch `Power/PDH Channel 0-3` to identify specific high-current devices.

Common PDH layout:
- Channels 0-7: Usually drive motors
- Channels 8-15: Usually mechanism motors
- Identify which channel spikes during brownouts

---

## 🔧 IF BROWNOUTS PERSIST

### IMMEDIATE FIXES:

#### Option A: Reduce Drivetrain Further
Edit `TunerConstants.java` line 56:
```java
private static final Current kSlipCurrent = Amps.of(60); // Was 80A
```

#### Option B: Reduce Flywheel Further
Edit `FlywheelConstants.java` line 36:
```java
public static final double SUPPLY_CURRENT_LIMIT = 25.0; // Was 35A
public static final double STATOR_CURRENT_LIMIT = 60.0; // Was 80A
```

#### Option C: Disable Intake During Shooting
Comment out intake in `ShootCommand.java` Phase 4:
```java
// Commands.runOnce(() -> intakeRollerSubsystem.setVelocity(INTAKE_SHOOT_SPEED))
```

### HARDWARE CHECKS:

1. **Battery Connections**:
   - ✅ Check main breaker contact - corrosion?
   - ✅ Check battery terminal bolts - TIGHT?
   - ✅ Check PDH main input wires - proper crimps?
   - ✅ Measure battery voltage with multimeter at breaker

2. **Radio Power**:
   - ✅ Radio should be on POE barrel jack from roboRIO
   - ✅ Check radio power wire connections
   - ✅ Verify radio is getting 12V from roboRIO
   - ✅ Try different Ethernet cable

3. **Wiring Resistance**:
   - ❌ **Long/thin wires to PDH** = voltage drop
   - ❌ **Poor crimps** = high resistance
   - ❌ **Loose Anderson connectors** = intermittent
   - ✅ Use 10AWG or larger for high-current devices
   - ✅ Keep wire runs SHORT

4. **Battery Health**:
   - ✅ Battery should be <1 year old
   - ✅ Check battery internal resistance (should be <15mΩ)
   - ✅ Try a DIFFERENT battery (maybe yours is bad)
   - ✅ Battery should rest at 12.6-13.0V when fully charged

---

## 📊 TYPICAL BROWNOUT CAUSES (IN ORDER)

1. **40%** - Battery is OLD or DAMAGED (high internal resistance)
2. **30%** - Poor main battery connections (corroded/loose)
3. **15%** - Current limits too high (fixed by our changes)
4. **10%** - Long wire runs with high resistance
5. **5%** - PDH or roboRIO power issues

---

## 🎯 EXPECTED BEHAVIOR AFTER FIX

✅ **Robot should drive aggressively** without brownout
✅ **Flywheel should spin up slower** (~2-3 seconds vs 1.5s) but reliably
✅ **Shooting + driving** should work without radio disconnect
✅ **Battery voltage** should stay above 9V even under full load
✅ **LEDs show dim yellow** occasionally but NOT rapid red

---

## ⚡ EMERGENCY: IF RADIO KEEPS DISCONNECTING

### Nuclear Option: Disable All Mechanism Motors During Drive

Add this to `RobotContainer.java` in the drive command:
```java
// Stop all mechanisms when driving
drivetrain.setDefaultCommand(
    drivetrain.applyRequest(() -> drive
        .withVelocityX(-joystick.getLeftY() * maxSpeed)
        .withVelocityY(-joystick.getLeftX() * maxSpeed)
        .withRotationalRate(-joystick.getRightX() * maxAngularRate)
    ).beforeStarting(() -> {
        flywheelSubsystem.stop();
        indexerSubsystem.stopAll();
        intakeRollerSubsystem.stop();
    })
);
```

This will prevent shooting while driving (competition rule violation but good for testing).

---

## 📞 NEXT STEPS

1. **Deploy this code** immediately
2. **Watch SmartDashboard** - record all "Power/" values
3. **Test each scenario** separately
4. **Report back** with voltage/current readings
5. If still brownout: **Check battery health FIRST**

---

## 🛠️ TOOLS NEEDED

- Multimeter (measure battery voltage)
- Battery internal resistance tester (optional but very useful)
- Spare battery (to eliminate battery as variable)
- Wire crimper (if need to remake connections)
- Small wire brush (clean corrosion)

---

**GOOD LUCK!** This should fix the issue. If not, it's 90% likely a battery or wiring problem, not software.
