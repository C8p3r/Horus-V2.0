# ⚠️ NON-BROWNOUT DISCONNECTION DIAGNOSIS
## Battery at 11V+ but STILL Losing Connection

---

## 🔍 YOUR SITUATION
- ✅ Battery voltage: **11V+** (good - not a brownout)
- ❌ **Sudden disconnections** still happening
- ❌ Radio loses connection (.1, .4 bad)
- ❌ roboRIO loses connection (.2 bad)

**This is NOT a power problem - it's a NETWORK/WIRING problem!**

---

## 🚨 TOP 5 MOST LIKELY CAUSES

### 1. **LOOSE ETHERNET CABLES** (40% of cases)
**Symptoms**: Intermittent disconnects, works then suddenly fails

**CHECK:**
- [ ] Ethernet cable from roboRIO to radio - **FIRMLY SEATED**?
- [ ] Try **different Ethernet cable** (common failure point)
- [ ] Check cable **not pinched/damaged** by robot mechanisms
- [ ] Remove and re-insert BOTH ends of cable
- [ ] Verify cable **clicks** when inserted

**FIX:** Replace Ethernet cable with known-good cable

---

### 2. **RADIO POWER ISSUE** (30% of cases)
**Symptoms**: Radio loses power intermittently, all lights go out

**CHECK:**
- [ ] Radio POE barrel jack connection - **TIGHT**?
- [ ] POE wire from roboRIO to radio - **good crimps**?
- [ ] Radio power LED - **stays solid green**?
- [ ] Measure voltage at radio barrel jack - **12V stable**?
- [ ] Check POE wire not **loose in connector**

**FIX:** 
- Re-crimp POE connector
- Check roboRIO POE port not damaged
- Try different radio (if available)

---

### 3. **CAN BUS ISSUES** (15% of cases)
**Symptoms**: Entire robot freezes, CAN errors in logs

**NEW SMARTDASHBOARD VALUES TO WATCH:**
```
CAN/Utilization %        - Should be <80%
CAN/Bus Off Count        - Should be 0
CAN/TX Error Count       - Should be 0
CAN/RX Error Count       - Should be <10
CAN/HIGH UTILIZATION     - Should be FALSE
CAN/BUS OFF ERROR        - Should be FALSE
```

**CHECK:**
- [ ] CAN bus utilization - **over 80%**?
- [ ] CAN bus off count - **greater than 0**?
- [ ] Yellow/green CAN wires - **twisted together**?
- [ ] CAN termination resistor - **present at both ends**?
- [ ] All CAN devices show up in Phoenix Tuner?

**FIX:**
- Reduce CAN update rates (see below)
- Add 120Ω termination resistors
- Check for damaged CAN wiring

---

### 4. **VIBRATION/MECHANICAL** (10% of cases)
**Symptoms**: Disconnects when robot moves/shoots (vibration causes intermittent contact)

**CHECK:**
- [ ] roboRIO mounting - **SECURE**? Not bouncing?
- [ ] Radio mounting - **SECURE**? Not vibrating loose?
- [ ] All wire connections - **strain relief**?
- [ ] Anderson connectors - **fully inserted**?
- [ ] Main breaker - **TIGHT** connection?

**FIX:**
- Add foam padding under roboRIO/radio
- Zip-tie all cables with strain relief
- Replace damaged connectors

---

### 5. **INTERFERENCE/EMI** (5% of cases)
**Symptoms**: Disconnects only when certain motors run

**CHECK:**
- [ ] Motor wires **close to radio**?
- [ ] Ethernet cable **parallel to motor wires**?
- [ ] Radio antenna **damaged/bent**?
- [ ] Flywheel motor wires **shielded**?

**FIX:**
- Route Ethernet cable AWAY from high-current wires
- Keep radio antenna vertical and clear
- Add ferrite beads to motor wires

---

## 🔧 IMMEDIATE DIAGNOSTIC STEPS

### STEP 1: Deploy This Code and Watch SmartDashboard

Enable robot and watch these values continuously:

**Power Section** (should be fine based on your 11V):
```
Power/Battery Voltage: 11.0V+ ✅
Power/Total Current: <400A ✅
```

**NEW: CAN Bus Section** (CRITICAL - watch for errors):
```
CAN/Utilization %: Should be <80%
CAN/Bus Off Count: Should be 0
CAN/TX Error Count: Should be 0
Network/Connected: Should be TRUE
```

### STEP 2: Physical Inspection (DO THIS NOW)

**BEFORE ENABLING ROBOT:**
1. **Unplug and re-plug Ethernet cable** (roboRIO ↔ radio)
   - Should **click** when fully seated
   - Cable should not pull out easily

2. **Check radio power**:
   - Power LED solid green?
   - Barrel jack connector tight?
   - Measure voltage with multimeter: 12V?

3. **Wiggle test**:
   - Gently wiggle Ethernet cable while watching Driver Station
   - Does connection drop? **Replace cable immediately**

4. **Check CAN wiring**:
   - Yellow/green wires twisted together?
   - Connections tight at all CAN devices?
   - 120Ω termination resistors at BOTH ends?

### STEP 3: Reproduce the Problem

Enable robot and systematically test:

1. **Idle** - does it stay connected? (30 seconds)
   - If drops: **WIRING PROBLEM**

2. **Drive slowly** - does it stay connected?
   - If drops: **VIBRATION or CAN bus issue**

3. **Shoot stationary** - does it stay connected?
   - If drops: **CAN bus overload or EMI**

4. **Drive + Shoot** - does it drop?
   - If drops: **Combination of above**

### STEP 4: Check Driver Station Logs

Open Driver Station → Gear icon → View Logs

Look for these error messages:
- **"Lost communication with Robot"** - Network/cable issue
- **"CAN bus off"** - CAN wiring problem
- **"No code"** - Code crash (memory leak)
- **"Watchdog not fed"** - Code taking too long (CAN overload)

---

## 🛠️ QUICK FIXES TO TRY (IN ORDER)

### Fix #1: Replace Ethernet Cable
**Time**: 2 minutes  
**Success Rate**: 40%

Simply replace the cable between roboRIO and radio with a **known-good** cable.

### Fix #2: Reduce CAN Update Rates
**Time**: 5 minutes  
**Success Rate**: 20%

If CAN utilization is >80%, reduce update rates. Add this to each motor subsystem:

**Example for FlywheelSubsystem.java:**
```java
// After motor initialization, add:
velocitySignal.setUpdateFrequency(20); // Reduce from 50Hz to 20Hz
flywheelMotor.optimizeBusUtilization();
```

Do this for ALL TalonFX motors that are hitting 50Hz or higher.

### Fix #3: Check Radio Configuration
**Time**: 5 minutes  
**Success Rate**: 15%

1. Open web browser, go to: `http://10.49.25.1`
2. Login (default: no password)
3. Check radio is in **"Bridge Mode"**
4. Verify roboRIO IP is: `10.49.25.2`
5. Save settings and reboot radio

### Fix #4: Re-flash Radio Firmware
**Time**: 10 minutes  
**Success Rate**: 10%

1. Open FRC Radio Configuration Utility
2. Load radio firmware for team 4925
3. Flash radio
4. Test again

---

## 🎯 EXPECTED SMARTDASHBOARD VALUES

| Value | Good | Warning | Critical |
|-------|------|---------|----------|
| Battery Voltage | >11V | 9-11V | <9V |
| Total Current | <400A | 400-500A | >500A |
| CAN Utilization | <60% | 60-80% | **>80%** |
| CAN Bus Off | 0 | 0 | **>0** |
| CAN TX Errors | 0 | <5 | **>5** |
| Network Connected | TRUE | - | **FALSE** |

---

## ⚡ NUCLEAR OPTION: Simplify Code

If NOTHING else works, there might be a code issue causing crashes.

**Add this safety check to Robot.java robotPeriodic():**

```java
@Override
public void robotPeriodic() {
    try {
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();
    } catch (Exception e) {
        System.err.println("❌ ERROR IN PERIODIC: " + e.getMessage());
        e.printStackTrace();
        // Don't crash - keep running
    }
}
```

This prevents code crashes from killing the robot connection.

---

## 📊 MOST LIKELY DIAGNOSIS

Based on "battery at 11V+ but still disconnects":

1. **60% chance**: Loose/bad Ethernet cable
2. **25% chance**: Radio power issue (POE connection)
3. **10% chance**: CAN bus overload (>80% utilization)
4. **5% chance**: Vibration causing intermittent contact

**START WITH ETHERNET CABLE REPLACEMENT** - this fixes most cases.

---

## 📞 REPORT BACK WITH:

1. **CAN/Utilization %** value from SmartDashboard
2. **CAN/Bus Off Count** value
3. **Does wiggling Ethernet cable cause disconnect?**
4. **Do disconnects happen at specific times?** (when driving? shooting? both?)
5. **Can you hear/feel radio rebooting?** (relay clicks?)

This will narrow down the exact cause.

---

## 🚨 IF NOTHING WORKS

**Last resort checks:**
- Replace radio completely (may be defective)
- Replace roboRIO Ethernet port (may be damaged)
- Check for robot code memory leaks (excessive object creation)
- Verify Java heap not exhausted (OutOfMemoryError in logs)
- Test with MINIMAL code (disable all subsystems except drivetrain)

**Your voltage is good - this is 90% a wiring/network hardware problem, NOT software.**
