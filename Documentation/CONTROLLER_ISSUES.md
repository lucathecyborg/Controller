# Controller PID Tuning - Issues Found ⚠️

## CRITICAL ISSUES

### 🚨 Issue #1: PID Defaults in Message Struct Don't Match Drone Defaults
**Location:** Line 14-18 (controller.cpp)
```cpp
struct message {
  // ...
  float kp=1.5, ki=0.05, kd=0.8;  // Defaults in struct
};
```

**Problem:** These defaults (1.5, 0.05, 0.8) do NOT match drone's conservative values:
```
Drone defaults (from main.cpp):
  rollPID = {0.8, 0.02, 0.4, ...}     ← kp=0.8, ki=0.02, kd=0.4
  pitchPID = {0.8, 0.02, 0.4, ...}
  yawPID = {1.5, 0.01, 0.05, ...}     ← yaw: ki=0.01, kd=0.05

Controller defaults:
  kp=1.5, ki=0.05, kd=0.8            ← WRONG for roll/pitch!
```

**Impact:** If drone receives message before calibration, it gets wrong gains!
- Roll/Pitch will be too aggressive (kp 1.5 vs expected 0.8)
- Integral will be way too high (ki 0.05 vs expected 0.02 or 0.01)
- System will oscillate on first flight

**FIX:** Change struct defaults to match drone:
```cpp
struct message {
  // ...
  float kp=0.8, ki=0.02, kd=0.4;  // Match drone defaults
};
```

---

### 🚨 Issue #2: Local Variable Defaults Don't Match Drone Defaults
**Location:** Lines 21-23
```cpp
float roll_kp = 1.5, roll_ki = 0.05, roll_kd = 0.8;  
float pitch_kp = 1.5, pitch_ki = 0.05, pitch_kd = 0.8;
float yaw_kp = 1.5, yaw_ki = 0.05, yaw_kd = 0.8;
```

**Problem:** Same issue - these defaults are WRONG
- Should be: roll_kp = 0.8, roll_ki = 0.02, roll_kd = 0.4
- And separate for yaw: yaw_kp = 1.5, yaw_ki = 0.01, yaw_kd = 0.05

**Impact:** When user starts tuning, they begin from wrong baseline
- If they don't change values, wrong gains get sent
- Tuning process starts with already-bad values

**FIX:** Match drone exactly:
```cpp
float roll_kp = 0.8, roll_ki = 0.02, roll_kd = 0.4;  
float pitch_kp = 0.8, pitch_ki = 0.02, pitch_kd = 0.4;
float yaw_kp = 1.5, yaw_ki = 0.01, yaw_kd = 0.05;
```

---

### ⚠️ Issue #3: Joysticks Disabled in sendData()
**Location:** Lines 168-177
```cpp
void sendData(){
  /* Data.joystickL = joystickL.getValues();
  Data.joystickR = joystickR.getValues();  */
  Data.joystickL.x = 512;    // HARDCODED to center!
  Data.joystickL.y = 512;
  Data.joystickL.button = 0;

  Data.joystickR.x = 512;    // HARDCODED to center!
  Data.joystickR.y = 512;
  Data.joystickR.button = 0;
```

**Problem:** Joysticks are commented out AND hardcoded to center (512)
- Drone won't respond to ANY stick input
- Drone only gets throttle (pot1)
- Can't control pitch/roll/yaw at all!

**Impact:** 
- Drone is completely uncontrollable
- Can only adjust gains, nothing else
- Aircraft will crash if you try to fly

**FIX:** Uncomment joysticks:
```cpp
Data.joystickL = joystickL.getValues();
Data.joystickR = joystickR.getValues();
```

---

### ⚠️ Issue #4: Gain Adjustment Steps Are Wrong for Yaw
**Location:** Line 265 (Yaw kd adjustment)
```cpp
case 2: // Yaw
  yaw_kd += delta * 0.02;  // ← 0.02 increment per notch
  drawPID(2, 2, yaw_kp, yaw_ki, yaw_kd);
```

**vs. Pitch/Roll:**
```cpp
case 2: // D for Pitch
  pitch_kd += delta * 0.2;  // ← 0.2 increment per notch
```

**Problem:** Yaw kd uses 0.02 (10× smaller) but others use 0.2
- Yaw tuning is 10× more sensitive
- User will get frustrated with tiny changes
- Inconsistent tuning experience

**Analysis:**
- Yaw kd starts at 0.05 (very small)
- With 0.02 step: Each notch changes by 40%!
- Roll/Pitch kd starts at 0.4
- With 0.2 step: Each notch changes by 50%

**Better approach:** Use consistent increment:
```cpp
yaw_kd += delta * 0.005;  // Smaller absolute, but proportional to base value
```

---

### ⚠️ Issue #5: Ki Adjustment Different for Yaw
**Location:** Lines 252, 258, 264
```cpp
case 1: // I
  switch (axis) {
    case 0: // Pitch
      pitch_ki += delta * 0.05;  // 0.05 step
    case 1: // Roll
      roll_ki += delta * 0.05;   // 0.05 step
    case 2: // Yaw
      yaw_ki += delta * 0.05;    // 0.05 step (same as others)
```

**Problem:** Yaw ki should be MUCH smaller increment
- Yaw ki default: 0.01 (small)
- Roll/Pitch ki default: 0.02 (also small)
- Increment 0.05 is massive for both!

**Example:** If user turns encoder once on yaw_ki:
- Yaw ki: 0.01 + 0.05 = 0.06 (6× increase!)
- Roll ki: 0.02 + 0.05 = 0.07 (3.5× increase)

**Better approach:** Use smaller increments
```cpp
pitch_ki += delta * 0.005;  // 0.5% per notch
roll_ki += delta * 0.005;
yaw_ki += delta * 0.002;    // Smaller for yaw
```

---

## ISSUES SUMMARY TABLE

| Issue | Severity | Impact | Fix Difficulty |
|-------|----------|--------|-----------------|
| **PID Defaults in Struct** | 🚨 CRITICAL | Wrong gains sent first | Easy |
| **Local PID Defaults** | 🚨 CRITICAL | Wrong tuning baseline | Easy |
| **Joysticks Disabled** | 🚨 CRITICAL | No control authority | Easy |
| **Yaw Kd Step Size** | ⚠️ MEDIUM | Poor tuning UX | Easy |
| **Ki Step Sizes** | ⚠️ MEDIUM | Coarse tuning granularity | Easy |

---

## QUICK FIXES

### Fix #1: Update Struct Defaults
```cpp
struct message {
  uint16_t pot1;
  joystickValues joystickL;
  joystickValues joystickR;
  uint8_t PidAxis=3;
  float kp=0.8, ki=0.02, kd=0.4;  // Changed!
};
```

### Fix #2: Update Local Variables
```cpp
float roll_kp = 0.8, roll_ki = 0.02, roll_kd = 0.4;  
float pitch_kp = 0.8, pitch_ki = 0.02, pitch_kd = 0.4;
float yaw_kp = 1.5, yaw_ki = 0.01, yaw_kd = 0.05;
```

### Fix #3: Enable Joysticks
```cpp
void sendData(){
  Data.joystickL = joystickL.getValues();
  Data.joystickR = joystickR.getValues();
  // Remove the hardcoded 512 values!
```

### Fix #4 & #5: Better Increment Values
Consider more granular control:
```cpp
// In Calibration() function:
pitch_ki += delta * 0.005;  // Smaller increment
roll_ki += delta * 0.005;
yaw_ki += delta * 0.002;    // Even smaller for yaw

yaw_kd += delta * 0.005;    // Consistent with others
```

---

## BEFORE FLYING

**DO NOT FLY until you fix these issues!**

Priority order:
1. ✅ Fix joysticks (Issue #3) - MANDATORY
2. ✅ Fix PID defaults (Issues #1 & #2) - MANDATORY
3. ✅ Fix increment values (Issues #4 & #5) - Recommended

Without fixes #1-3, your drone:
- Will be uncontrollable
- Will have wrong PID gains
- Will crash immediately

---

## VERIFICATION AFTER FIXES

After making changes:
1. Check that joysticks read values (not hardcoded 512)
2. Check that default gains match drone in main.cpp
3. In tuning mode, verify small increments per encoder notch
4. Test with propellers OFF first
5. Only then attempt flight

---

*Analysis Date: November 2025*
*Status: Issues Found - Fixes Needed*
