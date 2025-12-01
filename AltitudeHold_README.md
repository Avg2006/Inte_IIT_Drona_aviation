
# Altitude Hold System — Technical Documentation

## 1. Introduction
This document explains the internal working of the MagisV2 Altitude Hold subsystem.  
It includes altitude estimation, sensor fusion (Barometer + IMU + ToF), filtering,  
and throttle control mechanics. All explanations reflect your current firmware  
build where **LASER_ALT** and **LASER_TOF** are enabled, while **SONAR** and  
**LASER_TOF_L1x** are not.

This README is formatted for clean, professional GitHub documentation.

---

## 2. Build Configuration Summary

Your firmware defines:

| Macro | Status | Meaning |
|-------|--------|---------|
| `LASER_ALT` | ✔ Active | ToF fusion system enabled |
| `LASER_TOF` | ✔ Active | VL53L0X ToF data handled |
| `SONAR` | ✖ Disabled | No sonar pipeline compiled |
| `LASER_TOF_L1x` | ✖ Disabled | L1X ToF not included |

As a result:

- Barometer + IMU + ToF fusion is active  
- Sonar-based altitude is not used  
- Laser L1x model logic is not compiled  

---

## 3. Architecture Overview

### 3.1 High-Level Estimation Flow

```
IMU Acceleration
        │
        ▼
APM 3rd Order Complementary Filter  ◄── Barometer
        │                           ◄── ToF (short range)
        ▼
Kalman Filters (Altitude + Velocity smoothing)
        ▼
Fused Estimates:
   - EstAlt (cm)
   - VelocityZ (cm/s)
```

### 3.2 Control Flow

```
Fused Altitude & Velocity
        │
        ▼
Velocity PID Loop
        │
        ▼
Throttle Adjustment
        ▼
Mixed into rcCommand[THROTTLE]
```

---

## 4. Estimation Functions

### `apmCalculateEstimatedAltitude(uint32_t now)`
This is the **main altitude estimator** in your configuration.

It performs:

- Integration of IMU acceleration  
- Correction using `_position_error_z` from Baro / ToF  
- Update of:
  - `_position_base_z`
  - `_position_correction_z`
  - `_velocity_z`
- Writes final values:
  - `EstAlt` (cm)  
  - `VelocityZ` (cm/s)  
- Applies Kalman filters for smoothing

This function is recommended for all real flight altitude estimation.

---

### `calculateEstimatedAltitude(uint32_t now)`
Alternative barometer-driven estimator (≈40 Hz):

- Blends barometer + accelerometer using complementary filter  
- Uses Kalman filters for altitude and velocity  
- Produces `EstAlt` and `vel`  

This is used depending on scheduler and configuration but  
**APM method is primary** when enabled.

---

## 5. Sensor Fusion Functions

### `checkReading()`
Active because `LASER_ALT` is defined.

This function:

- Reads barometer updates  
- Reads ToF (if new and valid)  
- Applies tilt compensation (`cos(tilt)`)  
- Selects which correction to apply:

```
If ToF valid and in usable range:
    correctedWithTof(ToF_Height)
Else:
    correctedWithBaro(Baro_Height)
```

---

### `correctedWithTof(float h)`
Used when ToF height is reliable:

- Computes `_position_error_z = h - EstAlt`
- Sets `_time_constant_z = 1.5`
- Updates complementary filter gains
- Enables fast, accurate low-altitude correction

---

### `correctedWithBaro(float baroAlt, float dt)`
Used when:

- ToF unavailable  
- ToF out of range  
- Height too large  

Behavior:

- Retrieves historical base position to offset baro delay  
- Computes `_position_error_z = baroAlt - delayed_base_position`  
- Selects time constant based on tilt  
  - Normal: 2  
  - Tilted: 5  
- Updates filter gains  

---

## 6. Kalman Filtering

Two scalar Kalman filters are used:

1. **Altitude filter**  
2. **Velocity filter**

Functions:

- `kalmanFilterInit(KF, Q, R, initialValue)`
- `kalmanFilterUpdate(KF, measurement)`

These filters smooth measurement noise but do *not* replace  
the complementary filter—they refine its output.

Outputs:

- `EstAlt = filtered altitude`
- `VelocityZ = filtered vertical speed`

---

## 7. Throttle Control Loop

### Function: `calculateAltHoldThrottleAdjustment()`

### Modes:
#### 1. **Altitude Hold**
```
error = AltHold - EstAlt
setVel = P_alt * error
```

#### 2. **Velocity Control**
```
setVel = pilot input mapped to velocity
```

### Final throttle adjustment:
```
vel_error = setVel - velocity_z
P_term = P_vel * vel_error
I_term = ∫ I_vel * vel_error
D_term = D_vel * (accZ_tmp + accZ_old)
ThrottleAdjust = P + I - D
```

### Application:
`applyMultirotorAltHold()` or  
`applyFixedWingAltHold()` inject the throttle delta  
into `rcCommand[THROTTLE]`.

---

## 8. Obtaining Fused Altitude & Velocity

After calling the estimator:

```c
int32_t height_cm = getEstAltitude();
int32_t vel_cms   = getEstVelocity();
```

### Alternative getter:
```c
altitudeHoldGetEstimatedAltitude();
```

### Minimal example:
```c
uint32_t t = micros();

// Update estimator
apmCalculateEstimatedAltitude(t);

// Read fused values
int32_t h = getEstAltitude();   // cm
int32_t v = getEstVelocity();   // cm/s
```

---

## 9. Debug Signals

The module exposes debug variables such as:

- `altholdDebug`
- `altholdDebug1`
- `altholdDebug5`
- `altholdDebug8`
- `altholdDebug9`
- `velControlDebug[]`

These help observe:

- Fused altitude  
- Correction signals  
- Velocity error  
- Baro/ToF impact  
- PID contributions  

---

## 10. Practical Notes

- ToF used only at low altitudes (up to ~200–350 cm)
- ToF disabled automatically at large tilt  
- Velocity estimation stabilizes after ~5 cycles  
- `_time_constant_z` dynamically adjusts response  
- Kalman filters smooth final values  
- Barometer delay is compensated using history queue  

---

## 11. Full Data Pipeline Diagram

```
 Sensors
   │
   ├─ IMU Acceleration
   ├─ Barometer
   └─ ToF (if valid)
        │
        ▼
 Correction Functions:
   - correctedWithTof()
   - correctedWithBaro()
        │
        ▼
 3rd Order Complementary Filter
        │
        ▼
 Kalman Filters (alt + vel)
        │
        ▼
 Fused Outputs:
   - EstAlt (cm)
   - VelocityZ (cm/s)
        │
        ▼
 Velocity PID Controller
        │
        ▼
 altHoldThrottleAdjustment
        │
        ▼
 rcCommand[THROTTLE]
```

---

## 12. API Quick Reference

| Task | Function |
|------|----------|
| Update altitude/velocity estimator | `apmCalculateEstimatedAltitude()` |
| Read fused altitude | `getEstAltitude()` |
| Read fused velocity | `getEstVelocity()` |
| Apply ToF correction | `correctedWithTof()` |
| Apply baro correction | `correctedWithBaro()` |
| Reset alt-hold system | `AltRst()` |
| Set hold target | `setAltitude()` |

---

End of README.
