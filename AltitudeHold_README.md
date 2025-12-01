# Altitude Hold System — Internal Documentation

## Overview
This document explains how the MagisV2 Altitude Hold module works internally: estimator pipeline, ToF/barometer fusion, Kalman filters, and what functions you can call to obtain fused altitude and velocity.

## 1. System Summary
The altitude-hold subsystem:
1. Estimates altitude/vertical velocity using IMU + Barometer + (optional) ToF.
2. Fuses data with:
   - A 3rd‑order complementary filter (APM-style)
   - Kalman filters (altitude + velocity)
3. Controls altitude using a velocity-PID loop.

## 2. Active Features in Your Build
Because `LASER_ALT` and `LASER_TOF` are defined, but `SONAR` and `LASER_TOF_L1x` are not, the following apply:

| Feature | Active? | Notes |
|--------|--------|-------|
| Barometer | ✔️ | Always active |
| ToF Laser | ✔️ | Uses `LASER_TOF` block |
| ToF L1X | ❌ | Not compiled |
| Sonar | ❌ | Not compiled |
| APM Complementary Filter | ✔️ | Active |
| Kalman Filters | ✔️ | Active |
| Baro ↔ ToF Fusion | ✔️ | `correctedWithTof()` + `correctedWithBaro()` |

## 3. Key Functions

### Estimators
### `apmCalculateEstimatedAltitude(uint32_t now)`
Main estimator.
- Integrates acceleration  
- Applies baro/ToF corrections  
- Smooths with Kalman filters  
- Outputs:
  - `EstAlt` (cm)
  - `VelocityZ` (cm/s)

### `calculateEstimatedAltitude(uint32_t now)`
Alternate baro-driven estimator (40Hz). Also produces filtered altitude/velocity.

---

### Fusion Support Functions

#### `checkReading()`
Runs ToF + barometer fusion. Active because `LASER_ALT` is defined.

#### `correctedWithTof(float h)`
- Computes `_position_error_z = h - EstAlt`
- Sets time constant = 1.5 → fast correction
- Updates gains

#### `correctedWithBaro(float baroAlt, float dt)`
- Uses delayed history queue to compensate latency
- Updates `_position_error_z`
- Adapts time constant based on tilt

---

### Control Loop

#### `calculateAltHoldThrottleAdjustment(velocity_z, accZ_tmp, accZ_old)`
Performs:
1. Position-to-velocity conversion  
2. Velocity PID control  
3. Outputs throttle adjustment  

#### `applyAltHold()`
Applies the throttle correction to the pilot command.

---

## 4. Getting Fused Altitude & Velocity

Call the estimator first:

```c
apmCalculateEstimatedAltitude(now);
```

Then read fused values:

```c
int32_t height = getEstAltitude();   // cm
int32_t vel    = getEstVelocity();   // cm/s
```

Alternative altitude getter:
```c
altitudeHoldGetEstimatedAltitude();
```

---

## 5. Minimal Example

```c
uint32_t t = micros();

// Update estimator (recommended)
apmCalculateEstimatedAltitude(t);

// Read fused outputs
int32_t height_cm = getEstAltitude();
int32_t vel_cms   = getEstVelocity();
```

---

## 6. Notes & Caveats
- ToF is only used under ~200–350 cm and small tilt (<25°).
- Velocity requires a warm-up of ~5 samples.
- Kalman filters smooth outputs but the complementary filter does most of the fusion.
- `_time_constant_z` adjusts correction speed dynamically.

---

## 7. Pipeline Summary

```
IMU + BARO + TOF → correction functions → 3rd-order complementary filter
                                               ↓
                                        Kalman filters
                                               ↓
                                 EstAlt (cm) & VelocityZ (cm/s)
                                               ↓
                         calculateAltHoldThrottleAdjustment()
                                               ↓
                                   rcCommand[THROTTLE]
```

---

## 8. Quick Reference

| Purpose | Function |
|---------|----------|
| Update estimator | `apmCalculateEstimatedAltitude(now)` |
| Get fused height | `getEstAltitude()` |
| Get fused velocity | `getEstVelocity()` |
| ToF correction | `correctedWithTof()` |
| Baro correction | `correctedWithBaro()` |

---

