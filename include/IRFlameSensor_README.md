# Advanced Flame Detection Algorithm - Implementation Guide

## Overview
This document describes the advanced 5-channel IR Flame Sensor algorithm implemented for the ESP32 Fire Detector system. The algorithm implements sophisticated signal processing to detect real flames while rejecting false positives from ambient interference.

---

## System Architecture

### Four-Stage Detection Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│ STAGE 1: DATA CLEANING (OVERSAMPLING)                          │
│ ─ Read each of 5 analog channels                              │
│ ─ Collect 64 samples per measurement                          │
│ ─ Calculate average in millivolts (mV)                        │
│ ─ Use analogReadMilliVolts() for better linearity             │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│ STAGE 2: DYNAMIC BASELINE (ADAPTIVE THRESHOLDING)              │
│ ─ EMA tracks ambient IR level per channel                     │
│ ─ Formula: Baseline = (α × Current) + ((1-α) × Baseline)     │
│ ─ α = 0.01 (1% learning, 99% inertia)                        │
│ ─ Adapts to slow changes (door opening, sunlight)             │
│ ─ Ignores fast spikes (flames)                               │
│ ─ Calculate deviation: Deviation = Raw - Baseline             │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│ STAGE 3: SPATIAL VOTING FILTER                                  │
│ ─ Global Trigger: ≥4 sensors spiking = AMBIENT_INTERFERENCE   │
│   (e.g., direct sunlight, room-wide IR source)                │
│ ─ Point Source: 1-2 adjacent sensors spiking = POTENTIAL      │
│   (e.g., localized flame source)                              │
│ ─ Non-adjacent or >2 non-adjacent spikes = ignored            │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│ STAGE 4: TEMPORAL VERIFICATION & PEAK DETECTION                 │
│ ─ FLAME_POTENTIAL: Waiting for persistence                     │
│ ─ Requires ≥500ms continuous detection                        │
│ ─ Then: FLAME_DETECTED state triggered                         │
│ ─ Lost spike: Return to IDLE                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Key Configurations

### Sampling & Timing
```cpp
#define OVERSAMPLING_SAMPLES        64          // 64 samples per measurement
#define FLAME_DETECTION_UPDATE_MS   50          // Update baseline every 50ms
```

### Adaptive Baseline (EMA)
```cpp
#define EMA_ALPHA                   0.01f       // 1% learning rate
                                                 // Very low to ignore spikes
```

### Sensitivity & Thresholds
```cpp
#define SENSITIVITY_MARGIN          300         // mV above baseline to trigger spike
```

### Spatial Voting
```cpp
#define AMBIENT_INTERFERENCE_MIN    4           // ≥4 sensors = ambient interference
```

### Temporal Verification
```cpp
#define TEMPORAL_VERIFICATION_MS    500         // Persistence required (ms)
```

---

## Detection States

The flame sensor tracks four distinct states:

| State | Value | Description |
|-------|-------|-------------|
| `FLAME_IDLE` | 0 | No detection activity |
| `FLAME_POTENTIAL` | 1 | Point source detected, waiting for temporal verification |
| `FLAME_DETECTED` | 2 | Real flame confirmed (≥500ms persistence) |
| `FLAME_AMBIENT_INTERFERENCE` | 3 | >3 sensors triggered simultaneously (sunlight, etc.) |

---

## Algorithm Details

### Stage 1: Data Cleaning (Oversampling)

**Purpose**: Mitigate ESP32 ADC noise and jitter

**Method**:
```cpp
uint16_t readChannelMilliVolts(uint8_t channel) {
    long totalMV = 0;
    for (int i = 0; i < 64; i++) {
        totalMV += analogReadMilliVolts(IR_PINS[channel]);
        delayMicroseconds(10);  // Allow ADC settling
    }
    return (uint16_t)(totalMV / 64);  // Average in mV
}
```

**Benefits**:
- Reduces ADC noise by √N factor (√64 = 8x improvement)
- `analogReadMilliVolts()` provides better linearity than 12-bit raw readings
- 640µs sampling window per channel (64 × 10µs)

---

### Stage 2: Dynamic Baseline (Exponential Moving Average)

**Purpose**: Track ambient IR level and detect deviations

**EMA Formula**:
$$\text{Baseline}_{\text{new}} = (\alpha \times \text{Current}) + ((1-\alpha) \times \text{Baseline}_{\text{old}})$$

**With α = 0.01**:
$$\text{Baseline}_{\text{new}} = (0.01 \times \text{Current}) + (0.99 \times \text{Baseline}_{\text{old}})$$

**Characteristics**:
- **Low learning rate** (α = 0.01): 99% inertia filters out spikes
- **Adaptive**: Baseline slowly drifts with ambient light changes
- **Per-channel**: Each sensor tracks its own baseline independently
- **Time constant**: ~7 seconds for 63% adaptation (τ = -1/(ln(1-α) × update_rate))

**Example Timeline** (assuming 50ms updates):
```
Time    Current(mV)    Baseline(mV)   Deviation(mV)   Status
0ms     1500           0              1500            SPIKE!
50ms    1480           15             1465            SPIKE!
100ms   1450           30             1420            SPIKE!
500ms   400            65             335             POTENTIAL (persistent)
550ms   350            72             278             POTENTIAL (still)
1000ms  350            110            240             FLAME_DETECTED! (≥500ms)
2000ms  380            130            250             FLAME_DETECTED
3000ms  320            140            180             FLAME_DETECTED
5000ms  200            145            55              Lost spike → IDLE
```

---

### Stage 3: Spatial Voting Filter

**Channel Layout** (linear array):
```
    ┌───┬───┬───┬───┬───┐
    │ 0 │ 1 │ 2 │ 3 │ 4 │
    └───┴───┴───┴───┴───┘
    └───────────────────┘
    Adjacent pairs
```

**Spike Threshold**:
```cpp
isSpike = (Deviation > SENSITIVITY_MARGIN)  // e.g., >300mV above baseline
```

**Global Trigger (Ambient Interference)**:
```
Active Spikes ≥ 4:  FLAME_AMBIENT_INTERFERENCE
├─ Signature: All/most sensors detecting simultaneously
└─ Typical causes: Direct sunlight, room-wide reflection
```

**Point Source (Real Flame)**:
```
Active Spikes = 1:  FLAME_POTENTIAL ✓
├─ Single sensor showing spike
└─ Typical: Flame directly in line with one sensor

Active Spikes = 2 (adjacent):  FLAME_POTENTIAL ✓
├─ Two neighboring sensors (e.g., sensors 1-2)
└─ Typical: Flame between two sensors

Active Spikes = 2 (non-adjacent):  Rejected ✗
├─ Not physically adjacent
└─ Typical: Two independent reflections/noise

Active Spikes ≥ 3:  FLAME_AMBIENT_INTERFERENCE
└─ See above
```

---

### Stage 4: Temporal Verification

**Purpose**: Require 500ms persistence before declaring flame

**Timeline**:
```
Point Source Detected
        ↓
   FLAME_POTENTIAL
   Start Timer (t₀)
        ↓
    Every Update:
    Is point source still active?
        ├─ YES, t > 500ms → FLAME_DETECTED ✓✓✓
        ├─ YES, t < 500ms → Stay POTENTIAL
        └─ NO → Return to IDLE
```

**Benefits**:
- Filters transient interference
- Real flames typically have 50-500ms flicker frequency
- 500ms = 10 flicker cycles at 20Hz (typical fire flicker)
- False positives from camera flash/strobe require >500ms persistence

---

## Integration into Main Loop

### Initialization (setup())
```cpp
void setup() {
    flameSensor.init();  // Initialize and print configuration
}
```

### Fast Update Loop (100ms)
```cpp
if (now - lastFastCheck >= 100) {
    flameSensor.update();  // Non-blocking update
    bool flameDetected = flameSensor.isFlameDetected();

    // Update danger state based on flame + temperature + smoke
    bool dangerNow = flameDetected ||
                     (temp_value > THRESHOLD_TEMP && smokeDetected);
}
```

### Debug Output (5000ms)
```cpp
if (now - lastDebugPrint >= 5000) {
    flameSensor.printDebugInfo();  // Detailed state table
}
```

---

## Public API

### Core Detection
```cpp
// Get current state
FlameDetectionState getFlameState() const;

// Simple boolean check
bool isFlameDetected() const;
```

### Channel Data Access
```cpp
// Get data for single channel
const IRChannelData* getChannelData(uint8_t channel) const;

// Get all raw values
void getAllRawValues(uint16_t* values) const;

// Get all baselines
void getAllBaselines(float* baselines) const;
```

### Configuration
```cpp
// Adjust sensitivity at runtime
void setSensitivityMargin(uint16_t margin);
uint16_t getSensitivityMargin() const;

// Reset all baselines
void resetBaselines();
```

### Debug Output
```cpp
// Prints formatted table + Serial Plotter data
void printDebugInfo();
```

---

## Serial Output Examples

### Slow Check (2000ms interval)
```
========== SENSOR READINGS ==========
Status: Aman
Flame Detection State: IDLE
Temp: 28.5°C | Asap (MQ2): 12.3 PPM
IR Flame (Raw mV)  :  1250 |  1235 |  1245 |  1255 |  1240
IR Baseline (mV)   : 1240.5| 1235.0| 1245.0| 1250.0| 1240.0
Max IR Value: 1255 mV
Danger Count: 0
====================================
```

### Debug Print (5000ms interval)
```
================ FLAME DETECTOR STATUS ================
State: POTENTIAL
Active Spikes: 2/5
Sensitivity: 300 mV

Channel Data:
CH  |   Raw(mV)  |  Base(mV)  |  Dev(mV)  | Spike
----|------------|------------|-----------|------
 0  |       1550 |     1240.0 |     310.0 | YES
 1  |       1580 |     1235.0 |     345.0 | YES
 2  |       1245 |     1245.0 |       0.0 | NO
 3  |       1250 |     1250.0 |       0.0 | NO
 4  |       1240 |     1240.0 |       0.0 | NO
======================================================

[PLOTTER] 1550	1240.0	1580	1235.0	1245	1245.0	1250	1250.0	1240	1240.0
```

---

## Tuning Guide

### Adjusting Sensitivity

**To increase sensitivity** (catch smaller flames):
```cpp
flameSensor.setSensitivityMargin(200);  // Reduce margin to 200mV
```

**To decrease sensitivity** (reduce false positives):
```cpp
flameSensor.setSensitivityMargin(400);  // Increase margin to 400mV
```

### Adjusting Temporal Verification

Edit in `IRFlameSensor.h`:
```cpp
#define TEMPORAL_VERIFICATION_MS    500  // 500ms persistence
```

**Shorter (200ms)**: Faster response, more false positives
**Longer (1000ms)**: Slower response, fewer false positives

### Adjusting EMA Alpha

Edit in `IRFlameSensor.h`:
```cpp
#define EMA_ALPHA                   0.01f  // Current value
```

**Smaller (0.005)**: Slower baseline adaptation, more inertia
**Larger (0.02)**: Faster baseline adaptation, less inertia

---

## Performance Characteristics

### Timing Analysis

| Operation | Time | Notes |
|-----------|------|-------|
| Single channel 64-sample read | ~640µs | 64 × 10µs delays |
| All 5 channels update | ~3.2ms | 5 × 640µs |
| EMA calculation | <1ms | All 5 channels |
| Spatial voting | <1ms | 5 comparisons |
| Total update() call | ~5ms | Non-blocking |

### Memory Usage

```cpp
struct IRChannelData {      // Per channel
    uint16_t rawMilliVolts;    // 2 bytes
    float baseline;             // 4 bytes
    float deviation;            // 4 bytes
    bool isSpike;              // 1 byte
    unsigned long lastSpikeTime; // 4 bytes
};                           // = 15 bytes × 5 = 75 bytes total

IRFlameSensor instance: ~150 bytes (including overhead)
```

---

## Testing Recommendations

### Unit Tests

1. **Oversampling Test**
   - Verify 64-sample averaging reduces noise
   - Check `analogReadMilliVolts()` linearity

2. **EMA Test**
   - Feed constant value, verify baseline convergence
   - Apply step input, verify time constant (~7 seconds)

3. **Spatial Voting Test**
   - Simulate single sensor spike → POTENTIAL ✓
   - Simulate 4+ sensor spike → AMBIENT_INTERFERENCE ✓
   - Simulate non-adjacent spike pair → Reject ✓

4. **Temporal Test**
   - Maintain spike for 400ms → Stay POTENTIAL
   - Maintain spike for 600ms → FLAME_DETECTED ✓
   - Lose spike at 300ms → Return to IDLE

### Field Tests

1. **Real Flame Response**
   - Place candle at various distances
   - Verify consistent detection at >30cm
   - Record response time

2. **Sunlight Rejection**
   - Point sensors at window (direct sunlight)
   - Verify AMBIENT_INTERFERENCE state
   - Verify no FLAME_DETECTED trigger

3. **Reflection Rejection**
   - Test with mirrors, glass reflections
   - Verify spatial voting correctly identifies ambient sources

4. **Noise Robustness**
   - Test with LED strobe lights
   - Test with IR remotes
   - Test with thermal variations

---

## Code Structure

### File Organization
```
include/IRFlameSensor.h     - Class definition, constants
src/IRFlameSensor.cpp       - Implementation, algorithm
src/main.cpp                - Integration in main loop
```

### Key Methods
- `init()` - Configure and display settings
- `update()` - Main algorithm (call every 50ms+)
- `readChannelMilliVolts()` - 64-sample oversampling
- `updateBaselines()` - EMA calculation
- `evaluateSpatialPattern()` - Voting logic
- `evaluateTemporal()` - Persistence checking
- `printDebugInfo()` - Debug output

---

## Non-Blocking Design

All timing uses `millis()` instead of `delay()`:

```cpp
void update() {
    if (now - lastUpdateTime < FLAME_DETECTION_UPDATE_MS) {
        return;  // Skip, not time yet
    }
    lastUpdateTime = now;

    // Perform update...
}
```

Benefits:
- Main loop never blocks
- Blynk connectivity maintained
- Other sensors can update independently
- Can easily add more detection types

---

## Future Enhancements

1. **Frequency Analysis**
   - FFT on baseline variation to detect flame flicker
   - Flicker signature: 20-30Hz for real fires

2. **Adaptive Sensitivity**
   - Automatic SENSITIVITY_MARGIN adjustment based on ambient noise

3. **Multi-sensor Fusion**
   - Combine with thermal (DHT22) and smoke (MQ2) sensors
   - Bayesian probability of fire

4. **Kalman Filtering**
   - Replace EMA with Kalman filter for state estimation
   - Better noise rejection while maintaining responsiveness

5. **Machine Learning**
   - Train CNN on flame/non-flame spectrograms
   - 99%+ accuracy with pattern recognition

---

## References

- **EMA Time Constant**: $\tau = -\frac{1}{\ln(1-\alpha) \cdot f_s}$ where $f_s$ is sample frequency
- **ADC Noise Reduction**: Averaging reduces noise by factor of $\sqrt{N}$
- **Flame Flicker**: Typical 20-30Hz oscillation in thermal radiation
- **IR Sensor Characteristic**: Responsivity peaks in 800-1000nm region

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-01-04 | Initial implementation with 5-stage detection |
