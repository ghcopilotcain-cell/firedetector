#ifndef IR_FLAME_SENSOR_H
#define IR_FLAME_SENSOR_H

#include <Arduino.h>

// ============================================================================
// ADVANCED FLAME DETECTION ALGORITHM
// 5-Channel IR Flame Sensor with:
// - Oversampling (64 samples)
// - Dynamic Baseline (EMA)
// - Spatial Voting Filter
// - Peak Detection & Scoring
// ============================================================================

// Configuration Constants
#define IR_NUM_CHANNELS             5
#define OVERSAMPLING_SAMPLES        64          // 64 samples per measurement
#define EMA_ALPHA                   0.01f       // EMA coefficient (1%)
#define SENSITIVITY_MARGIN          300         // mV above baseline to detect
#define AMBIENT_INTERFERENCE_MIN    4           // >3 sensors trigger = ambient
#define TEMPORAL_VERIFICATION_MS    500         // 500ms persistence required
#define FLAME_DETECTION_UPDATE_MS   50          // Update baseline every 50ms

// Flame Detection States
enum FlameDetectionState {
    FLAME_IDLE,
    FLAME_POTENTIAL,
    FLAME_DETECTED,
    FLAME_AMBIENT_INTERFERENCE
};

// Structure to hold per-channel sensor data
struct IRChannelData {
    uint16_t rawMilliVolts;         // Current raw reading in mV
    float baseline;                  // Dynamic baseline (EMA)
    float deviation;                 // Current - Baseline
    bool isSpike;                    // True if exceeds threshold
    unsigned long lastSpikeTime;     // Timestamp of last spike
};

// Main Flame Sensor Class
class IRFlameSensor {
public:
    // Constructor
    IRFlameSensor();

    // Initialization
    void init();

    // Non-blocking update - call this regularly (every 50ms or more frequent)
    void update();

    // Get current flame detection state
    FlameDetectionState getFlameState() const;

    // Get flame detection boolean
    bool isFlameDetected() const;

    // Get data for a specific channel (0-4)
    const IRChannelData* getChannelData(uint8_t channel) const;

    // Get all channel raw values as array
    void getAllRawValues(uint16_t* values) const;

    // Get all channel baselines as array
    void getAllBaselines(float* baselines) const;

    // Print debug info to Serial (for Serial Plotter compatibility)
    void printDebugInfo();

    // Adjust sensitivity globally
    void setSensitivityMargin(uint16_t margin);
    uint16_t getSensitivityMargin() const;

    // Reset all baselines
    void resetBaselines();

private:
    // Channel data storage
    IRChannelData channels[IR_NUM_CHANNELS];

    // Detection state tracking
    FlameDetectionState currentState;
    unsigned long potentialFlameStartTime;

    // Configuration
    uint16_t sensitivityMargin;

    // Timing
    unsigned long lastUpdateTime;

    // Private methods
    uint16_t readChannelMilliVolts(uint8_t channel);
    void updateBaselines();
    void evaluateSpatialPattern();
    void evaluateTemporal();
    uint8_t countActiveSpikes() const;
    bool areChannelsAdjacent(uint8_t ch1, uint8_t ch2) const;
    bool isPointSource() const;
    bool isAmbientInterference() const;
};

#endif // IR_FLAME_SENSOR_H
