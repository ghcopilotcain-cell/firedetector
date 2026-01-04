#include "IRFlameSensor.h"
#include "Config.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================
IRFlameSensor::IRFlameSensor()
    : currentState(FLAME_IDLE),
      potentialFlameStartTime(0),
      sensitivityMargin(SENSITIVITY_MARGIN),
      lastUpdateTime(0) {
    // Initialize all channels with zero baseline
    for (int i = 0; i < IR_NUM_CHANNELS; i++) {
        channels[i].baseline = 0.0f;
        channels[i].rawMilliVolts = 0;
        channels[i].deviation = 0.0f;
        channels[i].isSpike = false;
        channels[i].lastSpikeTime = 0;
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================
void IRFlameSensor::init() {
    Serial.println("[IRFlameSensor] Initializing 5-channel advanced flame detector...");
    Serial.printf("[IRFlameSensor] Oversampling: %d samples per read\n", OVERSAMPLING_SAMPLES);
    Serial.printf("[IRFlameSensor] EMA Alpha: %.3f\n", EMA_ALPHA);
    Serial.printf("[IRFlameSensor] Sensitivity Margin: %d mV\n", sensitivityMargin);
    Serial.printf("[IRFlameSensor] Temporal Verification: %d ms\n", TEMPORAL_VERIFICATION_MS);
    Serial.printf("[IRFlameSensor] Ambient Interference Threshold: %d sensors\n", AMBIENT_INTERFERENCE_MIN);
    Serial.println("[IRFlameSensor] Initialization complete!");
}

// ============================================================================
// READ CHANNEL WITH OVERSAMPLING
// Returns analog value in millivolts (0-3300 mV for ESP32)
// ============================================================================
uint16_t IRFlameSensor::readChannelMilliVolts(uint8_t channel) {
    if (channel >= IR_NUM_CHANNELS) return 0;

    // Accumulate OVERSAMPLING_SAMPLES readings
    long totalMV = 0;
    for (int i = 0; i < OVERSAMPLING_SAMPLES; i++) {
        // analogReadMilliVolts() provides better linearity than analogRead()
        totalMV += analogReadMilliVolts(IR_PINS[channel]);
        // Very small delay to allow ADC capacitor to settle
        delayMicroseconds(10);
    }

    // Return average in millivolts
    return (uint16_t)(totalMV / OVERSAMPLING_SAMPLES);
}

// ============================================================================
// UPDATE BASELINES (EMA)
// Called regularly to adapt baseline to slow environmental changes
// ============================================================================
void IRFlameSensor::updateBaselines() {
    for (int i = 0; i < IR_NUM_CHANNELS; i++) {
        // EMA Formula: Baseline = (α × Current) + ((1 - α) × Baseline)
        // α = 0.01 means 99% inertia (ignores spikes, tracks slow changes)
        channels[i].baseline = (EMA_ALPHA * channels[i].rawMilliVolts) +
                               ((1.0f - EMA_ALPHA) * channels[i].baseline);

        // Calculate deviation from baseline
        channels[i].deviation = channels[i].rawMilliVolts - channels[i].baseline;

        // Determine if this channel shows a spike
        channels[i].isSpike = channels[i].deviation > sensitivityMargin;
    }
}

// ============================================================================
// SPATIAL VOTING: Count active spikes
// ============================================================================
uint8_t IRFlameSensor::countActiveSpikes() const {
    uint8_t count = 0;
    for (int i = 0; i < IR_NUM_CHANNELS; i++) {
        if (channels[i].isSpike) count++;
    }
    return count;
}

// ============================================================================
// SPATIAL VOTING: Check if two channels are adjacent
// Channels: 0-1-2-3-4 (linear array)
// ============================================================================
bool IRFlameSensor::areChannelsAdjacent(uint8_t ch1, uint8_t ch2) const {
    return (ch1 < IR_NUM_CHANNELS && ch2 < IR_NUM_CHANNELS &&
            abs((int)ch1 - (int)ch2) == 1);
}

// ============================================================================
// SPATIAL VOTING: Point Source Detection
// True if only 1 or 2 adjacent sensors show spike
// ============================================================================
bool IRFlameSensor::isPointSource() const {
    uint8_t spikeCount = countActiveSpikes();

    // Single sensor spike is a point source
    if (spikeCount == 1) return true;

    // Two adjacent sensors spike = point source (localized flame)
    if (spikeCount == 2) {
        for (int i = 0; i < IR_NUM_CHANNELS - 1; i++) {
            if (channels[i].isSpike && channels[i + 1].isSpike) {
                return true;  // Found adjacent pair
            }
        }
    }

    // Multiple non-adjacent spikes or >2 spikes = not point source
    return false;
}

// ============================================================================
// SPATIAL VOTING: Ambient Interference Detection
// True if >3 sensors (4 or 5) show spike simultaneously
// Typical signature of direct sunlight or room-wide IR source
// ============================================================================
bool IRFlameSensor::isAmbientInterference() const {
    uint8_t spikeCount = countActiveSpikes();
    return spikeCount >= AMBIENT_INTERFERENCE_MIN;
}

// ============================================================================
// SPATIAL PATTERN EVALUATION
// Categorize detection type and set state
// ============================================================================
void IRFlameSensor::evaluateSpatialPattern() {
    uint8_t spikeCount = countActiveSpikes();

    if (spikeCount == 0) {
        // No spikes detected
        if (currentState == FLAME_POTENTIAL) {
            // Lost spike, return to idle
            currentState = FLAME_IDLE;
            potentialFlameStartTime = 0;
        }
    } else if (isAmbientInterference()) {
        // >3 sensors spiking = ambient (sunlight, room reflection, etc.)
        currentState = FLAME_AMBIENT_INTERFERENCE;
        potentialFlameStartTime = 0;
    } else if (isPointSource()) {
        // 1-2 adjacent sensors = potential flame
        currentState = FLAME_POTENTIAL;

        // Record time of first potential flame detection
        if (potentialFlameStartTime == 0) {
            potentialFlameStartTime = millis();
        }
    }
}

// ============================================================================
// TEMPORAL VERIFICATION
// Require persistence for 500ms before declaring FLAME_DETECTED
// ============================================================================
void IRFlameSensor::evaluateTemporal() {
    if (currentState == FLAME_POTENTIAL) {
        unsigned long persistenceTime = millis() - potentialFlameStartTime;

        if (persistenceTime >= TEMPORAL_VERIFICATION_MS) {
            // Potential flame has persisted long enough
            currentState = FLAME_DETECTED;
            Serial.printf("[IRFlameSensor] FLAME DETECTED after %lu ms persistence!\n", persistenceTime);
        }
    }
}

// ============================================================================
// MAIN UPDATE FUNCTION
// Call this regularly (every 50ms or more frequent)
// Non-blocking operation
// ============================================================================
void IRFlameSensor::update() {
    unsigned long now = millis();

    // Check if it's time for update (FLAME_DETECTION_UPDATE_MS interval)
    if (now - lastUpdateTime < FLAME_DETECTION_UPDATE_MS) {
        return;  // Not yet time, skip this call
    }
    lastUpdateTime = now;

    // -------- STEP 1: DATA CLEANING (OVERSAMPLING) --------
    // Read all 5 channels with 64-sample averaging
    for (int i = 0; i < IR_NUM_CHANNELS; i++) {
        channels[i].rawMilliVolts = readChannelMilliVolts(i);
    }

    // -------- STEP 2: DYNAMIC BASELINE (EMA) --------
    updateBaselines();

    // -------- STEP 3: SPATIAL VOTING --------
    evaluateSpatialPattern();

    // -------- STEP 4: TEMPORAL VERIFICATION --------
    evaluateTemporal();
}

// ============================================================================
// PUBLIC GETTERS
// ============================================================================
FlameDetectionState IRFlameSensor::getFlameState() const {
    return currentState;
}

bool IRFlameSensor::isFlameDetected() const {
    return (currentState == FLAME_DETECTED);
}

const IRChannelData* IRFlameSensor::getChannelData(uint8_t channel) const {
    if (channel >= IR_NUM_CHANNELS) return nullptr;
    return &channels[channel];
}

void IRFlameSensor::getAllRawValues(uint16_t* values) const {
    for (int i = 0; i < IR_NUM_CHANNELS; i++) {
        values[i] = channels[i].rawMilliVolts;
    }
}

void IRFlameSensor::getAllBaselines(float* baselines) const {
    for (int i = 0; i < IR_NUM_CHANNELS; i++) {
        baselines[i] = channels[i].baseline;
    }
}

void IRFlameSensor::setSensitivityMargin(uint16_t margin) {
    sensitivityMargin = margin;
    Serial.printf("[IRFlameSensor] Sensitivity margin updated to %d mV\n", margin);
}

uint16_t IRFlameSensor::getSensitivityMargin() const {
    return sensitivityMargin;
}

void IRFlameSensor::resetBaselines() {
    Serial.println("[IRFlameSensor] Resetting all baselines...");
    for (int i = 0; i < IR_NUM_CHANNELS; i++) {
        channels[i].baseline = 0.0f;
        channels[i].isSpike = false;
        channels[i].lastSpikeTime = 0;
    }
    currentState = FLAME_IDLE;
    potentialFlameStartTime = 0;
}

// ============================================================================
// DEBUG OUTPUT
// Optimized for Serial Plotter visualization
// Format: Raw0,Baseline0,Raw1,Baseline1,...
// ============================================================================
void IRFlameSensor::printDebugInfo() {
    Serial.println("\n================ FLAME DETECTOR STATUS ================");

    // State info
    const char* stateStr;
    switch (currentState) {
        case FLAME_IDLE:
            stateStr = "IDLE";
            break;
        case FLAME_POTENTIAL:
            stateStr = "POTENTIAL";
            break;
        case FLAME_DETECTED:
            stateStr = "DETECTED";
            break;
        case FLAME_AMBIENT_INTERFERENCE:
            stateStr = "AMBIENT_INTERFERENCE";
            break;
        default:
            stateStr = "UNKNOWN";
    }

    Serial.printf("State: %s\n", stateStr);
    Serial.printf("Active Spikes: %d/5\n", countActiveSpikes());
    Serial.printf("Sensitivity: %d mV\n", sensitivityMargin);
    Serial.println("\nChannel Data:");
    Serial.println("CH  |   Raw(mV)  |  Base(mV)  |  Dev(mV)  | Spike");
    Serial.println("----|------------|------------|-----------|------");

    for (int i = 0; i < IR_NUM_CHANNELS; i++) {
        Serial.printf(" %d  | %10d | %10.1f | %9.1f | %s\n",
                      i,
                      channels[i].rawMilliVolts,
                      channels[i].baseline,
                      channels[i].deviation,
                      channels[i].isSpike ? "YES" : "NO");
    }

    Serial.println("======================================================\n");

    // CSV format for Serial Plotter (one line, tab or comma separated)
    Serial.print("[PLOTTER] ");
    for (int i = 0; i < IR_NUM_CHANNELS; i++) {
        if (i > 0) Serial.print("\t");
        Serial.printf("%.0f\t%.0f", channels[i].rawMilliVolts * 1.0f, channels[i].baseline);
    }
    Serial.println();
}
