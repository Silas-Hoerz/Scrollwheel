#pragma once

#include <Arduino.h>
#include <Wire.h>

// Update modes for sensor reading
enum class UpdateMode
{
    kInterruptOnly,
    kPollingOnly,
    kHybrid
};

class Scrollwheel
{
public:
    // Constructor with default I2C address
    explicit Scrollwheel(uint8_t i2c_addr = 0x37);

    // Initializes the library, I2C, and checks configuration
    // Returns true if config is intact, false if a reconfiguration was needed
    bool begin(int interrupt_pin = -1);

    // Configures the behavior of the update() method
    void setUpdateMode(UpdateMode mode, unsigned long poll_interval_ms = 50);

    // Non-blocking update routine. Must be called repeatedly in loop()
    // Returns true if new data was read in this cycle
    bool update();

    // --- Data Retrieval ---
    // These methods return cached values from the last update()
    bool isTouched() const;
    int getSliderAngle(int offset = 0) const;
    bool isButtonPressed() const;

    // --- Dynamic Configuration ---
    void setDebounce(uint8_t cycles);
    void setScanPeriod(uint8_t value);

    // Advanced users: Overwrite specific config bytes before applying
    void setConfigByte(uint8_t offset, uint8_t value);

    // Writes the cached 128-byte configuration to the sensor's non-volatile memory
    void applyConfig();

    // --- Debugging ---
    void printDiagnostics();

private:
    uint8_t i2c_addr_;
    int int_pin_;
    bool use_interrupt_;

    UpdateMode update_mode_;
    unsigned long poll_interval_;
    unsigned long last_poll_time_;

    // Cached sensor states
    uint8_t last_slider_pos_;
    bool is_button_pressed_;

    // 128 Byte local configuration copy
    uint8_t config_data_[128];

    // Internal I2C helpers
    uint16_t calculateCrc(const uint8_t *data, uint8_t len) const;
    void writeCommand(uint8_t cmd);
    uint8_t readReg8(uint8_t reg);
    uint16_t readReg16(uint8_t reg);
};