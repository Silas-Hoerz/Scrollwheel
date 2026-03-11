#include "Scrollwheel.h"

// --- Register Map Constants ---
constexpr uint8_t kRegSensorEn = 0x00;
constexpr uint8_t kRegConfigCrc = 0x7E;
constexpr uint8_t kRegCommand = 0x86;
constexpr uint8_t kRegSysStatus = 0x8A;
constexpr uint8_t kRegFamilyId = 0x8F;
constexpr uint8_t kRegDeviceId = 0x90;
constexpr uint8_t kRegChipRev = 0x92;
constexpr uint8_t kRegVddShort = 0x9A;
constexpr uint8_t kRegGndShort = 0x9C;
constexpr uint8_t kRegButtonStat = 0xAA;
constexpr uint8_t kRegSliderPos = 0xB0;

constexpr uint8_t kCmdSaveToFlash = 0x02;
constexpr uint8_t kCmdSoftReset = 0xFF;

// Button CS11 bitmask (Bit 11 in the 16-bit status register)
constexpr uint16_t kButtonCs11Mask = (1 << 11);

// Global ISR flag (used if interrupt pin is attached)
volatile bool g_cy8_irq_flag = false;

void scrollwheelIsr()
{
    g_cy8_irq_flag = true;
}

// Default 128-byte configuration for the Scrollwheel PCB
const uint8_t kDefaultConfig[128] = {
    0xC0u, 0x0Fu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu,
    0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x80u,
    0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x03u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x05u, 0x00u, 0x00u, 0x02u, 0x00u, 0x02u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x01u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x24u, 0x03u, 0x01u, 0x59u,
    0x00u, 0x37u, 0x01u, 0x00u, 0x00u, 0x0Au, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x25u, 0xFEu, 0x80u, 0x00u, 0x00u, 0x00u, 0x03u,
    0x2Du, 0x80u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x90u, 0x5Du};

Scrollwheel::Scrollwheel(uint8_t i2c_addr)
    : i2c_addr_(i2c_addr),
      int_pin_(-1),
      use_interrupt_(false),
      update_mode_(UpdateMode::kPollingOnly),
      poll_interval_(50),
      last_poll_time_(0),
      last_slider_pos_(255),
      is_button_pressed_(false)
{
}

bool Scrollwheel::begin(int interrupt_pin)
{
    Wire.begin();

    if (interrupt_pin >= 0)
    {
        int_pin_ = interrupt_pin;
        use_interrupt_ = true;
        update_mode_ = UpdateMode::kHybrid; // Best-practice default when pin is connected
        pinMode(int_pin_, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(int_pin_), scrollwheelIsr, FALLING);
    }
    else
    {
        use_interrupt_ = false;
        update_mode_ = UpdateMode::kPollingOnly;
    }

    memcpy(config_data_, kDefaultConfig, 128);

    // Verify configuration integrity
    uint16_t current_crc = readReg16(kRegConfigCrc);
    uint16_t target_crc = config_data_[126] | (config_data_[127] << 8);

    if (current_crc != target_crc)
    {
        applyConfig();
        return false; // Indicates configuration had to be rewritten
    }
    return true; // Config is intact
}

void Scrollwheel::setUpdateMode(UpdateMode mode, unsigned long poll_interval_ms)
{
    update_mode_ = mode;
    poll_interval_ = poll_interval_ms;
}

bool Scrollwheel::update()
{
    bool should_read = false;
    unsigned long current_millis = millis();

    // 1. Check for hardware interrupt (Hybrid or Interrupt Only mode)
    if (update_mode_ == UpdateMode::kInterruptOnly || update_mode_ == UpdateMode::kHybrid)
    {
        if (use_interrupt_ && g_cy8_irq_flag)
        {
            should_read = true;
            g_cy8_irq_flag = false;
        }
    }

    // 2. Check for cyclic polling (Hybrid or Polling Only mode)
    if (update_mode_ == UpdateMode::kPollingOnly || update_mode_ == UpdateMode::kHybrid)
    {
        if (current_millis - last_poll_time_ >= poll_interval_)
        {
            should_read = true;
            g_cy8_irq_flag = false; // Clear any hanging interrupts
        }
    }

    if (!should_read)
        return false;

    // Reset timer as we are reading now
    last_poll_time_ = current_millis;

    // Fetch data from I2C. This also clears the interrupt line on the chip.
    last_slider_pos_ = readReg8(kRegSliderPos);
    uint16_t btn_stat = readReg16(kRegButtonStat);

    is_button_pressed_ = (btn_stat & kButtonCs11Mask) != 0;

    return true; // New data processed
}

int Scrollwheel::getSliderAngle(int offset) const
{
    if (last_slider_pos_ == 255)
        return -1; // No touch detected

    // Cast to uint32_t prevents integer overflow for high positional values
    int angle = ((uint32_t)last_slider_pos_ * 360) / 254;

    angle = (angle + offset) % 360;
    if (angle < 0)
        angle += 360;

    return angle;
}

bool Scrollwheel::isTouched() const
{
    return last_slider_pos_ != 255;
}

bool Scrollwheel::isButtonPressed() const
{
    return is_button_pressed_;
}

void Scrollwheel::setDebounce(uint8_t cycles)
{
    config_data_[28] = cycles;
}

void Scrollwheel::setScanPeriod(uint8_t value)
{
    config_data_[82] = value;
}

void Scrollwheel::setConfigByte(uint8_t offset, uint8_t value)
{
    if (offset < 126)
    { // Protect CRC bytes
        config_data_[offset] = value;
    }
}

void Scrollwheel::applyConfig()
{
    // Recalculate and update CRC in config array
    uint16_t new_crc = calculateCrc(config_data_, 126);
    config_data_[126] = new_crc & 0xFF;
    config_data_[127] = (new_crc >> 8) & 0xFF;

    // Write configuration in 16-byte chunks
    for (uint8_t offset = 0; offset < 128; offset += 16)
    {
        Wire.beginTransmission(i2c_addr_);
        Wire.write(offset);
        for (uint8_t i = 0; i < 16; i++)
        {
            Wire.write(config_data_[offset + i]);
        }
        Wire.endTransmission();
    }

    // Save to non-volatile memory and perform a soft reset
    writeCommand(kCmdSaveToFlash);
    delay(250);
    writeCommand(kCmdSoftReset);
    delay(100);
}

void Scrollwheel::printDiagnostics()
{
    Serial.println("\n--- SCROLLWHEEL METADATA ---");
    Serial.print("Device ID:   0x");
    Serial.println(readReg16(kRegDeviceId), HEX);
    Serial.print("Family ID:   0x");
    Serial.println(readReg8(kRegFamilyId), HEX);
    Serial.print("Chip Rev:    ");
    Serial.println(readReg8(kRegChipRev));
    Serial.print("Sys Status:  0x");
    Serial.println(readReg8(kRegSysStatus), HEX);
    Serial.print("Config CRC:  0x");
    Serial.println(readReg16(kRegConfigCrc), HEX);

    uint16_t gnd_short = readReg16(kRegGndShort);
    uint16_t vdd_short = readReg16(kRegVddShort);

    if (gnd_short)
    {
        Serial.print("ERROR GND-Short at: 0b");
        Serial.println(gnd_short, BIN);
    }
    if (vdd_short)
    {
        Serial.print("ERROR VDD-Short at: 0b");
        Serial.println(vdd_short, BIN);
    }
    Serial.println("----------------------------");
}

uint16_t Scrollwheel::calculateCrc(const uint8_t *data, uint8_t len) const
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= (data[i] << 8);
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

void Scrollwheel::writeCommand(uint8_t cmd)
{
    Wire.beginTransmission(i2c_addr_);
    Wire.write(kRegCommand);
    Wire.write(cmd);
    Wire.endTransmission();
}

uint8_t Scrollwheel::readReg8(uint8_t reg)
{
    Wire.beginTransmission(i2c_addr_);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)i2c_addr_, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

uint16_t Scrollwheel::readReg16(uint8_t reg)
{
    Wire.beginTransmission(i2c_addr_);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)i2c_addr_, (uint8_t)2);
    if (Wire.available() >= 2)
    {
        uint8_t lsb = Wire.read();
        uint8_t msb = Wire.read();
        return (msb << 8) | lsb;
    }
    return 0;
}