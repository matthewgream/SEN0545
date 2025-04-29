
// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#pragma once

#include <stdint.h>
#include <optional>

// #define DEBUG_SEN0545
#ifdef DEBUG_SEN0545
#define DEBUG_SEN0545_PRINTF Serial.printf
#else
#define DEBUG_SEN0545_PRINTF(...)                                                                                                                                                                                                                                                                                              \
    do {                                                                                                                                                                                                                                                                                                                       \
    } while (0)
#endif

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

// ZLG RS200 Rainfall Detection Module - https://www.zlg.cn/tm/tm/product/id/259.html

static inline constexpr uint8_t FrameHeader = 0x3A;

enum class FrameProperty : uint8_t { Read = 0, Write = 1 };

enum class FrameType : uint8_t {
    FirmwareVersion = 0,
    RainfallStatus = 1,
    SystemStatus = 2,
    OpticalSystem = 3,
    RealtimeRainfall = 4,
    OutputFrequency = 5,
    ThresholdV1 = 6,
    ThresholdV2 = 7,
    ThresholdV3 = 8,
    ThresholdS1 = 9,
    ThresholdS2 = 10,
    ThresholdS3 = 11,
    ThresholdN1 = 12,
    ThresholdN2 = 13,
    ThresholdN3 = 14,
    AmbientLight = 15,
    Temperature = 16,
    SleepMode = 17
};

static inline constexpr uint8_t FrameFCS = 0x00;

// Type 0: Firmware Version - two 8-bit values
struct __attribute__ ((__packed__)) frame_data_firmware_version_t {
    uint8_t major;
    uint8_t backup;
};

// Type 1: Rainfall Status - enumerated status
struct __attribute__ ((__packed__)) frame_data_rainfall_status_t {
    enum class status_t : uint16_t { NoRain = 0, LightRain = 1, ModerateRain = 2, HeavyRain = 3 } status;
    static constexpr const char *toString (const status_t &status) {
        switch (status) {
        case status_t::NoRain :
            return "none";
        case status_t::LightRain :
            return "light";
        case status_t::ModerateRain :
            return "moderate";
        case status_t::HeavyRain :
            return "heavy";
        default :
            return "unknown";
        }
    }
};

// Type 2: System Status - enumerated status
struct __attribute__ ((__packed__)) frame_data_system_status_t {
    enum class status_t : uint16_t { Normal = 0, CommunicationError = 1, LEDADamaged = 2, LEDBDamaged = 3, CalibrationNotGood = 4, ConfigFailure = 5, SerialError = 6, LowVoltage = 7 } status;
    static constexpr const char *toString (const status_t &status) {
        switch (status) {
        case status_t::Normal :
            return "normal";
        case status_t::CommunicationError :
            return "communication-error";
        case status_t::LEDADamaged :
            return "leda-damaged";
        case status_t::LEDBDamaged :
            return "ledb-damaged";
        case status_t::CalibrationNotGood :
            return "calibration-not-good";
        case status_t::ConfigFailure :
            return "config-failure";
        case status_t::SerialError :
            return "serial-error";
        case status_t::LowVoltage :
            return "low-voltage";
        default :
            return "unknown";
        }
    }
};

// Type 3: Optical System - command enum
struct __attribute__ ((__packed__)) frame_data_optical_system_t {
    enum class command_t : uint16_t { Calibrate = 0, SendValue = 1 } command;
    static constexpr const char *toString (const command_t &command) {
        switch (command) {
        case command_t::Calibrate :
            return "calibrate";
        case command_t::SendValue :
            return "send-value";
        default :
            return "unknown";
        }
    }
};

// Type 4: Real-time Rainfall Mode - command enum
// Type 15: Ambient Light Mode - command enum
// Type 17: Sleep Mode - command enum
struct __attribute__ ((__packed__)) frame_data_enterorexit_t {
    enum class command_t : uint16_t { Exit = 0, Enter = 1 } command;
    static constexpr const char *toString (const command_t &command) {
        switch (command) {
        case command_t::Exit :
            return "exit";
        case command_t::Enter :
            return "enter";
        default :
            return "unknown";
        }
    }
};

// Type 5: Output Frequency - integer range 0-9: 0=disabled, 1-9=frequency in 50ms units
struct __attribute__ ((__packed__)) frame_data_output_frequency_t {
    enum class frequency_t : uint16_t { Disabled = 0, Minimum = 1, Maximum = 9 } frequency;
};

// Types 6-8: V Thresholds - 16-bit range 0-65535
struct __attribute__ ((__packed__)) frame_data_threshold_v_t {
    uint16_t threshold;
};

// Types 9-11: S Thresholds - 16-bit range 0-65535
struct __attribute__ ((__packed__)) frame_data_threshold_s_t {
    uint16_t threshold;
};

// Types 12-14: N Thresholds - range 1-10
struct __attribute__ ((__packed__)) frame_data_threshold_n_t {
    enum class count_t : uint16_t { Minimum = 1, Maximum = 10 } count;
};

// Type 16: Temperature Reading - command value: always 0 for reading temperature
struct __attribute__ ((__packed__)) frame_data_temperature_t {
    uint16_t value;
    static inline constexpr float toFloat (uint16_t temperature) {
        return (static_cast<float> (temperature) - 605.36) / -1.5596;    // Figure 2.1. ZLG RS200
    }
};

// Union of all frame data types
union __attribute__ ((__packed__)) frame_data_t {
    frame_data_firmware_version_t firmware_version;
    frame_data_rainfall_status_t rainfall_status;
    frame_data_system_status_t system_status;
    frame_data_optical_system_t optical_system;
    frame_data_enterorexit_t realtime_rainfall;
    frame_data_output_frequency_t output_frequency;
    frame_data_threshold_v_t threshold_v;
    frame_data_threshold_s_t threshold_s;
    frame_data_threshold_n_t threshold_n;
    frame_data_enterorexit_t ambient_light;
    frame_data_temperature_t temperature;
    frame_data_enterorexit_t sleep_mode;
    uint8_t __bytes [2];
};

struct __attribute__ ((__packed__)) frame_command_t {
    uint8_t flags;
    frame_data_t data;
};

static inline constexpr FrameType FRAME_FLAGS_DECODE_TYPE (const uint8_t flags) { return static_cast<FrameType> (flags & 0x7FU); }
static inline constexpr uint8_t FRAME_FLAGS_ENCODE_PROPERTY (const FrameProperty property) { return (static_cast<uint8_t> (property) << 7) & 0x80U; }
static inline constexpr uint8_t FRAME_FLAGS_ENCODE_TYPE (const FrameType type) { return static_cast<uint8_t> (type) & 0x7FU; }
static inline constexpr uint8_t FRAME_FLAGS_ENCODE (const FrameProperty property, const FrameType type) { return FRAME_FLAGS_ENCODE_PROPERTY (property) | FRAME_FLAGS_ENCODE_TYPE (type); }
// little endian
template <typename T>
static inline constexpr frame_data_t FRAME_DATA_ENCODE (const T &type) {
    return { .__bytes = { static_cast<uint8_t> ((reinterpret_cast<const uint16_t &> (type) >> 0) & 0xFF), static_cast<uint8_t> ((reinterpret_cast<const uint16_t &> (type) >> 8) & 0xFF) } };
}
static inline constexpr uint16_t FRAME_DATA_DECODE (const frame_data_t &data) { return static_cast<uint16_t> (data.__bytes [1] << 8) | static_cast<uint16_t> (data.__bytes [0]); }

struct __attribute__ ((__packed__)) frame_t {
    const uint8_t header = FrameHeader;
    frame_command_t data;
    uint8_t fcs = FrameFCS;
};

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

template <uint8_t poly>
static inline constexpr uint32_t __crc8_bit (uint32_t crc) {
    return (crc << 1) ^ ((crc & 0x80) ? poly : 0);
}
template <uint8_t poly>
static inline constexpr uint32_t __crc8_byte (uint32_t crc) {
    for (int i = 0; i < 8; i++)
        crc = __crc8_bit<poly> (crc);
    return crc;
}
template <uint8_t poly, size_t SIZE, size_t INDEX = 0>
static inline constexpr uint32_t __crc8_bytes (const uint8_t *data, uint32_t crc = 0xFF) {
    static_assert (SIZE <= 16, "crc8 limited to 16 bytes for performance reasons");
    if constexpr (INDEX < SIZE)
        return __crc8_bytes<poly, SIZE, INDEX + 1> (data, __crc8_byte<poly> (crc ^ data [INDEX]));
    return crc;
}
template <uint8_t poly, size_t SIZE>
static inline constexpr uint8_t crc8 (const uint8_t *data) {
    return __crc8_bytes<poly, SIZE> (data) & 0xFF;
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

class frame_transceiver_t {
public:
    using tx_function_t = std::function<size_t (const uint8_t *, size_t)>;
    using rx_function_t = std::function<size_t (uint8_t *, size_t)>;
    using bk_function_t = std::function<void (void)>;

    frame_transceiver_t (const tx_function_t tx, const rx_function_t rx, const bk_function_t bk) :
        tx_function (tx),
        rx_function (rx),
        bk_function (bk) { }

    bool transmit (const frame_command_t &command) const {
        const frame_t frame { .data = command, .fcs = crc8<0x31, sizeof (frame_command_t)> (reinterpret_cast<const uint8_t *> (&command)) };
        const uint8_t *frame_raw = reinterpret_cast<const uint8_t *> (&frame);
        const bool result = tx_function (frame_raw, sizeof (frame_t)) == sizeof (frame_t);
        DEBUG_SEN0545_PRINTF ("SEN0545::TX: %02X %02X %02X %02X %02X [%s]\n", frame_raw [0], frame_raw [1], frame_raw [2], frame_raw [3], frame_raw [4], result ? "true" : "false");
        return result;
    }
    template <bool blocking = true, unsigned long timeout = 100>
    std::optional<frame_command_t> receive () const {
        frame_t frame;
        uint8_t *frame_raw = reinterpret_cast<uint8_t *> (&frame);
        size_t bytesRead = 0;
        const unsigned long startTime = millis ();
        while (bytesRead < sizeof (frame_t)) {
            if constexpr (! blocking)
                if ((millis () - startTime) >= timeout)
                    break;
            if (bytesRead == 0) {
                if (rx_function (&frame_raw [0], 1) != 1) {
                    if constexpr (blocking) {
                        if (bk_function)
                            bk_function ();
                    } else
                        return std::nullopt;
                    continue;
                }
                if (frame.header != FrameHeader)
                    continue;
                bytesRead++;
            } else {
                const size_t result = rx_function (frame_raw + bytesRead, sizeof (frame_t) - bytesRead);
                if (result == 0) {
                    if constexpr (blocking) {
                        if (bk_function)
                            bk_function ();
                    } else
                        return std::nullopt;
                    continue;
                }
                bytesRead += result;
            }
        }
        if (bytesRead != sizeof (frame_t)) {
            DEBUG_SEN0545_PRINTF ("SEN0545::RX: short read, only %d bytes\n", bytesRead);
            return std::nullopt;
        }
        DEBUG_SEN0545_PRINTF ("SEN0545::RX: %02X %02X %02X %02X %02X\n", frame_raw [0], frame_raw [1], frame_raw [2], frame_raw [3], frame_raw [4]);
        const uint8_t fcs = crc8<0x31, sizeof (frame_command_t)> (reinterpret_cast<uint8_t *> (&frame.data));
        if (fcs != frame.fcs) {
            DEBUG_SEN0545_PRINTF ("SEN0545::RX: mismatched fcs, received 0x%02X, calculated 0x%02X\n", frame.fcs, fcs);
            return std::nullopt;
        }
        return frame.data;
    }

private:
    tx_function_t tx_function;
    rx_function_t rx_function;
    bk_function_t bk_function;
};

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

class RainSensor {
public:
    RainSensor (frame_transceiver_t::tx_function_t tx, frame_transceiver_t::rx_function_t rx, const frame_transceiver_t::bk_function_t bk) :
        transceiver (tx, rx, bk) { }

    template <typename T>
    bool read (const FrameType &type, T &value) const {
        if (! transceiver.transmit (frame_command_t { .flags = FRAME_FLAGS_ENCODE (FrameProperty::Read, type) }))
            return false;
        const auto response = transceiver.receive ();
        if (! response || FRAME_FLAGS_DECODE_TYPE (response->flags) != type)
            return false;
        value = static_cast<T> (FRAME_DATA_DECODE (response->data));
        return true;
    }
    template <typename T>
    bool write (const FrameType &type, const T &value) const {
        return transceiver.transmit (frame_command_t { .flags = FRAME_FLAGS_ENCODE (FrameProperty::Write, type), .data = FRAME_DATA_ENCODE (value) });
    }

    bool _setEnterOrExitMode (const FrameType &type, const bool enabled) const { return write (type, frame_data_enterorexit_t { .command = enabled ? frame_data_enterorexit_t::command_t::Enter : frame_data_enterorexit_t::command_t::Exit }); }

    //

    using FirmwareVersion = frame_data_firmware_version_t;
    bool getFirmwareVersion (FirmwareVersion &version) const { return read (FrameType::FirmwareVersion, version); }
    using SystemStatus = frame_data_system_status_t::status_t;
    bool getSystemStatus (frame_data_system_status_t::status_t &status) const { return read (FrameType::SystemStatus, status); }
    using RainfallStatus = frame_data_rainfall_status_t::status_t;
    bool getRainfallStatus (frame_data_rainfall_status_t::status_t &status) const { return read (FrameType::RainfallStatus, status); }
    bool performCalibration () const { return write (FrameType::OpticalSystem, frame_data_optical_system_t { .command = frame_data_optical_system_t::command_t::Calibrate }); }
    bool getOpticalCalibrationValue () const { return write (FrameType::OpticalSystem, frame_data_optical_system_t { .command = frame_data_optical_system_t::command_t::SendValue }); }
    bool setRealtimeMode (const bool enabled) const { return _setEnterOrExitMode (FrameType::RealtimeRainfall, enabled); }
    using FrequencyType = uint16_t;
    bool setOutputFrequency (const FrequencyType frequency) const {
        if (frequency > static_cast<uint16_t> (frame_data_output_frequency_t::frequency_t::Maximum))
            return false;
        return write (FrameType::OutputFrequency, frame_data_output_frequency_t { .frequency = static_cast<frame_data_output_frequency_t::frequency_t> (frequency) });
    }
    bool getOutputFrequency (uint16_t &frequency) const { return read (FrameType::OutputFrequency, frequency); }
    enum class Thresholds : uint8_t {
        V1 = static_cast<uint8_t> (FrameType::ThresholdV1),
        V2 = static_cast<uint8_t> (FrameType::ThresholdV2),
        V3 = static_cast<uint8_t> (FrameType::ThresholdV3),
        S1 = static_cast<uint8_t> (FrameType::ThresholdS1),
        S2 = static_cast<uint8_t> (FrameType::ThresholdS2),
        S3 = static_cast<uint8_t> (FrameType::ThresholdS3),
        N1 = static_cast<uint8_t> (FrameType::ThresholdN1),
        N2 = static_cast<uint8_t> (FrameType::ThresholdN2),
        N3 = static_cast<uint8_t> (FrameType::ThresholdN3)
    };
    using ThresholdType = uint16_t;
    bool setThreshold (const Thresholds type, const ThresholdType value) const {
        if (type == Thresholds::V1 || type == Thresholds::V2 || type == Thresholds::V3)
            return write (static_cast<FrameType> (type), frame_data_threshold_v_t { .threshold = value });
        if (type == Thresholds::S1 || type == Thresholds::S2 || type == Thresholds::S3)
            return write (static_cast<FrameType> (type), frame_data_threshold_s_t { .threshold = value });
        if (type == Thresholds::N1 || type == Thresholds::N2 || type == Thresholds::N3)
            if ((value >= static_cast<uint8_t> (frame_data_threshold_n_t::count_t::Minimum) && value <= static_cast<uint8_t> (frame_data_threshold_n_t::count_t::Maximum)))
                return write (static_cast<FrameType> (type), frame_data_threshold_n_t { .count = static_cast<frame_data_threshold_n_t::count_t> (value) });
        return false;
    }
    bool getThreshold (const Thresholds type, ThresholdType &value) const { return read (static_cast<FrameType> (type), value); }
    using TemperatureType = float;
    bool getTemperature (TemperatureType &value) const {
        frame_data_temperature_t temperature {};
        if (! read (FrameType::Temperature, temperature))
            return false;
        value = frame_data_temperature_t::toFloat (temperature.value);
        return true;
    }
    bool setAmbientLightMode (const bool enabled) const { return _setEnterOrExitMode (FrameType::AmbientLight, enabled); }
    bool setSleepMode (const bool enabled) const { return _setEnterOrExitMode (FrameType::SleepMode, enabled); }

private:
    frame_transceiver_t transceiver;
};

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
