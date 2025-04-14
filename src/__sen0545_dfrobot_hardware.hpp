
// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#pragma once

#include <stdint.h>
#include <optional>

// #define DEBUG_SEN0545
#ifdef DEBUG_SEN0545
#define DEBUG_SEN0545_PRINTF    Serial.printf
#define DEBUG_SEN0545_ONLY(...) __VA_ARGS__
#else
#define DEBUG_SEN0545_PRINTF(...) \
    do {                          \
    } while (0)
#define DEBUG_SEN0545_ONLY(...)
#endif

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

static inline constexpr uint8_t FRAME_HEADER = 0x3A;

enum class FrameProperty : uint8_t {
    Read = 0,
    Write = 1
};

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

static inline constexpr uint8_t FRAME_FCS = 0x00;

// Type 0: Firmware Version - two 8-bit values
typedef struct {
    uint8_t major;
    uint8_t backup;
} __attribute__ ((__packed__)) frame_data_firmware_version_t;

// Type 1: Rainfall Status - enumerated status
typedef struct {
    enum class status_t : uint16_t {
        NoRain = 0,
        LightRain = 1,
        ModerateRain = 2,
        HeavyRain = 3
    } status;
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
} __attribute__ ((__packed__)) frame_data_rainfall_status_t;

// Type 2: System Status - enumerated status
typedef struct {
    enum class status_t : uint16_t {
        Normal = 0,
        CommunicationError = 1,
        LEDADamaged = 2,
        LEDBDamaged = 3,
        CalibrationNotGood = 4,
        ConfigFailure = 5,
        SerialError = 6,
        LowVoltage = 7
    } status;
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
} __attribute__ ((__packed__)) frame_data_system_status_t;

// Type 3: Optical System - command enum
typedef struct {
    enum class command_t : uint16_t {
        Calibrate = 0,
        SendValue = 1
    } command;
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
} __attribute__ ((__packed__)) frame_data_optical_system_t;

// Type 4: Real-time Rainfall Mode - command enum
// Type 15: Ambient Light Mode - command enum
// Type 17: Sleep Mode - command enum
typedef struct {
    enum class command_t : uint16_t {
        Exit = 0,
        Enter = 1
    } command;
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
} __attribute__ ((__packed__)) frame_data_enterorexit_t;

// Type 5: Output Frequency - integer range 0-9: 0=disabled, 1-9=frequency in 50ms units
typedef struct {
    enum class frequency_t : uint16_t {
        Disabled = 0,
        Minimum = 1,
        Maximum = 9
    } frequency;
} __attribute__ ((__packed__)) frame_data_output_frequency_t;

// Types 6-8: V Thresholds - 16-bit range 0-65535
typedef struct {
    uint16_t threshold;
} __attribute__ ((__packed__)) frame_data_threshold_v_t;

// Types 9-11: S Thresholds - 16-bit range 0-65535
typedef struct {
    uint16_t threshold;
} __attribute__ ((__packed__)) frame_data_threshold_s_t;

// Types 12-14: N Thresholds - range 1-10
typedef struct {
    enum class count_t : uint16_t {
        Minimum = 1,
        Maximum = 10
    } count;
} __attribute__ ((__packed__)) frame_data_threshold_n_t;

// Type 16: Temperature Reading - command value: always 0 for reading temperature
typedef struct {
    uint16_t command;
} __attribute__ ((__packed__)) frame_data_temperature_t;

// Union of all frame data types
typedef union {
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
} __attribute__ ((__packed__)) frame_data_t;

typedef struct {
    FrameProperty property : 1;
    FrameType type : 7;
} __attribute__ ((__packed__)) frame_flags_t;

typedef struct {
    frame_flags_t flags;
    frame_data_t data;
} __attribute__ ((__packed__)) frame_command_t;

#define FRAME_DATA_ENCODE(type)                                                          \
    {                                                                                    \
        .__bytes = {(uint8_t) ((reinterpret_cast<const uint16_t &> (type) >> 8) & 0xFF), \
                    (uint8_t) (reinterpret_cast<const uint16_t &> (type) & 0xFF) }       \
    }
#define FRAME_DATA_DECODE(data) (uint16_t) ((data).__bytes [0] << 8) | (uint16_t) ((data).__bytes [1])

typedef struct {
    const uint8_t header = FRAME_HEADER;
    frame_command_t data;
    uint8_t fcs = FRAME_FCS;
} __attribute__ ((__packed__)) frame_t;

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

// static inline uint8_t crc8_poly0x31 (const uint8_t *data, size_t len) {
//     uint8_t crc = 0xFF;
//     while (len--) {
//         crc ^= *data++;
//         for (uint8_t i = 0; i < 8; i++)
//             crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
//     }
//     return crc;
// }

static inline constexpr uint32_t __crc8x31_bit (const uint32_t crc) {
    return (crc << 1) ^ ((crc & 0x100) ? 0x31 : 0);
}
static inline constexpr uint32_t __crc8x31_byte (const uint32_t crc) {
    return __crc8x31_bit (__crc8x31_bit (__crc8x31_bit (__crc8x31_bit (__crc8x31_bit (__crc8x31_bit (__crc8x31_bit (__crc8x31_bit (crc))))))));
}
template <size_t INDEX, size_t SIZE>
static inline constexpr uint32_t __crc8x31_bytes (const uint32_t crc, const uint8_t *data) {
    static_assert (SIZE <= 16, "crc8 limited to 16 bytes for performance reasons");
    if constexpr (INDEX < SIZE)
        return __crc8x31_bytes<INDEX + 1, SIZE> (__crc8x31_byte (crc ^ data [INDEX]), data);
    else
        return crc;
}
template <size_t SIZE>
static inline constexpr uint8_t crc8_poly0x31 (const uint8_t *data) {
    return __crc8x31_bytes<0, SIZE> (0x000000FF, data) & 0xFF;
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

class frame_transceiver_t {
public:
    using tx_function_t = std::function<size_t (const uint8_t *, const size_t)>;
    using rx_function_t = std::function<size_t (uint8_t *, const size_t)>;
    using bk_function_t = std::function<void (void)>;

    frame_transceiver_t (const tx_function_t tx, const rx_function_t rx, const bk_function_t bk = nullptr) :
        tx_function (tx),
        rx_function (rx),
        bk_function (bk) { }

    bool transmit (const frame_command_t &data) const {
        const frame_t frame { .data = data, .fcs = crc8_poly0x31<sizeof (frame_command_t)> (reinterpret_cast<const uint8_t *> (&data)) };
        const uint8_t *frame_raw = reinterpret_cast<const uint8_t *> (&frame);
        DEBUG_SEN0545_PRINTF ("SEN0545::TX: %02X %02X %02X %02X %02X\n", frame_raw [0], frame_raw [1], frame_raw [2], frame_raw [3], frame_raw [4]);
        return tx_function (frame_raw, sizeof (frame_t)) == sizeof (frame_t);
    }
    template <bool blocking = true, unsigned long timeout = 100>
    std::optional<frame_command_t> receive () const {
        frame_t frame;
        uint8_t *frame_raw = reinterpret_cast<uint8_t *> (&frame);
        size_t bytesRead = 0;
        unsigned long startTime = millis ();
        // XXX. needs to be more robust to prevent lockup
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
                if (frame.header != FRAME_HEADER)
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
        const uint8_t fcs = crc8_poly0x31<sizeof (frame_command_t)> (reinterpret_cast<uint8_t *> (&frame.data));
        if (fcs != frame.fcs) {
            DEBUG_SEN0545_PRINTF ("SEN0545::RX: mismatched fcs, received 0x%02X, calculated 0x%02X\n", frame.fcs, fcs);
            return std::nullopt;
        }
        return frame.data;
    }

private:
    const tx_function_t tx_function;
    const rx_function_t rx_function;
    const bk_function_t bk_function;
};

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

class RainSensor {
public:
    RainSensor (const frame_transceiver_t::tx_function_t tx, const frame_transceiver_t::rx_function_t rx, const frame_transceiver_t::bk_function_t block) :
        transceiver (tx, rx, block) { }

    template <typename T>
    bool read (const FrameType type, T &value) const {
        if (! transceiver.transmit (frame_command_t {
                .flags = { .property = FrameProperty::Read, .type = type }
        }))
            return false;
        delay (1);
        const auto response = transceiver.receive ();
        if (! response || response->flags.type != type)
            return false;
        value = static_cast<T> (FRAME_DATA_DECODE (response->data));
        return true;
    }
    template <typename T>
    bool write (const FrameType type, const T &value) const {
        return transceiver.transmit (frame_command_t {
            .flags = { .property = FrameProperty::Write, .type = type },
            .data = FRAME_DATA_ENCODE (value)
        });
    }

    bool _setEnterOrExitMode (const FrameType type, const bool enabled) const {
        return write (type, frame_data_enterorexit_t { .command = enabled ? frame_data_enterorexit_t::command_t::Enter : frame_data_enterorexit_t::command_t::Exit });
    }

    //

    using FirmwareVersion = frame_data_firmware_version_t;
    bool getFirmwareVersion (FirmwareVersion &version) const {
        return read (FrameType::FirmwareVersion, version);
    }
    using SystemStatus = frame_data_system_status_t::status_t;
    bool getSystemStatus (SystemStatus &status) const {
        return read (FrameType::SystemStatus, status);
    }
    using RainfallStatus = frame_data_rainfall_status_t::status_t;
    bool getRainfallStatus (RainfallStatus &status) const {
        return read (FrameType::RainfallStatus, status);
    }
    bool performCalibration () const {
        return write (FrameType::OpticalSystem, frame_data_optical_system_t { .command = frame_data_optical_system_t::command_t::Calibrate });
    }
    bool getOpticalCalibrationValue () const {
        return write (FrameType::OpticalSystem, frame_data_optical_system_t { .command = frame_data_optical_system_t::command_t::SendValue });
    }
    bool setRealtimeMode (const bool enabled) const {
        return _setEnterOrExitMode (FrameType::RealtimeRainfall, enabled);
    }
    using FrequencyType = uint16_t;
    bool setOutputFrequency (const FrequencyType frequency) const {
        if (frequency > static_cast<uint16_t> (frame_data_output_frequency_t::frequency_t::Maximum))
            return false;
        return write (FrameType::OutputFrequency, frame_data_output_frequency_t { .frequency = static_cast<frame_data_output_frequency_t::frequency_t> (frequency) });
    }
    bool getOutputFrequency (FrequencyType &frequency) const {
        return read (FrameType::OutputFrequency, frequency);
    }
    enum class Thresholds : uint8_t { V1 = static_cast<uint8_t> (FrameType::ThresholdV1),
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
    bool getThreshold (const Thresholds type, ThresholdType &value) const {
        return read (static_cast<FrameType> (type), value);
    }
    using TemperatureType = float;
    bool getTemperature (TemperatureType &value) const {
        uint16_t temp;
        if (! read (FrameType::Temperature, temp))
            return false;
        value = static_cast<float> (temp) / 1000.0f;
        return true;
    }
    bool setAmbientLightMode (const bool enabled) const {
        return _setEnterOrExitMode (FrameType::AmbientLight, enabled);
    }
    bool setSleepMode (const bool enabled) const {
        return _setEnterOrExitMode (FrameType::SleepMode, enabled);
    }

private:
    frame_transceiver_t transceiver;
};

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
