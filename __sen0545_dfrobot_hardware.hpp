
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

#define FRAME_HEADER                 0x3A
#define FRAME_PROPERTY_READ          0
#define FRAME_PROPERTY_WRITE         1

#define FRAME_TYPE_FIRMWARE_VERSION  0
#define FRAME_TYPE_RAINFALL_STATUS   1
#define FRAME_TYPE_SYSTEM_STATUS     2
#define FRAME_TYPE_OPTICAL_SYSTEM    3
#define FRAME_TYPE_REALTIME_RAINFALL 4
#define FRAME_TYPE_OUTPUT_FREQUENCY  5
#define FRAME_TYPE_THRESHOLD_V1      6
#define FRAME_TYPE_THRESHOLD_V2      7
#define FRAME_TYPE_THRESHOLD_V3      8
#define FRAME_TYPE_THRESHOLD_S1      9
#define FRAME_TYPE_THRESHOLD_S2      10
#define FRAME_TYPE_THRESHOLD_S3      11
#define FRAME_TYPE_THRESHOLD_N1      12
#define FRAME_TYPE_THRESHOLD_N2      13
#define FRAME_TYPE_THRESHOLD_N3      14
#define FRAME_TYPE_AMBIENT_LIGHT     15
#define FRAME_TYPE_TEMPERATURE       16
#define FRAME_TYPE_SLEEP_MODE        17

#define FRAME_FCS_NULL               0x00

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
    static String toString (const status_t &status) {
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
            return "unknown(0x" + String ((uint16_t) status, HEX) + ")";
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
    static String toString (const status_t &status) {
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
            return "unknown(0x" + String ((uint16_t) status, HEX) + ")";
        }
    }
} __attribute__ ((__packed__)) frame_data_system_status_t;

// Type 3: Optical System - command enum
typedef struct {
    enum class command_t : uint16_t {
        Calibrate = 0,
        SendValue = 1
    } command;
    static String toString (const command_t &command) {
        switch (command) {
        case command_t::Calibrate :
            return "calibrate";
        case command_t::SendValue :
            return "send-value";
        default :
            return "unknown(0x" + String ((uint16_t) command, HEX) + ")";
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
    static String toString (const command_t &command) {
        switch (command) {
        case command_t::Exit :
            return "exit";
        case command_t::Enter :
            return "enter";
        default :
            return "unknown(0x" + String ((uint16_t) command, HEX) + ")";
        }
    }
} __attribute__ ((__packed__)) frame_data_enterorexit_t;

// Type 5: Output Frequency - integer range 0-9
typedef struct {
    uint16_t frequency;    // 0-9, 0=disabled, 1-9=frequency in 50ms units
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
    uint16_t count;
} __attribute__ ((__packed__)) frame_data_threshold_n_t;

// Type 16: Temperature Reading - command value
typedef struct {
    uint16_t command;    // Always 0 for reading temperature
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
    uint16_t value;
    uint8_t bytes [2];
} __attribute__ ((__packed__)) frame_data_t;

typedef struct {
    uint8_t flags;
    frame_data_t data;
} __attribute__ ((__packed__)) frame_command_t;
#define FRAME_FLAGS_PROPERTY_READ_OR_WRITE_DECODE(flags) (((uint8_t) (flags) >> 7) & 0x01U)
#define FRAME_FLAGS_TYPE_DECODE(flags)                   ((uint8_t) (flags) & 0x7FU)
#define FRAME_FLAGS_PROPERTY_READ_OR_WRITE_ENCODE(value) ((uint8_t) (((uint8_t) (value) << 7) & 0x80U))
#define FRAME_FLAGS_TYPE_ENCODE(value)                   ((uint8_t) ((uint8_t) (value) & 0x7FU))
#define FRAME_FLAGS_MAKE(property_read_or_write, type)   (uint8_t) (FRAME_FLAGS_PROPERTY_READ_OR_WRITE_ENCODE (property_read_or_write) | FRAME_FLAGS_TYPE_ENCODE (type))
#define FRAME_TYPES_MAKE(value16)                        { (uint8_t) ((value16 >> 8) & 0xFF), (uint8_t) (value16 & 0xFF) }

typedef struct {
    uint8_t header = FRAME_HEADER;
    frame_command_t data;
    uint8_t fcs = FRAME_FCS_NULL;
} __attribute__ ((__packed__)) frame_t;

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

class frame_transceiver_t {
public:
    using tx_function_t = std::function<size_t (const uint8_t *, size_t)>;
    using rx_function_t = std::function<size_t (uint8_t *, size_t)>;

    frame_transceiver_t (const tx_function_t tx, const rx_function_t rx) :
        tx_function (tx),
        rx_function (rx) { }

    bool transmit (const frame_command_t &command) const {
        const frame_t frame { .data = command, .fcs = _calculate_crc8 ((const uint8_t *) &command, sizeof (frame_command_t)) };
        const uint8_t *frame_raw = (const uint8_t *) &frame;
        DEBUG_SEN0545_PRINTF ("SEN0545::TX: %02X %02X %02X %02X %02X\n", frame_raw [0], frame_raw [1], frame_raw [2], frame_raw [3], frame_raw [4]);
        return tx_function (frame_raw, sizeof (frame_t)) == sizeof (frame_t);
    }
    std::optional<frame_command_t> receive (bool block = true) const {
        frame_t frame;
        uint8_t *frame_raw = (uint8_t *) &frame;
        size_t bytesRead = 0;
        unsigned long startTime = millis ();
        // XXX. needs to be more robust to preve4nt lockup
        while (bytesRead < sizeof (frame_t) && (block || (millis () - startTime < 100))) {
            if (bytesRead == 0) {
                if (rx_function (&frame_raw [0], 1) != 1) {
                    if (! block)
                        return std::nullopt;
                    delay (50);
                    continue;
                }
                if (frame.header != FRAME_HEADER)
                    continue;
                bytesRead++;
            } else {
                const size_t result = rx_function (frame_raw + bytesRead, sizeof (frame_t) - bytesRead);
                if (result == 0) {
                    if (! block)
                        return std::nullopt;
                    delay (50);
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
        const uint8_t fcs = _calculate_crc8 ((uint8_t *) &frame.data, sizeof (frame_command_t));
        if (fcs != frame.fcs) {
            DEBUG_SEN0545_PRINTF ("SEN0545::RX: mismatched fcs, received 0x%02X, calculated 0x%02X\n", frame.fcs, fcs);
            return std::nullopt;
        }
        return frame.data;
    }

private:
    tx_function_t tx_function;
    rx_function_t rx_function;
    static inline uint8_t _calculate_crc8 (const uint8_t *data, size_t len) {
        uint8_t crc = 0xFF;
        while (len--) {
            crc ^= *data++;
            for (uint8_t i = 0; i < 8; i++)
                crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
        return crc;
    }
};

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

class RainSensor {
public:
    RainSensor (frame_transceiver_t::tx_function_t tx, frame_transceiver_t::rx_function_t rx) :
        transceiver (tx, rx) { }

    template <typename T>
    bool read (uint8_t type, T &value) const {
        if (! transceiver.transmit (frame_command_t { .flags = FRAME_FLAGS_MAKE (FRAME_PROPERTY_READ, type) }))
            return false;
        delay (1);
        const auto response = transceiver.receive (true);
        if (! response || FRAME_FLAGS_TYPE_DECODE (response->flags) != type)
            return false;
        uint16_t value16 = (uint16_t) (response->data.bytes [0] << 8) | (uint16_t) (response->data.bytes [1]);
        value = reinterpret_cast<T &> (value16);
        return true;
    }
    template <typename T>
    bool write (uint8_t type, const T &value) const {
        uint16_t value16 = reinterpret_cast<const uint16_t &> (value);
        return transceiver.transmit (frame_command_t { .flags = FRAME_FLAGS_MAKE (FRAME_PROPERTY_WRITE, type), .data = { .bytes = FRAME_TYPES_MAKE (value16) } });
    }

    bool _enterOrExitMode (const uint8_t type, const bool enabled) const {
        return write (type, frame_data_enterorexit_t { .command = enabled ? frame_data_enterorexit_t::command_t::Enter : frame_data_enterorexit_t::command_t::Exit });
    }

    //

    bool getFirmwareVersion (uint8_t &major, uint8_t &backup) const {
        uint16_t version;
        if (! read (FRAME_TYPE_FIRMWARE_VERSION, version))
            return false;
        major = (version >> 8) & 0xFF;
        backup = version & 0xFF;
        return true;
    }
    bool getSystemStatus (frame_data_system_status_t::status_t &status) const {
        return read (FRAME_TYPE_SYSTEM_STATUS, status);
    }
    bool getRainfallStatus (frame_data_rainfall_status_t::status_t &status) const {
        return read (FRAME_TYPE_RAINFALL_STATUS, status);
    }
    bool performCalibration () const {
        return write (FRAME_TYPE_OPTICAL_SYSTEM, frame_data_optical_system_t { .command = frame_data_optical_system_t::command_t::Calibrate });
    }
    bool getOpticalCalibrationValue () const {
        return write (FRAME_TYPE_OPTICAL_SYSTEM, frame_data_optical_system_t { .command = frame_data_optical_system_t::command_t::SendValue });
    }
    bool setRealtimeMode (const bool enabled) const {
        return _enterOrExitMode (FRAME_TYPE_REALTIME_RAINFALL, enabled);
    }
    bool setOutputFrequency (const uint16_t frequency) const {
        return (frequency > 9) ? false : write (FRAME_TYPE_OUTPUT_FREQUENCY, frame_data_output_frequency_t { .frequency = frequency });
    }
    bool getOutputFrequency (uint16_t &frequency) const {
        return read (FRAME_TYPE_OUTPUT_FREQUENCY, frequency);
    }
    typedef enum : uint8_t { ThresholdV1 = FRAME_TYPE_THRESHOLD_V1,
                             ThresholdV2 = FRAME_TYPE_THRESHOLD_V2,
                             ThresholdV3 = FRAME_TYPE_THRESHOLD_V3 } ThresholdsV;
    bool setThreshold (ThresholdsV type, uint16_t threshold) const {
        return write (type, frame_data_threshold_v_t { .threshold = threshold });
    }
    bool getThreshold (ThresholdsV type, uint16_t &threshold) const {
        return read (type, threshold);
    }
    typedef enum : uint8_t { ThresholdS1 = FRAME_TYPE_THRESHOLD_S1,
                             ThresholdS2 = FRAME_TYPE_THRESHOLD_S2,
                             ThresholdS3 = FRAME_TYPE_THRESHOLD_S3 } ThresholdsS;
    bool setThreshold (ThresholdsS type, uint16_t threshold) const {
        return write (type, frame_data_threshold_s_t { .threshold = threshold });
    }
    bool getThreshold (ThresholdsS type, uint16_t &threshold) const {
        return read (type, threshold);
    }
    typedef enum : uint8_t { ThresholdN1 = FRAME_TYPE_THRESHOLD_N1,
                             ThresholdN2 = FRAME_TYPE_THRESHOLD_N2,
                             ThresholdN3 = FRAME_TYPE_THRESHOLD_N3 } ThresholdsN;
    bool setThreshold (ThresholdsN type, uint16_t count) const {
        return (count < 1 || count > 10) ? false : write (type, frame_data_threshold_n_t { .count = count });
    }
    bool getThreshold (ThresholdsN type, uint16_t &count) const {
        return read (type, count);
    }
    bool getTemperature (uint16_t &temp) const {
        return read (FRAME_TYPE_TEMPERATURE, temp);
    }
    bool setAmbientLightMode (const bool enabled) const {
        return _enterOrExitMode (FRAME_TYPE_AMBIENT_LIGHT, enabled);
    }
    bool setSleepMode (const bool enabled) const {
        return _enterOrExitMode (FRAME_TYPE_SLEEP_MODE, enabled);
    }

private:
    frame_transceiver_t transceiver;
};

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
