
// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

struct Result {
    bool success;
    String details;
    Result (const bool s = false, const String &d = String ()) :
        success (s),
        details (d) { }
};

enum MeasurementStatus {
    MeasurementError,
    MeasurementNotUpdated,
    MeasurementUpdatedButNotValid,
    MeasurementUpdatedAndValid,
};

String toString (const MeasurementStatus &status) {
    switch (status) {
    case MeasurementError :
        return "Error";
    case MeasurementNotUpdated :
        return "NotUpdated";
    case MeasurementUpdatedButNotValid :
        return "UpdatedAndNotValid";
    case MeasurementUpdatedAndValid :
        return "UpdatedAndValid";
    default :
        return "Unknown";
    }
}
struct MeasurementResult {
    MeasurementStatus status;
    String details;
    MeasurementResult (const MeasurementStatus s = MeasurementError, const String &d = String ()) :
        status (s),
        details (d) { }
};

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#include "__sen0545_dfrobot_hardware.hpp"
#include "DFRobot_IICSerial.h"

#include <functional>

class SensorDevice_DfrobotSEN0545 : public i2cbus_device {
    static inline constexpr unsigned long DEFAULT_SENSOR_SERIAL_BAUD = 115200;
    static inline constexpr unsigned char DEFAULT_SENSOR_SERIAL_OPTS = IICSerial_8N1;

public:
    struct Config {
        bool A0, A1;
        uint8_t uart;
    };

    using RainfallType = frame_data_rainfall_status_t::status_t;
    using StatusType = frame_data_system_status_t::status_t;
    struct Measurements {
        RainfallType rainfall;
        float t;
        StatusType s;
    };

    struct Information {
        struct {
            uint8_t major, backup;
        } firmware;
    };

private:
    const Config &config;

    DFRobot_IICSerial _serial;
    RainSensor _device;
    Measurements _measurements;
    Information _info;

    static uint8_t _address (const bool a1, const bool a0, const int uart) {
        return (uint8_t) ((0 << 7) | (static_cast<uint8_t> (a1) << 6) | (static_cast<uint8_t> (a0) << 5) | (1 << 4) | (0 << 3) | ((uart & 0x03) << 1) | (0 << 0));
    }

public:
    Result begin () {
        if (_serial.begin (DEFAULT_SENSOR_SERIAL_BAUD, DEFAULT_SENSOR_SERIAL_OPTS) != 0)
            return Result (false, "DFRobot_IICSerial::begin");
        // if (! _device.setSleepMode (false))
        //     return ComponentResult (false, "RainSensor::setSleepMode (false)");
        if (! _device.getFirmwareVersion (_info.firmware.major, _info.firmware.backup))
            return Result (false, "RainSensor::getFirmwareVersion");
        StatusType status;
        if (! _device.getSystemStatus (status))
            return Result (false, "RainSensor::getSystemStatus");
        _measurements.s = status;
        return true;
    }
    Result end () {
        // (void) _device.setSleepMode (true);
        _serial.end ();
        return true;
    }
    String information () const {
        return "i2c=0x" + String (_address (config.A1, config.A0, config.uart), HEX) + ", firmware=" + String (_info.firmware.major) + "." + String (_info.firmware.backup) + ", status=" + frame_data_system_status_t::toString (_measurements.s);
    }
    void debugDump () const {
        Serial.printf ("Temperature = %.2f\n", _measurements.t);
        Serial.printf ("Rainfall = %s\n", frame_data_rainfall_status_t::toString (_measurements.rainfall).c_str ());
        Serial.printf ("Status = %s\n", frame_data_system_status_t::toString (_measurements.s).c_str ());
    }

    MeasurementResult measurementsExecute () {
        uint16_t temperature;
        RainfallType rainfall;
        StatusType status;
        if (! _device.getTemperature (temperature))
            return MeasurementResult (MeasurementError, "RainSensor::getTemperature");
        if (! _device.getRainfallStatus (rainfall))
            return MeasurementResult (MeasurementError, "RainSensor::getRainfallStatus");
        if (! _device.getSystemStatus (status))
            return MeasurementResult (MeasurementError, "RainSensor::getSystemStatus");
        _measurements.t = static_cast<float> (temperature) / 1000.0f;
        _measurements.s = status;
        _measurements.rainfall = rainfall;
        return MeasurementUpdatedAndValid;
    }

    i2cbus_device::device_info i2cdevice () const override {
        const i2cbus_device::address address_lower = _address (config.A1, config.A0, config.uart), address_upper = address_lower + 0x07;
        return I2CBUS_DEVICE_INFO_RANGE ("SEN0545", address_lower, address_upper);
    }

public:
    SensorDevice_DfrobotSEN0545 (const Config &conf, i2cbus &i2cbus) :
        i2cbus_device (i2cbus),
        config (conf),
        _serial (i2cbus_device::wire (), config.uart, config.A1, config.A0),
        _device (
            [&] (const uint8_t *data, size_t len) -> size_t {
                return _serial.write (data, len);
            },
            [&] (uint8_t *data, size_t len) -> size_t {
                size_t got = 0;
                int value;
                while (got < len && (value = _serial.read ()) != -1)
                    data [got++] = value & 0xFF;
                return got;
            }) { }
};

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
