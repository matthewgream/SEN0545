
// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#include "__sen0545_dfrobot_hardware.hpp"
#include "DFRobot_IICSerial.h"

#include <functional>

class Hardware_DfrobotSEN0545 : public SensorComponentWithMeasurements, public i2cbus_device {
public:
    static inline constexpr unsigned long DEFAULT_SENSOR_SERIAL_BAUD = 115200;
    static inline constexpr unsigned char DEFAULT_SENSOR_SERIAL_OPTS = IICSerial_8N1;
    using TemperatureType = float;
    using RainfallType = frame_data_rainfall_status_t::status_t;
    using StatusType = frame_data_system_status_t::status_t;
    struct Config {
        uint8_t uart, IA1, IA0;
    };
    struct Measurements {
        SensorMeasurement<TemperatureType> temperature { "temperature", SensorMeasurementType::Internal };
        SensorMeasurement<RainfallType> rainfall { "rainfall", SensorMeasurementType::External, SensorMeasurement<RainfallType>::Convertor(frame_data_rainfall_status_t::toString) };
        SensorMeasurement<StatusType> status { "status", SensorMeasurementType::Internal,  SensorMeasurement<StatusType>::Convertor(frame_data_system_status_t::toString)  };
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

    static uint8_t _iicserial_address (const int uart, const int ia1, const int ia0) {
        return (uint8_t) ((0 << 7) | (ia1 << 6) | (ia0 << 5) | (1 << 4) | (0 << 3) | ((uart & 0x03) << 1) | (0 << 0));
    }

    bool sensorEnable () override {
        return true;
    }
    bool sensorDisable () override {
        return true;
    }
    SensorResult sensorConnectAndConfigure () override {
        if (_serial.begin (DEFAULT_SENSOR_SERIAL_BAUD, DEFAULT_SENSOR_SERIAL_OPTS) != 0)
            return SensorResult (false, "DFRobot_IICSerial::begin");
        // if (! _device.setSleepMode (false))
        //     return SensorResult (false, "RainSensor::setSleepMode (false)");
        if (! _device.getFirmwareVersion (_info.firmware.major, _info.firmware.backup))
            return SensorResult (false, "RainSensor::getFirmwareVersion");
        StatusType status;
        if (! _device.getSystemStatus (status))
            return SensorResult (false, "RainSensor::getSystemStatus");
        _measurements.status = status;
        return true;
    }
    SensorResult sensorDisconnect () override {
        // (void) _device.setSleepMode (true);
        _serial.end ();
        return true;
    }
    String sensorInformation () const override {
        return "i2c=0x" + String (_iicserial_address (config.uart, config.IA1, config.IA0), HEX) + ", firmware=" + String ( _info.firmware.major) + "." + String (_info.firmware.backup) + ", status=" + _measurements.status.toString ();
    }
    SensorResult sensorMeasurementsEnable (const interval_t interval) override {
        return true;
    }
    SensorResult sensorMeasurementsDisable () override {
        return true;
    }
    MeasurementResult sensorMeasurementsExecute () override {
        uint16_t temperature;
        RainfallType rainfall;
        StatusType status;
        if (! _device.getTemperature (temperature))
            return MeasurementResult (MeasurementError, "RainSensor::getTemperature");
        if (! _device.getRainfallStatus (rainfall))
            return MeasurementResult (MeasurementError, "RainSensor::getRainfallStatus");
        if (! _device.getSystemStatus (status))
            return MeasurementResult (MeasurementError, "RainSensor::getSystemStatus");
        _measurements.temperature = (TemperatureType) temperature / 1000.0f;
        _measurements.rainfall = rainfall;
        _measurements.status = status;
        return MeasurementUpdatedAndValid;
    }


public:
    i2cbus_device_info i2cdevice () const override {
        const uint8_t address_lower = _iicserial_address (config.uart, config.IA1, config.IA0), address_upper = address_lower + 0x07;
        return I2CBUS_DEVICE_INFO_RANGE (SensorComponent::name (), address_lower, address_upper);
    }
    Hardware_DfrobotSEN0545 (const Config &conf, TwoWire &wire) :
        SensorComponentWithMeasurements ("DfrobotSEN0545", "sen0545", { &_measurements.rainfall, &_measurements.status, &_measurements.temperature }),
        i2cbus_device (wire),
        config (conf),
        _serial (i2cbus_device::wire (), config.uart, config.IA1, config.IA0),
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