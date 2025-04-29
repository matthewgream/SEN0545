
#ifdef SEN0545_STANDALONE

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#include <Arduino.h>

#include <memory>

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

typedef unsigned long interval_t;
typedef unsigned long counter_t;

void append (const char *separator, String &csv, const String &add) {
    if (! csv.isEmpty ())
        csv += separator;
    csv += add;
}

template <typename... Args>
String join (const char *delimiter, const Args &...args) {
    String result;
    size_t index = 0;
    ((result += (index++ > 0 ? String (delimiter) : String ()) + String (args)), ...);
    return result;
}

class Enableable {
    bool _enabled;

public:
    explicit Enableable (const bool enabled = false) :
        _enabled (enabled) { }

    inline void operator++ (int) {
        _enabled = true;
    }
    inline operator bool () const {
        return _enabled;
    }
    void operator= (const bool state) {
        _enabled = state;
    }
};

class Intervalable {
    interval_t _interval, _previous;
    counter_t _exceeded = 0;

public:
    explicit Intervalable (const interval_t interval = 0, const interval_t previous = 0) :
        _interval (interval),
        _previous (previous) { }
    operator bool () {
        const interval_t current = millis ();
        if (current - _previous > _interval) {
            _previous = current;
            return true;
        }
        return false;
    }
    bool passed (interval_t *interval = nullptr, const bool atstart = false) {
        const interval_t current = millis ();
        if ((atstart && _previous == 0) || current - _previous > _interval) {
            if (interval != nullptr)
                (*interval) = current - _previous;
            _previous = current;
            return true;
        }
        return false;
    }
    void reset (const interval_t interval = std::numeric_limits<interval_t>::max ()) {
        if (interval != std::numeric_limits<interval_t>::max ())
            _interval = interval;
        _previous = millis ();
    }
    void setat (const interval_t place) {
        _previous = millis () - ((_interval - place) % _interval);
    }
    void wait () {
        const interval_t current = millis ();
        if (current - _previous < _interval)
            delay (_interval - (current - _previous));
        else if (_previous > 0)
            _exceeded++;
        _previous = millis ();
    }
    counter_t exceeded () const {
        return _exceeded;
    }
};

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#include "Hardware_I2C.hpp"
#include "SensorDevice_DfrobotSEN0545.hpp"

std::unique_ptr<i2cbus> i2cbus0;
std::unique_ptr<SensorDevice_DfrobotSEN0545> sen0545;

void setup () {
    Serial.begin (115200);
    delay (5 * 1000);
    Serial.println ("UP");

    i2cbus0 = std::make_unique<i2cbus> (i2cbus::Config { .num = 0, .pin_SDA = GPIO_NUM_5, .pin_SCL = GPIO_NUM_4 });
    i2cbus0->begin ();
    Serial.printf ("i2cbus0: %s\n", i2cbus0->status ().c_str ());

    sen0545 = std::make_unique<SensorDevice_DfrobotSEN0545> (SensorDevice_DfrobotSEN0545::Config { .A0 = false, .A1 = true, .uart = 0 }, *i2cbus0);
    const auto result = sen0545->begin ();
    if (!result.success) {
        Serial.printf ("sen0545: failed to begin(): %s\n", result.details.c_str ());
        return;
    }
    Serial.printf ("sen0545: %s\n", sen0545->information ().c_str ());
}

// -----------------------------------------------------------------------------------------------

Intervalable second (15 * 1000);

void loop () {
    second.wait ();
    Serial.println ("***");
    const auto result = sen0545->measurementsExecute ();
    Serial.printf ("sen0545: %s [%s]\n", toString (result.status).c_str (), result.details.c_str ());
    sen0545->debugDump ();
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#endif
