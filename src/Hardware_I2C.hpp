
// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#include <Wire.h>

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

class i2cbus;

class i2cbus_device {
    TwoWire &_wire;

public:
    typedef uint8_t address;
    static inline constexpr size_t address_max = 8;
    struct address_range {
        address lower, upper;
    };
    struct address_group {
        uint8_t length;
        std::array<address, address_max> addresses;
    };
    struct device_address {
        enum { single = 1,
               range = 2,
               group = 3 } type;
        union {
            address single;
            address_range range;
            address_group group;
        } addr;
    };
    using device_info = std::pair<device_address, String>;

    explicit i2cbus_device (i2cbus &bus);

    TwoWire &wire () const {
		return _wire;
	}
    virtual device_info i2cdevice () const = 0;
};

// clang-format off
#define I2CBUS_DEVICE_INFO_SINGLE(name, addx)                  i2cbus_device::device_info ({ .type = i2cbus_device::device_address::single, .addr = { .single = addx } }, name)
#define I2CBUS_DEVICE_INFO_RANGE(name, addx_lower, addx_upper) i2cbus_device::device_info ({ .type = i2cbus_device::device_address::range,  .addr = { .range = { .lower = addx_lower, .upper = addx_upper } } }, name)
#define I2CBUS_DEVICE_INFO_GROUP(name, ...)                    i2cbus_device::device_info ({ .type = i2cbus_device::device_address::group,  .addr = { .group = { .length = VA_NARGS(__VA_ARGS__), .addresses = { __VA_ARGS__ } } } }, name)
#define VA_NARGS_IMPL(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, N, ...) N
#define VA_NARGS(...) VA_NARGS_IMPL(__VA_ARGS__, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1)
#define I2CBUS_DEVICE_INFO_ARRAY(name, addx_data, addx_size)   i2cbus_device::device_info ({ .type = i2cbus_device::device_address::group,  .addr = { .group = { .length = static_cast <uint8_t> (std::min (addx_size, i2cbus_device::address_max)), .addresses = [&]() -> std::array<i2cbus_device::address, i2cbus_device::address_max> { std::array<i2cbus_device::address, i2cbus_device::address_max> addrs; for (uint8_t i = 0; i < std::min (addx_size, i2cbus_device::address_max); i++) addrs[i] = addx_data[i]; return addrs; }() } } }, name)
#define I2CBUS_DEVICE_ADDRESS_ITER_GET(addx, iter)              \
    (addx.type == i2cbus_device::device_address::single ? (iter == 0 ? addx.addr.single : 0xFF) : \
        (addx.type == i2cbus_device::device_address::range ? ((addx.addr.range.lower + iter) <= addx.addr.range.upper ? addx.addr.range.lower + iter : 0xFF) : \
            (iter < addx.addr.group.length ? addx.addr.group.addresses [iter] : 0xFF) \
        ) \
    )
#define I2CBUS_DEVICE_ADDRESS_ITER_END(addx, iter)             (0xFF)
#define I2CBUS_DEVICE_ADDRESS_ITER_NXT(addx, iter)             (iter) ++
// clang-format on

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

class i2cbus {

public:
    struct Config {
        uint8_t num;
        gpio_num_t pin_SDA, pin_SCL;
    };

private:
    using address = uint8_t;
    using addresses = std::vector<address>;
    using devices = std::vector<i2cbus_device *>;

    const Config config;
    TwoWire _wire;
    Enableable _available;
    devices _devices;
    addresses _addresses;

protected:
    void scan () {
        _addresses.clear ();
        for (uint8_t address = 1; address < 127; address++) {
            _wire.beginTransmission (address);
            if (_wire.endTransmission () == 0)
                _addresses.push_back (address);
        }
    }

public:
    explicit i2cbus (const Config &conf) :
        config (conf),
        _wire (config.num) { }

    TwoWire &wire () {
        return _wire;
    }
    void begin () {
        _wire.setPins (config.pin_SDA, config.pin_SCL);
        if (! _wire.begin ()) {
            Serial.printf ("i2cbus(%d):: sda=%d, sdl=%d, FAILED TO INITIALISE\n", config.num, config.pin_SDA, config.pin_SCL);
            return;
        }
        _available++;
    }
    void insert (i2cbus_device &device) {
        _devices.push_back (&device);
    }
    String status () {
        if (!_available)
            return "UNAVAILABLE";
        String s;
        scan ();
        addresses addresses (_addresses);
        for (const auto &device : _devices) {
            const auto &[device_address, device_name] = device->i2cdevice ();
            int iter = 0;
            uint8_t address;
            while ((address = I2CBUS_DEVICE_ADDRESS_ITER_GET (device_address, iter)) != I2CBUS_DEVICE_ADDRESS_ITER_END (device_address, iter)) {
                const auto &found = std::find (addresses.begin (), addresses.end (), address);
                append (", ", s, device_name + ":0x" + String (address, HEX) + "/" + String (found != addresses.end () ? "+" : "-"));
                if (found != addresses.end ())
                    addresses.erase (found);
                I2CBUS_DEVICE_ADDRESS_ITER_NXT (device_address, iter);
            }
        }
        for (const auto &address : addresses)
            append (", ", s, String ("?:0x") + String (address, HEX) + "/+");
        return join (", ", "num=" + String (config.num), "sda/scl=" + String (config.pin_SDA) + "/" + String (config.pin_SCL), "clk=" + String (_wire.getClock () / 1000) + String ("kHz :: ")) + s;
    }
};

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

i2cbus_device::i2cbus_device (i2cbus &bus) :
    _wire (bus.wire ()) {
    bus.insert (*this);
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
