// BME280_example.cpp

#include <Arduino.h>

//#define SERIAL_DEBUG_DISABLED

#define USE_LIB_WEBSOCKET true

#include "sensesp_app.h"
#include "sensors/analog_input.h"
#include "sensors/bme280.h"
#include "sensors/digital_input.h"
#include "signalk/signalk_output.h"
#include "transforms/air_density.h"
#include "transforms/dew_point.h"
#include "transforms/frequency.h"
#include "transforms/heat_index.h"
#include "transforms/lambda_transform.h"
#include "transforms/linear.h"
#include "transforms/typecast.h"
#include "transforms/voltagedivider.h"

#include "signalk/signalk_position.h"
#include "signalk/signalk_time.h"
#include "transforms/angle_correction.h"

#include "wiring_helpers.h"

ReactESP app([]() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  sensesp_app = new SensESPApp();

  // Create a BME280, which represents the physical sensor.
  // 0x77 is the default address. Some chips use 0x76, which is shown here.
  auto *bme280 = new BME280(0x77);

  // If you want to change any of the settings that are set by
  // Adafruit_BME280::setSampling(), do that here, like this:
  // bme280->adafruit_bme280->setSampling(); // pass in the parameters you want
  {
    // Define the read_delays you're going to use:
    const uint read_delay = 1000;           // once per second
    const uint pressure_read_delay = 60000; // once per minute

    // Create a BME280Value, which is used to read a specific value from the
    // BME280, and send its output to Signal K as a number (float). This one is
    // for the temperature reading.
    auto *bme_temperature = new BME280Value(bme280, BME280Value::temperature,
                                            read_delay, "/Outside/Temperature");

    bme_temperature->connect_to(
        new SKOutputNumber("environment.outside.temperature"));

    // Do the same for the barometric pressure value. Its read_delay is longer,
    // since barometric pressure can't change all that quickly. It could be much
    // longer for that reason.
    auto *bme_pressure =
        new BME280Value(bme280, BME280Value::pressure, pressure_read_delay,
                        "/Outside/Pressure");

    bme_pressure->connect_to(
        new SKOutputNumber("environment.outside.pressure"));

    // Do the same for the humidity value.
    auto *bme_humidity = new BME280Value(bme280, BME280Value::humidity,
                                         read_delay, "/Outside/Humidity");

    bme_humidity->connect_to(
        new SKOutputNumber("environment.outside.humidity"));

    // Use the transform dewPoint to calculate the dewpoint based upon the
    // temperature and humidity.
    auto *dew_point = new DewPoint();

    dew_point->connect_from(bme_temperature, bme_humidity)
        ->connect_to(
            new SKOutputNumber("environment.outside.dewPointTemperature"));

    // Use the transform airDensity to calculate the air density of humid air
    // based upon the temperature, humidity and pressure.
    auto *airDensity = new AirDensity();

    airDensity->connect_from(bme_temperature, bme_humidity, bme_pressure)
        ->connect_to(new SKOutputNumber("environment.outside.airDensity"));

    // Use the transform heatIndex to calculate the heat index based upon the
    // temperature and humidity.
    auto *heat_index_temperature = new HeatIndexTemperature();

    heat_index_temperature->connect_from(bme_temperature, bme_humidity)
        ->connect_to(
            new SKOutputNumber("environment.outside.heatIndexTemperature"))
        ->connect_to(new HeatIndexEffect)
        ->connect_to(new SKOutputString("environment.outside.heatIndexEffect"));
  }

  // Wind direction comes to us via ADC. A count of zero implies due north,
  // full count is 359 degrees. Signal K expects this in radians, so we scale it
  // to 0-2*PI. We report it as apparent wind as seen by a boat heading north,
  // with negative values being wind from port.
  {
    uint8_t pin = 34 /* ADC 6 */;
    uint read_interval_ms = 3 * 1000 /* read every 3s */;

    auto *sensor = new AnalogInput(pin, read_interval_ms, "", 2 * PI);
    sensor
        ->connect_to(new LambdaTransform<float, float>(
            [](float inRadians) {
              // Convert from true wind direction to apparent wind direction
              // with the boat heading true north.
              return inRadians < PI ? inRadians : (inRadians - 2 * PI);
            },
            "" /* no config */))
        ->connect_to(new SKOutputNumber("environment.wind.angleApparent"));
  }

  // Wind speed. 1Hz is 1.026m/s.
  {
    uint8_t pin = 19;
    uint read_interval_ms = 3 * 1000 /* read every 3s */;

    auto *sensor =
        new DigitalInputCounter(pin, INPUT_PULLUP, RISING, read_interval_ms);
    sensor->connect_to(new Frequency(1.026, "/Outside/Windspeed/calibrate"))
        ->connect_to(new SKOutputNumber("environment.wind.speedApparent"));
  }

  // Rain sensor. Report count every 5 minutes.
  {
    const uint8_t pin = 18;
    const uint read_interval_ms = 5 * 60 * 1000 /* 5 minutes */;
    const unsigned int ignore_interval_ms = 200; // switch is kinda noisy
    const float multiplier = 0.18;               // mm per count

    // There's no path in the Signal K spec for rain, so let's make one.
    SKMetadata *rain_meta = new SKMetadata();
    rain_meta->units_ = "mm";
    rain_meta->description_ = rain_meta->display_name_ =
        rain_meta->short_name_ = "Rainfall";

    auto *sensor = new DigitalInputDebounceCounter(
        pin, INPUT_PULLUP, FALLING, read_interval_ms, ignore_interval_ms);
    sensor->connect_to(new Typecast<int, float>())
        ->connect_to(new Linear(multiplier, 0.0, "/Outside/Rain/calibrate"))
        ->connect_to(
            new SKOutputNumber("environment.rain.volume5min", rain_meta));
  }

  //////////
  // connect a RPM meter. A DigitalInputCounter implements an interrupt
  // to count pulses and reports the readings every read_delay ms
  // (500 in the example). A Frequency
  // transform takes a number of pulses and converts that into
  // a frequency. The sample multiplier converts the 97 tooth
  // tach output into Hz, SK native units.
  {

    const char *sk_path = "propulsion.main.revolutions";
    // const char *config_path = "/sensors/engine_rpm";
    const char *config_path_calibrate = "/sensors/engine_rpm/calibrate";
    const char *config_path_skpath = "/sensors/engine_rpm/sk";

    const float multiplier = 1.0 / 97.0;
    const uint read_delay = 500;

    // Wire it all up by connecting the producer directly to the consumer
    // ESP8266 pins are specified as DX
    // ESP32 pins are specified as just the X in GPIOX
#ifdef ESP8266
    uint8_t pin = D5;
#elif defined(ESP32)
    uint8_t pin = 4;
#endif
    auto *sensor =
        new DigitalInputCounter(pin, INPUT_PULLUP, RISING, read_delay);

    sensor
        ->connect_to(new Frequency(
            multiplier, config_path_calibrate)) // connect the output of sensor
                                                // to the input of Frequency()
        ->connect_to(new SKOutputNumber(
            sk_path, config_path_skpath)); // connect the output of Frequency()
                                           // to a Signal K Output as a number
  }

  // Turbo boost sensor
  {
    const char *sk_path = "propulsion.main.boostPressure";
    const char *config_path_calibrate = "/engine/boostPressure/calibrate";
    const char *config_path_skpath = "/sensors/boostPressure/sk";

    // The sensor produces values from 0 to 1/2 Vcc, i.e. 2048 counts in our 12
    // bit ADC. AnalogInput scales the value to 0->1024, so a max-value reading
    // will be 512 counts. The maximum reading for the sensor is 50 psi, or
    // about 344737 Pa.
    const float multiplier = 344737 / 512.0f;
    uint8_t pin = 36 /* ADC 0 */;

    auto *analog_input = new AnalogInput(pin);

    analog_input->connect_to(new Linear(multiplier, 0.0, config_path_calibrate))
        ->connect_to(new SKOutputNumber(sk_path, config_path_skpath));
  }

  // Oil pressure sensor
  {
    const char *sk_path = "propulsion.main.oilPressure";
    const char *config_path_calibrate = "/engine/oilPressure/calibrate";
    const char *config_path_skpath = "/sensors/oilPressure/sk";

    // The sensor produces values from 0 to 1/2 Vcc, i.e. 2048 counts in our 12
    // bit ADC. AnalogInput scales the value to 0->1024, so a max-value reading
    // will be 512 counts. The maximum reading for the sensor is 100 psi, or
    // about 689475 Pa.
    const float multiplier = 689475 / 512.0f;
    uint8_t pin = 39 /* ADC 3 */;

    auto *analog_input = new AnalogInput(pin);

    analog_input->connect_to(new Linear(multiplier, 0.0, config_path_calibrate))
        ->connect_to(new SKOutputNumber(sk_path, config_path_skpath));
  }

  // GPS sentences
  {
    uint8_t gps_rx_pin = 5;
    uint16_t gps_baud = 9600;

    HardwareSerial *serial = &Serial1;

    serial->begin(gps_baud, SERIAL_8N1, gps_rx_pin, -1, false, 256);

    // HardwareSerial* serial = &Serial1;
    // serial->begin(gps_baud, SERIAL_8N1, gps_rx_pin, -1);

    auto *gps = new GPSInput(serial);
    gps->nmea_data_.position.connect_to(
        new SKOutputPosition("navigation.position", ""));
    gps->nmea_data_.gnss_quality.connect_to(
        new SKOutputString("navigation.methodQuality", ""));
    gps->nmea_data_.num_satellites.connect_to(
        new SKOutputInt("navigation.satellites", ""));
    gps->nmea_data_.horizontal_dilution.connect_to(
        new SKOutputNumber("navigation.horizontalDilution", ""));
    gps->nmea_data_.geoidal_separation.connect_to(
        new SKOutputNumber("navigation.geoidalSeparation", ""));
    gps->nmea_data_.dgps_age.connect_to(
        new SKOutputNumber("navigation.differentialAge", ""));
    gps->nmea_data_.dgps_id.connect_to(
        new SKOutputNumber("navigation.differentialReference", ""));
    gps->nmea_data_.datetime.connect_to(
        new SKOutputTime("navigation.datetime", ""));
    gps->nmea_data_.speed.connect_to(
        new SKOutputNumber("navigation.speedOverGround", ""));
    gps->nmea_data_.true_course.connect_to(
        new SKOutputNumber("navigation.courseOverGroundTrue", ""));
    gps->nmea_data_.variation.connect_to(
        new SKOutputNumber("navigation.magneticVariation", ""));
    gps->nmea_data_.rtk_age.connect_to(
        new SKOutputNumber("navigation.rtkAge", ""));
    gps->nmea_data_.rtk_ratio.connect_to(
        new SKOutputNumber("navigation.rtkRatio", ""));
    gps->nmea_data_.baseline_length.connect_to(
        new SKOutputNumber("navigation.rtkBaselineLength", ""));
    gps->nmea_data_.baseline_course
        .connect_to(new SKOutputNumber("navigation.rtkBaselineCourse"))
        ->connect_to(new AngleCorrection(0, 0, "/sensors/heading/correction"))
        ->connect_to(new SKOutputNumber("navigation.headingTrue", ""));

    //  GPSInput* gps = setup_gps(serial);
  }

  sensesp_app->enable();
});