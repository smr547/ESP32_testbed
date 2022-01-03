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
#include "transforms/analogvoltage.h"
#include "transforms/curveinterpolator.h"
#include "transforms/dew_point.h"
#include "transforms/frequency.h"
#include "transforms/heat_index.h"
#include "transforms/lambda_transform.h"
#include "transforms/linear.h"
#include "transforms/voltagedivider.h"

namespace {
class BoostPressureInterpolator : public CurveInterpolator {
public:
  BoostPressureInterpolator(String config_path = "")
      : CurveInterpolator(nullptr, config_path) {
    clearSamples();
    addSample(CurveInterpolator::Sample(0, 0));
    addSample(CurveInterpolator::Sample(512, 517100 /* 75 psi in Pa */));
  }
};

class OilPressureInterpolator : public CurveInterpolator {
public:
  OilPressureInterpolator(String config_path = "")
      : CurveInterpolator(nullptr, config_path) {
    clearSamples();
    addSample(CurveInterpolator::Sample(0, 0));
    addSample(CurveInterpolator::Sample(512, 689476 /* 100 psi in Pa */));
  }
};

} // namespace

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

    const float Vin = 3.3;
    const float R1 = 5000;

    uint8_t pin = 36 /* ADC 0 */;

    auto *analog_input = new AnalogInput(pin);

    analog_input->connect_to(new AnalogVoltage())
        ->connect_to(
            new VoltageDividerR2(R1, Vin, "/engine/boostPressure/sender"))
        ->connect_to(
            new BoostPressureInterpolator("/engine/boostPressure/curve"))
        ->connect_to(new Linear(1.0, 0.0, config_path_calibrate))
        ->connect_to(new SKOutputNumber(sk_path, config_path_skpath));
  }

  // Turbo boost sensor
  {
    const char *sk_path = "propulsion.main.boostPressure";
    const char *config_path_calibrate = "/engine/boostPressure/calibrate";
    const char *config_path_skpath = "/sensors/boostPressure/sk";

    const float Vin = 3.3;
    const float R1 = 5000;

    uint8_t pin = 36 /* ADC 0 */;

    auto *analog_input = new AnalogInput(pin);

    analog_input->connect_to(new AnalogVoltage())
        ->connect_to(
            new VoltageDividerR2(R1, Vin, "/engine/boostPressure/sender"))
        ->connect_to(
            new BoostPressureInterpolator("/engine/boostPressure/curve"))
        ->connect_to(new Linear(1.0, 0.0, config_path_calibrate))
        ->connect_to(new SKOutputNumber(sk_path, config_path_skpath));
  }

  // Oil pressure sensor
  {
    const char *sk_path = "propulsion.main.oilPressure";
    const char *config_path_calibrate = "/engine/oilPressure/calibrate";
    const char *config_path_skpath = "/sensors/oilPressure/sk";

    const float Vin = 3.3;
    const float R1 = 5000;

    uint8_t pin = 39 /* ADC 3 */;

    auto *analog_input = new AnalogInput(pin);

    analog_input->connect_to(new AnalogVoltage())
        ->connect_to(
            new VoltageDividerR2(R1, Vin, "/engine/oilPressure/sender"))
        ->connect_to(new OilPressureInterpolator("/engine/oilPressure/curve"))
        ->connect_to(new Linear(1.0, 0.0, config_path_calibrate))
        ->connect_to(new SKOutputNumber(sk_path, config_path_skpath));
  }

  sensesp_app->enable();
});