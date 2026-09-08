#include "myEnvironmentSensorManager.h"

#include <Wire.h>

#include <TCA9548.h>

PCA9546 muxSensors(0x70, &Wire1);

#if ENV_PIN_SDA && ENV_PIN_SCL
#define TELEM_WIRE &Wire1  // Use Wire1 as the I2C bus for Environment Sensors
#else
#define TELEM_WIRE &Wire  // Use default I2C bus for Environment Sensors
#endif

// ============================================================
// Sensor library includes and static driver instances
// ============================================================

#if ENV_INCLUDE_BME680_BSEC
#ifndef TELEM_BME680_ADDRESS
#define TELEM_BME680_ADDRESS 0x76
#endif
#define TELEM_BME680_SEALEVELPRESSURE_HPA (1013.25)
#include <bsec.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
static const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_28d/bsec_iaq.txt" // 3.3v, LP, 28 day background calibration window
};
static Bsec     bsec_iaq;
static float    bsec_temperature     = 0;
static float    bsec_humidity        = 0;
static float    bsec_pressure_hpa    = 0;
static float    bsec_iaq_val         = 0;
static uint8_t  bsec_accuracy        = 0;
static bool     bsec_active          = false;
static bool     bsec_data_ready      = false;
static bool     bsec_first_save_done = false;
static uint32_t bsec_last_save_ms    = 0;
#define BSEC_STATE_FILE "/bsec_state.bin"
#define BSEC_SAVE_INTERVAL_MS (8UL * 60 * 60 * 1000) // 8 hour state-save interval
#endif

#ifdef ENV_INCLUDE_BME680
#ifndef TELEM_BME680_ADDRESS
#define TELEM_BME680_ADDRESS 0x76
#endif
#define TELEM_BME680_SEALEVELPRESSURE_HPA (1013.25)
#include <Adafruit_BME680.h>
static Adafruit_BME680 BME680(TELEM_WIRE);
#endif

#if ENV_INCLUDE_AHTX0
#ifndef TELEM_AHTX_ADDRESS
#define TELEM_AHTX_ADDRESS      0x38      // AHT10, AHT20 temperature and humidity sensor I2C address
#endif
#include <Adafruit_AHTX0.h>
static Adafruit_AHTX0 AHTX0;
#endif

#if ENV_INCLUDE_BME280
#ifndef TELEM_BME280_ADDRESS
#define TELEM_BME280_ADDRESS    0x76      // BME280 environmental sensor I2C address
#endif
#define TELEM_BME280_SEALEVELPRESSURE_HPA (1013.25)    // Atmospheric pressure at sea level
#include <Adafruit_BME280.h>
static Adafruit_BME280 BME280;
#endif

#if ENV_INCLUDE_BME280B
#ifndef TELEM_BME280B_ADDRESS
#define TELEM_BME280B_ADDRESS    0x77      // BME280 environmental sensor I2C address
#endif
#define TELEM_BME280B_SEALEVELPRESSURE_HPA (1013.25)    // Atmospheric pressure at sea level
#include <Adafruit_BME280.h>
static Adafruit_BME280 BME280B;
#endif

#if ENV_INCLUDE_BMP280
#ifndef TELEM_BMP280_ADDRESS
#define TELEM_BMP280_ADDRESS    0x76      // BMP280 environmental sensor I2C address
#endif
#define TELEM_BMP280_SEALEVELPRESSURE_HPA (1013.25)    // Atmospheric pressure at sea level
#include <Adafruit_BMP280.h>
static Adafruit_BMP280 BMP280(TELEM_WIRE);
#endif

#if ENV_INCLUDE_INA3221
#ifndef TELEM_INA3221_ADDRESS
#define TELEM_INA3221_ADDRESS     0x42    // INA3221 3 channel current sensor I2C address
#endif
#ifndef TELEM_INA3221_SHUNT_VALUE
#define TELEM_INA3221_SHUNT_VALUE 0.100 // most variants will have a 0.1 ohm shunts
#endif
#ifndef TELEM_INA3221_NUM_CHANNELS
#define TELEM_INA3221_NUM_CHANNELS 3
#endif
#include <Adafruit_INA3221.h>
static Adafruit_INA3221 INA3221;
#endif

#if ENV_INCLUDE_INA3221E1A
#ifndef TELEM_INA3221E1A_ADDRESS
#define TELEM_INA3221E1A_ADDRESS     0x40    // INA3221 3 channel current sensor I2C address
#endif
#ifndef TELEM_INA3221E1A_SHUNT_VALUE
#define TELEM_INA3221E1A_SHUNT_VALUE 0.033 // most variants will have a 0.1 ohm shunts
#endif
#ifndef TELEM_INA3221E1A_NUM_CHANNELS
#define TELEM_INA3221E1A_NUM_CHANNELS 3
#endif
#include <Adafruit_INA3221.h>
static Adafruit_INA3221 INA3221E1A;
#endif

#if ENV_INCLUDE_INA3221E1B
#ifndef TELEM_INA3221E1B_ADDRESS
#define TELEM_INA3221E1B_ADDRESS     0x41    // INA3221 3 channel current sensor I2C address
#endif
#ifndef TELEM_INA3221E1B_SHUNT_VALUE
#define TELEM_INA3221E1B_SHUNT_VALUE 0.033 // most variants will have a 0.1 ohm shunts
#endif
#ifndef TELEM_INA3221E1B_NUM_CHANNELS
#define TELEM_INA3221E1B_NUM_CHANNELS 3
#endif
#include <Adafruit_INA3221.h>
static Adafruit_INA3221 INA3221E1B;
#endif

#if ENV_INCLUDE_INA3221E1C
#ifndef TELEM_INA3221E1C_ADDRESS
#define TELEM_INA3221E1C_ADDRESS     0x42    // INA3221 3 channel current sensor I2C address
#endif
#ifndef TELEM_INA3221E1C_SHUNT_VALUE
#define TELEM_INA3221E1C_SHUNT_VALUE 0.012 // most variants will have a 0.1 ohm shunts
#endif
#ifndef TELEM_INA3221E1C_NUM_CHANNELS
#define TELEM_INA3221E1C_NUM_CHANNELS 3
#endif
#include <Adafruit_INA3221.h>
static Adafruit_INA3221 INA3221E1C;
#endif


// ============================================================
// I2C bus scanner
// Probes every valid address and records which ones ACK.
// This runs before any sensor library is touched, so a missing
// or misbehaving device cannot stall or crash the boot sequence.
// ============================================================

static void scanI2CBus(TwoWire* wire, bool found[128]) {
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    wire->beginTransmission(addr);
    found[addr] = (wire->endTransmission() == 0);
  }
}

// ============================================================
// Per-sensor init and query functions
//
// init(wire, address) — called only when the address was seen
//   on the bus. Returns 0 on failure, or the number of
//   telemetry channels the sensor will consume (1 for all
//   single-output sensors; INA3221 returns one per enabled
//   hardware channel; MLX90614 and RAK12035+calibration
//   return 2).
//
// query(channel, sub_channel, lpp) — called once per active
//   sensor entry during querySensors(). sub_channel is always
//   0 for single-output sensors.
// ============================================================

#if ENV_INCLUDE_AHTX0
static uint8_t init_ahtx0(TwoWire* wire, uint8_t addr) {
  return AHTX0.begin(wire, 0, addr) ? 1 : 0;
}
static void query_ahtx0(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  sensors_event_t humidity, temp;
  AHTX0.getEvent(&humidity, &temp);
  lpp.addTemperature(ch, temp.temperature);
  lpp.addRelativeHumidity(ch, humidity.relative_humidity);
}
#endif

#ifdef ENV_INCLUDE_BME680
static uint8_t init_bme680(TwoWire*, uint8_t addr) {
  // Wire was set in the static constructor; begin() takes address only.
  return BME680.begin(addr) ? 1 : 0;
}
static void query_bme680(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  if (BME680.performReading()) {
    lpp.addTemperature(ch, BME680.temperature);
    lpp.addRelativeHumidity(ch, BME680.humidity);
    const float pressure_hpa = BME680.pressure / 100.0f;
    lpp.addBarometricPressure(ch, pressure_hpa);
    lpp.addAltitude(ch, 44330.0f * (1.0f - powf(pressure_hpa / (float)TELEM_BME680_SEALEVELPRESSURE_HPA, 0.1903f)));
    lpp.addGenericSensor(ch, BME680.gas_resistance);
  }
}
#endif

#if ENV_INCLUDE_BME280
static uint8_t init_bme280(TwoWire* wire, uint8_t addr) {
  if (!BME280.begin(addr, wire)) return 0;
  BME280.setSampling(Adafruit_BME280::MODE_FORCED,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::FILTER_OFF,
                     Adafruit_BME280::STANDBY_MS_1000);
  return 1;
}
static void query_bme280(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  if (BME280.takeForcedMeasurement()) {
    lpp.addTemperature(ch, BME280.readTemperature());
    lpp.addRelativeHumidity(ch, BME280.readHumidity());
    lpp.addBarometricPressure(ch, BME280.readPressure() / 100);
    //lpp.addAltitude(ch, BME280.readAltitude(TELEM_BME280_SEALEVELPRESSURE_HPA));
  }
}
#endif

#if ENV_INCLUDE_BME280B
static uint8_t init_bme280b(TwoWire* wire, uint8_t addr) {
  if (!BME280B.begin(addr, wire)) return 0;
  BME280B.setSampling(Adafruit_BME280::MODE_FORCED,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::FILTER_OFF,
                     Adafruit_BME280::STANDBY_MS_1000);
  return 1;
}
static void query_bme280b(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  if (BME280.takeForcedMeasurement()) {
    lpp.addTemperature(ch, BME280B.readTemperature());
    lpp.addRelativeHumidity(ch, BME280B.readHumidity());
    lpp.addBarometricPressure(ch, BME280B.readPressure() / 100);
    //lpp.addAltitude(ch, BME280B.readAltitude(TELEM_BME280B_SEALEVELPRESSURE_HPA));
  }
}
#endif

#if ENV_INCLUDE_BMP280
static uint8_t init_bmp280(TwoWire*, uint8_t addr) {
  // BMP280 static instance was constructed with TELEM_WIRE; begin() uses it.
  return BMP280.begin(addr) ? 1 : 0;
}
static void query_bmp280(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  lpp.addTemperature(ch, BMP280.readTemperature());
  lpp.addBarometricPressure(ch, BMP280.readPressure() / 100);
  lpp.addAltitude(ch, BMP280.readAltitude(TELEM_BMP280_SEALEVELPRESSURE_HPA));
}
#endif

#if ENV_INCLUDE_INA3221
static uint8_t init_ina3221(TwoWire* wire, uint8_t addr) {
  if (!INA3221.begin(addr, wire)) return 0;
  for (int i = 0; i < TELEM_INA3221_NUM_CHANNELS; i++) {
    INA3221.setShuntResistance(i, TELEM_INA3221_SHUNT_VALUE);
  }
  // Each enabled hardware channel becomes its own telemetry channel.
  uint8_t enabled = 0;
  for (int i = 0; i < TELEM_INA3221_NUM_CHANNELS; i++) {
    if (INA3221.isChannelEnabled(i)) enabled++;
  }
  return enabled > 0 ? enabled : 1;
}
static void query_ina3221(uint8_t ch, uint8_t sub_ch, CayenneLPP& lpp) {
  // sub_ch is the index of the nth enabled hardware channel.
  uint8_t seen = 0;
  for (int i = 0; i < TELEM_INA3221_NUM_CHANNELS; i++) {
    if (INA3221.isChannelEnabled(i)) {
      if (seen == sub_ch) {
        float v = INA3221.getBusVoltage(i);
        float c = INA3221.getCurrentAmps(i);
        lpp.addVoltage(ch, v);
        lpp.addCurrent(ch, c);
        //lpp.addPower(ch, v * c);
        return;
      }
      seen++;
    }
  }
}
#endif

#if ENV_INCLUDE_INA3221E1A
static uint8_t init_ina3221e1a(TwoWire* wire, uint8_t addr) {
  muxSensors.enableChannel(0);
  if (!INA3221E1A.begin(addr, wire)) return 0;
  for (int i = 0; i < TELEM_INA3221E1A_NUM_CHANNELS; i++) {
    INA3221E1A.setShuntResistance(i, TELEM_INA3221E1A_SHUNT_VALUE);
  }
  // Each enabled hardware channel becomes its own telemetry channel.
  uint8_t enabled = 0;
  for (int i = 0; i < TELEM_INA3221E1A_NUM_CHANNELS; i++) {
    if (INA3221E1A.isChannelEnabled(i)) enabled++;
  }
  muxSensors.disableChannel(0);
  return enabled > 0 ? enabled : 1;
}
static void query_ina3221e1a(uint8_t ch, uint8_t sub_ch, CayenneLPP& lpp) {
  // sub_ch is the index of the nth enabled hardware channel.
  uint8_t seen = 0;
  muxSensors.enableChannel(0);
  for (int i = 0; i < TELEM_INA3221E1A_NUM_CHANNELS; i++) {
    if (INA3221E1A.isChannelEnabled(i)) {
      if (seen == sub_ch) {
        float v = INA3221E1A.getBusVoltage(i);
        float c = INA3221E1A.getCurrentAmps(i);
        lpp.addVoltage(ch, v);
        lpp.addCurrent(ch, c);
        //lpp.addPower(ch, v * c);
        return;
      }
      seen++;
    }
  }
  muxSensors.disableChannel(0);
}
#endif

#if ENV_INCLUDE_INA3221E1B
static uint8_t init_ina3221e1b(TwoWire* wire, uint8_t addr) {
  muxSensors.enableChannel(0);
  if (!INA3221E1B.begin(addr, wire)) return 0;
  for (int i = 0; i < TELEM_INA3221E1B_NUM_CHANNELS; i++) {
    INA3221E1B.setShuntResistance(i, TELEM_INA3221E1B_SHUNT_VALUE);
  }
  // Each enabled hardware channel becomes its own telemetry channel.
  uint8_t enabled = 0;
  for (int i = 0; i < TELEM_INA3221E1B_NUM_CHANNELS; i++) {
    if (INA3221E1B.isChannelEnabled(i)) enabled++;
  }
  muxSensors.disableChannel(0);
  return enabled > 0 ? enabled : 1;
}
static void query_ina3221e1b(uint8_t ch, uint8_t sub_ch, CayenneLPP& lpp) {
  // sub_ch is the index of the nth enabled hardware channel.
  uint8_t seen = 0;
  muxSensors.enableChannel(0);
  for (int i = 0; i < TELEM_INA3221E1B_NUM_CHANNELS; i++) {
    if (INA3221E1B.isChannelEnabled(i)) {
      if (seen == sub_ch) {
        float v = INA3221E1B.getBusVoltage(i);
        float c = INA3221E1B.getCurrentAmps(i);
        lpp.addVoltage(ch, v);
        lpp.addCurrent(ch, c);
        //lpp.addPower(ch, v * c);
        return;
      }
      seen++;
    }
  }
  muxSensors.disableChannel(0);
}
#endif

#if ENV_INCLUDE_INA3221E1C
static uint8_t init_ina3221e1c(TwoWire* wire, uint8_t addr) {
  muxSensors.enableChannel(0);
  if (!INA3221E1C.begin(addr, wire)) return 0;
  for (int i = 0; i < TELEM_INA3221E1C_NUM_CHANNELS; i++) {
    INA3221E1C.setShuntResistance(i, TELEM_INA3221E1C_SHUNT_VALUE);
  }
  // Each enabled hardware channel becomes its own telemetry channel.
  uint8_t enabled = 0;
  for (int i = 0; i < TELEM_INA3221E1C_NUM_CHANNELS; i++) {
    if (INA3221E1C.isChannelEnabled(i)) enabled++;
  }
  muxSensors.disableChannel(0);
  return enabled > 0 ? enabled : 1;
}
static void query_ina3221e1c(uint8_t ch, uint8_t sub_ch, CayenneLPP& lpp) {
  // sub_ch is the index of the nth enabled hardware channel.
  uint8_t seen = 0;
  muxSensors.enableChannel(0);
  for (int i = 0; i < TELEM_INA3221E1C_NUM_CHANNELS; i++) {
    if (INA3221E1C.isChannelEnabled(i)) {
      if (seen == sub_ch) {
        float v = INA3221E1C.getBusVoltage(i);
        float c = INA3221E1C.getCurrentAmps(i);
        lpp.addVoltage(ch, v);
        lpp.addCurrent(ch, c);
        //lpp.addPower(ch, v * c);
        return;
      }
      seen++;
    }
  }
  muxSensors.disableChannel(0);
}
#endif


#if ENV_INCLUDE_BME680_BSEC
static void bsec_load_state() {
  using namespace Adafruit_LittleFS_Namespace;
  File f = InternalFS.open(BSEC_STATE_FILE, FILE_O_READ);
  if (!f) return;
  uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
  f.read(state, BSEC_MAX_STATE_BLOB_SIZE);
  f.close();
  bsec_iaq.setState(state);
}

static void bsec_save_state() {
  using namespace Adafruit_LittleFS_Namespace;
  uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
  bsec_iaq.getState(state);
  InternalFS.remove(BSEC_STATE_FILE);
  File f = InternalFS.open(BSEC_STATE_FILE, FILE_O_WRITE);
  if (!f) return;
  f.write(state, BSEC_MAX_STATE_BLOB_SIZE);
  f.close();
}

static uint8_t init_bme680_bsec(TwoWire* wire, uint8_t addr) {
  bsec_iaq.begin(addr, *wire);
  if (bsec_iaq.bsecStatus != BSEC_OK) return 0;

  bsec_iaq.setConfig(bsec_config_iaq);
  if (bsec_iaq.bsecStatus != BSEC_OK) return 0;

  bsec_virtual_sensor_t outputs[] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
  };
  bsec_iaq.updateSubscription(outputs, 6, BSEC_SAMPLE_RATE_LP);
  if (bsec_iaq.bsecStatus != BSEC_OK) return 0;

  bsec_load_state();
  bsec_active = true;
  return 1;
}

static void query_bme680_bsec(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  if (!bsec_data_ready) return;
  lpp.addTemperature(ch, bsec_temperature);
  lpp.addRelativeHumidity(ch, bsec_humidity);
  lpp.addBarometricPressure(ch, bsec_pressure_hpa);
  lpp.addAltitude(ch, 44330.0f * (1.0f - powf(bsec_pressure_hpa / (float)TELEM_BME680_SEALEVELPRESSURE_HPA, 0.1903f)));
  lpp.addGenericSensor(ch, (uint16_t)bsec_iaq_val);
  lpp.addAnalogInput(ch, (float)bsec_accuracy);
}
#endif

// ============================================================
// Sensor descriptor table
//
// Each entry maps an I2C address to a sensor's init and query
// functions. Only entries whose ENV_INCLUDE_* guard is defined
// are compiled in. The sentinel at the end keeps the array
// non-empty regardless of which sensors are enabled.
//
// Ordering here determines channel assignment at runtime:
// the first detected+initialized sensor gets channel 2, the
// next gets channel 3, and so on.
// ============================================================

struct SensorDef {
  uint8_t     address;
  const char* name;
  uint8_t   (*init)(TwoWire* wire, uint8_t address);
  void      (*query)(uint8_t channel, uint8_t sub_channel, CayenneLPP& telemetry);
};

static const SensorDef SENSOR_TABLE[] = {
#if ENV_INCLUDE_AHTX0
  { TELEM_AHTX_ADDRESS,    "AHT10/AHT20", init_ahtx0,    query_ahtx0    },
#endif
#ifdef ENV_INCLUDE_BME680
  { TELEM_BME680_ADDRESS,  "BME680",       init_bme680,   query_bme680   },
#endif
#if ENV_INCLUDE_BME680_BSEC
  { TELEM_BME680_ADDRESS,  "BME680+BSEC",   init_bme680_bsec, query_bme680_bsec },
#endif
#if ENV_INCLUDE_BME280
  { TELEM_BME280_ADDRESS,  "BME280",       init_bme280,   query_bme280   },
#endif
#if ENV_INCLUDE_BME280B
  { TELEM_BME280B_ADDRESS,  "BME280B",       init_bme280b,   query_bme280b   },
#endif
#if ENV_INCLUDE_BMP280
  { TELEM_BMP280_ADDRESS,  "BMP280",       init_bmp280,   query_bmp280   },
#endif
#if ENV_INCLUDE_INA3221
  { TELEM_INA3221_ADDRESS, "INA3221",      init_ina3221,  query_ina3221  },
#endif
  { 0, nullptr, nullptr, nullptr }  // sentinel — keeps the array non-empty
};
static const size_t SENSOR_TABLE_SIZE = (sizeof(SENSOR_TABLE) / sizeof(SENSOR_TABLE[0])) - 1;

static const SensorDef SENSOR2_TABLE[] = {
#if ENV_INCLUDE_INA3221E1A
  { TELEM_INA3221E1A_ADDRESS, "INA3221E1A",      init_ina3221e1a,  query_ina3221e1a  },
#endif
#if ENV_INCLUDE_INA3221E1B
{ TELEM_INA3221E1B_ADDRESS, "INA3221E1B",      init_ina3221e1b,  query_ina3221e1b  },
#endif
#if ENV_INCLUDE_INA3221E1C
{ TELEM_INA3221E1C_ADDRESS, "INA3221E1C",      init_ina3221e1c,  query_ina3221e1c  },
#endif
  { 0, nullptr, nullptr, nullptr }  // sentinel — keeps the array non-empty
};
static const size_t SENSOR2_TABLE_SIZE = (sizeof(SENSOR2_TABLE) / sizeof(SENSOR2_TABLE[0])) - 1;

// ============================================================
// begin() — scan the I2C bus, then initialize only what was
// found. A sensor whose address does not ACK during the scan
// is never touched by a library call, preventing hangs or
// crashes caused by absent or misbehaving hardware.
// ============================================================

bool EnvironmentSensorManager::begin() {
  #if ENV_INCLUDE_GPS
  #ifdef RAK_WISBLOCK_GPS
  rakGPSInit();
  #else
  initBasicGPS();
  #endif
  #endif

  #if ENV_PIN_SDA && ENV_PIN_SCL
    #ifdef NRF52_PLATFORM
  Wire1.setPins(ENV_PIN_SDA, ENV_PIN_SCL);
  Wire1.setClock(100000);
  Wire1.begin();
    #else
  Wire1.begin(ENV_PIN_SDA, ENV_PIN_SCL, 100000);
    #endif
  MESH_DEBUG_PRINTLN("Second I2C initialized on pins SDA: %d SCL: %d", ENV_PIN_SDA, ENV_PIN_SCL);
  #endif

  // Scan the I2C bus before touching any sensor library.
  bool detected[128] = {};
  scanI2CBus(TELEM_WIRE, detected);

  // Walk the sensor table and initialize only detected devices.
  _active_sensor_count = 0;
  for (size_t i = 0; i < SENSOR_TABLE_SIZE && _active_sensor_count < MAX_ACTIVE_SENSORS; i++) {
    const SensorDef& def = SENSOR_TABLE[i];
    if (!detected[def.address]) {
      MESH_DEBUG_PRINTLN("%s not detected at I2C address %02X", def.name, def.address);
      continue;
    }
    uint8_t n = def.init(TELEM_WIRE, def.address);
    if (n == 0) {
      MESH_DEBUG_PRINTLN("%s found at %02X but failed to initialize", def.name, def.address);
      continue;
    }
    MESH_DEBUG_PRINTLN("Found %s at address: %02X", def.name, def.address);
    for (uint8_t sub = 0; sub < n && _active_sensor_count < MAX_ACTIVE_SENSORS; sub++) {
      _active_sensors[_active_sensor_count++] = { def.query, sub };
    }
  }

  // MUX I2C
  if (muxSensors.begin()) {
    delay(100);
    muxSensors.disableAllChannels();
    MESH_DEBUG_PRINTLN("MUX SENSORS Initialized");
  }
  else
    MESH_DEBUG_PRINTLN("MUX SENSORS NOT Found");

  muxSensors.enableChannel(0);
  scanI2CBus(&Wire1, detected);

  // Walk the sensor table and initialize only detected devices.
  for (size_t i = 0; i < SENSOR2_TABLE_SIZE && _active_sensor_count < MAX_ACTIVE_SENSORS; i++) {
    const SensorDef& def = SENSOR2_TABLE[i];
    if (!detected[def.address]) {
      MESH_DEBUG_PRINTLN("%s not detected at I2C address %02X", def.name, def.address);
      continue;
    }
    uint8_t n = def.init(&Wire1, def.address);
    if (n == 0) {
      MESH_DEBUG_PRINTLN("%s found at %02X but failed to initialize", def.name, def.address);
      continue;
    }
    MESH_DEBUG_PRINTLN("Found %s at address: %02X", def.name, def.address);
    for (uint8_t sub = 0; sub < n && _active_sensor_count < MAX_ACTIVE_SENSORS; sub++) {
      _active_sensors[_active_sensor_count++] = { def.query, sub };
    }
  }
  muxSensors.enableChannel(0);
  return true;
}

// ============================================================
// querySensors() — GPS stays on channel 1; each active sensor
// gets the next available channel in the order it was
// initialized.
// ============================================================

bool EnvironmentSensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  next_available_channel = TELEM_CHANNEL_SELF + 1;

  if (requester_permissions & TELEM_PERM_LOCATION && gps_active) {
    telemetry.addGPS(TELEM_CHANNEL_SELF, node_lat, node_lon, node_altitude);
  }

  if (requester_permissions & TELEM_PERM_ENVIRONMENT) {
    for (int i = 0; i < _active_sensor_count; i++) {
      _active_sensors[i].query(next_available_channel, _active_sensors[i].sub_channel, telemetry);
      next_available_channel++;
    }
  }

  return true;
}


int EnvironmentSensorManager::getNumSettings() const {
  int settings = 0;
  #if ENV_INCLUDE_GPS
    if (gps_detected) settings++;  // only show GPS setting if GPS is detected
  #endif
  return settings;
}

const char* EnvironmentSensorManager::getSettingName(int i) const {
  int settings = 0;
  #if ENV_INCLUDE_GPS
    if (gps_detected && i == settings++) {
      return "gps";
    }
  #endif
  return NULL;
}

const char* EnvironmentSensorManager::getSettingValue(int i) const {
  int settings = 0;
  #if ENV_INCLUDE_GPS
    if (gps_detected && i == settings++) {
      return gps_active ? "1" : "0";
    }
  #endif
  return NULL;
}

bool EnvironmentSensorManager::setSettingValue(const char* name, const char* value) {
  #if ENV_INCLUDE_GPS
  if (gps_detected && strcmp(name, "gps") == 0) {
    if (strcmp(value, "0") == 0) {
      stop_gps();
    } else {
      start_gps();
    }
    return true;
  }
  if (strcmp(name, "gps_interval") == 0) {
    uint32_t interval_seconds = atoi(value);
    gps_update_interval_sec = interval_seconds > 0 ? interval_seconds : 1;
    return true;
  }
  #endif
  return false;  // not supported
}

#if ENV_INCLUDE_GPS
void EnvironmentSensorManager::initBasicGPS() {
#if (RP2040_PLATFORM)
  Serial1.setPinout(PIN_GPS_TX, PIN_GPS_RX);
#else
  Serial1.setPins(PIN_GPS_TX, PIN_GPS_RX);
#endif

  #ifdef GPS_BAUD_RATE
  Serial1.begin(GPS_BAUD_RATE);
  #else
  Serial1.begin(9600);
  #endif

  // Try to detect if GPS is physically connected to determine if we should expose the setting
  _location->begin();
  _location->reset();

  #ifndef PIN_GPS_EN
    MESH_DEBUG_PRINTLN("No GPS wake/reset pin found for this board. Continuing on...");
  #endif

  // Give GPS a moment to power up and send data
  delay(1000);

  // We'll consider GPS detected if we see any data on Serial1
#ifdef ENV_SKIP_GPS_DETECT
  gps_detected = true;
#else
  gps_detected = (Serial1.available() > 0);
#endif

  if (gps_detected) {
    MESH_DEBUG_PRINTLN("GPS detected");
    #ifdef PERSISTANT_GPS
      gps_active = true;
      return;
    #endif
  } else {
    MESH_DEBUG_PRINTLN("No GPS detected");
  }
  _location->stop();
  gps_active = false; //Set GPS visibility off until setting is changed
}

// gps code for rak might be moved to MicroNMEALoactionProvider
// or make a new location provider ...
#ifdef RAK_WISBLOCK_GPS
void EnvironmentSensorManager::rakGPSInit(){

  Serial1.setPins(PIN_GPS_TX, PIN_GPS_RX);

  #ifdef GPS_BAUD_RATE
  Serial1.begin(GPS_BAUD_RATE);
  #else
  Serial1.begin(9600);
  #endif

  //search for the correct IO standby pin depending on socket used
  if(gpsIsAwake(WB_IO2)){
  }
  else if(gpsIsAwake(WB_IO4)){
  }
  else if(gpsIsAwake(WB_IO5)){
  }
  else{
    MESH_DEBUG_PRINTLN("No GPS found");
    gps_active = false;
    gps_detected = false;
    Serial1.end();
    return;
  }

  #ifndef FORCE_GPS_ALIVE // for use with repeaters, until GPS toggle is implimented
  //Now that GPS is found and set up, set to sleep for initial state
  stop_gps();
  #endif
}

bool EnvironmentSensorManager::gpsIsAwake(uint8_t ioPin){

  #if defined(ETHERNET_ENABLED) && defined(RAK_BOARD)
    if (ioPin == WB_IO2) {
      // WB_IO2 powers the Ethernet module on RAK baseboards.
      return false;
    }
  #endif

  //set initial waking state
  pinMode(ioPin,OUTPUT);
  digitalWrite(ioPin,LOW);
  delay(500);
  digitalWrite(ioPin,HIGH);
  delay(500);

  //Try to init RAK12500 on I2C
  if (ublox_GNSS.begin(Wire) == true){
    MESH_DEBUG_PRINTLN("RAK12500 GPS init correctly with pin %i",ioPin);
    ublox_GNSS.setI2COutput(COM_TYPE_UBX);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_SBAS);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_IMES);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS);
    ublox_GNSS.setMeasurementRate(1000);
    ublox_GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
    gpsResetPin = ioPin;
    i2cGPSFlag = true;
    gps_active = true;
    gps_detected = true;

    _location = &RAK12500_provider;
    return true;
  } else if (Serial1.available()) {
    MESH_DEBUG_PRINTLN("Serial GPS init correctly and is turned on");
#ifdef PIN_GPS_EN
    if(PIN_GPS_EN){
      gpsResetPin = PIN_GPS_EN;
    }
#endif
    serialGPSFlag = true;
    gps_active = true;
    gps_detected = true;
    return true;
  }

  pinMode(ioPin, INPUT);
  MESH_DEBUG_PRINTLN("GPS did not init with this IO pin... try the next");
  return false;
}
#endif

void EnvironmentSensorManager::start_gps() {
  gps_active = true;
  #ifdef RAK_WISBLOCK_GPS
    pinMode(gpsResetPin, OUTPUT);
    digitalWrite(gpsResetPin, HIGH);
    return;
  #endif

  _location->begin();
  _location->reset();

#ifndef PIN_GPS_EN
  MESH_DEBUG_PRINTLN("Start GPS is N/A on this board. Actual GPS state unchanged");
#endif
}

void EnvironmentSensorManager::stop_gps() {
  gps_active = false;
  #ifdef RAK_WISBLOCK_GPS
    pinMode(gpsResetPin, OUTPUT);
    digitalWrite(gpsResetPin, LOW);
    return;
  #endif

  _location->stop();

  #ifndef PIN_GPS_EN
  MESH_DEBUG_PRINTLN("Stop GPS is N/A on this board. Actual GPS state unchanged");
  #endif
}
#endif // ENV_INCLUDE_GPS

#if ENV_INCLUDE_GPS || defined(ENV_INCLUDE_BME680_BSEC)
void EnvironmentSensorManager::loop() {

  #if ENV_INCLUDE_GPS
  static unsigned long next_gps_update = 0;
  if (gps_active) {
    _location->loop();
  }
  if ((long)(millis() - next_gps_update) > 0) {

    if(gps_active){
    #ifdef RAK_WISBLOCK_GPS
    if ((i2cGPSFlag || serialGPSFlag) && _location->isValid()) {
      node_lat = ((double)_location->getLatitude())/1000000.;
      node_lon = ((double)_location->getLongitude())/1000000.;
      MESH_DEBUG_PRINTLN("lat %f lon %f", node_lat, node_lon);
      node_altitude = ((double)_location->getAltitude()) / 1000.0;
      MESH_DEBUG_PRINTLN("lat %f lon %f alt %f", node_lat, node_lon, node_altitude);
    }
    #else
    if (_location->isValid()) {
      node_lat = ((double)_location->getLatitude())/1000000.;
      node_lon = ((double)_location->getLongitude())/1000000.;
      MESH_DEBUG_PRINTLN("lat %f lon %f", node_lat, node_lon);
      node_altitude = ((double)_location->getAltitude()) / 1000.0;
      MESH_DEBUG_PRINTLN("lat %f lon %f alt %f", node_lat, node_lon, node_altitude);
    }
    #endif
    }
    next_gps_update = millis() + (gps_update_interval_sec * 1000);
  }
  #endif
  #if ENV_INCLUDE_BME680_BSEC
  if (bsec_active && bsec_iaq.run()) {
    uint8_t prev_accuracy = bsec_accuracy;
    bsec_temperature  = bsec_iaq.temperature;
    bsec_humidity     = bsec_iaq.humidity;
    bsec_pressure_hpa = bsec_iaq.pressure / 100.0f;
    bsec_iaq_val      = bsec_iaq.iaq;
    bsec_accuracy     = bsec_iaq.iaqAccuracy;
    bsec_data_ready   = true;

    if (bsec_accuracy == 3) {
      if (!bsec_first_save_done) {
        bsec_save_state();
        bsec_last_save_ms = millis();
        bsec_first_save_done = true;
      } else if ((millis() - bsec_last_save_ms) >= BSEC_SAVE_INTERVAL_MS) {
        bsec_save_state();
        bsec_last_save_ms = millis();
      }
    }
  }
  #endif  // ENV_INCLUDE_BME680_BSEC
}
#endif // ENV_INCLUDE_GPS || ENV_INCLUDE_BME680_BSEC
