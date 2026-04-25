# Chapter 7: Communication Protocols and IoT Integration

## Connecting Smart Sensors to the Cloud and Beyond

Wireless connectivity transforms standalone smart sensors into networked intelligence systems. This chapter examines protocols, architectures, and best practices for sensor-to-cloud communication.

---

## Protocol Selection Matrix

### Key Decision Factors

**Range vs Power Trade-off:**
- Short range (< 100m): BLE, Zigbee, Thread → Ultra-low power
- Medium range (100m - 1km): Wi-Fi, Sub-GHz → Medium power
- Long range (> 1km): LoRaWAN, NB-IoT → Low-medium power

**Data Rate Requirements:**
- Low (< 10 kbps): LoRaWAN, NB-IoT
- Medium (10-500 kbps): BLE, Zigbee, Thread
- High (> 1 Mbps): Wi-Fi, BLE 5.0

**Latency Tolerance:**
- Real-time (< 100 ms): BLE, Wi-Fi, Thread
- Near real-time (< 1 s): Zigbee, NB-IoT (class C)
- Delay-tolerant (> 1 s): LoRaWAN, NB-IoT (class A)

**Infrastructure:**
- Existing infrastructure: Wi-Fi, NB-IoT (cellular)
- Gateway required: BLE, Zigbee, Thread, LoRaWAN
- Point-to-point: BLE, custom Sub-GHz

---

## Bluetooth Low Energy (BLE) 5.x

### Why BLE for Smart Sensors?

**Advantages:**
- Ultra-low power (< 10 µA average for periodic advertising)
- Ubiquitous support (smartphones, laptops, gateways)
- Easy pairing and setup
- Good throughput (up to 2 Mbps in BLE 5.0)

**Challenges:**
- Limited range (50-100m typical)
- Connection overhead
- Smartphone dependency for many applications

### BLE Architecture

**GAP (Generic Access Profile):**
- Broadcaster / Observer (advertising, scanning)
- Peripheral / Central (connection roles)

**GATT (Generic Attribute Profile):**
- Services: Collections of related characteristics
- Characteristics: Individual data values
- Descriptors: Metadata about characteristics

**Example: Environmental Sensor GATT Service**

```cpp
// Environmental Sensing Service (ESS)
#define UUID_ESS 0x181A

// Characteristics
#define UUID_TEMPERATURE 0x2A6E
#define UUID_HUMIDITY 0x2A6F
#define UUID_PRESSURE 0x2A6D

typedef struct {
    uint16_t service_handle;
    uint16_t temp_handle;
    uint16_t humidity_handle;
    uint16_t pressure_handle;
} ESS_Service;

void ess_init(ESS_Service* ess) {
    // Create service
    ble_gatts_add_service(UUID_ESS, &ess->service_handle);

    // Add temperature characteristic
    ble_gatts_add_char(ess->service_handle, UUID_TEMPERATURE,
                      BLE_GATT_CHAR_PROP_READ | BLE_GATT_CHAR_PROP_NOTIFY,
                      &ess->temp_handle);

    // Add humidity characteristic
    ble_gatts_add_char(ess->service_handle, UUID_HUMIDITY,
                      BLE_GATT_CHAR_PROP_READ | BLE_GATT_CHAR_PROP_NOTIFY,
                      &ess->humidity_handle);

    // Add pressure characteristic
    ble_gatts_add_char(ess->service_handle, UUID_PRESSURE,
                      BLE_GATT_CHAR_PROP_READ | BLE_GATT_CHAR_PROP_NOTIFY,
                      &ess->pressure_handle);
}

void ess_update_temperature(ESS_Service* ess, float temp_c) {
    // Convert to BLE format (temperature in 0.01°C units)
    int16_t temp_ble = (int16_t)(temp_c * 100);

    // Update characteristic
    ble_gatts_set_value(ess->temp_handle, (uint8_t*)&temp_ble, 2);

    // Send notification if client subscribed
    if (ble_gatts_is_subscribed(ess->temp_handle)) {
        ble_gatts_notify(ess->temp_handle);
    }
}
```

### BLE Power Optimization

**Connection Parameters:**

```cpp
// Power-optimized connection parameters
ble_gap_conn_params_t conn_params = {
    .min_conn_interval = 400,  // 400 × 1.25ms = 500ms
    .max_conn_interval = 400,
    .slave_latency = 4,        // Skip 4 intervals (total 2.5s)
    .conn_sup_timeout = 4000   // 40s timeout
};

// Power consumption:
// - High-rate (7.5ms interval): ~500 µA average
// - Power-optimized (above): ~20 µA average
// 25× reduction!
```

**Advertising Optimization:**

```cpp
// Beacon mode (non-connectable advertising)
ble_gap_adv_params_t adv_params = {
    .type = BLE_GAP_ADV_TYPE_NONCONN_IND,
    .interval = 1600,  // 1600 × 0.625ms = 1s
    .timeout = 0       // Infinite
};

// Advertising data (sensor readings in advertisement)
uint8_t adv_data[] = {
    0x02, 0x01, 0x06,  // Flags
    0x11, 0x16,        // Service Data (16-byte)
    0x1A, 0x18,        // ESS UUID
    0x12, 0x34,        // Temperature (0x3412 = 132.34°C in 0.01°C units)
    0x56, 0x78,        // Humidity
    0x9A, 0xBC, 0xDE, 0xF0  // Pressure
};

ble_gap_adv_set_data(adv_data, sizeof(adv_data));
ble_gap_adv_start(&adv_params);

// Power: ~10 µA (1s advertising interval)
```

### BLE Mesh for Sensor Networks

**Benefits:**
- Multi-hop routing (extended range)
- Self-healing network
- Large-scale deployments (1000+ nodes)

**Challenges:**
- Higher power consumption (relaying)
- Increased latency
- Complex provisioning

---

## LoRaWAN for Wide-Area Sensing

### Why LoRaWAN?

**Strengths:**
- Extreme range (2-15 km)
- Ultra-low power (10-year battery life)
- License-free spectrum
- Low cost (module < $5)

**Limitations:**
- Low data rate (0.3-50 kbps)
- Duty cycle restrictions (1% in EU)
- Latency (seconds to minutes)

### LoRaWAN Architecture

**Device Classes:**

**Class A (lowest power):**
- TX whenever needed
- RX windows only after TX
- Power: ~1-5 µA average
- Use: Battery sensors with uplink-only

**Class B (scheduled RX):**
- Periodic RX slots (beacon-synchronized)
- Power: ~10-50 µA average
- Use: Sensors needing occasional downlink

**Class C (always listening):**
- RX window always open
- Power: ~10-50 mA average
- Use: Mains-powered sensors, actuators

### LoRaWAN Message Flow

```cpp
#include "lorawan.h"

// Device configuration
const uint8_t DEV_EUI[8] = {0x00, 0x11, ...};  // Unique device ID
const uint8_t APP_EUI[8] = {0x70, 0xB3, ...};  // Application ID
const uint8_t APP_KEY[16] = {0x2B, 0x7E, ...}; // AES-128 key

void lorawan_init() {
    // OTAA (Over-The-Air Activation)
    lorawan_join_otaa(DEV_EUI, APP_EUI, APP_KEY);

    // Wait for join accept (can take 5-10 seconds)
    while (!lorawan_is_joined()) {
        delay_ms(100);
    }
}

void send_sensor_data() {
    // Read sensors
    float temp = read_temperature();
    float humidity = read_humidity();
    uint16_t battery_mv = read_battery_voltage();

    // Pack data (minimize payload size)
    uint8_t payload[6];
    payload[0] = (int16_t)(temp * 100) >> 8;      // Temp MSB
    payload[1] = (int16_t)(temp * 100) & 0xFF;    // Temp LSB
    payload[2] = (uint8_t)(humidity * 2);         // Humidity (0.5% resolution)
    payload[3] = battery_mv >> 8;                 // Battery MSB
    payload[4] = battery_mv & 0xFF;               // Battery LSB
    payload[5] = get_error_flags();               // Status byte

    // Send on port 1, unconfirmed
    lorawan_send(1, payload, sizeof(payload), false);
}

void lorawan_task() {
    lorawan_init();

    while (1) {
        send_sensor_data();

        // Sleep for 10 minutes (adaptive data rate will optimize SF)
        lorawan_sleep(10 * 60 * 1000);
    }
}
```

### Power Budget Example

**Transmission profile:**
- SF7, 125 kHz BW, 14 dBm TX power
- Payload: 6 bytes
- Airtime: ~56 ms

**Power breakdown:**
- TX: 40 mA × 56 ms = 2.24 mAs
- RX1 window: 15 mA × 20 ms = 0.3 mAs
- RX2 window: 15 mA × 20 ms = 0.3 mAs (if no downlink in RX1)
- Deep sleep: 5 µA × 600 s = 3 mAs
- **Total per cycle: 5.84 mAs**

**Battery life (2× AA, 3000 mAh):**
- 3000 mAh × 3600 / 5.84 = 1,849,315 cycles
- At 10 min/cycle: 12,840 days = **35 years**

(Limited to ~10 years by battery shelf life)

---

## MQTT for Sensor-to-Cloud

### Why MQTT?

**Advantages:**
- Lightweight protocol (2-byte header minimum)
- Publish/Subscribe pattern (decouples sensors and applications)
- QoS levels (reliability vs. overhead)
- Widely supported (AWS IoT, Azure IoT Hub, Google Cloud IoT)

**Typical Architecture:**

```
[Sensor] → [WiFi/BLE] → [Gateway] → [MQTT Broker] → [Cloud Apps]
                                        ↓
                                   [Database]
```

### MQTT Protocol Basics

**Topics:** Hierarchical message routing

```
sensors/building1/floor2/temp
sensors/building1/floor2/humidity
sensors/building1/floor2/co2
```

**QoS Levels:**
- **QoS 0** (At most once): Fire and forget, no acknowledgment
- **QoS 1** (At least once): Acknowledged, may duplicate
- **QoS 2** (Exactly once): Four-way handshake, guaranteed

**Retained Messages:** Last message persists for late subscribers

### MQTT on Embedded Devices

```cpp
#include "mqtt_client.h"

mqtt_client_t mqtt;
const char* broker = "mqtt.example.com";
const char* client_id = "sensor_001";

void mqtt_connect() {
    mqtt_connect_params_t params = {
        .broker = broker,
        .port = 1883,
        .client_id = client_id,
        .username = "sensor_user",
        .password = "secret",
        .keep_alive = 60,  // 60s heartbeat
        .clean_session = true
    };

    mqtt_connect(&mqtt, &params);
}

void mqtt_publish_sensor_data() {
    // JSON payload
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"temp\":%.2f,\"humidity\":%.1f,\"battery\":%d}",
             temperature, humidity, battery_percent);

    // Publish to topic
    mqtt_publish(&mqtt,
                "sensors/env/sensor_001",
                payload,
                strlen(payload),
                MQTT_QOS_1,      // QoS 1
                false);          // Not retained
}

void mqtt_task() {
    mqtt_connect();

    while (1) {
        // Publish every 60 seconds
        read_sensors();
        mqtt_publish_sensor_data();

        // Keep connection alive (ping if needed)
        mqtt_loop(&mqtt);

        delay_ms(60000);
    }
}
```

### Power Optimization for MQTT

**Keep-alive tuning:**

```cpp
// Long keep-alive = less overhead
mqtt_params.keep_alive = 300;  // 5 minutes (vs. 60s default)

// Power savings:
// - 60s keep-alive: Ping every 60s (2 bytes × 2 = 4 bytes + TCP overhead)
// - 300s keep-alive: Ping every 300s
// 5× reduction in keep-alive traffic
```

**Persistent sessions:**

```cpp
// Avoid reconnection overhead
mqtt_params.clean_session = false;

// Broker remembers subscriptions and QoS state
// Saves ~200 bytes and 100ms on reconnect
```

**Batching:**

```cpp
// Instead of publishing every minute (60 publishes/hour):
void publish_batch() {
    char payload[512];
    int len = 0;

    // Collect 10 minutes of data
    len += snprintf(payload + len, sizeof(payload) - len, "[");
    for (int i = 0; i < 10; i++) {
        read_sensors();
        len += snprintf(payload + len, sizeof(payload) - len,
                       "{\"t\":%lu,\"temp\":%.2f,\"hum\":%.1f}%s",
                       timestamp, temp, humidity,
                       (i < 9) ? "," : "");
        delay_ms(60000);
    }
    len += snprintf(payload + len, sizeof(payload) - len, "]");

    // Single publish (10 samples)
    mqtt_publish(&mqtt, topic, payload, len, MQTT_QOS_1, false);
}

// Reduction: 60 publishes/hour → 6 publishes/hour (10× less overhead)
```

---

## CoAP for Constrained Devices

### Why CoAP?

**Designed specifically for IoT:**
- UDP-based (less overhead than TCP)
- RESTful (GET, POST, PUT, DELETE)
- Compact binary format
- Built-in discovery
- Observability (subscribe to resources)

**Comparison:**

| Feature | HTTP | MQTT | CoAP |
|---------|------|------|------|
| Transport | TCP | TCP | UDP |
| Header | ~200 bytes | ~2 bytes | ~4 bytes |
| Pattern | Request/Response | Pub/Sub | Request/Response |
| Power | Medium | Low-Medium | Low |

### CoAP Example

```cpp
#include "coap.h"

// Resource handler
static void temp_handler(coap_context_t *ctx,
                        coap_resource_t *resource,
                        coap_session_t *session,
                        coap_pdu_t *request,
                        coap_binary_t *token,
                        coap_string_t *query,
                        coap_pdu_t *response) {
    // Read temperature
    float temp = read_temperature();

    // Format response (JSON)
    char payload[32];
    int len = snprintf(payload, sizeof(payload), "{\"temp\":%.2f}", temp);

    // Set response
    coap_add_data_blocked_response(resource, session, request, response,
                                   token, COAP_MEDIATYPE_APPLICATION_JSON, 0,
                                   len, (uint8_t*)payload);
}

void coap_server_init() {
    coap_context_t *ctx = coap_new_context(NULL);

    // Create resource: coap://sensor.local/temperature
    coap_resource_t *resource = coap_resource_init(
        coap_make_str_const("temperature"), 0);

    // Register GET handler
    coap_register_handler(resource, COAP_REQUEST_GET, temp_handler);

    // Make resource observable (clients can subscribe)
    coap_resource_set_get_observable(resource, 1);

    // Add to context
    coap_add_resource(ctx, resource);
}

void coap_notify_observers() {
    // When temperature changes significantly, notify observers
    coap_resource_notify_observers(temperature_resource, NULL);
}
```

**Client (observing temperature):**

```cpp
void coap_client_observe() {
    // Send OBSERVE request
    coap_pdu_t *pdu = coap_pdu_init(COAP_MESSAGE_CON,
                                   COAP_REQUEST_GET,
                                   coap_new_message_id(session),
                                   coap_session_max_pdu_size(session));

    // Add OBSERVE option (0 = register)
    coap_insert_option(pdu, COAP_OPTION_OBSERVE, 1, (uint8_t*)"\x00");

    // Add URI
    coap_add_option(pdu, COAP_OPTION_URI_PATH, 11, (uint8_t*)"temperature");

    // Send
    coap_send(session, pdu);

    // Now server will notify when temperature changes
    // No polling needed!
}
```

---

## Edge-to-Cloud Architectures

### Architecture Pattern 1: Direct Connection

```
[Sensor] → WiFi/Cellular → [Cloud]
```

**Pros:**
- Simple
- No gateway cost

**Cons:**
- High sensor power (WiFi/cellular)
- Cloud dependency

**Use case:** Mains-powered, high-bandwidth sensors

### Architecture Pattern 2: Gateway Hub

```
[Sensors] → BLE/Zigbee → [Gateway] → WiFi/Ethernet → [Cloud]
```

**Pros:**
- Low sensor power
- Local processing at gateway
- Multiple sensors per gateway

**Cons:**
- Gateway cost
- Gateway deployment

**Use case:** Battery sensors, large deployments

### Architecture Pattern 3: Mesh Network

```
[Sensors] ← mesh → [Border Router] → [Cloud]
```

**Pros:**
- Self-healing
- Extended range
- Scalable

**Cons:**
- Complex
- Higher power (routing)

**Use case:** Large-area coverage, difficult environments

### Architecture Pattern 4: Edge Computing

```
[Sensors] → [Edge Server] ⇄ [Cloud]
             (local ML,      (model updates,
              aggregation)    dashboards)
```

**Pros:**
- Low latency
- Bandwidth reduction
- Privacy (data stays local)
- Works offline

**Cons:**
- Edge server cost
- Management complexity

**Use case:** Real-time applications, privacy-sensitive

---

## Protocol Implementation Tips

### Minimize Connection Overhead

```cpp
// BAD: Connect/disconnect per message
for (int i = 0; i < 100; i++) {
    wifi_connect();
    http_post(data[i]);
    wifi_disconnect();
}
// Connection overhead: 100× × 2s = 200s

// GOOD: Persistent connection
wifi_connect();
for (int i = 0; i < 100; i++) {
    http_post(data[i]);
}
wifi_disconnect();
// Connection overhead: 1× × 2s = 2s (100× better!)
```

### Error Handling and Retry

```cpp
bool send_with_retry(uint8_t* data, int len, int max_retries) {
    for (int attempt = 0; attempt < max_retries; attempt++) {
        if (lorawan_send(data, len)) {
            return true;  // Success
        }

        // Exponential backoff
        int delay_ms = 1000 * (1 << attempt);  // 1s, 2s, 4s, 8s, ...
        delay_ms += rand() % 1000;  // Jitter to avoid collision

        sleep_ms(delay_ms);
    }

    return false;  // Failed after retries
}
```

### Data Compression

```cpp
// Instead of JSON (verbose):
// {"temp":23.45,"hum":67.8,"press":1013.2,"batt":87}
// 52 bytes

// Binary packed:
typedef struct __attribute__((packed)) {
    int16_t temp;     // 0.01°C resolution
    uint8_t humidity; // 0.5% resolution
    uint16_t pressure;// 0.1 hPa resolution
    uint8_t battery;  // 1% resolution
} SensorData;

SensorData data = {
    .temp = 2345,      // 23.45°C
    .humidity = 135,   // 67.5%
    .pressure = 10132, // 1013.2 hPa
    .battery = 87      // 87%
};

// 6 bytes (8.7× smaller!)
```

---

## Review Questions

1. **BLE Power Optimization**: Compare high-rate BLE connection (7.5ms interval, no latency) versus power-optimized (500ms interval, latency=4). Given ~500 µA for high-rate and ~20 µA for optimized, calculate the power reduction factor. Explain how slave latency works and why it enables such dramatic savings.

2. **LoRaWAN Battery Life Calculation**: A LoRaWAN sensor transmits 6 bytes every 10 minutes using SF7. Given: TX 2.24 mAs, RX1 0.3 mAs, RX2 0.3 mAs, sleep 3 mAs. Calculate total energy per cycle (5.84 mAs) and verify the 35-year battery life claim with 2× AA batteries (3000 mAh).

3. **MQTT Batching Benefits**: An MQTT sensor can publish every minute (60× per hour) or batch 10 samples and publish every 10 minutes (6× per hour). Calculate the overhead reduction factor and explain why batching is critical for power-constrained devices. What's the trade-off with real-time responsiveness?

4. **CoAP vs HTTP Overhead**: Compare header sizes: HTTP (~200 bytes), MQTT (~2 bytes), CoAP (~4 bytes). For a 10-byte sensor payload transmitted 1000 times/day, calculate total overhead bytes for each protocol. Why is UDP-based CoAP more suitable than TCP-based HTTP for constrained devices?

5. **Edge Architecture Selection**: Compare four architectures (direct connection, gateway hub, mesh network, edge computing) for a deployment of 100 battery-powered sensors requiring real-time ML processing. Which architecture would you choose and why? Consider power, latency, cost, and privacy.

6. **Data Compression Impact**: JSON sensor message is 52 bytes: {"temp":23.45,"hum":67.8,"press":1013.2,"batt":87}. The binary packed equivalent is 6 bytes. Calculate compression ratio (8.7×) and LoRaWAN airtime savings at SF7 (assume 50 bytes/s bitrate). Why is this critical for duty-cycle-limited protocols?

7. **Protocol Selection Matrix**: For an agricultural monitoring system with 5 km range, 10-year battery life, 15-minute update interval, and 50-byte payload, compare BLE, WiFi, and LoRaWAN. Calculate energy per transmission for each (assume: BLE 0.1 mJ, WiFi 50 mJ, LoRaWAN SF9 20.5 mJ) and recommend the optimal protocol.

## Key Takeaways

- **BLE Power Optimization**: Power-optimized connection parameters (500ms interval, slave latency=4) achieve **~20 µA average** versus **~500 µA for high-rate (7.5ms)**, delivering **25× power reduction**. Periodic advertising at 1s intervals consumes **~10 µA**, enabling months-to-years battery life.

- **LoRaWAN Ultra-Long Range**: LoRaWAN Class A provides **2-15 km range** with **10-year battery life** (2× AA) by combining ultra-low duty cycle (1 TX per 10 min = 5.84 mAs/cycle) with **1-5 µA deep sleep**. SF7 achieves **35-year theoretical life** (limited by shelf life).

- **MQTT Lightweight Pub/Sub**: MQTT-over-WiFi enables sensor-to-cloud with **~2-byte headers** (vs. ~200-byte HTTP), while 300s keep-alive and persistent sessions reduce overhead by **5×**. Batching 10 samples reduces publishes from **60/hour to 6/hour** (10× overhead reduction).

- **CoAP for Constrained Devices**: UDP-based CoAP with **~4-byte headers** and built-in observability (subscribe to resources) eliminates polling. RESTful design (GET/POST/PUT/DELETE) familiar to HTTP developers but optimized for ultra-low-power IoT with **10× lower overhead** than HTTP.

- **Edge Computing Architecture**: Local edge server processing reduces latency to **< 100 ms**, provides **1000× bandwidth reduction** (transmit decisions vs. raw data), enables **privacy-preserving local ML**, and maintains operation during cloud outages, at cost of edge infrastructure deployment.

- **Data Compression Criticality**: Binary packing reduces payload from **52 bytes JSON to 6 bytes** (**8.7× compression**), enabling LoRaWAN SF7 airtime reduction from ~450 ms to ~50 ms and satisfying **1% duty cycle** limits (EU) for more frequent transmissions.

- **Connection Overhead Elimination**: Persistent connections save **100× overhead** versus connect/disconnect per message (2s × 100 = 200s overhead reduced to 2s). Exponential backoff with jitter (1s, 2s, 4s, 8s + random) prevents retry collisions in dense deployments.

---

**Next Chapter**: Security, privacy, and OTA updates for smart sensors.

---

© 2025 SmileStory Inc. / WIA
弘익人間 (Hongik Ingan) · Benefit All Humanity
