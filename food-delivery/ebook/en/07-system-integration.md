# Chapter 7: System Integration

---

## 7.1 Overview

This chapter covers integration with external systems essential for food delivery operations: restaurant POS systems, payment gateways, mapping services, and IoT platforms.

Successful integration enables seamless data flow between your delivery platform and partner systems.

---

## 7.2 Restaurant POS Integration

### 7.2.1 Common POS Systems

**Major Platforms:**
- Toast (30% market share)
- Square (25%)
- Clover (15%)
- Aloha (10%)
- Others (20%)

**Integration Methods:**
1. **REST API**: Most modern POS systems
2. **Webhook**: Real-time order notifications
3. **ODATA**: Open Data Protocol
4. **FTP/SFTP**: Legacy systems (file exchange)

### 7.2.2 Order Flow Integration

**Restaurant → Delivery Platform:**

```typescript
// POS System sends new order to delivery platform
interface POSOrder {
  posOrderId: string;
  restaurantId: string;
  items: POSItem[];
  subtotal: number;
  tax: number;
  customer: {
    name: string;
    phone: string;
    address: string;
  };
  pickupTime: Date;
  specialInstructions?: string;
}

// Example webhook from POS to delivery platform
app.post('/webhooks/pos/order-ready', async (req, res) => {
  const { posOrderId, restaurantId, readyTime } = req.body;

  // Find corresponding delivery order
  const order = await Order.findOne({
    externalId: posOrderId,
    restaurantId: restaurantId
  });

  if (order) {
    // Update order status
    await order.updateStatus('ready');

    // Notify assigned driver
    await notifyDriver(order.driverId, {
      type: 'order_ready',
      orderId: order.id,
      restaurantName: order.restaurant.name,
      pickupAddress: order.pickupLocation.address
    });
  }

  res.json({ success: true });
});
```

**Delivery Platform → POS:**

```typescript
// Notify POS when driver arrives
async function notifyPOSDriverArrived(order: Order) {
  const posConfig = await getPOSConfig(order.restaurantId);

  await axios.post(posConfig.webhookUrl, {
    event: 'driver_arrived',
    orderId: order.externalId,
    driverName: order.driver.name,
    driverPhoto: order.driver.photo,
    estimatedPickupTime: new Date(Date.now() + 5 * 60000)  // +5 min
  }, {
    headers: {
      'Authorization': `Bearer ${posConfig.apiKey}`,
      'Content-Type': 'application/json'
    }
  });
}

// Notify POS when order picked up
async function notifyPOSPickedUp(order: Order) {
  const posConfig = await getPOSConfig(order.restaurantId);

  await axios.post(posConfig.webhookUrl, {
    event: 'order_picked_up',
    orderId: order.externalId,
    timestamp: new Date(),
    driverId: order.driverId,
    estimatedDeliveryTime: order.estimatedDelivery
  });
}

// Notify POS when delivery completed
async function notifyPOSDelivered(order: Order) {
  const posConfig = await getPOSConfig(order.restaurantId);

  await axios.post(posConfig.webhookUrl, {
    event: 'order_delivered',
    orderId: order.externalId,
    deliveryTime: order.actualDelivery,
    proofOfDelivery: {
      photo: order.photos[0].url,
      signature: order.signature,
      location: order.deliveryLocation
    }
  });
}
```

### 7.2.3 Menu Synchronization

**Sync restaurant menu from POS:**

```typescript
interface MenuItem {
  id: string;
  name: string;
  description: string;
  category: string;
  price: number;
  available: boolean;
  modifiers?: MenuModifier[];
  allergens?: string[];
  temperature: 'hot' | 'cold' | 'ambient' | 'frozen';
  prepTime: number;  // minutes
}

async function syncMenuFromPOS(restaurantId: string) {
  const posConfig = await getPOSConfig(restaurantId);

  // Fetch menu from POS
  const response = await axios.get(
    `${posConfig.apiUrl}/menu`,
    {
      headers: {
        'Authorization': `Bearer ${posConfig.apiKey}`
      }
    }
  );

  const posMenu = response.data.items;

  // Transform POS menu to our format
  const menuItems = posMenu.map(item => ({
    id: generateId(),
    externalId: item.id,
    restaurantId: restaurantId,
    name: item.name,
    description: item.description,
    category: item.category,
    price: item.price,
    available: item.available && item.in_stock,
    modifiers: transformModifiers(item.modifiers),
    temperature: inferTemperature(item.category, item.name),
    prepTime: item.prep_time || 15,  // Default 15 min
    syncedAt: new Date()
  }));

  // Bulk update menu
  await MenuItem.bulkWrite(
    menuItems.map(item => ({
      updateOne: {
        filter: { externalId: item.externalId, restaurantId: item.restaurantId },
        update: { $set: item },
        upsert: true
      }
    }))
  );

  console.log(`Synced ${menuItems.length} items for ${restaurantId}`);
}

// Run sync every 15 minutes
setInterval(() => {
  const activeRestaurants = await Restaurant.find({ status: 'active' });
  for (const restaurant of activeRestaurants) {
    await syncMenuFromPOS(restaurant.id);
  }
}, 15 * 60 * 1000);
```

---

## 7.3 Payment Gateway Integration

### 7.3.1 Supported Gateways

- **Stripe**: Most popular, best API
- **Square**: Integrated POS + payments
- **PayPal/Braintree**: Global reach
- **Adyen**: International

### 7.3.2 Payment Flow

**1. Authorization (Hold Funds):**

```typescript
import Stripe from 'stripe';
const stripe = new Stripe(process.env.STRIPE_SECRET_KEY);

async function authorizePayment(order: Order): Promise<string> {
  try {
    // Create payment intent (authorization)
    const paymentIntent = await stripe.paymentIntents.create({
      amount: order.total,  // Cents
      currency: 'usd',
      customer: order.customer.stripeCustomerId,
      payment_method: order.customer.defaultPaymentMethod,
      capture_method: 'manual',  // Don't capture yet
      description: `Order ${order.confirmationCode}`,
      metadata: {
        orderId: order.id,
        restaurantId: order.restaurantId,
        customerId: order.customerId
      }
    });

    // Confirm payment intent (authorize card)
    const confirmed = await stripe.paymentIntents.confirm(
      paymentIntent.id,
      { payment_method: order.customer.defaultPaymentMethod }
    );

    if (confirmed.status === 'requires_capture') {
      // Authorization successful
      await order.update({
        paymentIntentId: confirmed.id,
        paymentStatus: 'authorized'
      });

      return confirmed.id;
    } else {
      throw new Error(`Payment authorization failed: ${confirmed.status}`);
    }

  } catch (error) {
    console.error('Payment authorization error:', error);
    throw error;
  }
}

// 2. Capture (Charge Card)
async function capturePayment(order: Order): Promise<void> {
  try {
    // Capture the authorized payment
    const captured = await stripe.paymentIntents.capture(
      order.paymentIntentId
    );

    if (captured.status === 'succeeded') {
      await order.update({
        paymentStatus: 'captured',
        paidAt: new Date()
      });

      // Split payment to restaurant, driver, platform
      await splitPayment(order, captured.id);
    }

  } catch (error) {
    console.error('Payment capture error:', error);
    throw error;
  }
}

// 3. Split Payment (Settlement)
async function splitPayment(order: Order, chargeId: string) {
  const restaurantAmount = order.subtotal + order.tax;  // 90% of food
  const driverAmount = order.deliveryFee + order.tip;
  const platformAmount = order.serviceFee;

  // Transfer to restaurant
  await stripe.transfers.create({
    amount: restaurantAmount,
    currency: 'usd',
    destination: order.restaurant.stripeAccountId,
    source_transaction: chargeId,
    description: `Order ${order.confirmationCode}`,
    metadata: { orderId: order.id }
  });

  // Transfer to driver
  await stripe.transfers.create({
    amount: driverAmount,
    currency: 'usd',
    destination: order.driver.stripeAccountId,
    source_transaction: chargeId,
    description: `Delivery ${order.confirmationCode}`
  });

  // Platform keeps service fee (automatic)

  await order.update({
    settlementStatus: 'completed',
    settledAt: new Date()
  });
}

// 4. Refund
async function refundOrder(
  order: Order,
  reason: string,
  amount?: number
): Promise<void> {
  try {
    const refundAmount = amount || order.total;

    const refund = await stripe.refunds.create({
      payment_intent: order.paymentIntentId,
      amount: refundAmount,
      reason: 'requested_by_customer',
      metadata: {
        orderId: order.id,
        reason: reason
      }
    });

    await order.update({
      refundAmount: refundAmount,
      refundStatus: 'completed',
      refundReason: reason,
      refundedAt: new Date()
    });

  } catch (error) {
    console.error('Refund error:', error);
    throw error;
  }
}
```

---

## 7.4 Mapping Services Integration

### 7.4.1 Google Maps Platform

**Services Used:**
- Geocoding API: Address → Coordinates
- Reverse Geocoding: Coordinates → Address
- Directions API: Route calculation
- Distance Matrix API: Bulk distance calculations
- Roads API: Snap to roads
- Places API: Address autocomplete

**Implementation:**

```typescript
import { Client } from '@googlemaps/google-maps-services-js';
const mapsClient = new Client({});

// Geocode address
async function geocodeAddress(address: string) {
  try {
    const response = await mapsClient.geocode({
      params: {
        address: address,
        key: process.env.GOOGLE_MAPS_API_KEY
      }
    });

    if (response.data.results.length > 0) {
      const result = response.data.results[0];
      return {
        latitude: result.geometry.location.lat,
        longitude: result.geometry.location.lng,
        formattedAddress: result.formatted_address,
        placeId: result.place_id,
        verified: true
      };
    } else {
      return { verified: false };
    }

  } catch (error) {
    console.error('Geocoding error:', error);
    throw error;
  }
}

// Calculate route
async function calculateRoute(
  origin: { lat: number; lng: number },
  destination: { lat: number; lng: number },
  departureTime?: Date
) {
  try {
    const response = await mapsClient.directions({
      params: {
        origin: `${origin.lat},${origin.lng}`,
        destination: `${destination.lat},${destination.lng}`,
        mode: 'bicycling',  // bike, ebike, scooter
        departure_time: departureTime || new Date(),
        traffic_model: 'best_guess',
        key: process.env.GOOGLE_MAPS_API_KEY
      }
    });

    if (response.data.routes.length > 0) {
      const route = response.data.routes[0];
      const leg = route.legs[0];

      return {
        distance: leg.distance.value / 1000,  // meters → km
        duration: leg.duration.value / 60,     // seconds → minutes
        durationInTraffic: leg.duration_in_traffic?.value / 60,
        polyline: route.overview_polyline.points,
        steps: leg.steps.map(step => ({
          instruction: step.html_instructions.replace(/<[^>]*>/g, ''),
          distance: step.distance.value / 1000,
          duration: step.duration.value / 60
        }))
      };
    } else {
      throw new Error('No route found');
    }

  } catch (error) {
    console.error('Route calculation error:', error);
    throw error;
  }
}

// Distance matrix (bulk calculations)
async function calculateDistanceMatrix(
  origins: Location[],
  destinations: Location[]
) {
  try {
    const response = await mapsClient.distancematrix({
      params: {
        origins: origins.map(o => `${o.latitude},${o.longitude}`),
        destinations: destinations.map(d => `${d.latitude},${d.longitude}`),
        mode: 'bicycling',
        key: process.env.GOOGLE_MAPS_API_KEY
      }
    });

    // Parse matrix
    const matrix = [];
    for (let i = 0; i < origins.length; i++) {
      const row = [];
      for (let j = 0; j < destinations.length; j++) {
        const element = response.data.rows[i].elements[j];
        if (element.status === 'OK') {
          row.push({
            distance: element.distance.value / 1000,  // km
            duration: element.duration.value / 60     // minutes
          });
        } else {
          row.push(null);
        }
      }
      matrix.push(row);
    }

    return matrix;

  } catch (error) {
    console.error('Distance matrix error:', error);
    throw error;
  }
}
```

### 7.4.2 Mapbox Integration

**Alternative to Google Maps with similar functionality:**

```typescript
import MapboxClient from '@mapbox/mapbox-sdk/services/geocoding';
import MapboxDirections from '@mapbox/mapbox-sdk/services/directions';

const geocodingClient = MapboxClient({
  accessToken: process.env.MAPBOX_ACCESS_TOKEN
});

const directionsClient = MapboxDirections({
  accessToken: process.env.MAPBOX_ACCESS_TOKEN
});

// Geocode with Mapbox
async function mapboxGeocode(address: string) {
  const response = await geocodingClient
    .forwardGeocode({
      query: address,
      limit: 1
    })
    .send();

  if (response.body.features.length > 0) {
    const feature = response.body.features[0];
    return {
      latitude: feature.center[1],
      longitude: feature.center[0],
      formattedAddress: feature.place_name,
      verified: true
    };
  }
}

// Route with Mapbox
async function mapboxRoute(origin: Location, destination: Location) {
  const response = await directionsClient
    .getDirections({
      profile: 'cycling',
      waypoints: [
        { coordinates: [origin.longitude, origin.latitude] },
        { coordinates: [destination.longitude, destination.latitude] }
      ],
      geometries: 'geojson',
      overview: 'full'
    })
    .send();

  if (response.body.routes.length > 0) {
    const route = response.body.routes[0];
    return {
      distance: route.distance / 1000,  // meters → km
      duration: route.duration / 60,    // seconds → minutes
      geometry: route.geometry,
      steps: route.legs[0].steps.map(step => ({
        instruction: step.maneuver.instruction,
        distance: step.distance / 1000,
        duration: step.duration / 60
      }))
    };
  }
}
```

---

## 7.5 IoT Sensor Integration

### 7.5.1 Temperature Sensor Setup

**Supported Sensors:**
- Bluetooth Low Energy (BLE) sensors
- Wi-Fi enabled sensors
- Cellular IoT sensors

**Example: AWS IoT Core Integration:**

```typescript
import AWS from 'aws-sdk';
import mqtt from 'mqtt';

const iot = new AWS.Iot({ region: 'us-east-1' });
const iotData = new AWS.IotData({
  endpoint: process.env.AWS_IOT_ENDPOINT
});

// Register new sensor
async function registerSensor(sensorId: string, driverId: string) {
  // Create IoT thing
  await iot.createThing({
    thingName: sensorId,
    attributePayload: {
      attributes: {
        driverId: driverId,
        registeredAt: new Date().toISOString()
      }
    }
  }).promise();

  // Create certificate
  const cert = await iot.createKeysAndCertificate({
    setAsActive: true
  }).promise();

  // Attach policy to certificate
  await iot.attachPolicy({
    policyName: 'TemperatureSensorPolicy',
    target: cert.certificateArn
  }).promise();

  // Attach certificate to thing
  await iot.attachThingPrincipal({
    thingName: sensorId,
    principal: cert.certificateArn
  }).promise();

  return {
    certificateArn: cert.certificateArn,
    certificatePem: cert.certificatePem,
    privateKey: cert.keyPair.PrivateKey,
    publicKey: cert.keyPair.PublicKey
  };
}

// Subscribe to sensor data
function subscribeSensorData(callback: (reading: any) => void) {
  const client = mqtt.connect(`mqtts://${process.env.AWS_IOT_ENDPOINT}`, {
    clientId: `server_${Date.now()}`,
    cert: process.env.IOT_CERT,
    key: process.env.IOT_PRIVATE_KEY,
    ca: process.env.IOT_CA
  });

  client.on('connect', () => {
    console.log('Connected to AWS IoT');

    // Subscribe to all sensors
    client.subscribe('sensors/+/temperature', (err) => {
      if (err) console.error('Subscribe error:', err);
    });
  });

  client.on('message', (topic, message) => {
    // Parse temperature reading
    const reading = JSON.parse(message.toString());
    const sensorId = topic.split('/')[1];

    callback({
      sensorId,
      ...reading
    });
  });
}

// Process temperature reading
subscribeSensorData(async (reading) => {
  const {
    sensorId,
    temperature,
    humidity,
    battery,
    timestamp
  } = reading;

  // Find associated order
  const order = await Order.findOne({
    status: { $in: ['picked_up', 'in_transit'] },
    'driver.sensorId': sensorId
  });

  if (order) {
    // Store reading
    await TemperatureReading.create({
      orderId: order.id,
      driverId: order.driverId,
      sensorId: sensorId,
      temperature: temperature,
      humidity: humidity,
      batteryLevel: battery,
      timestamp: new Date(timestamp),
      location: order.driver.location
    });

    // Check for alerts
    const monitor = new TemperatureMonitor();
    const alert = monitor.check_temperature(
      reading,
      order.temperatureRequirement
    );

    if (alert !== AlertLevel.NONE) {
      await monitor.handle_alert(alert, reading, order.id);
    }
  }
});
```

### 7.5.2 Mobile App Sensor Integration

**For sensors that connect via Bluetooth to driver's phone:**

```typescript
// React Native example
import { BleManager } from 'react-native-ble-plx';

const bleManager = new BleManager();

// Scan for temperature sensors
function scanForSensors(onSensorFound: (sensor: Device) => void) {
  bleManager.startDeviceScan(
    ['180D'],  // Temperature service UUID
    null,
    (error, device) => {
      if (error) {
        console.error('Scan error:', error);
        return;
      }

      if (device && device.name?.includes('TempSensor')) {
        onSensorFound(device);
        bleManager.stopDeviceScan();
      }
    }
  );
}

// Connect to sensor
async function connectSensor(deviceId: string) {
  const device = await bleManager.connectToDevice(deviceId);
  await device.discoverAllServicesAndCharacteristics();

  // Subscribe to temperature notifications
  device.monitorCharacteristicForService(
    '180D',  // Service UUID
    '2A1C',  // Temperature characteristic UUID
    (error, characteristic) => {
      if (error) {
        console.error('Monitor error:', error);
        return;
      }

      if (characteristic?.value) {
        const temperature = parseTemperature(characteristic.value);

        // Send to server
        sendTemperatureReading({
          sensorId: deviceId,
          temperature: temperature,
          timestamp: new Date().toISOString()
        });
      }
    }
  );
}

// Parse BLE temperature data
function parseTemperature(base64Value: string): number {
  const buffer = Buffer.from(base64Value, 'base64');
  // Assuming 16-bit signed integer in little-endian
  const raw = buffer.readInt16LE(0);
  return raw / 100;  // Convert to Celsius
}

// Send reading to server
async function sendTemperatureReading(reading: any) {
  await fetch(`${API_URL}/temperature-readings`, {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${authToken}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify(reading)
  });
}
```

---

## 7.6 Third-Party Platform Aggregation

### 7.6.1 Multi-Platform Integration

**Aggregate orders from multiple delivery platforms:**

```typescript
// Unified order processor
class OrderAggregator {
  private platforms = {
    ubereats: new UberEatsAdapter(),
    doordash: new DoorDashAdapter(),
    grubhub: new GrubHubAdapter()
  };

  async processIncomingOrder(
    platform: string,
    externalOrder: any
  ): Promise<Order> {
    const adapter = this.platforms[platform];

    if (!adapter) {
      throw new Error(`Unknown platform: ${platform}`);
    }

    // Transform to our standard format
    const order = adapter.transform(externalOrder);

    // Create in our system
    const created = await Order.create({
      ...order,
      source: platform,
      externalId: externalOrder.id
    });

    // Notify restaurant
    await notifyRestaurant(created);

    return created;
  }

  async updateExternalOrder(
    order: Order,
    status: string
  ) {
    const adapter = this.platforms[order.source];

    if (adapter) {
      await adapter.updateStatus(order.externalId, status);
    }
  }
}

// Platform-specific adapters
class UberEatsAdapter {
  transform(externalOrder: any): Partial<Order> {
    return {
      restaurantId: externalOrder.store.id,
      items: externalOrder.eater_order.items.map(item => ({
        name: item.title,
        quantity: item.quantity,
        price: item.price
      })),
      deliveryLocation: {
        address: externalOrder.eater.delivery_address,
        latitude: externalOrder.eater.latitude,
        longitude: externalOrder.eater.longitude
      }
      // ... other fields
    };
  }

  async updateStatus(externalId: string, status: string) {
    // Call Uber Eats API to update status
    await axios.patch(
      `https://api.uber.com/v1/eats/orders/${externalId}`,
      { status: mapStatusToUberEats(status) },
      {
        headers: {
          'Authorization': `Bearer ${process.env.UBER_EATS_TOKEN}`
        }
      }
    );
  }
}
```

---

## 7.7 Summary

This chapter covered integration with:

1. **Restaurant POS**: Order sync, menu sync, status updates
2. **Payment Gateways**: Authorization, capture, split payments, refunds
3. **Mapping Services**: Geocoding, routing, distance calculations
4. **IoT Sensors**: Temperature monitoring via AWS IoT or Bluetooth
5. **Third-Party Platforms**: Multi-platform order aggregation

All integrations use:
- **Standard protocols**: REST, webhooks, MQTT
- **Error handling**: Retries, fallbacks, logging
- **Security**: API keys, certificates, encryption

---

**Next Chapter**: [Chapter 8: Implementation Guide →](08-implementation.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
