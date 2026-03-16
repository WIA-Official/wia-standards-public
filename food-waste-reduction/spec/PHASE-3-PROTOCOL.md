# WIA-AGRI-032: Food Waste Reduction Standard
## Phase 3: Protocol Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26

---

## 1. Overview

This specification defines communication protocols, security requirements, data synchronization patterns, and notification mechanisms for the Food Waste Reduction standard.

### 1.1 Protocol Stack

```
Application Layer:  WIA-AGRI-032 Food Waste Reduction
Transport Layer:    HTTPS, WebSocket, MQTT
Security Layer:     TLS 1.3, OAuth 2.0
Data Format:        JSON, XML, CSV
```

---

## 2. Communication Protocols

### 2.1 HTTP/HTTPS (RESTful API)

**Primary protocol for:**
- CRUD operations on food items
- Batch data imports
- Query and reporting
- Configuration management

**Requirements:**
- TLS 1.3 minimum
- HTTP/2 preferred
- Compression: gzip, brotli
- Keep-alive connections supported

**Example Request:**
```http
POST /api/v1/inventory/items HTTP/2
Host: api.wia-foodwaste.org
Authorization: Bearer wia_live_xxxxxxxxxxxxx
Content-Type: application/json
Accept-Encoding: gzip, br
Connection: keep-alive

{
  "facilityId": "FACILITY-2025-001",
  "product": {
    "name": "Fresh Milk",
    "quantity": 100
  }
}
```

### 2.2 WebSocket (Real-Time Updates)

**Use cases:**
- Live expiration alerts
- Real-time inventory updates
- Donation matching notifications
- Dashboard live metrics

**Connection URL:**
```
wss://api.wia-foodwaste.org/v1/stream?token=wia_live_xxx
```

**Protocol Flow:**
```javascript
// 1. Connect
const ws = new WebSocket('wss://api.wia-foodwaste.org/v1/stream?token=xxx');

// 2. Subscribe to topics
ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    topics: ['expiration_alerts', 'waste_events'],
    filters: {
      facilityId: 'FACILITY-2025-001',
      severity: ['high', 'critical']
    }
  }));
};

// 3. Receive messages
ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  console.log('Event:', message.event, message.data);
};

// 4. Heartbeat (every 30 seconds)
setInterval(() => {
  ws.send(JSON.stringify({ action: 'ping' }));
}, 30000);
```

**Message Format:**
```json
{
  "messageId": "MSG-20251226-001",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "event": "expiration_alert",
  "data": {
    "trackingId": "TRACK-20251226-001",
    "daysUntilExpiry": 1,
    "severity": "high"
  }
}
```

### 2.3 MQTT (IoT Device Integration)

**Use cases:**
- Smart refrigerator sensors
- Temperature monitoring devices
- Automated waste scales
- Barcode scanners

**Broker Configuration:**
```
Host: mqtt.wia-foodwaste.org
Port: 8883 (TLS), 1883 (non-TLS)
Protocol: MQTT 5.0
QoS Levels: 0, 1, 2 supported
```

**Topic Structure:**
```
wia/v1/{facilityId}/inventory/items
wia/v1/{facilityId}/waste/events
wia/v1/{facilityId}/temperature/alerts
wia/v1/{facilityId}/donations/requests
```

**Publish Example:**
```javascript
const mqtt = require('mqtt');
const client = mqtt.connect('mqtts://mqtt.wia-foodwaste.org:8883', {
  clientId: 'FACILITY-2025-001',
  username: 'wia_mqtt_user',
  password: 'wia_mqtt_pass',
  protocolVersion: 5
});

client.on('connect', () => {
  client.publish('wia/v1/FACILITY-2025-001/inventory/items', JSON.stringify({
    trackingId: 'TRACK-20251226-001',
    temperature: 4.5,
    timestamp: new Date().toISOString()
  }), { qos: 1 });
});
```

### 2.4 Webhook (Event-Driven Notifications)

**Supported Events:**
- `expiration_alert`: Item approaching expiration
- `waste_event`: Waste recorded
- `donation_confirmed`: Donation pickup confirmed
- `inventory_low`: Stock below threshold
- `temperature_alert`: Temperature out of range

**Webhook Signature Verification:**
```javascript
const crypto = require('crypto');

function verifyWebhook(payload, signature, secret) {
  const hmac = crypto.createHmac('sha256', secret);
  const digest = hmac.update(payload).digest('hex');
  return crypto.timingSafeEqual(
    Buffer.from(signature),
    Buffer.from('sha256=' + digest)
  );
}

// Express middleware
app.post('/webhooks/wia', (req, res) => {
  const signature = req.headers['x-wia-signature'];
  const payload = JSON.stringify(req.body);

  if (!verifyWebhook(payload, signature, webhookSecret)) {
    return res.status(401).send('Invalid signature');
  }

  // Process webhook
  handleWebhookEvent(req.body);
  res.status(200).send('OK');
});
```

---

## 3. Security Protocols

### 3.1 Transport Layer Security (TLS)

**Requirements:**
- TLS 1.3 (preferred)
- TLS 1.2 (minimum)
- Certificate validation required
- Perfect Forward Secrecy (PFS)

**Cipher Suites (Recommended):**
```
TLS_AES_128_GCM_SHA256
TLS_AES_256_GCM_SHA384
TLS_CHACHA20_POLY1305_SHA256
```

### 3.2 Authentication Protocols

#### API Key Authentication

**Format:**
```
Authorization: Bearer wia_{environment}_{random_32_chars}
```

**Environments:**
- `wia_live_`: Production
- `wia_test_`: Testing/Sandbox

**Scopes:**
- `inventory:read`: Read inventory data
- `inventory:write`: Modify inventory
- `waste:track`: Record waste events
- `donations:manage`: Create/manage donations
- `analytics:view`: Access analytics

#### OAuth 2.0 Flow

**Client Credentials Grant:**
```http
POST /oauth/token HTTP/1.1
Host: api.wia-foodwaste.org
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=your_client_id
&client_secret=your_client_secret
&scope=inventory:read inventory:write
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "inventory:read inventory:write"
}
```

**Authorization Code Flow (for user delegation):**
```
1. Redirect to: https://api.wia-foodwaste.org/oauth/authorize
   ?response_type=code
   &client_id=your_client_id
   &redirect_uri=https://your-app.com/callback
   &scope=inventory:read
   &state=random_state_string

2. User authorizes

3. Callback: https://your-app.com/callback?code=AUTH_CODE&state=random_state_string

4. Exchange code for token:
   POST /oauth/token
   grant_type=authorization_code
   &code=AUTH_CODE
   &client_id=your_client_id
   &client_secret=your_client_secret
   &redirect_uri=https://your-app.com/callback
```

### 3.3 Data Encryption

**At Rest:**
- AES-256-GCM for stored data
- Encrypted backups
- Key rotation every 90 days

**In Transit:**
- TLS 1.3 for all communications
- Certificate pinning for mobile apps
- End-to-end encryption for sensitive fields

**Field-Level Encryption:**
```json
{
  "facilityId": "FACILITY-2025-001",
  "contactEmail": {
    "encrypted": true,
    "value": "enc_AES256_xxxxxxxxxxxxx",
    "keyId": "key_20251226_001"
  }
}
```

---

## 4. Data Synchronization Protocols

### 4.1 Real-Time Sync (WebSocket)

**Push-based updates:**
```javascript
// Server pushes updates immediately
{
  "sync": "realtime",
  "event": "inventory_update",
  "trackingId": "TRACK-20251226-001",
  "changes": {
    "quantity": { "old": 100, "new": 80 },
    "status": { "old": "fresh", "new": "near_expiry" }
  }
}
```

### 4.2 Polling (HTTP)

**Client-initiated periodic checks:**
```http
GET /api/v1/inventory/changes?since=2025-12-26T10:30:00.000Z
```

**Response:**
```json
{
  "changes": [
    {
      "trackingId": "TRACK-20251226-001",
      "changeType": "update",
      "timestamp": "2025-12-26T10:35:00.000Z",
      "fields": ["quantity", "status"]
    }
  ],
  "nextPoll": "2025-12-26T10:40:00.000Z",
  "moreChanges": false
}
```

### 4.3 Batch Sync

**Periodic bulk synchronization:**
```http
POST /api/v1/sync/batch
```

**Request:**
```json
{
  "syncId": "SYNC-20251226-001",
  "lastSyncTimestamp": "2025-12-25T10:30:00.000Z",
  "operations": [
    {
      "operation": "create",
      "localId": "LOCAL-001",
      "data": { /* food item */ }
    },
    {
      "operation": "update",
      "trackingId": "TRACK-20251225-050",
      "changes": { "quantity": 50 }
    }
  ]
}
```

**Response:**
```json
{
  "syncId": "SYNC-20251226-001",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "results": [
    {
      "localId": "LOCAL-001",
      "trackingId": "TRACK-20251226-002",
      "status": "success"
    },
    {
      "trackingId": "TRACK-20251225-050",
      "status": "success"
    }
  ],
  "conflicts": [],
  "nextSyncTimestamp": "2025-12-26T10:30:00.000Z"
}
```

### 4.4 Conflict Resolution

**Last Write Wins (LWW):**
```json
{
  "conflict": {
    "trackingId": "TRACK-20251226-001",
    "field": "quantity",
    "serverValue": 80,
    "serverTimestamp": "2025-12-26T10:30:00.000Z",
    "clientValue": 75,
    "clientTimestamp": "2025-12-26T10:29:50.000Z",
    "resolution": "server_wins",
    "finalValue": 80
  }
}
```

---

## 5. Notification Protocols

### 5.1 Email Notifications

**SMTP Configuration:**
```
Host: smtp.wia-foodwaste.org
Port: 587 (STARTTLS)
Authentication: Required
```

**Email Template:**
```json
{
  "notificationId": "NOTIF-EMAIL-001",
  "type": "expiration_alert",
  "recipient": "manager@grocery.com",
  "subject": "⚠️ 15 Products Expiring Within 24 Hours",
  "priority": "high",
  "body": {
    "html": "<html>...</html>",
    "text": "Plain text version..."
  },
  "attachments": [
    {
      "filename": "expiring-products.pdf",
      "contentType": "application/pdf",
      "data": "base64_encoded_data"
    }
  ]
}
```

### 5.2 SMS Notifications

**Twilio Integration:**
```javascript
POST /api/v1/notifications/sms

{
  "recipient": "+1-555-0123",
  "message": "URGENT: 15 products expiring today. View: https://app.wia.org/alerts/001",
  "priority": "high"
}
```

### 5.3 Push Notifications

**Firebase Cloud Messaging (FCM):**
```json
{
  "to": "device_token_xxxxxxxxxxxxx",
  "notification": {
    "title": "Expiration Alert",
    "body": "15 products expiring within 24 hours",
    "icon": "https://cdn.wia.org/icons/expiration.png",
    "click_action": "OPEN_ALERTS"
  },
  "data": {
    "type": "expiration_alert",
    "count": 15,
    "urgency": "high",
    "url": "/alerts/001"
  },
  "priority": "high"
}
```

### 5.4 In-App Notifications

**WebSocket channel:**
```json
{
  "event": "notification",
  "notification": {
    "id": "NOTIF-20251226-001",
    "type": "expiration_alert",
    "severity": "high",
    "title": "Products Expiring Soon",
    "message": "15 products will expire within 24 hours",
    "timestamp": "2025-12-26T10:30:00.000Z",
    "actions": [
      {
        "label": "View Items",
        "action": "navigate",
        "url": "/inventory?filter=near_expiry"
      },
      {
        "label": "Create Donation",
        "action": "navigate",
        "url": "/donations/create"
      }
    ]
  }
}
```

---

## 6. Protocol Compliance

### 6.1 Data Retention

**Requirements:**
- Transaction data: 7 years minimum
- Analytics data: 3 years minimum
- Logs: 1 year minimum
- Backups: 90 days rolling

### 6.2 Audit Logging

**All operations must log:**
```json
{
  "auditLog": {
    "logId": "LOG-20251226-001",
    "timestamp": "2025-12-26T10:30:00.000Z",
    "actor": {
      "userId": "USER-001",
      "apiKey": "wia_live_xxx...xxx",
      "ipAddress": "192.168.1.100"
    },
    "action": "inventory.item.update",
    "resource": {
      "type": "food_item",
      "id": "TRACK-20251226-001"
    },
    "changes": {
      "quantity": { "from": 100, "to": 80 }
    },
    "result": "success"
  }
}
```

### 6.3 GDPR Compliance

**Right to be forgotten:**
```http
DELETE /api/v1/data/user/{userId}?confirm=true
```

**Data export:**
```http
GET /api/v1/data/export?userId={userId}&format=json
```

**Consent management:**
```json
{
  "consent": {
    "userId": "USER-001",
    "purposes": [
      {
        "purpose": "analytics",
        "granted": true,
        "timestamp": "2025-12-26T10:30:00.000Z"
      },
      {
        "purpose": "marketing",
        "granted": false,
        "timestamp": "2025-12-26T10:30:00.000Z"
      }
    ]
  }
}
```

---

## 7. Performance Requirements

### 7.1 Latency Targets

- API Response: p95 < 200ms, p99 < 500ms
- WebSocket Message Delivery: < 100ms
- MQTT Publish-to-Receive: < 50ms
- Batch Sync: < 5 seconds for 1000 items

### 7.2 Throughput

- API Requests: 10,000 req/sec per region
- WebSocket Connections: 100,000 concurrent
- MQTT Messages: 50,000 msg/sec
- Batch Operations: 100,000 items/minute

### 7.3 Availability

- Uptime: 99.95% SLA
- Planned Maintenance: < 4 hours/month
- Data Durability: 99.999999999% (11 nines)

---

**Next Phase:** [Phase 4: Integration](./PHASE-4-INTEGRATION.md)

---

© 2025 WIA Standards · MIT License
弘益人間 · Benefit All Humanity
