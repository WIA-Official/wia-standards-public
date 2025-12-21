# Phase 3: Pet Care Robot Protocol Specification

## WIA-PET-CARE-ROBOT Protocol Standard

**Version**: 1.0.0
**Date**: 2025-12-18
**Status**: Draft
**Standard ID**: WIA-PET-CARE-ROBOT-PHASE3-001
**Primary Color**: #F59E0B (Amber)

---

## 1. Overview

### 1.1 Purpose

This specification defines the communication protocols, security mechanisms, and system integration standards for pet care robot ecosystems. It ensures reliable, secure, and interoperable communication between robots, cloud services, edge devices, and client applications.

**Protocol Objectives**:
- Define real-time communication channels
- Establish security and encryption standards
- Enable reliable command and control
- Support offline operation and synchronization
- Facilitate multi-robot coordination
- Ensure data integrity and privacy
- Enable cross-platform compatibility

### 1.2 Protocol Stack

| Layer | Protocol | Purpose |
|-------|----------|---------|
| **Application** | WIA-PET-CARE-ROBOT | Pet care operations and data |
| **Messaging** | MQTT 5.0, WebSocket | Real-time bidirectional communication |
| **Transport** | TLS 1.3, DTLS 1.3 | Secure data transmission |
| **Network** | IPv4, IPv6 | Network connectivity |
| **Physical** | WiFi, Ethernet, LTE | Device connectivity |

### 1.3 Communication Patterns

| Pattern | Use Case | Protocol | Latency |
|---------|----------|----------|---------|
| **Request-Response** | API operations | HTTPS/REST | <500ms |
| **Publish-Subscribe** | Status updates | MQTT | <100ms |
| **Streaming** | Video/Audio | WebRTC | <50ms |
| **Command-Control** | Robot operations | MQTT/CoAP | <200ms |
| **Batch Sync** | Historical data | HTTPS | Variable |

---

## 2. MQTT Protocol Implementation

### 2.1 Broker Configuration

```yaml
broker:
  version: "5.0"
  endpoints:
    - mqtt://mqtt.petcare.wia.org:1883
    - mqtts://mqtt.petcare.wia.org:8883
    - wss://mqtt.petcare.wia.org:443/mqtt

  authentication:
    method: "username_password"
    tls_client_cert: true
    oauth2: true

  qos_levels:
    - 0  # At most once (status updates)
    - 1  # At least once (commands)
    - 2  # Exactly once (critical operations)

  persistence:
    enabled: true
    type: "disk"
    retention: "7d"

  limits:
    max_packet_size: 268435456  # 256MB
    max_connections: 10000
    max_subscriptions_per_client: 100
```

### 2.2 Topic Structure

```
wia/pet-care/{organizationId}/{robotId}/{category}/{action}
```

**Topic Hierarchy:**

| Topic Pattern | Description | QoS | Retained |
|--------------|-------------|-----|----------|
| `wia/pet-care/{orgId}/{robotId}/status` | Robot status updates | 0 | Yes |
| `wia/pet-care/{orgId}/{robotId}/command` | Robot commands | 1 | No |
| `wia/pet-care/{orgId}/{robotId}/telemetry` | Sensor telemetry | 0 | No |
| `wia/pet-care/{orgId}/{robotId}/feeding/event` | Feeding events | 1 | No |
| `wia/pet-care/{orgId}/{robotId}/play/event` | Play events | 1 | No |
| `wia/pet-care/{orgId}/{robotId}/health/alert` | Health alerts | 2 | Yes |
| `wia/pet-care/{orgId}/{robotId}/error` | Error notifications | 2 | Yes |
| `wia/pet-care/{orgId}/broadcast` | Organization-wide messages | 1 | No |

### 2.3 MQTT Message Format

```json
{
  "version": "1.0",
  "messageId": "MSG-ABC123456789",
  "timestamp": "2025-12-18T10:00:00Z",
  "source": {
    "robotId": "PCR-ABC123456789",
    "deviceType": "multi_function",
    "firmwareVersion": "2.5.1"
  },
  "payload": {
    "type": "status_update",
    "data": {
      "operational": "online",
      "batteryLevel": 85,
      "foodLevel": 65,
      "waterLevel": 80
    }
  },
  "metadata": {
    "priority": "normal",
    "ttl": 300,
    "correlationId": "CORR-XYZ789"
  }
}
```

### 2.4 Command Message Structure

```json
{
  "version": "1.0",
  "commandId": "CMD-20251218-001",
  "timestamp": "2025-12-18T10:05:00Z",
  "source": {
    "clientId": "mobile-app-12345",
    "userId": "USER-ABC123"
  },
  "target": {
    "robotId": "PCR-ABC123456789"
  },
  "command": {
    "action": "dispense_food",
    "parameters": {
      "petId": "PET-DOG001",
      "portionSize": 100,
      "foodType": "dry_kibble"
    },
    "timeout": 30,
    "retryPolicy": {
      "maxRetries": 3,
      "retryDelay": 5
    }
  },
  "responseRequired": true,
  "responseTimeout": 10
}
```

### 2.5 Response Message Structure

```json
{
  "version": "1.0",
  "responseId": "RESP-20251218-001",
  "timestamp": "2025-12-18T10:05:08Z",
  "correlationId": "CMD-20251218-001",
  "source": {
    "robotId": "PCR-ABC123456789"
  },
  "result": {
    "status": "success",
    "code": 200,
    "message": "Food dispensed successfully",
    "data": {
      "eventId": "FEED-20251218-001",
      "dispensedAmount": 100,
      "remainingFood": 3150
    }
  },
  "executionTime": 8.2
}
```

### 2.6 MQTT Connection Example

```python
import paho.mqtt.client as mqtt
import json
import ssl
from datetime import datetime

class PetCareRobotMQTT:
    def __init__(self, broker, port, org_id, robot_id, username, password):
        self.broker = broker
        self.port = port
        self.org_id = org_id
        self.robot_id = robot_id
        self.client = mqtt.Client(
            client_id=f"{robot_id}",
            protocol=mqtt.MQTTv5
        )

        # Set credentials
        self.client.username_pw_set(username, password)

        # Configure TLS
        self.client.tls_set(
            ca_certs="/path/to/ca.crt",
            certfile="/path/to/client.crt",
            keyfile="/path/to/client.key",
            tls_version=ssl.PROTOCOL_TLSv1_2
        )

        # Set callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            print(f"Connected to MQTT broker")

            # Subscribe to command topic
            command_topic = f"wia/pet-care/{self.org_id}/{self.robot_id}/command"
            client.subscribe(command_topic, qos=1)

            # Subscribe to broadcast topic
            broadcast_topic = f"wia/pet-care/{self.org_id}/broadcast"
            client.subscribe(broadcast_topic, qos=1)

            # Publish online status
            self.publish_status("online")
        else:
            print(f"Connection failed with code {rc}")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())

            if "command" in payload:
                self.handle_command(payload)
            elif "broadcast" in payload:
                self.handle_broadcast(payload)

        except json.JSONDecodeError as e:
            print(f"Failed to decode message: {e}")

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print(f"Unexpected disconnection. Reconnecting...")
            self.connect()

    def connect(self):
        self.client.connect(self.broker, self.port, keepalive=60)
        self.client.loop_start()

    def publish_status(self, operational_status):
        topic = f"wia/pet-care/{self.org_id}/{self.robot_id}/status"

        message = {
            "version": "1.0",
            "messageId": f"MSG-{datetime.now().timestamp()}",
            "timestamp": datetime.now().isoformat(),
            "source": {
                "robotId": self.robot_id,
                "deviceType": "multi_function"
            },
            "payload": {
                "type": "status_update",
                "data": {
                    "operational": operational_status,
                    "batteryLevel": 85,
                    "foodLevel": 65,
                    "waterLevel": 80
                }
            }
        }

        self.client.publish(
            topic,
            payload=json.dumps(message),
            qos=0,
            retain=True
        )

    def publish_feeding_event(self, event_data):
        topic = f"wia/pet-care/{self.org_id}/{self.robot_id}/feeding/event"

        message = {
            "version": "1.0",
            "messageId": f"MSG-{datetime.now().timestamp()}",
            "timestamp": datetime.now().isoformat(),
            "source": {"robotId": self.robot_id},
            "payload": {
                "type": "feeding_event",
                "data": event_data
            }
        }

        self.client.publish(topic, payload=json.dumps(message), qos=1)

    def handle_command(self, payload):
        command = payload.get("command", {})
        action = command.get("action")
        parameters = command.get("parameters", {})

        # Execute command
        if action == "dispense_food":
            result = self.dispense_food(parameters)
        elif action == "start_play":
            result = self.start_play_session(parameters)
        else:
            result = {"status": "error", "message": "Unknown command"}

        # Send response
        self.send_response(payload["commandId"], result)

    def send_response(self, command_id, result):
        topic = f"wia/pet-care/{self.org_id}/{self.robot_id}/response"

        response = {
            "version": "1.0",
            "responseId": f"RESP-{datetime.now().timestamp()}",
            "timestamp": datetime.now().isoformat(),
            "correlationId": command_id,
            "source": {"robotId": self.robot_id},
            "result": result
        }

        self.client.publish(topic, payload=json.dumps(response), qos=1)

    def dispense_food(self, parameters):
        # Simulate food dispensing
        return {
            "status": "success",
            "code": 200,
            "message": "Food dispensed successfully",
            "data": {
                "dispensedAmount": parameters.get("portionSize"),
                "remainingFood": 3150
            }
        }

    def start_play_session(self, parameters):
        # Simulate play session start
        return {
            "status": "success",
            "code": 200,
            "message": "Play session started",
            "data": {
                "sessionId": f"PLAY-{datetime.now().timestamp()}"
            }
        }

# Usage
robot = PetCareRobotMQTT(
    broker="mqtt.petcare.wia.org",
    port=8883,
    org_id="ORG-001",
    robot_id="PCR-ABC123456789",
    username="robot-user",
    password="secure-password"
)

robot.connect()
```

---

## 3. WebSocket Protocol

### 3.1 WebSocket Handshake

```http
GET /ws/v1/robots/PCR-ABC123456789 HTTP/1.1
Host: ws.petcare.wia.org
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response:**
```http
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=
```

### 3.2 WebSocket Message Types

| Message Type | Direction | Purpose | Frequency |
|-------------|-----------|---------|-----------|
| `ping` | Bidirectional | Keep-alive | Every 30s |
| `pong` | Response | Keep-alive response | On ping |
| `subscribe` | Client → Server | Subscribe to events | On connect |
| `unsubscribe` | Client → Server | Unsubscribe from events | As needed |
| `command` | Client → Server | Control commands | On demand |
| `event` | Server → Client | Real-time events | As they occur |
| `status` | Server → Client | Status updates | Every 5s |
| `error` | Server → Client | Error notifications | On error |

### 3.3 WebSocket Frame Format

```json
{
  "type": "event",
  "id": "WS-MSG-001",
  "timestamp": "2025-12-18T10:10:00Z",
  "event": "feeding.completed",
  "data": {
    "eventId": "FEED-20251218-001",
    "robotId": "PCR-ABC123456789",
    "petId": "PET-DOG001",
    "portionSize": 150,
    "dispensed": true
  }
}
```

### 3.4 WebSocket Client Implementation

```javascript
class PetCareWebSocket {
  constructor(url, token) {
    this.url = url;
    this.token = token;
    this.ws = null;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 5;
    this.reconnectDelay = 1000;
    this.handlers = new Map();
  }

  connect() {
    this.ws = new WebSocket(`${this.url}?token=${this.token}`);

    this.ws.onopen = () => {
      console.log('WebSocket connected');
      this.reconnectAttempts = 0;
      this.startHeartbeat();

      // Subscribe to events
      this.subscribe(['feeding.*', 'play.*', 'health.*']);
    };

    this.ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        this.handleMessage(message);
      } catch (error) {
        console.error('Failed to parse message:', error);
      }
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    this.ws.onclose = () => {
      console.log('WebSocket disconnected');
      this.stopHeartbeat();
      this.reconnect();
    };
  }

  reconnect() {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      const delay = this.reconnectDelay * Math.pow(2, this.reconnectAttempts - 1);

      console.log(`Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts})`);

      setTimeout(() => this.connect(), delay);
    } else {
      console.error('Max reconnection attempts reached');
    }
  }

  startHeartbeat() {
    this.heartbeatInterval = setInterval(() => {
      if (this.ws.readyState === WebSocket.OPEN) {
        this.send({ type: 'ping' });
      }
    }, 30000);
  }

  stopHeartbeat() {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
    }
  }

  send(message) {
    if (this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    } else {
      console.warn('WebSocket not connected');
    }
  }

  subscribe(events) {
    this.send({
      type: 'subscribe',
      events: events
    });
  }

  unsubscribe(events) {
    this.send({
      type: 'unsubscribe',
      events: events
    });
  }

  sendCommand(robotId, action, parameters) {
    this.send({
      type: 'command',
      id: `CMD-${Date.now()}`,
      timestamp: new Date().toISOString(),
      robotId: robotId,
      action: action,
      parameters: parameters
    });
  }

  on(eventType, handler) {
    if (!this.handlers.has(eventType)) {
      this.handlers.set(eventType, []);
    }
    this.handlers.get(eventType).push(handler);
  }

  handleMessage(message) {
    const { type, event } = message;

    // Handle by message type
    if (this.handlers.has(type)) {
      this.handlers.get(type).forEach(handler => handler(message));
    }

    // Handle by event type (for event messages)
    if (event && this.handlers.has(event)) {
      this.handlers.get(event).forEach(handler => handler(message));
    }
  }

  disconnect() {
    if (this.ws) {
      this.stopHeartbeat();
      this.ws.close();
    }
  }
}

// Usage
const ws = new PetCareWebSocket(
  'wss://ws.petcare.wia.org/v1/robots/PCR-ABC123456789',
  'your-auth-token'
);

ws.on('feeding.completed', (message) => {
  console.log('Feeding completed:', message.data);
});

ws.on('health.alert', (message) => {
  console.log('Health alert:', message.data);
  // Show notification to user
});

ws.connect();

// Send a command
ws.sendCommand('PCR-ABC123456789', 'dispense_food', {
  petId: 'PET-DOG001',
  portionSize: 100
});
```

---

## 4. Security Protocols

### 4.1 Authentication Methods

| Method | Use Case | Security Level | Recommended For |
|--------|----------|----------------|-----------------|
| **OAuth 2.0** | User applications | High | Mobile apps, web apps |
| **API Keys** | Service-to-service | Medium | Backend integrations |
| **Client Certificates** | Device authentication | Very High | Robot devices |
| **JWT Tokens** | Session management | High | All API access |
| **HMAC Signatures** | Webhook verification | High | Webhook endpoints |

### 4.2 TLS Configuration

```nginx
# NGINX TLS Configuration
ssl_protocols TLSv1.2 TLSv1.3;
ssl_ciphers 'ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-RSA-AES128-GCM-SHA256:ECDHE-ECDSA-AES256-GCM-SHA384:ECDHE-RSA-AES256-GCM-SHA384';
ssl_prefer_server_ciphers on;
ssl_session_cache shared:SSL:10m;
ssl_session_timeout 10m;
ssl_stapling on;
ssl_stapling_verify on;

# Client certificate verification
ssl_client_certificate /path/to/ca.crt;
ssl_verify_client optional;
ssl_verify_depth 2;
```

### 4.3 Message Encryption

**End-to-End Encryption for Sensitive Data:**

```python
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from cryptography.hazmat.backends import default_backend
import os
import json

class SecureMessageHandler:
    def __init__(self, private_key, peer_public_key):
        self.private_key = private_key
        self.peer_public_key = peer_public_key

    def encrypt_message(self, plaintext_data):
        """Encrypt message using AES-256-GCM with RSA key exchange"""
        # Generate random AES key
        aes_key = os.urandom(32)  # 256 bits
        iv = os.urandom(12)  # 96 bits for GCM

        # Encrypt data with AES-GCM
        cipher = Cipher(
            algorithms.AES(aes_key),
            modes.GCM(iv),
            backend=default_backend()
        )
        encryptor = cipher.encryptor()

        plaintext = json.dumps(plaintext_data).encode()
        ciphertext = encryptor.update(plaintext) + encryptor.finalize()

        # Encrypt AES key with recipient's RSA public key
        encrypted_key = self.peer_public_key.encrypt(
            aes_key,
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        )

        return {
            "version": "1.0",
            "encrypted_key": encrypted_key.hex(),
            "iv": iv.hex(),
            "ciphertext": ciphertext.hex(),
            "tag": encryptor.tag.hex()
        }

    def decrypt_message(self, encrypted_message):
        """Decrypt received message"""
        # Decrypt AES key with private RSA key
        encrypted_key = bytes.fromhex(encrypted_message["encrypted_key"])
        aes_key = self.private_key.decrypt(
            encrypted_key,
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        )

        # Decrypt data with AES-GCM
        iv = bytes.fromhex(encrypted_message["iv"])
        ciphertext = bytes.fromhex(encrypted_message["ciphertext"])
        tag = bytes.fromhex(encrypted_message["tag"])

        cipher = Cipher(
            algorithms.AES(aes_key),
            modes.GCM(iv, tag),
            backend=default_backend()
        )
        decryptor = cipher.decryptor()

        plaintext = decryptor.update(ciphertext) + decryptor.finalize()

        return json.loads(plaintext.decode())

# Generate keys
private_key = rsa.generate_private_key(
    public_exponent=65537,
    key_size=2048,
    backend=default_backend()
)
public_key = private_key.public_key()

# Usage
handler = SecureMessageHandler(private_key, public_key)
encrypted = handler.encrypt_message({"command": "dispense_food", "amount": 100})
decrypted = handler.decrypt_message(encrypted)
```

### 4.4 API Request Signing

```python
import hmac
import hashlib
import time
from datetime import datetime

class RequestSigner:
    def __init__(self, api_key, api_secret):
        self.api_key = api_key
        self.api_secret = api_secret

    def sign_request(self, method, path, body=None):
        """Generate HMAC signature for API request"""
        timestamp = str(int(time.time()))
        nonce = os.urandom(16).hex()

        # Create signature base string
        parts = [
            method.upper(),
            path,
            timestamp,
            nonce
        ]

        if body:
            body_str = json.dumps(body, sort_keys=True)
            body_hash = hashlib.sha256(body_str.encode()).hexdigest()
            parts.append(body_hash)

        signature_base = '\n'.join(parts)

        # Generate HMAC signature
        signature = hmac.new(
            self.api_secret.encode(),
            signature_base.encode(),
            hashlib.sha256
        ).hexdigest()

        return {
            "X-API-Key": self.api_key,
            "X-Timestamp": timestamp,
            "X-Nonce": nonce,
            "X-Signature": signature
        }

    def verify_signature(self, method, path, headers, body=None):
        """Verify incoming request signature"""
        timestamp = headers.get("X-Timestamp")
        nonce = headers.get("X-Nonce")
        received_signature = headers.get("X-Signature")

        # Check timestamp (reject if > 5 minutes old)
        if abs(int(time.time()) - int(timestamp)) > 300:
            return False

        # Recreate signature
        parts = [method.upper(), path, timestamp, nonce]

        if body:
            body_str = json.dumps(body, sort_keys=True)
            body_hash = hashlib.sha256(body_str.encode()).hexdigest()
            parts.append(body_hash)

        signature_base = '\n'.join(parts)

        expected_signature = hmac.new(
            self.api_secret.encode(),
            signature_base.encode(),
            hashlib.sha256
        ).hexdigest()

        return hmac.compare_digest(expected_signature, received_signature)

# Usage
signer = RequestSigner("api-key-123", "api-secret-456")
headers = signer.sign_request("POST", "/api/v1/feeding/dispense", {"amount": 100})
```

---

## 5. Offline Operation Protocol

### 5.1 Local Storage Schema

```json
{
  "version": "1.0",
  "lastSync": "2025-12-18T10:00:00Z",
  "pendingOperations": [
    {
      "operationId": "OP-LOCAL-001",
      "timestamp": "2025-12-18T10:05:00Z",
      "type": "feeding",
      "status": "pending_sync",
      "data": {
        "eventId": "FEED-LOCAL-001",
        "petId": "PET-DOG001",
        "portionSize": 150,
        "dispensed": true
      },
      "retryCount": 0,
      "maxRetries": 3
    }
  ],
  "scheduledEvents": [
    {
      "scheduleId": "SCHED-FEED-001",
      "nextExecution": "2025-12-18T18:00:00Z",
      "action": "dispense_food",
      "parameters": {
        "petId": "PET-DOG001",
        "portionSize": 150
      }
    }
  ],
  "cachedData": {
    "petProfiles": [],
    "configuration": {}
  }
}
```

### 5.2 Synchronization Protocol

| Phase | Action | Direction | Priority |
|-------|--------|-----------|----------|
| **1. Connect** | Establish connection | Robot → Cloud | - |
| **2. Auth** | Authenticate device | Robot → Cloud | Critical |
| **3. Time Sync** | Synchronize clock | Cloud → Robot | High |
| **4. Config Pull** | Fetch updated configuration | Cloud → Robot | High |
| **5. Upload Pending** | Upload offline operations | Robot → Cloud | Medium |
| **6. Download Events** | Fetch missed events | Cloud → Robot | Medium |
| **7. Conflict Resolution** | Resolve data conflicts | Bidirectional | High |
| **8. Heartbeat** | Maintain connection | Bidirectional | Low |

### 5.3 Conflict Resolution

```python
class ConflictResolver:
    def __init__(self):
        self.resolution_strategies = {
            'feeding': 'server_wins',
            'schedule': 'merge',
            'configuration': 'server_wins',
            'health_observation': 'keep_both'
        }

    def resolve_conflict(self, local_data, server_data, data_type):
        strategy = self.resolution_strategies.get(data_type, 'server_wins')

        if strategy == 'server_wins':
            return server_data

        elif strategy == 'client_wins':
            return local_data

        elif strategy == 'merge':
            return self.merge_data(local_data, server_data)

        elif strategy == 'keep_both':
            return {
                'local': local_data,
                'server': server_data,
                'requires_manual_resolution': True
            }

        elif strategy == 'latest_timestamp':
            local_ts = local_data.get('timestamp', '')
            server_ts = server_data.get('timestamp', '')
            return local_data if local_ts > server_ts else server_data

    def merge_data(self, local_data, server_data):
        """Merge two data objects, preferring non-null server values"""
        merged = local_data.copy()

        for key, value in server_data.items():
            if value is not None:
                merged[key] = value

        return merged

# Usage
resolver = ConflictResolver()
resolved = resolver.resolve_conflict(
    local_data={'scheduleId': 'SCHED-001', 'enabled': False},
    server_data={'scheduleId': 'SCHED-001', 'enabled': True},
    data_type='schedule'
)
```

---

## 6. Multi-Robot Coordination

### 6.1 Coordination Scenarios

| Scenario | Challenge | Solution | Protocol |
|----------|-----------|----------|----------|
| **Shared Resources** | Multiple robots accessing same pet | Token-based locking | MQTT with QoS 2 |
| **Failover** | Primary robot offline | Backup robot takes over | Heartbeat monitoring |
| **Load Balancing** | Distribute tasks efficiently | Central coordinator | REST API |
| **Synchronized Actions** | Coordinated multi-robot tasks | Event synchronization | MQTT broadcast |
| **Data Consistency** | Consistent pet data across robots | Eventual consistency | Sync protocol |

### 6.2 Resource Locking Protocol

```typescript
interface ResourceLock {
  lockId: string;
  resourceType: 'pet' | 'location' | 'device';
  resourceId: string;
  acquiredBy: string;
  acquiredAt: string;
  expiresAt: string;
  renewable: boolean;
}

class ResourceLockManager {
  private locks: Map<string, ResourceLock>;
  private lockTTL: number = 300; // 5 minutes

  constructor() {
    this.locks = new Map();
  }

  async acquireLock(
    resourceType: string,
    resourceId: string,
    robotId: string
  ): Promise<ResourceLock | null> {
    const lockKey = `${resourceType}:${resourceId}`;
    const existingLock = this.locks.get(lockKey);

    // Check if lock exists and is valid
    if (existingLock) {
      const now = new Date();
      const expiresAt = new Date(existingLock.expiresAt);

      if (expiresAt > now && existingLock.acquiredBy !== robotId) {
        // Lock is held by another robot
        return null;
      }
    }

    // Create new lock
    const lock: ResourceLock = {
      lockId: `LOCK-${Date.now()}`,
      resourceType,
      resourceId,
      acquiredBy: robotId,
      acquiredAt: new Date().toISOString(),
      expiresAt: new Date(Date.now() + this.lockTTL * 1000).toISOString(),
      renewable: true
    };

    this.locks.set(lockKey, lock);

    // Publish lock acquisition
    this.publishLockEvent('acquired', lock);

    return lock;
  }

  async releaseLock(lockId: string, robotId: string): Promise<boolean> {
    for (const [key, lock] of this.locks.entries()) {
      if (lock.lockId === lockId) {
        if (lock.acquiredBy !== robotId) {
          return false; // Can't release someone else's lock
        }

        this.locks.delete(key);
        this.publishLockEvent('released', lock);
        return true;
      }
    }

    return false;
  }

  async renewLock(lockId: string, robotId: string): Promise<boolean> {
    for (const [key, lock] of this.locks.entries()) {
      if (lock.lockId === lockId && lock.acquiredBy === robotId) {
        lock.expiresAt = new Date(Date.now() + this.lockTTL * 1000).toISOString();
        this.locks.set(key, lock);
        return true;
      }
    }

    return false;
  }

  private publishLockEvent(event: string, lock: ResourceLock): void {
    const message = {
      event: `lock.${event}`,
      timestamp: new Date().toISOString(),
      data: lock
    };

    // Publish to MQTT topic
    // Implementation depends on MQTT client
  }

  // Cleanup expired locks
  cleanupExpiredLocks(): void {
    const now = new Date();

    for (const [key, lock] of this.locks.entries()) {
      const expiresAt = new Date(lock.expiresAt);

      if (expiresAt < now) {
        this.locks.delete(key);
        this.publishLockEvent('expired', lock);
      }
    }
  }
}

// Usage
const lockManager = new ResourceLockManager();

// Robot 1 tries to feed pet
const lock = await lockManager.acquireLock('pet', 'PET-DOG001', 'PCR-ROBOT001');

if (lock) {
  // Perform feeding operation
  await performFeeding();

  // Release lock
  await lockManager.releaseLock(lock.lockId, 'PCR-ROBOT001');
} else {
  console.log('Pet is being handled by another robot');
}
```

### 6.3 Failover Protocol

```go
package coordination

import (
    "context"
    "time"
)

type RobotStatus struct {
    RobotID       string
    Status        string
    LastHeartbeat time.Time
    IsPrimary     bool
    BackupFor     []string
}

type FailoverCoordinator struct {
    robots          map[string]*RobotStatus
    heartbeatTimeout time.Duration
    checkInterval   time.Duration
}

func NewFailoverCoordinator() *FailoverCoordinator {
    return &FailoverCoordinator{
        robots:           make(map[string]*RobotStatus),
        heartbeatTimeout: 60 * time.Second,
        checkInterval:    10 * time.Second,
    }
}

func (fc *FailoverCoordinator) RegisterRobot(robotID string, isPrimary bool, backupFor []string) {
    fc.robots[robotID] = &RobotStatus{
        RobotID:       robotID,
        Status:        "online",
        LastHeartbeat: time.Now(),
        IsPrimary:     isPrimary,
        BackupFor:     backupFor,
    }
}

func (fc *FailoverCoordinator) UpdateHeartbeat(robotID string) {
    if robot, exists := fc.robots[robotID]; exists {
        robot.LastHeartbeat = time.Now()
        robot.Status = "online"
    }
}

func (fc *FailoverCoordinator) MonitorHealth(ctx context.Context) {
    ticker := time.NewTicker(fc.checkInterval)
    defer ticker.Stop()

    for {
        select {
        case <-ctx.Done():
            return
        case <-ticker.C:
            fc.checkRobotHealth()
        }
    }
}

func (fc *FailoverCoordinator) checkRobotHealth() {
    now := time.Now()

    for _, robot := range fc.robots {
        timeSinceHeartbeat := now.Sub(robot.LastHeartbeat)

        if timeSinceHeartbeat > fc.heartbeatTimeout {
            if robot.Status == "online" {
                robot.Status = "offline"

                // Trigger failover if this was a primary robot
                if robot.IsPrimary {
                    fc.triggerFailover(robot.RobotID)
                }
            }
        }
    }
}

func (fc *FailoverCoordinator) triggerFailover(failedRobotID string) {
    // Find backup robot
    for _, robot := range fc.robots {
        if robot.Status == "online" {
            for _, backupFor := range robot.BackupFor {
                if backupFor == failedRobotID {
                    // Promote backup to primary
                    fc.promoteBackup(robot.RobotID, failedRobotID)
                    return
                }
            }
        }
    }
}

func (fc *FailoverCoordinator) promoteBackup(backupRobotID, failedRobotID string) {
    // Send command to backup robot to take over
    // Implementation would use MQTT or API call

    // Update status
    if backup, exists := fc.robots[backupRobotID]; exists {
        backup.IsPrimary = true
    }
}
```

---

## 7. Data Synchronization Protocol

### 7.1 Sync Message Format

```json
{
  "syncId": "SYNC-20251218-001",
  "timestamp": "2025-12-18T10:30:00Z",
  "source": {
    "robotId": "PCR-ABC123456789",
    "lastSyncTimestamp": "2025-12-18T09:00:00Z"
  },
  "changesets": [
    {
      "entity": "feeding_event",
      "operation": "create",
      "data": {
        "eventId": "FEED-20251218-001",
        "timestamp": "2025-12-18T10:00:00Z",
        "petId": "PET-DOG001",
        "portionSize": 150,
        "dispensed": true
      },
      "checksum": "abc123def456"
    },
    {
      "entity": "robot_status",
      "operation": "update",
      "data": {
        "robotId": "PCR-ABC123456789",
        "foodLevel": 63
      },
      "checksum": "def789ghi012"
    }
  ],
  "requestedData": [
    {
      "entity": "pet_profile",
      "filter": {
        "updatedAfter": "2025-12-18T09:00:00Z"
      }
    },
    {
      "entity": "feeding_schedule",
      "filter": {
        "robotId": "PCR-ABC123456789"
      }
    }
  ]
}
```

### 7.2 Sync Response Format

```json
{
  "syncId": "SYNC-20251218-001",
  "timestamp": "2025-12-18T10:30:05Z",
  "status": "success",
  "processed": {
    "accepted": 2,
    "rejected": 0,
    "conflicts": 0
  },
  "responseData": [
    {
      "entity": "pet_profile",
      "records": [
        {
          "petId": "PET-DOG001",
          "name": "Max",
          "updatedAt": "2025-12-18T09:30:00Z"
        }
      ]
    },
    {
      "entity": "feeding_schedule",
      "records": [
        {
          "scheduleId": "SCHED-FEED-001",
          "enabled": true,
          "nextExecution": "2025-12-18T18:00:00Z"
        }
      ]
    }
  ],
  "nextSyncRecommended": "2025-12-18T10:35:00Z"
}
```

---

## 8. Protocol Compliance Testing

### 8.1 Test Scenarios

| Test Case | Description | Expected Outcome |
|-----------|-------------|------------------|
| **MQTT Connection** | Connect to broker with TLS | Successful authenticated connection |
| **Topic Subscribe** | Subscribe to command topic | Receive subscription acknowledgment |
| **Command Execution** | Send dispense command | Command executed, response received |
| **Offline Operation** | Disconnect, perform action, reconnect | Data synced successfully |
| **Failover** | Primary robot goes offline | Backup robot takes over within 60s |
| **Encryption** | Send encrypted message | Message decrypted successfully |
| **Rate Limiting** | Exceed rate limit | 429 error received |
| **WebSocket Reconnect** | Force disconnect | Auto-reconnect successful |

### 8.2 Performance Benchmarks

| Metric | Target | Maximum | Measurement |
|--------|--------|---------|-------------|
| **Command Latency** | <200ms | <500ms | Time from send to execution |
| **Event Delivery** | <100ms | <300ms | Time from event to notification |
| **Sync Duration** | <5s | <30s | Time to sync 100 records |
| **Connection Establishment** | <2s | <10s | Time to connect and authenticate |
| **Heartbeat Frequency** | 30s | 60s | Time between heartbeats |
| **Offline Storage** | 7 days | 30 days | Duration of offline operation |

---

**弘익人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
