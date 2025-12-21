# WIA Cryo-Consent Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## Table of Contents

1. [Overview](#overview)
2. [Protocol Architecture](#protocol-architecture)
3. [Message Types](#message-types)
4. [Real-Time Notifications](#real-time-notifications)
5. [Event Streaming](#event-streaming)
6. [Security & Authentication](#security--authentication)
7. [Implementation Examples](#implementation-examples)
8. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Cryo-Consent Protocol Standard defines real-time communication protocols for consent change notifications, event streaming, and multi-party synchronization. This standard enables immediate notification of consent modifications, revocations, and legal status changes across all stakeholders in the cryopreservation ecosystem.

**Core Objectives**:
- Enable real-time consent change notifications to all authorized parties
- Support WebSocket and Server-Sent Events (SSE) for live updates
- Facilitate multi-party consent verification workflows
- Ensure secure, encrypted notification delivery
- Enable event replay and audit trail reconstruction
- Support high-availability and fault-tolerant architectures

### 1.2 Use Cases

| Use Case | Description | Stakeholders |
|----------|-------------|--------------|
| Consent Modification | Real-time notification of consent changes | Facilities, Legal, Proxies |
| Emergency Revocation | Immediate consent withdrawal notification | All Parties |
| Proxy Authorization | New proxy delegation alerts | Grantor, Facilities, Legal |
| Legal Status Change | Jurisdiction compliance updates | Legal, Compliance |
| Multi-Party Signing | Collaborative witnessing workflows | Witnesses, Notaries |
| Audit Monitoring | Real-time audit trail updates | Auditors, Regulators |

### 1.3 Protocol Stack

| Layer | Technology | Purpose |
|-------|-----------|---------|
| Application | JSON Messages | Consent event payloads |
| Transport | WebSocket / SSE / HTTP/2 | Real-time bidirectional communication |
| Security | TLS 1.3 / mTLS | Encryption and authentication |
| Infrastructure | Load Balancer / Message Queue | Scalability and reliability |

---

## Protocol Architecture

### 2.1 System Architecture

```
┌──────────────┐         ┌──────────────┐         ┌──────────────┐
│   Client A   │         │   Client B   │         │   Client C   │
│  (Grantor)   │         │ (Healthcare) │         │   (Legal)    │
└──────┬───────┘         └──────┬───────┘         └──────┬───────┘
       │                        │                        │
       │ WebSocket              │ WebSocket              │ SSE
       │                        │                        │
       └────────────────────────┼────────────────────────┘
                                │
                    ┌───────────▼───────────┐
                    │   WIA Consent Hub     │
                    │  - Connection Manager │
                    │  - Event Router       │
                    │  - Auth Gateway       │
                    └───────────┬───────────┘
                                │
                    ┌───────────▼───────────┐
                    │   Message Broker      │
                    │  (Kafka / RabbitMQ)   │
                    └───────────┬───────────┘
                                │
                ┌───────────────┼───────────────┐
                │               │               │
        ┌───────▼──────┐ ┌─────▼──────┐ ┌─────▼──────┐
        │   Consent    │ │   Legal    │ │ Blockchain │
        │   Service    │ │  Validation│ │   Anchor   │
        └──────────────┘ └────────────┘ └────────────┘
```

### 2.2 Connection Types

| Type | Protocol | Use Case | Latency |
|------|----------|----------|---------|
| WebSocket | WSS | Bidirectional real-time | <100ms |
| Server-Sent Events | HTTPS/SSE | Unidirectional streaming | <200ms |
| HTTP/2 Server Push | HTTPS | Push notifications | <300ms |
| Webhook | HTTPS POST | Asynchronous callbacks | Variable |

### 2.3 Quality of Service

| Metric | Target | Description |
|--------|--------|-------------|
| Latency (p95) | <500ms | 95th percentile message delivery |
| Throughput | 10,000 msg/sec | Messages per second per node |
| Availability | 99.95% | Service uptime |
| Connection Limit | 100,000 | Concurrent connections per instance |
| Message Retention | 7 days | Event replay capability |

---

## Message Types

### 3.1 Event Message Structure

All protocol messages follow this structure:

```json
{
  "eventId": "evt_550e8400-e29b-41d4-a716-446655440001",
  "eventType": "consent.modified",
  "timestamp": "2025-01-15T10:30:00.123Z",
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "version": 2,
  "actor": {
    "id": "PERSON-001",
    "type": "grantor"
  },
  "payload": {
    // Event-specific data
  },
  "meta": {
    "signature": "0xabcdef...",
    "hash": "sha256:...",
    "previousEventId": "evt_prev..."
  }
}
```

### 3.2 Event Types

| Event Type | Description | Priority |
|------------|-------------|----------|
| `consent.created` | New consent record created | NORMAL |
| `consent.modified` | Consent modified | HIGH |
| `consent.revoked` | Consent permanently revoked | CRITICAL |
| `consent.verified` | Third-party verification complete | NORMAL |
| `proxy.added` | New proxy authorized | HIGH |
| `proxy.revoked` | Proxy authorization removed | HIGH |
| `legal.witnessed` | Witness signature added | NORMAL |
| `legal.notarized` | Notarization complete | HIGH |
| `jurisdiction.updated` | Legal jurisdiction changed | NORMAL |
| `blockchain.anchored` | Blockchain anchoring complete | NORMAL |
| `compliance.alert` | Compliance issue detected | CRITICAL |
| `audit.requested` | Audit trail requested | NORMAL |

### 3.3 Event Priority Levels

| Priority | Delivery SLA | Retry Policy | Description |
|----------|--------------|--------------|-------------|
| CRITICAL | Immediate | 10 attempts, exponential backoff | Life-critical events |
| HIGH | <5 seconds | 5 attempts | Important status changes |
| NORMAL | <30 seconds | 3 attempts | Standard notifications |
| LOW | Best effort | 1 attempt | Informational updates |

---

## Real-Time Notifications

### 4.1 WebSocket Connection

#### 4.1.1 Connection Establishment

```typescript
const ws = new WebSocket('wss://events.wia.live/cryo-consent/v1/stream');

// Authentication
ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'eyJhbGciOiJSUzI1NiIs...',
    subscriptions: [
      'consent.modified',
      'consent.revoked',
      'proxy.added'
    ],
    filters: {
      consentId: '550e8400-e29b-41d4-a716-446655440001'
    }
  }));
};

// Handle authentication response
ws.onmessage = (event) => {
  const message = JSON.parse(event.data);

  if (message.type === 'auth.success') {
    console.log('Connected and authenticated');
  } else if (message.type === 'event') {
    handleConsentEvent(message.data);
  }
};

// Error handling
ws.onerror = (error) => {
  console.error('WebSocket error:', error);
};

// Reconnection logic
ws.onclose = (event) => {
  if (!event.wasClean) {
    setTimeout(reconnect, 5000);
  }
};
```

#### 4.1.2 Subscription Management

```json
{
  "type": "subscribe",
  "subscriptions": [
    {
      "eventType": "consent.modified",
      "filters": {
        "grantorId": "PERSON-001",
        "jurisdiction": ["US-CA"]
      }
    },
    {
      "eventType": "consent.revoked",
      "filters": {
        "facilityId": "FAC-001"
      }
    }
  ]
}
```

**Response:**
```json
{
  "type": "subscription.confirmed",
  "subscriptionId": "sub_1234567890",
  "subscriptions": [
    "consent.modified",
    "consent.revoked"
  ],
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 4.2 Server-Sent Events (SSE)

```typescript
const eventSource = new EventSource(
  'https://events.wia.live/cryo-consent/v1/stream/sse?' +
  'token=eyJhbGciOiJSUzI1NiIs...' +
  '&events=consent.modified,consent.revoked'
);

eventSource.addEventListener('consent.modified', (event) => {
  const data = JSON.parse(event.data);
  console.log('Consent modified:', data);
});

eventSource.addEventListener('consent.revoked', (event) => {
  const data = JSON.parse(event.data);
  console.log('Consent revoked:', data);
  // Trigger emergency procedures
  handleEmergencyRevocation(data);
});

eventSource.onerror = (error) => {
  console.error('SSE error:', error);
  // Automatic reconnection handled by browser
};
```

### 4.3 Webhook Notifications

#### 4.3.1 Webhook Registration

```http
POST /api/v1/webhooks
Content-Type: application/json
Authorization: Bearer eyJhbGciOiJSUzI1NiIs...

{
  "url": "https://facility.example.com/webhooks/consent",
  "events": [
    "consent.modified",
    "consent.revoked",
    "proxy.added"
  ],
  "filters": {
    "facilityId": "FAC-001"
  },
  "secret": "whsec_1234567890abcdef"
}
```

**Response:**
```json
{
  "webhookId": "wh_550e8400-e29b-41d4-a716-446655440001",
  "url": "https://facility.example.com/webhooks/consent",
  "events": ["consent.modified", "consent.revoked", "proxy.added"],
  "status": "active",
  "createdAt": "2025-01-15T10:30:00Z"
}
```

#### 4.3.2 Webhook Payload

```http
POST /webhooks/consent HTTP/1.1
Host: facility.example.com
Content-Type: application/json
X-WIA-Event-Type: consent.modified
X-WIA-Event-ID: evt_550e8400-e29b-41d4-a716-446655440001
X-WIA-Signature: sha256=a5b9c3d4e5f6...
X-WIA-Timestamp: 2025-01-15T10:30:00Z

{
  "eventId": "evt_550e8400-e29b-41d4-a716-446655440001",
  "eventType": "consent.modified",
  "timestamp": "2025-01-15T10:30:00Z",
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "version": 2,
  "actor": {
    "id": "PERSON-001",
    "type": "grantor"
  },
  "payload": {
    "modifications": {
      "consent.scope.research.categories": ["medical", "scientific", "commercial"]
    },
    "previousVersion": 1
  },
  "meta": {
    "signature": "0xabcdef...",
    "hash": "sha256:..."
  }
}
```

#### 4.3.3 Webhook Signature Verification

```python
import hmac
import hashlib

def verify_webhook_signature(
    payload: bytes,
    signature: str,
    secret: str,
    timestamp: str
) -> bool:
    """
    Verify webhook signature using HMAC-SHA256.

    Args:
        payload: Raw request body
        signature: X-WIA-Signature header value
        secret: Webhook secret
        timestamp: X-WIA-Timestamp header value

    Returns:
        True if signature is valid
    """
    # Prevent replay attacks (5-minute tolerance)
    from datetime import datetime, timedelta
    event_time = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
    if abs((datetime.utcnow() - event_time)) > timedelta(minutes=5):
        return False

    # Compute expected signature
    signed_payload = f"{timestamp}.{payload.decode('utf-8')}"
    expected_signature = hmac.new(
        secret.encode('utf-8'),
        signed_payload.encode('utf-8'),
        hashlib.sha256
    ).hexdigest()

    # Compare signatures
    signature_value = signature.split('=')[1] if '=' in signature else signature
    return hmac.compare_digest(expected_signature, signature_value)
```

---

## Event Streaming

### 5.1 Event Stream Subscription

```typescript
interface EventStreamOptions {
  apiKey: string;
  events: string[];
  filters?: Record<string, any>;
  fromTimestamp?: string;
  maxRetries?: number;
}

class ConsentEventStream {
  private ws: WebSocket | null = null;
  private reconnectAttempts = 0;
  private readonly maxReconnectAttempts = 10;

  constructor(private options: EventStreamOptions) {}

  connect(): void {
    const url = new URL('wss://events.wia.live/cryo-consent/v1/stream');

    this.ws = new WebSocket(url.toString());

    this.ws.onopen = () => {
      console.log('Connected to event stream');
      this.reconnectAttempts = 0;
      this.authenticate();
    };

    this.ws.onmessage = (event) => {
      this.handleMessage(JSON.parse(event.data));
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    this.ws.onclose = (event) => {
      console.log('Connection closed:', event.code);
      this.reconnect();
    };
  }

  private authenticate(): void {
    if (!this.ws) return;

    this.ws.send(JSON.stringify({
      type: 'auth',
      token: this.options.apiKey,
      subscriptions: this.options.events,
      filters: this.options.filters,
      fromTimestamp: this.options.fromTimestamp
    }));
  }

  private handleMessage(message: any): void {
    switch (message.type) {
      case 'auth.success':
        console.log('Authenticated successfully');
        break;
      case 'auth.error':
        console.error('Authentication failed:', message.error);
        break;
      case 'event':
        this.onEvent(message.data);
        break;
      case 'heartbeat':
        this.onHeartbeat();
        break;
      default:
        console.warn('Unknown message type:', message.type);
    }
  }

  private reconnect(): void {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.error('Max reconnection attempts reached');
      return;
    }

    const delay = Math.min(1000 * Math.pow(2, this.reconnectAttempts), 30000);
    this.reconnectAttempts++;

    console.log(`Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts})`);
    setTimeout(() => this.connect(), delay);
  }

  onEvent(event: any): void {
    // Override in implementation
  }

  onHeartbeat(): void {
    // Override in implementation
  }

  disconnect(): void {
    if (this.ws) {
      this.ws.close(1000, 'Client disconnect');
      this.ws = null;
    }
  }
}
```

### 5.2 Event Replay

```http
GET /api/v1/events/replay
```

**Query Parameters:**
- `consentId` (string, required): Consent ID
- `fromTimestamp` (string, required): ISO 8601 timestamp
- `toTimestamp` (string, optional): ISO 8601 timestamp
- `eventTypes` (string[], optional): Filter by event types

**Response:**
```json
{
  "events": [
    {
      "eventId": "evt_001",
      "eventType": "consent.created",
      "timestamp": "2025-01-15T10:30:00Z",
      "consentId": "550e8400-e29b-41d4-a716-446655440001",
      "payload": { /* ... */ }
    },
    {
      "eventId": "evt_002",
      "eventType": "consent.modified",
      "timestamp": "2025-02-20T14:00:00Z",
      "consentId": "550e8400-e29b-41d4-a716-446655440001",
      "payload": { /* ... */ }
    }
  ],
  "meta": {
    "total": 2,
    "fromTimestamp": "2025-01-15T10:30:00Z",
    "toTimestamp": "2025-02-20T14:00:00Z"
  }
}
```

### 5.3 Event Filtering

| Filter | Type | Description | Example |
|--------|------|-------------|---------|
| `consentId` | string | Filter by consent ID | `550e8400-e29b-41d4-a716-446655440001` |
| `grantorId` | string | Filter by grantor | `PERSON-001` |
| `facilityId` | string | Filter by facility | `FAC-001` |
| `jurisdiction` | string[] | Filter by jurisdiction | `["US-CA", "US-NY"]` |
| `eventType` | string[] | Filter by event type | `["consent.modified", "consent.revoked"]` |
| `priority` | string | Filter by priority | `CRITICAL` |
| `fromTimestamp` | timestamp | Events after timestamp | `2025-01-15T00:00:00Z` |
| `toTimestamp` | timestamp | Events before timestamp | `2025-12-31T23:59:59Z` |

---

## Security & Authentication

### 6.1 Authentication Methods

| Method | Transport | Use Case |
|--------|-----------|----------|
| JWT Bearer Token | WebSocket/SSE | Standard authentication |
| OAuth 2.0 | WebSocket/SSE | Third-party integrations |
| API Key | Webhook callbacks | Server-to-server |
| mTLS | WebSocket | High-security environments |

### 6.2 Connection Authentication

```json
{
  "type": "auth",
  "method": "bearer",
  "token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "subscriptions": ["consent.modified"],
  "filters": {
    "consentId": "550e8400-e29b-41d4-a716-446655440001"
  }
}
```

**Success Response:**
```json
{
  "type": "auth.success",
  "subscriptionId": "sub_1234567890",
  "expiresAt": "2025-01-15T12:30:00Z",
  "capabilities": [
    "subscribe",
    "replay",
    "filter"
  ]
}
```

**Error Response:**
```json
{
  "type": "auth.error",
  "error": {
    "code": "INVALID_TOKEN",
    "message": "JWT token has expired"
  }
}
```

### 6.3 Encryption

| Layer | Protocol | Key Size |
|-------|----------|----------|
| Transport | TLS 1.3 | 256-bit |
| Message | AES-GCM | 256-bit |
| Signature | ECDSA | P-256 |

---

## Implementation Examples

### 7.1 TypeScript - Complete Event Listener

```typescript
import WebSocket from 'ws';

interface ConsentEvent {
  eventId: string;
  eventType: string;
  timestamp: string;
  consentId: string;
  version: number;
  actor: {
    id: string;
    type: string;
  };
  payload: any;
  meta: {
    signature: string;
    hash: string;
  };
}

class ConsentEventListener {
  private ws: WebSocket | null = null;
  private heartbeatInterval: NodeJS.Timeout | null = null;

  constructor(
    private apiKey: string,
    private eventHandlers: Map<string, (event: ConsentEvent) => void>
  ) {}

  connect(): void {
    this.ws = new WebSocket('wss://events.wia.live/cryo-consent/v1/stream');

    this.ws.on('open', () => {
      console.log('Connected to consent event stream');
      this.authenticate();
      this.startHeartbeat();
    });

    this.ws.on('message', (data: string) => {
      const message = JSON.parse(data);
      this.handleMessage(message);
    });

    this.ws.on('error', (error) => {
      console.error('WebSocket error:', error);
    });

    this.ws.on('close', () => {
      console.log('Connection closed');
      this.stopHeartbeat();
      this.reconnect();
    });
  }

  private authenticate(): void {
    if (!this.ws) return;

    const subscriptions = Array.from(this.eventHandlers.keys());

    this.ws.send(JSON.stringify({
      type: 'auth',
      token: this.apiKey,
      subscriptions: subscriptions
    }));
  }

  private handleMessage(message: any): void {
    switch (message.type) {
      case 'auth.success':
        console.log('Authentication successful');
        break;

      case 'event':
        this.handleEvent(message.data);
        break;

      case 'heartbeat':
        // Server heartbeat received
        break;

      default:
        console.warn('Unknown message type:', message.type);
    }
  }

  private handleEvent(event: ConsentEvent): void {
    console.log(`Received event: ${event.eventType} for consent ${event.consentId}`);

    const handler = this.eventHandlers.get(event.eventType);
    if (handler) {
      try {
        handler(event);
      } catch (error) {
        console.error(`Error handling event ${event.eventType}:`, error);
      }
    }
  }

  private startHeartbeat(): void {
    this.heartbeatInterval = setInterval(() => {
      if (this.ws?.readyState === WebSocket.OPEN) {
        this.ws.send(JSON.stringify({ type: 'ping' }));
      }
    }, 30000); // 30 seconds
  }

  private stopHeartbeat(): void {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
      this.heartbeatInterval = null;
    }
  }

  private reconnect(): void {
    setTimeout(() => {
      console.log('Attempting to reconnect...');
      this.connect();
    }, 5000);
  }

  disconnect(): void {
    this.stopHeartbeat();
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }
}

// Usage
const eventHandlers = new Map<string, (event: ConsentEvent) => void>([
  ['consent.modified', (event) => {
    console.log(`Consent ${event.consentId} was modified by ${event.actor.id}`);
    // Update local cache, notify stakeholders, etc.
  }],

  ['consent.revoked', (event) => {
    console.log(`CRITICAL: Consent ${event.consentId} was revoked!`);
    // Trigger emergency procedures
    handleEmergencyRevocation(event);
  }],

  ['proxy.added', (event) => {
    console.log(`New proxy added for consent ${event.consentId}`);
    // Update access control lists
  }]
]);

const listener = new ConsentEventListener(
  'wia_live_sk_1234567890',
  eventHandlers
);

listener.connect();

// Graceful shutdown
process.on('SIGINT', () => {
  console.log('Shutting down...');
  listener.disconnect();
  process.exit(0);
});

function handleEmergencyRevocation(event: ConsentEvent): void {
  // Implementation for emergency revocation handling
  console.log('Emergency revocation procedures initiated');

  // 1. Notify all relevant parties
  // 2. Update preservation records
  // 3. Document the revocation
  // 4. Generate audit trail
}
```

### 7.2 Python - SSE Event Stream

```python
import requests
import json
from typing import Callable, Dict
from datetime import datetime

class ConsentEventStream:
    def __init__(self, api_key: str):
        self.api_key = api_key
        self.base_url = "https://events.wia.live/cryo-consent/v1"
        self.handlers: Dict[str, Callable] = {}

    def on(self, event_type: str, handler: Callable):
        """Register an event handler."""
        self.handlers[event_type] = handler

    def listen(self):
        """
        Start listening to the event stream using SSE.
        """
        url = f"{self.base_url}/stream/sse"
        headers = {
            'Authorization': f'Bearer {self.api_key}',
            'Accept': 'text/event-stream'
        }

        params = {
            'events': ','.join(self.handlers.keys())
        }

        try:
            with requests.get(
                url,
                headers=headers,
                params=params,
                stream=True,
                timeout=None
            ) as response:
                response.raise_for_status()

                print("Connected to event stream")

                for line in response.iter_lines():
                    if line:
                        self._handle_sse_line(line.decode('utf-8'))

        except requests.exceptions.RequestException as e:
            print(f"Error connecting to event stream: {e}")
            # Implement reconnection logic

    def _handle_sse_line(self, line: str):
        """Parse and handle SSE message line."""
        if line.startswith('event:'):
            self.current_event_type = line.split(':', 1)[1].strip()
        elif line.startswith('data:'):
            data = line.split(':', 1)[1].strip()
            event = json.loads(data)
            self._dispatch_event(self.current_event_type, event)

    def _dispatch_event(self, event_type: str, event: dict):
        """Dispatch event to registered handler."""
        print(f"Received event: {event_type}")

        handler = self.handlers.get(event_type)
        if handler:
            try:
                handler(event)
            except Exception as e:
                print(f"Error handling event {event_type}: {e}")

# Usage
stream = ConsentEventStream('wia_live_sk_1234567890')

@stream.on('consent.modified')
def handle_consent_modified(event):
    print(f"Consent {event['consentId']} was modified")
    print(f"Version: {event['version']}")
    print(f"Actor: {event['actor']['id']}")

@stream.on('consent.revoked')
def handle_consent_revoked(event):
    print(f"CRITICAL: Consent {event['consentId']} was revoked!")
    # Trigger emergency procedures

    # Send notifications
    notify_stakeholders(event['consentId'])

    # Update preservation records
    update_preservation_status(event['consentId'], 'revoked')

@stream.on('proxy.added')
def handle_proxy_added(event):
    print(f"New proxy added for consent {event['consentId']}")
    print(f"Proxy: {event['payload']['proxyId']}")

# Start listening
stream.listen()
```

### 7.3 Python - Webhook Server

```python
from flask import Flask, request, jsonify
import hmac
import hashlib
from datetime import datetime, timedelta

app = Flask(__name__)

WEBHOOK_SECRET = "whsec_1234567890abcdef"

def verify_webhook_signature(request) -> bool:
    """Verify webhook signature."""
    signature = request.headers.get('X-WIA-Signature', '')
    timestamp = request.headers.get('X-WIA-Timestamp', '')

    # Check timestamp to prevent replay attacks
    try:
        event_time = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
        if abs((datetime.utcnow() - event_time).total_seconds()) > 300:
            return False
    except ValueError:
        return False

    # Compute expected signature
    payload = request.get_data()
    signed_payload = f"{timestamp}.{payload.decode('utf-8')}"
    expected_signature = hmac.new(
        WEBHOOK_SECRET.encode('utf-8'),
        signed_payload.encode('utf-8'),
        hashlib.sha256
    ).hexdigest()

    # Extract signature value
    signature_value = signature.split('=')[1] if '=' in signature else signature

    return hmac.compare_digest(expected_signature, signature_value)

@app.route('/webhooks/consent', methods=['POST'])
def handle_webhook():
    """Handle incoming webhook."""

    # Verify signature
    if not verify_webhook_signature(request):
        return jsonify({'error': 'Invalid signature'}), 401

    # Parse event
    event = request.get_json()
    event_type = request.headers.get('X-WIA-Event-Type')

    print(f"Received webhook: {event_type}")

    # Handle event based on type
    if event_type == 'consent.modified':
        handle_consent_modified(event)
    elif event_type == 'consent.revoked':
        handle_consent_revoked(event)
    elif event_type == 'proxy.added':
        handle_proxy_added(event)
    else:
        print(f"Unknown event type: {event_type}")

    # Return 200 to acknowledge receipt
    return jsonify({'received': True}), 200

def handle_consent_modified(event):
    """Handle consent modification event."""
    consent_id = event['consentId']
    version = event['version']

    print(f"Processing consent modification: {consent_id} v{version}")

    # Update local database
    # Notify relevant parties
    # Trigger business logic

def handle_consent_revoked(event):
    """Handle consent revocation event."""
    consent_id = event['consentId']

    print(f"CRITICAL: Processing consent revocation: {consent_id}")

    # Emergency procedures
    # Update preservation records
    # Notify all stakeholders

def handle_proxy_added(event):
    """Handle proxy addition event."""
    consent_id = event['consentId']
    proxy_id = event['payload']['proxyId']

    print(f"Processing proxy addition: {proxy_id} for {consent_id}")

    # Update access control
    # Notify proxy

if __name__ == '__main__':
    app.run(port=8080)
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

## Appendix A: Related Standards

| Standard | Relationship |
|----------|--------------|
| WIA Cryo-Consent Phase 1 | Data format for events |
| WIA Cryo-Consent Phase 2 | API endpoints for queries |
| WebSocket Protocol RFC 6455 | Real-time transport |
| Server-Sent Events W3C | Unidirectional streaming |
| CloudEvents CNCF | Event format interoperability |

---

<div align="center">

**WIA Cryo-Consent Protocol Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
