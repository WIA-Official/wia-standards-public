# @wia/tls-lite

> WIA-TLS-LITE - Lightweight Transport Layer Security for IoT and Constrained Devices

## Overview

`@wia/tls-lite` provides a TypeScript implementation of the WIA-TLS-LITE standard, designed specifically for IoT devices and constrained environments where traditional TLS implementations are too resource-intensive.

### Features

- **Lightweight**: Optimized cipher suites for minimal overhead
- **Modern Security**: Support for TLS 1.2 and TLS 1.3
- **Session Resumption**: Reduce handshake overhead with session tickets
- **PSK Support**: Pre-Shared Key mode for ultra-lightweight scenarios
- **ALPN**: Application-Layer Protocol Negotiation
- **SNI**: Server Name Indication for virtual hosting
- **Certificate Validation**: Flexible certificate validation with custom validators
- **Event-Driven**: Comprehensive event system for monitoring
- **TypeScript**: Full type safety and IntelliSense support

## Installation

```bash
npm install @wia/tls-lite
```

## Quick Start

### Basic Client Connection

```typescript
import { createTLSLite, TLSVersion, CipherSuite } from '@wia/tls-lite';

const tls = createTLSLite();

// Perform handshake
const session = await tls.handshake({
  version: TLSVersion.TLS_1_3_LITE,
  cipherSuites: [
    CipherSuite.TLS_AES_128_GCM_SHA256,
    CipherSuite.TLS_CHACHA20_POLY1305_SHA256
  ],
  serverName: 'iot.example.com',
  alpnProtocols: ['mqtt', 'coap'],
  enableSessionResumption: true
});

console.log('Connected:', session.id);

// Establish secure channel
await tls.establishSecureChannel({ session });

// Send encrypted data
await tls.send(Buffer.from('Hello, secure world!'));

// Close connection
await tls.close();
```

### Using Pre-Shared Keys (PSK)

```typescript
import { createTLSLite, TLSVersion, CipherSuite } from '@wia/tls-lite';

const tls = createTLSLite();

const session = await tls.handshake({
  version: TLSVersion.TLS_1_3_LITE,
  cipherSuites: [CipherSuite.TLS_PSK_WITH_AES_128_GCM_SHA256],
  psk: {
    identity: 'device-001',
    key: Buffer.from('secret-shared-key', 'utf8')
  },
  enableSessionResumption: true
});
```

### Session Resumption

```typescript
import { createTLSLite, TLSVersion, CipherSuite } from '@wia/tls-lite';

const tls = createTLSLite();

// Initial handshake
const session1 = await tls.handshake({
  version: TLSVersion.TLS_1_3_LITE,
  cipherSuites: [CipherSuite.TLS_AES_128_GCM_SHA256],
  serverName: 'iot.example.com',
  enableSessionResumption: true
});

// Save session ticket
const ticket = session1.ticket;

// Later, resume session with ticket
const session2 = await tls.handshake({
  version: TLSVersion.TLS_1_3_LITE,
  cipherSuites: [CipherSuite.TLS_AES_128_GCM_SHA256],
  serverName: 'iot.example.com',
  sessionTicket: ticket,
  enableSessionResumption: true
});

console.log('Session resumed:', session2.id === session1.id);
```

### Certificate Validation

```typescript
import { createTLSLite, Certificate, CertificateType } from '@wia/tls-lite';

const tls = createTLSLite();

const cert: Certificate = {
  type: CertificateType.X509,
  data: Buffer.from('...'),
  subject: 'CN=iot.example.com',
  issuer: 'CN=Example CA',
  notBefore: new Date('2025-01-01'),
  notAfter: new Date('2026-01-01')
};

// Validate certificate
const isValid = await tls.validateCertificate(cert, {
  allowSelfSigned: false,
  checkRevocation: true,
  customValidator: async (cert) => {
    // Custom validation logic
    return cert.subject?.includes('example.com') ?? false;
  }
});

console.log('Certificate valid:', isValid);
```

### Event Handling

```typescript
import { createTLSLite, TLSEventType } from '@wia/tls-lite';

const tls = createTLSLite();

// Listen to handshake events
tls.on(TLSEventType.HANDSHAKE_START, (event) => {
  console.log('Handshake started:', event.timestamp);
});

tls.on(TLSEventType.HANDSHAKE_COMPLETE, (event) => {
  console.log('Handshake completed:', event.sessionId);
});

tls.on(TLSEventType.SESSION_RESUMED, (event) => {
  console.log('Session resumed:', event.sessionId);
});

tls.on(TLSEventType.ERROR, (event) => {
  console.error('TLS error:', event.error);
});

// Perform handshake
await tls.handshake({
  version: TLSVersion.TLS_1_3_LITE,
  cipherSuites: [CipherSuite.TLS_AES_128_GCM_SHA256],
  serverName: 'iot.example.com'
});
```

### Statistics and Monitoring

```typescript
import { createTLSLite } from '@wia/tls-lite';

const tls = createTLSLite();

// Perform some operations...
await tls.handshake({ /* ... */ });
await tls.send(Buffer.from('data'));

// Get statistics
const stats = tls.getStats();
console.log('Statistics:', {
  totalHandshakes: stats.totalHandshakes,
  successRate: stats.successfulHandshakes / stats.totalHandshakes,
  avgHandshakeTime: stats.avgHandshakeTime,
  activeSessions: stats.activeSessions,
  bytesSent: stats.bytesSent,
  bytesReceived: stats.bytesReceived
});
```

### Session Management

```typescript
import { createTLSLite } from '@wia/tls-lite';

const tls = createTLSLite();

// Get all active sessions
const activeSessions = tls.getActiveSessions();
console.log('Active sessions:', activeSessions.length);

// Get specific session
const session = tls.getSession('session-id');

// Clear expired sessions
const cleared = tls.clearExpiredSessions();
console.log('Cleared expired sessions:', cleared);
```

## API Reference

### WIATLSLite Class

#### Methods

- `handshake(config: HandshakeConfig): Promise<TLSSession>` - Perform TLS handshake
- `resumeSession(ticket: Buffer): Promise<TLSSession | null>` - Resume session from ticket
- `validateCertificate(cert: Certificate, options?: CertificateValidationOptions): Promise<boolean>` - Validate certificate
- `negotiateCipherSuite(clientSuites: CipherSuite[], serverSuites: CipherSuite[]): CipherSuite | null` - Negotiate cipher suite
- `establishSecureChannel(config: SecureChannelConfig): Promise<void>` - Establish secure channel
- `send(data: Buffer): Promise<void>` - Send encrypted data
- `receive(): Promise<Buffer>` - Receive encrypted data
- `close(): Promise<void>` - Close connection
- `getSession(sessionId: string): TLSSession | undefined` - Get session by ID
- `getActiveSessions(): TLSSession[]` - Get all active sessions
- `clearExpiredSessions(): number` - Clear expired sessions
- `getStats(): TLSStats` - Get statistics
- `on(eventType: TLSEventType, handler: (event: TLSEvent) => void): void` - Register event handler
- `off(eventType: TLSEventType, handler: (event: TLSEvent) => void): void` - Unregister event handler
- `getVersion(): string` - Get SDK version

### Types

See [types.ts](./src/types.ts) for complete type definitions including:

- `TLSVersion` - TLS protocol versions
- `CipherSuite` - Supported cipher suites
- `Certificate` - Certificate structure
- `HandshakeConfig` - Handshake configuration
- `TLSSession` - Session information
- `TLSEvent` - Event structure
- `TLSStats` - Statistics structure

## Cipher Suites

### TLS 1.3 (Recommended for IoT)

- `TLS_AES_128_GCM_SHA256` - Best balance for IoT
- `TLS_AES_256_GCM_SHA384` - Higher security
- `TLS_CHACHA20_POLY1305_SHA256` - Excellent for software-only devices

### TLS 1.2

- `TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256` - Widely supported
- `TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256` - Lower compute overhead
- `TLS_ECDHE_ECDSA_WITH_CHACHA20_POLY1305_SHA256` - Best for constrained devices

### Pre-Shared Key (Ultra-Lightweight)

- `TLS_PSK_WITH_AES_128_GCM_SHA256` - Minimal handshake overhead
- `TLS_PSK_WITH_CHACHA20_POLY1305_SHA256` - Ultra-lightweight

## Best Practices

### For Battery-Powered Devices

```typescript
// Use session resumption to minimize handshakes
const session = await tls.handshake({
  version: TLSVersion.TLS_1_3_LITE,
  cipherSuites: [CipherSuite.TLS_AES_128_GCM_SHA256],
  enableSessionResumption: true,
  enable0RTT: true // Enable 0-RTT for even faster resumption
});
```

### For Memory-Constrained Devices

```typescript
// Use PSK to avoid certificate overhead
const session = await tls.handshake({
  version: TLSVersion.TLS_1_3_LITE,
  cipherSuites: [CipherSuite.TLS_PSK_WITH_CHACHA20_POLY1305_SHA256],
  psk: {
    identity: 'device-id',
    key: preSharedKey
  }
});
```

### For Software-Only Devices

```typescript
// Use ChaCha20-Poly1305 for devices without AES hardware
const session = await tls.handshake({
  version: TLSVersion.TLS_1_3_LITE,
  cipherSuites: [CipherSuite.TLS_CHACHA20_POLY1305_SHA256]
});
```

## Contributing

Contributions are welcome! Please see the [WIA Standards repository](https://github.com/WIA-Official/wia-standards) for contribution guidelines.

## License

MIT © WIA (World Certification Industry Association)

## Philosophy

弘益人間 (홍익인간) - Benefit All Humanity

---

For more information, visit the [WIA-TLS-LITE specification](../../spec/WIA-TLS-LITE-v1.0.md).
