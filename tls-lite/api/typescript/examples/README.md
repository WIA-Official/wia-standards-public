# WIA-TLS-LITE Examples

This directory contains practical examples demonstrating various features of the WIA-TLS-LITE SDK.

## Prerequisites

```bash
# Install dependencies
npm install

# Build the SDK
npm run build
```

## Running Examples

Each example can be run using `ts-node` or by compiling first:

### Using ts-node (Development)

```bash
# Install ts-node if not already installed
npm install -g ts-node

# Run any example
ts-node examples/basic-client.ts
ts-node examples/psk-mode.ts
ts-node examples/session-resumption.ts
ts-node examples/certificate-validation.ts
```

### Using Node.js (Production)

```bash
# Compile TypeScript
npx tsc examples/basic-client.ts --outDir dist/examples

# Run compiled JavaScript
node dist/examples/basic-client.js
```

## Examples Overview

### 1. Basic Client (`basic-client.ts`)

**What it demonstrates:**
- Basic TLS handshake
- Secure channel establishment
- Event handling
- Sending encrypted data
- Statistics monitoring

**Best for:** Understanding the fundamentals

```bash
ts-node examples/basic-client.ts
```

**Expected output:**
```
🔐 WIA-TLS-LITE Basic Client Example
📡 Handshake started...
✅ Handshake completed!
📊 Session Details:
   Version: 1.3-lite
   Cipher Suite: TLS_AES_128_GCM_SHA256
...
```

---

### 2. PSK Mode (`psk-mode.ts`)

**What it demonstrates:**
- Pre-Shared Key authentication
- Ultra-lightweight handshake
- No certificate overhead
- Ideal configuration for constrained devices

**Best for:** Memory-limited IoT devices

```bash
ts-node examples/psk-mode.ts
```

**Key benefits shown:**
- No certificate validation overhead
- Faster handshake times
- Lower memory usage
- Perfect for sensors and actuators

---

### 3. Session Resumption (`session-resumption.ts`)

**What it demonstrates:**
- Initial full handshake
- Session ticket generation
- Fast session resumption
- Performance comparison
- 0-RTT data transmission

**Best for:** Battery-powered devices that reconnect frequently

```bash
ts-node examples/session-resumption.ts
```

**Performance gains:**
- Up to 80% faster reconnection
- Reduced battery consumption
- Lower network overhead

---

### 4. Certificate Validation (`certificate-validation.ts`)

**What it demonstrates:**
- Standard X.509 validation
- Self-signed certificate handling
- Custom validators
- Trust anchor configuration
- Various certificate types (X.509, Raw Public Key, Compact X.509)

**Best for:** Understanding security policies

```bash
ts-node examples/certificate-validation.ts
```

**Certificate types covered:**
- X.509 (standard)
- Raw Public Key (lightweight)
- Compact X.509 (IoT-optimized)
- PSK identity (certificate-less)

---

## Use Cases by Device Type

### Ultra-Constrained Devices (< 256 KB RAM)
**Recommended example:** `psk-mode.ts`
- Use PSK authentication
- ChaCha20-Poly1305 cipher
- Minimal handshake overhead

### Battery-Powered Devices
**Recommended example:** `session-resumption.ts`
- Enable session resumption
- Use 0-RTT when possible
- Reduce reconnection overhead

### Standard IoT Devices (> 1 MB RAM)
**Recommended example:** `basic-client.ts`
- Full TLS 1.3 with certificates
- AES-128-GCM cipher
- Complete security features

### Industrial IoT
**Recommended example:** `certificate-validation.ts`
- Custom certificate validation
- Trust anchor configuration
- Strict security policies

---

## Common Patterns

### Pattern 1: Quick Start

```typescript
import { createTLSLite, TLSVersion, CipherSuite } from '@wia/tls-lite';

const tls = createTLSLite();
const session = await tls.handshake({
  version: TLSVersion.TLS_1_3_LITE,
  cipherSuites: [CipherSuite.TLS_AES_128_GCM_SHA256],
  serverName: 'your-server.com'
});
```

### Pattern 2: Error Handling

```typescript
import { TLSEventType } from '@wia/tls-lite';

tls.on(TLSEventType.ERROR, (event) => {
  console.error('TLS Error:', event.error);
  // Handle reconnection, logging, etc.
});
```

### Pattern 3: Session Management

```typescript
// Save ticket for later
const ticket = session.ticket;
localStorage.setItem('tls-ticket', ticket.toString('base64'));

// Resume later
const savedTicket = Buffer.from(localStorage.getItem('tls-ticket'), 'base64');
const resumedSession = await tls.handshake({
  sessionTicket: savedTicket,
  enableSessionResumption: true
});
```

---

## Troubleshooting

### Issue: "No active secure channel"

**Solution:** Call `establishSecureChannel()` after handshake:

```typescript
const session = await tls.handshake(config);
await tls.establishSecureChannel({ session });
await tls.send(data); // Now works
```

### Issue: Handshake timeout

**Solution:** Increase timeout or check network:

```typescript
await tls.handshake({
  ...config,
  timeout: 60000 // 60 seconds instead of default 30
});
```

### Issue: Certificate validation failed

**Solution:** Check certificate validity period or use custom validator:

```typescript
const isValid = await tls.validateCertificate(cert, {
  allowSelfSigned: true, // For development
  customValidator: async (cert) => {
    // Your custom logic
    return true;
  }
});
```

---

## Performance Tips

1. **Use PSK when possible** - Eliminates certificate overhead
2. **Enable session resumption** - Dramatically faster reconnection
3. **Choose right cipher suite**:
   - ChaCha20-Poly1305: Best for software-only devices
   - AES-GCM: Best when hardware acceleration available
4. **Adjust record size** - Smaller for constrained devices
5. **Use 0-RTT** - Fastest resumption (TLS 1.3 only)

---

## Security Considerations

1. **PSK Management**: Store pre-shared keys securely
2. **Certificate Validation**: Always validate in production
3. **Session Tickets**: Encrypt and protect ticket storage
4. **Cipher Suite Selection**: Use strong, modern ciphers
5. **Key Rotation**: Implement periodic key updates

---

## Additional Resources

- [WIA-TLS-LITE Specification](../../spec/WIA-TLS-LITE-v1.0.md)
- [API Documentation](../README.md)
- [WIA Standards Repository](https://github.com/WIA-Official/wia-standards)

---

## Contributing

Found an issue or want to add an example? Please contribute to the [WIA Standards repository](https://github.com/WIA-Official/wia-standards).

---

## License

MIT © WIA (World Certification Industry Association)

弘益人間 (홍익인간) - Benefit All Humanity
