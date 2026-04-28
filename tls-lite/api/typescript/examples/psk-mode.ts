/**
 * Pre-Shared Key (PSK) Mode Example
 * Demonstrates ultra-lightweight TLS for memory-constrained devices
 */

import {
  createTLSLite,
  TLSVersion,
  CipherSuite,
  TLSEventType
} from '@wia/tls-lite';

async function main() {
  console.log('🔑 WIA-TLS-LITE PSK Mode Example\n');
  console.log('Perfect for: Memory-constrained IoT devices\n');

  const tls = createTLSLite();

  // Register event handlers
  tls.on(TLSEventType.HANDSHAKE_START, () => {
    console.log('📡 Starting PSK handshake (no certificates!)...');
  });

  tls.on(TLSEventType.HANDSHAKE_COMPLETE, (event) => {
    console.log('✅ PSK handshake completed');
    console.log('   Session:', event.sessionId);
  });

  // Pre-Shared Key configuration
  // In production, this would be provisioned during device manufacturing
  const deviceIdentity = 'device-sensor-001';
  const preSharedKey = Buffer.from('ultra-secret-shared-key-32-bytes!', 'utf8');

  console.log('🔐 PSK Configuration:');
  console.log('   Device Identity:', deviceIdentity);
  console.log('   Key Size:', preSharedKey.length, 'bytes\n');

  try {
    // PSK handshake - minimal overhead, no certificates
    const session = await tls.handshake({
      version: TLSVersion.TLS_1_3_LITE,
      cipherSuites: [
        // ChaCha20 is ideal for software-only devices
        CipherSuite.TLS_PSK_WITH_CHACHA20_POLY1305_SHA256,
        // AES-GCM as fallback
        CipherSuite.TLS_PSK_WITH_AES_128_GCM_SHA256
      ],
      psk: {
        identity: deviceIdentity,
        key: preSharedKey
      },
      enableSessionResumption: true,
      timeout: 10000 // PSK handshake is fast, 10s is plenty
    });

    console.log('📊 Session Established:');
    console.log('   Cipher:', session.cipherSuite);
    console.log('   Version:', session.version);
    console.log('   Resumable:', session.resumable);

    // Establish secure channel
    await tls.establishSecureChannel({
      session,
      maxRecordSize: 4096, // Smaller for constrained devices
      heartbeatInterval: 120000 // 2 minutes
    });

    // Send sensor data
    console.log('\n📡 Sending sensor data...');
    const sensorData = JSON.stringify({
      deviceId: deviceIdentity,
      temperature: 23.5,
      humidity: 65.2,
      timestamp: new Date().toISOString()
    });

    await tls.send(Buffer.from(sensorData, 'utf8'));
    console.log('   Data sent:', sensorData.length, 'bytes');

    // Show memory savings
    const stats = tls.getStats();
    console.log('\n💡 PSK Benefits:');
    console.log('   ✓ No certificate overhead');
    console.log('   ✓ Faster handshake');
    console.log('   ✓ Lower memory usage');
    console.log('   ✓ Ideal for constrained devices');
    console.log('\n📈 Handshake Time:', `${stats.avgHandshakeTime.toFixed(2)}ms`);

    // Close
    await tls.close();
    console.log('\n✅ Connection closed\n');

  } catch (error) {
    console.error('\n❌ Error:', error);
    process.exit(1);
  }
}

main().catch(console.error);
