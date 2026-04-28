/**
 * Basic TLS-Lite Client Example
 * Demonstrates basic TLS handshake and secure communication
 */

import {
  createTLSLite,
  TLSVersion,
  CipherSuite,
  TLSEventType,
  HandshakeConfig
} from '@wia/tls-lite';

async function main() {
  console.log('🔐 WIA-TLS-LITE Basic Client Example\n');

  // Create TLS-Lite instance
  const tls = createTLSLite();
  console.log(`SDK Version: ${tls.getVersion()}\n`);

  // Register event handlers
  tls.on(TLSEventType.HANDSHAKE_START, (event) => {
    console.log('📡 Handshake started at', event.timestamp);
  });

  tls.on(TLSEventType.HANDSHAKE_COMPLETE, (event) => {
    console.log('✅ Handshake completed!');
    console.log('   Session ID:', event.sessionId);
  });

  tls.on(TLSEventType.ERROR, (event) => {
    console.error('❌ Error:', event.error?.message);
  });

  // Configure handshake
  const config: HandshakeConfig = {
    version: TLSVersion.TLS_1_3_LITE,
    cipherSuites: [
      CipherSuite.TLS_AES_128_GCM_SHA256,
      CipherSuite.TLS_CHACHA20_POLY1305_SHA256
    ],
    serverName: 'iot.example.com',
    alpnProtocols: ['mqtt', 'coap'],
    enableSessionResumption: true,
    timeout: 30000 // 30 seconds
  };

  try {
    // Perform handshake
    console.log('🤝 Initiating TLS handshake...');
    const session = await tls.handshake(config);

    console.log('\n📊 Session Details:');
    console.log('   Version:', session.version);
    console.log('   Cipher Suite:', session.cipherSuite);
    console.log('   Server Name:', session.serverName);
    console.log('   ALPN Protocol:', session.alpnProtocol);
    console.log('   Created:', session.createdAt.toISOString());
    console.log('   Expires:', session.expiresAt.toISOString());
    console.log('   Resumable:', session.resumable);

    // Establish secure channel
    console.log('\n🔒 Establishing secure channel...');
    await tls.establishSecureChannel({
      session,
      maxRecordSize: 16384,
      heartbeatInterval: 60000 // 1 minute
    });

    // Send encrypted data
    console.log('📤 Sending encrypted data...');
    const message = Buffer.from('Hello from WIA-TLS-LITE client!');
    await tls.send(message);
    console.log(`   Sent ${message.length} bytes`);

    // Get statistics
    const stats = tls.getStats();
    console.log('\n📈 Statistics:');
    console.log('   Total Handshakes:', stats.totalHandshakes);
    console.log('   Successful:', stats.successfulHandshakes);
    console.log('   Failed:', stats.failedHandshakes);
    console.log('   Avg Handshake Time:', `${stats.avgHandshakeTime.toFixed(2)}ms`);
    console.log('   Bytes Sent:', stats.bytesSent);
    console.log('   Active Sessions:', stats.activeSessions);

    // Save session ticket for resumption
    if (session.ticket) {
      console.log('\n💾 Session ticket saved for future resumption');
      console.log('   Ticket size:', session.ticket.length, 'bytes');
    }

    // Close connection
    console.log('\n👋 Closing connection...');
    await tls.close();
    console.log('✅ Connection closed successfully\n');

  } catch (error) {
    console.error('\n❌ Failed:', error);
    process.exit(1);
  }
}

// Run example
main().catch(console.error);
