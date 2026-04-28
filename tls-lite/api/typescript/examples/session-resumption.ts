/**
 * Session Resumption Example
 * Demonstrates fast reconnection using session tickets
 */

import {
  createTLSLite,
  TLSVersion,
  CipherSuite,
  TLSEventType,
  TLSSession
} from '@wia/tls-lite';

async function performHandshake(
  tls: ReturnType<typeof createTLSLite>,
  ticket?: Buffer | Uint8Array
): Promise<TLSSession> {
  return await tls.handshake({
    version: TLSVersion.TLS_1_3_LITE,
    cipherSuites: [CipherSuite.TLS_AES_128_GCM_SHA256],
    serverName: 'iot.example.com',
    sessionTicket: ticket,
    enableSessionResumption: true,
    enable0RTT: true // Enable 0-RTT for even faster resumption
  });
}

async function main() {
  console.log('🎫 WIA-TLS-LITE Session Resumption Example\n');
  console.log('Ideal for: Battery-powered devices that frequently reconnect\n');

  const tls = createTLSLite();

  // Track handshake types
  let fullHandshakes = 0;
  let resumptions = 0;

  tls.on(TLSEventType.HANDSHAKE_COMPLETE, () => {
    fullHandshakes++;
  });

  tls.on(TLSEventType.SESSION_RESUMED, () => {
    resumptions++;
    console.log('⚡ Session resumed (fast!)');
  });

  try {
    // === First Connection: Full Handshake ===
    console.log('═══ First Connection ═══');
    console.log('🤝 Performing full TLS handshake...');
    const startFull = Date.now();
    const session1 = await performHandshake(tls);
    const fullTime = Date.now() - startFull;

    console.log('✅ Full handshake completed');
    console.log('   Time:', `${fullTime}ms`);
    console.log('   Session ID:', session1.id);

    // Save the session ticket
    const ticket = session1.ticket;
    if (!ticket) {
      throw new Error('Server did not provide session ticket');
    }
    console.log('   Ticket size:', ticket.length, 'bytes');

    // Simulate sending some data
    await tls.establishSecureChannel({ session: session1 });
    await tls.send(Buffer.from('Data from first connection'));
    await tls.close();

    // Simulate device going to sleep
    console.log('\n💤 Device sleeping... (simulating disconnect)\n');
    await new Promise(resolve => setTimeout(resolve, 1000));

    // === Second Connection: Resumption ===
    console.log('═══ Second Connection (with ticket) ═══');
    console.log('🎫 Resuming session with ticket...');
    const startResume = Date.now();
    const session2 = await performHandshake(tls, ticket);
    const resumeTime = Date.now() - startResume;

    console.log('✅ Session resumed');
    console.log('   Time:', `${resumeTime}ms`);
    console.log('   Session ID:', session2.id);
    console.log('   Same session:', session1.id === session2.id ? 'Yes ✓' : 'No ✗');

    await tls.establishSecureChannel({ session: session2 });
    await tls.send(Buffer.from('Data from resumed session'));
    await tls.close();

    // === Third Connection: Another Resumption ===
    console.log('\n💤 Device sleeping again...\n');
    await new Promise(resolve => setTimeout(resolve, 1000));

    console.log('═══ Third Connection (with ticket) ═══');
    console.log('🎫 Resuming session again...');
    const startResume2 = Date.now();
    const session3 = await performHandshake(tls, ticket);
    const resumeTime2 = Date.now() - startResume2;

    console.log('✅ Session resumed again');
    console.log('   Time:', `${resumeTime2}ms`);

    await tls.establishSecureChannel({ session: session3 });
    await tls.send(Buffer.from('Data from second resumption'));
    await tls.close();

    // === Show Benefits ===
    console.log('\n═══ Performance Comparison ═══');
    console.log('\n⏱️  Timing:');
    console.log('   Full handshake:', `${fullTime}ms`);
    console.log('   Resumption #1:', `${resumeTime}ms`);
    console.log('   Resumption #2:', `${resumeTime2}ms`);

    const avgResume = (resumeTime + resumeTime2) / 2;
    const speedup = ((fullTime - avgResume) / fullTime * 100).toFixed(1);
    console.log('\n📊 Statistics:');
    console.log('   Speed improvement:', `${speedup}% faster`);
    console.log('   Full handshakes:', fullHandshakes - resumptions);
    console.log('   Resumptions:', resumptions);

    const stats = tls.getStats();
    console.log('   Total handshakes:', stats.totalHandshakes);
    console.log('   Session resumptions:', stats.sessionResumptions);

    console.log('\n💡 Benefits:');
    console.log('   ✓ Faster reconnection');
    console.log('   ✓ Lower power consumption');
    console.log('   ✓ Reduced network traffic');
    console.log('   ✓ Better battery life');

    // Show active sessions
    const activeSessions = tls.getActiveSessions();
    console.log('\n📋 Active sessions:', activeSessions.length);
    activeSessions.forEach((session, i) => {
      console.log(`   ${i + 1}. ${session.id}`);
      console.log(`      Created: ${session.createdAt.toISOString()}`);
      console.log(`      Expires: ${session.expiresAt.toISOString()}`);
    });

    console.log('\n✅ Example completed\n');

  } catch (error) {
    console.error('\n❌ Error:', error);
    process.exit(1);
  }
}

main().catch(console.error);
