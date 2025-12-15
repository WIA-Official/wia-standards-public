/**
 * WIA AAC Full Demo
 * Phase 4: WIA Ecosystem Integration
 *
 * Demonstrates the complete AAC pipeline:
 * Sensor Input → Text Generation → Multi-Output
 */

import {
  // Core (Phase 2)
  WiaAac,
  MockAdapter,
  // Output (Phase 4)
  OutputManager,
  MockTTSAdapter,
  MockSignLanguageAdapter,
  MockBrailleAdapter,
  OutputType
} from 'wia-aac';

async function main() {
  console.log('=== WIA AAC Full Demo ===\n');

  // 1. Initialize AAC Core (Phase 1-3)
  console.log('1. Initializing AAC Core...');
  const aac = new WiaAac({
    sensorType: 'eye_tracker',
    autoConnect: false,
    bufferSize: 100
  });

  // 2. Initialize Output Manager (Phase 4)
  console.log('2. Initializing Output Manager...');
  const output = new OutputManager();

  // 3. Initialize and register output adapters
  console.log('3. Setting up output adapters...');

  // TTS Adapter
  const tts = new MockTTSAdapter();
  await tts.initialize();
  output.register(tts);
  console.log('   - TTS adapter registered');

  // Sign Language Adapter
  const signLanguage = new MockSignLanguageAdapter();
  await signLanguage.initialize();
  output.register(signLanguage);
  console.log('   - Sign Language adapter registered');

  // Braille Adapter
  const braille = new MockBrailleAdapter();
  await braille.initialize();
  output.register(braille);
  console.log('   - Braille adapter registered');

  // 4. Set up event handlers
  console.log('4. Setting up event handlers...\n');

  output.on('outputStart', (data) => {
    console.log(`   [Output Started] ${JSON.stringify(data)}`);
  });

  output.on('outputEnd', (data) => {
    console.log(`   [Output Completed] ${JSON.stringify(data)}`);
  });

  output.on('error', (data) => {
    console.error(`   [Output Error] ${JSON.stringify(data)}`);
  });

  // 5. Demo: Output to each adapter individually
  console.log('5. Demo: Individual Adapter Output\n');

  const testText = '안녕하세요';
  console.log(`   Test text: "${testText}"\n`);

  // TTS Output
  console.log('   --- TTS Output ---');
  await output.outputTo('tts', testText, { speed: 1.0 });
  console.log();

  // Sign Language Output
  console.log('   --- Sign Language Output ---');
  const ispCodes = await signLanguage.textToISP(testText);
  console.log(`   ISP Codes: ${JSON.stringify(ispCodes, null, 2)}`);
  await output.outputTo('sign_language', testText);
  console.log();

  // Braille Output
  console.log('   --- Braille Output ---');
  const brailleOutput = await braille.textToBraille(testText);
  console.log(`   IPA: ${brailleOutput.ipa}`);
  console.log(`   Braille: ${brailleOutput.braille}`);
  console.log(`   Unicode: ${brailleOutput.unicode.join(', ')}`);
  await output.outputTo('braille', testText);
  console.log();

  // 6. Demo: Broadcast to all adapters
  console.log('6. Demo: Broadcast to All Adapters\n');

  const broadcastText = '감사합니다';
  console.log(`   Broadcasting: "${broadcastText}"\n`);
  await output.broadcast(broadcastText, { speed: 0.8 });
  console.log();

  // 7. Cleanup
  console.log('7. Cleaning up...');
  await output.dispose();
  await aac.disconnect();

  console.log('\n=== Demo Complete ===');
}

// Run the demo
main().catch(console.error);
