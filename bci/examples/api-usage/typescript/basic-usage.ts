/**
 * WIA BCI Basic Usage Example (TypeScript)
 *
 * Demonstrates basic connection, streaming, and data handling.
 */

import { WiaBci, SignalProcessor } from 'wia-bci';

async function main() {
  // Create BCI instance with logging
  const bci = new WiaBci({ logLevel: 'info' });

  console.log('=== WIA BCI Basic Usage Example ===\n');

  try {
    // List available devices
    console.log('Scanning for devices...');
    const devices = await bci.listDevices();
    console.log(`Found ${devices.length} device(s):`);
    devices.forEach((d) => console.log(`  - ${d.name} (${d.type})`));

    // Connect to simulator
    console.log('\nConnecting to simulator...');
    await bci.connect({
      type: 'simulator',
      acquisition: {
        samplingRate: 250,
        channels: ['Fp1', 'Fp2', 'C3', 'C4', 'O1', 'O2', 'P3', 'P4'],
      },
    });

    console.log('Connected!');
    console.log('Device:', bci.getDeviceInfo()?.name);
    console.log('Channels:', bci.getChannels().map((c) => c.label).join(', '));

    // Set up event handlers
    let sampleCount = 0;
    const buffer: Float32Array[] = [];

    bci.on('signal', (event) => {
      sampleCount++;
      buffer.push(event.data);

      // Log every 250 samples (1 second)
      if (sampleCount % 250 === 0) {
        console.log(`\nReceived ${sampleCount} samples`);

        // Calculate band powers for channel 0
        if (buffer.length >= 250) {
          const epoch = concatenateBuffers(buffer.slice(-250));
          const powers = SignalProcessor.allBandPowers(epoch, 250);

          console.log('Band Powers (Ch0):');
          console.log(`  Delta: ${powers.delta.toFixed(2)}`);
          console.log(`  Theta: ${powers.theta.toFixed(2)}`);
          console.log(`  Alpha: ${powers.alpha.toFixed(2)}`);
          console.log(`  Beta:  ${powers.beta.toFixed(2)}`);
          console.log(`  Gamma: ${powers.gamma.toFixed(2)}`);
        }
      }
    });

    bci.on('error', (error) => {
      console.error('Error:', error.message);
    });

    // Start streaming
    console.log('\nStarting stream...');
    await bci.startStream();

    // Stream for 5 seconds
    console.log('Streaming for 5 seconds...');
    await sleep(5000);

    // Stop and disconnect
    console.log('\nStopping stream...');
    await bci.stopStream();

    console.log('Disconnecting...');
    await bci.disconnect();

    console.log('\n=== Example Complete ===');
    console.log(`Total samples received: ${sampleCount}`);

  } catch (error) {
    console.error('Error:', error);
  } finally {
    bci.dispose();
  }
}

// Helper: Concatenate Float32Arrays (first channel only)
function concatenateBuffers(buffers: Float32Array[]): Float32Array {
  const result = new Float32Array(buffers.length);
  for (let i = 0; i < buffers.length; i++) {
    result[i] = buffers[i][0]; // Channel 0
  }
  return result;
}

// Helper: Sleep for ms
function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

// Run
main();
