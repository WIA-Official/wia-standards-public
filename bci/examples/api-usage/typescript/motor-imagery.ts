/**
 * WIA BCI Motor Imagery Example (TypeScript)
 *
 * Demonstrates a simple motor imagery BCI with feature extraction.
 */

import { WiaBci, SignalProcessor, ClassificationEvent } from 'wia-bci';

// Simple classifier (placeholder for real ML model)
class SimpleClassifier {
  private classes = ['rest', 'left_hand', 'right_hand', 'feet'];

  predict(features: { alpha: number; beta: number }): { classId: number; className: string; confidence: number } {
    // Very simple rule-based classification (for demo only)
    const ratio = features.beta / (features.alpha + 0.001);

    let classId: number;
    let confidence: number;

    if (ratio < 0.3) {
      classId = 0; // rest
      confidence = 0.7;
    } else if (ratio < 0.5) {
      classId = 1; // left_hand
      confidence = 0.6;
    } else if (ratio < 0.7) {
      classId = 2; // right_hand
      confidence = 0.6;
    } else {
      classId = 3; // feet
      confidence = 0.5;
    }

    return {
      classId,
      className: this.classes[classId],
      confidence,
    };
  }
}

async function main() {
  const bci = new WiaBci({ logLevel: 'info' });
  const classifier = new SimpleClassifier();

  console.log('=== Motor Imagery BCI Example ===\n');

  // Data buffer for epoching
  const buffer: Float32Array[] = [];
  const EPOCH_SIZE = 250; // 1 second @ 250 Hz
  const EPOCH_OVERLAP = 125; // 50% overlap

  try {
    // Connect
    await bci.connect({
      type: 'simulator',
      acquisition: {
        samplingRate: 250,
        channels: ['C3', 'C4', 'Cz'], // Motor cortex channels
      },
    });

    console.log('Connected to:', bci.getDeviceInfo()?.name);

    // Process incoming signals
    bci.on('signal', (event) => {
      buffer.push(event.data);

      // Process when we have enough data
      if (buffer.length >= EPOCH_SIZE) {
        // Extract epoch
        const epochData = buffer.slice(0, EPOCH_SIZE);
        buffer.splice(0, EPOCH_OVERLAP); // Remove with overlap

        // Process each channel
        const c3Data = extractChannel(epochData, 0);
        const c4Data = extractChannel(epochData, 1);

        // Band-pass filter (8-30 Hz for mu/beta)
        const c3Filtered = SignalProcessor.bandpass(c3Data, 8, 30, 250);
        const c4Filtered = SignalProcessor.bandpass(c4Data, 8, 30, 250);

        // Calculate band powers
        const c3Powers = SignalProcessor.allBandPowers(c3Filtered, 250);
        const c4Powers = SignalProcessor.allBandPowers(c4Filtered, 250);

        // Average powers from both motor cortex channels
        const avgAlpha = (c3Powers.alpha + c4Powers.alpha) / 2;
        const avgBeta = (c3Powers.beta + c4Powers.beta) / 2;

        // Classify
        const prediction = classifier.predict({
          alpha: avgAlpha,
          beta: avgBeta,
        });

        // Emit classification event
        const classEvent: ClassificationEvent = {
          timestamp: Date.now(),
          classId: prediction.classId,
          className: prediction.className,
          confidence: prediction.confidence,
        };

        bci.emit('classification', classEvent);
      }
    });

    // Handle classification results
    bci.on('classification', (event) => {
      const bar = '█'.repeat(Math.round(event.confidence * 20));
      console.log(
        `[${event.className.padEnd(10)}] ${bar} (${(event.confidence * 100).toFixed(0)}%)`
      );
    });

    // Start streaming
    await bci.startStream();
    console.log('\nStreaming... Press Ctrl+C to stop\n');

    // Run for 30 seconds
    await sleep(30000);

  } catch (error) {
    console.error('Error:', error);
  } finally {
    await bci.stopStream();
    await bci.disconnect();
    bci.dispose();
    console.log('\nExample complete.');
  }
}

// Helper: Extract single channel from multi-channel samples
function extractChannel(samples: Float32Array[], channelIndex: number): Float32Array {
  const result = new Float32Array(samples.length);
  for (let i = 0; i < samples.length; i++) {
    result[i] = samples[i][channelIndex] ?? 0;
  }
  return result;
}

// Helper: Sleep
function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

// Run
main();
