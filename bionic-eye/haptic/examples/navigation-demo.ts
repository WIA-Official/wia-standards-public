/**
 * WIA Haptic Standard - Navigation Demo
 *
 * Example demonstrating haptic-enabled navigation for users with visual impairments.
 */

import { HapticDeviceManager, BluetoothHapticAdapter } from '../api/typescript/src';
import { GoogleMapsHapticPlugin } from '../integrations/navigation/google-maps-plugin';
import { NAVIGATION_PATTERNS } from '../integrations/navigation/patterns';
import { Obstacle } from '../integrations/navigation/types';

/**
 * Navigation Demo Application
 *
 * This demo shows how to integrate WIA haptic feedback with
 * navigation for users who are blind or have low vision.
 */
async function runNavigationDemo() {
  console.log('=== WIA Haptic Navigation Demo ===\n');

  // Step 1: Connect to haptic device
  console.log('1. Connecting to haptic device...');

  const deviceManager = new HapticDeviceManager();

  // Discover available devices
  const devices = await deviceManager.discover();
  console.log(`   Found ${devices.length} device(s)`);

  if (devices.length === 0) {
    console.log('   No devices found. Using simulation mode.');
    // In real app, would use software simulation
  }

  // Connect to first available device
  const device = devices[0] || await deviceManager.getSimulatedDevice();
  await device.connect();
  console.log(`   Connected to: ${device.name}\n`);

  // Step 2: Initialize navigation plugin
  console.log('2. Initializing navigation haptic plugin...');

  const navPlugin = new GoogleMapsHapticPlugin(device, {
    turnWarningDistances: {
      far: 200,      // First warning at 200m
      near: 50,      // Second warning at 50m
      immediate: 10, // Turn now at 10m
    },
    obstacleSettings: {
      enabled: true,
      minDistance: 10,
      criticalDistance: 1,
      updateInterval: 100,
    },
  });

  navPlugin.start();
  console.log('   Navigation haptics active\n');

  // Step 3: Simulate navigation events
  console.log('3. Simulating navigation route...\n');

  // Simulate approaching a turn
  console.log('   → Approaching left turn (150m)');
  navPlugin.turnByTurn.onApproachingTurn('left', 150);
  await delay(2000);

  console.log('   → Approaching left turn (50m)');
  navPlugin.turnByTurn.onApproachingTurn('left', 50);
  await delay(2000);

  console.log('   → Turn left now!');
  navPlugin.turnByTurn.onTurn('left');
  await delay(2000);

  // Simulate obstacle detection
  console.log('\n   → Obstacle detected: Person ahead (3m)');
  const obstacle: Obstacle = {
    id: 'obstacle-1',
    type: 'person',
    direction: 0,      // Straight ahead
    distance: 3,       // 3 meters
    confidence: 0.95,
  };
  navPlugin.obstacleDetection.onObstacleDetected(obstacle);
  await delay(2000);

  console.log('   → Obstacle approaching (1m - critical!)');
  navPlugin.obstacleDetection.onObstacleUpdated({
    ...obstacle,
    distance: 1,
  });
  await delay(2000);

  console.log('   → Path clear');
  navPlugin.obstacleDetection.onPathClear();
  await delay(2000);

  // Continue navigation
  console.log('\n   → Slight right turn');
  navPlugin.turnByTurn.onTurn('slight_right');
  await delay(2000);

  console.log('   → Destination approaching (30m)');
  navPlugin.turnByTurn.onDestinationNear(30);
  await delay(2000);

  console.log('   → Destination reached!');
  navPlugin.turnByTurn.onDestinationReached();
  await delay(2000);

  // Step 4: Cleanup
  console.log('\n4. Cleaning up...');
  navPlugin.stop();
  await device.disconnect();
  console.log('   Demo complete!\n');

  // Summary
  console.log('=== Navigation Haptic Patterns Used ===');
  console.log('• TURN_APPROACHING_FAR  - Gentle notification');
  console.log('• TURN_APPROACHING_NEAR - Double pulse');
  console.log('• TURN_LEFT             - Strong left-side pulse');
  console.log('• OBSTACLE_DETECTED     - Warning pattern');
  console.log('• OBSTACLE_CRITICAL     - Rapid urgent pulses');
  console.log('• PATH_CLEAR            - All-clear confirmation');
  console.log('• DESTINATION_REACHED   - Triumphant completion');
}

/**
 * Delay helper
 */
function delay(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

// Run the demo
runNavigationDemo().catch(console.error);
