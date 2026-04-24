/**
 * WIA Haptic Standard - Smart Home Demo
 *
 * Example demonstrating haptic-enabled smart home control
 * for users with visual impairments.
 */

import { HapticDeviceManager } from '../api/typescript/src';
import { MatterHapticBridge } from '../integrations/smarthome/matter-bridge';
import { SMARTHOME_PATTERNS, encodeTemperature } from '../integrations/smarthome/patterns';

/**
 * Smart Home Demo Application
 *
 * This demo shows how to control smart home devices using
 * haptic feedback for spatial awareness and confirmation.
 */
async function runSmartHomeDemo() {
  console.log('=== WIA Haptic Smart Home Demo ===\n');

  // Step 1: Connect to haptic device
  console.log('1. Connecting to haptic device...');

  const deviceManager = new HapticDeviceManager();
  const device = await deviceManager.getSimulatedDevice();
  await device.connect();
  console.log(`   Connected to: ${device.name}\n`);

  // Step 2: Initialize smart home bridge
  console.log('2. Initializing Matter haptic bridge...');

  const bridge = new MatterHapticBridge(device, {
    spatialSelection: {
      enabled: true,
      angleThreshold: 30,
      confirmationDelay: 500,
    },
    temperatureRange: {
      min: 15,
      max: 30,
    },
  });

  // Discover devices
  const devices = await bridge.discoverDevices();
  console.log(`   Discovered ${devices.length} device(s):`);
  devices.forEach(d => {
    console.log(`   - ${d.name} (${d.type}) at ${d.direction}°`);
  });
  console.log();

  // Step 3: Demonstrate spatial device selection
  console.log('3. Spatial device selection demo...\n');

  // Point in different directions
  console.log('   Pointing at 0° (forward)...');
  const device1 = bridge.pointToDevice(0);
  if (device1) {
    console.log(`   → Hovering over: ${device1.name}`);
  }
  await delay(1000);

  console.log('   Pointing at 90° (right)...');
  const device2 = bridge.pointToDevice(90);
  if (device2) {
    console.log(`   → Hovering over: ${device2.name}`);
  }
  await delay(1000);

  console.log('   Pointing at 180° (behind)...');
  const device3 = bridge.pointToDevice(180);
  if (device3) {
    console.log(`   → Hovering over: ${device3.name}`);
  }
  await delay(1000);

  // Step 4: Control devices with haptic feedback
  console.log('\n4. Device control with haptic feedback...\n');

  // Control light
  const light = devices.find(d => d.type === 'light');
  if (light) {
    console.log(`   Turning on ${light.name}...`);
    await bridge.controlDevice(light.id, { type: 'on' });
    console.log('   → Light ON pattern played (rising brightness)');
    await delay(1500);

    console.log('   Setting brightness to 50%...');
    await bridge.controlDevice(light.id, { type: 'setBrightness', value: 50 });
    console.log('   → Brightness pattern played');
    await delay(1500);

    console.log('   Turning off light...');
    await bridge.controlDevice(light.id, { type: 'off' });
    console.log('   → Light OFF pattern played (falling)');
    await delay(1500);
  }

  // Control door lock
  const doorLock = devices.find(d => d.type === 'door_lock');
  if (doorLock) {
    console.log(`\n   Unlocking ${doorLock.name}...`);
    await bridge.controlDevice(doorLock.id, { type: 'unlock' });
    console.log('   → Unlock pattern played');
    await delay(1500);

    console.log('   Locking door...');
    await bridge.controlDevice(doorLock.id, { type: 'lock' });
    console.log('   → Lock pattern played (secure confirmation)');
    await delay(1500);
  }

  // Control thermostat
  const thermostat = devices.find(d => d.type === 'thermostat');
  if (thermostat) {
    console.log(`\n   Reading temperature from ${thermostat.name}...`);

    // Demonstrate temperature encoding
    const temps = [18, 22, 26];
    for (const temp of temps) {
      console.log(`   → Temperature ${temp}°C:`);
      const pattern = encodeTemperature(temp, { min: 15, max: 30 });
      console.log(`     Frequency: ${pattern.primitives[0].frequency}Hz`);
      console.log(`     (Lower = cold, Higher = warm)`);
      await delay(1000);
    }

    console.log('\n   Setting target temperature to 21°C...');
    await bridge.controlDevice(thermostat.id, { type: 'setTemperature', value: 21 });
    console.log('   → Thermostat set pattern played');
    await delay(1500);
  }

  // Step 5: Simulate alerts
  console.log('\n5. Alert demonstration...\n');

  console.log('   Simulating motion sensor trigger...');
  bridge.onDeviceEvent({
    deviceId: 'motion-1',
    type: 'triggered',
    newState: {},
    timestamp: Date.now(),
  });
  await delay(2000);

  console.log('   Simulating device disconnect...');
  bridge.onDeviceEvent({
    deviceId: light?.id || 'light-1',
    type: 'disconnected',
    newState: {},
    timestamp: Date.now(),
  });
  await delay(2000);

  console.log('   Simulating device reconnect...');
  bridge.onDeviceEvent({
    deviceId: light?.id || 'light-1',
    type: 'connected',
    newState: { on: false, brightness: 100 },
    timestamp: Date.now(),
  });
  await delay(2000);

  // Step 6: Cleanup
  console.log('\n6. Cleaning up...');
  bridge.cancelSelection();
  await device.disconnect();
  console.log('   Demo complete!\n');

  // Summary
  console.log('=== Smart Home Haptic Patterns Used ===');
  console.log('• DEVICE_HOVER    - Subtle tick on pointing');
  console.log('• DEVICE_SELECT   - Confirmation double-pulse');
  console.log('• LIGHT_ON/OFF    - Rising/falling brightness');
  console.log('• DOOR_LOCKED     - Secure double-click');
  console.log('• DOOR_UNLOCKED   - Single confirmation');
  console.log('• THERMOSTAT_SET  - Temperature setting');
  console.log('• MOTION_DETECTED - Alert pattern');
  console.log('• Temperature     - Frequency-encoded (low=cold)');
}

/**
 * Delay helper
 */
function delay(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

// Run the demo
runSmartHomeDemo().catch(console.error);
