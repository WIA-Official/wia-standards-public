/**
 * WIA AAC Standard - TypeScript API Example
 *
 * This example demonstrates how to use the WIA AAC TypeScript API
 * to connect to sensors, handle signals, and process selections.
 */

import {
  WiaAac,
  SensorType,
  EventType,
  ConnectionState,
  WiaAacSignal,
  SelectionEvent,
  WiaAacError
} from 'wia-aac';

async function main() {
  // Create WiaAac instance with options
  const aac = new WiaAac({
    autoReconnect: true,
    reconnectInterval: 3000,
    maxReconnectAttempts: 5,
    signalBufferSize: 100,
    validateSignals: true,
    logLevel: 'info'
  });

  // Register event handlers
  aac.on(EventType.SIGNAL, (signal: WiaAacSignal) => {
    console.log(`Signal received: ${signal.type}`);

    switch (signal.type) {
      case SensorType.EYE_TRACKER:
        if (signal.data.gaze_point) {
          console.log(`  Gaze: (${signal.data.gaze_point.x}, ${signal.data.gaze_point.y})`);
        }
        break;

      case SensorType.SWITCH:
        console.log(`  Switch ${signal.data.switch_id}: ${signal.data.state}`);
        break;

      case SensorType.MUSCLE_SENSOR:
        console.log(`  EMG Activation: ${(signal.data.activation_level * 100).toFixed(1)}%`);
        break;

      case SensorType.BRAIN_INTERFACE:
        console.log(`  BCI Command: ${signal.data.mental_command || 'neutral'}`);
        break;

      case SensorType.BREATH:
        console.log(`  Breath: ${signal.data.action}`);
        break;

      case SensorType.HEAD_MOVEMENT:
        console.log(`  Head position: (${signal.data.position.x}, ${signal.data.position.y})`);
        break;
    }
  });

  aac.on(EventType.SELECTION, (event: SelectionEvent) => {
    console.log(`Selection: ${event.targetId} via ${event.method}`);
  });

  aac.on(EventType.ERROR, (error: WiaAacError) => {
    console.error(`Error: ${error.message} (${error.code})`);
  });

  aac.on(EventType.CONNECTED, (info) => {
    console.log(`Connected to: ${info.name}`);
  });

  aac.on(EventType.DISCONNECTED, (reason) => {
    console.log(`Disconnected: ${reason}`);
  });

  try {
    // Connect to an eye tracker
    await aac.connect({
      type: SensorType.EYE_TRACKER,
      device: {
        manufacturer: 'Tobii'
      },
      options: {
        dwellTime: 1000,
        smoothing: 0.3
      }
    });

    console.log(`Connection state: ${aac.getConnectionState()}`);

    // Let it run for a while (in a real app, this would be event-driven)
    await new Promise(resolve => setTimeout(resolve, 5000));

    // Get last signal
    const lastSignal = aac.getLastSignal();
    if (lastSignal) {
      console.log(`Last signal timestamp: ${lastSignal.timestamp}`);
    }

    // Disconnect
    await aac.disconnect();

  } catch (error) {
    console.error('Failed to connect:', error);
  }
}

main().catch(console.error);
