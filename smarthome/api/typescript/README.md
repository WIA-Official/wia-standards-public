# WIA Smart Home TypeScript SDK

**弘益人間 (Benefit All Humanity)**

Comprehensive TypeScript SDK for accessible smart home automation with support for Matter, Thread, Zigbee, and Z-Wave protocols.

## Features

- **Device Control**: Control lights, thermostats, locks, sensors, and more
- **Scene Management**: Create and activate multi-device scenes
- **Automation Rules**: Build complex automation with triggers, conditions, and actions
- **Security Monitoring**: Monitor and control home security systems
- **Energy Tracking**: Track device and home energy consumption
- **Voice Control**: Integrate with voice assistants (Alexa, Google, Siri, WIA)
- **Multi-Protocol Support**: Matter, Thread, Zigbee, Z-Wave, WiFi, Bluetooth
- **Accessibility First**: Built-in support for assistive technologies and accessibility features
- **Event-Driven**: Real-time event handling and notifications

## Installation

```bash
npm install @wia/smarthome
```

## Quick Start

```typescript
import { createWIASmartHome } from '@wia/smarthome';

// Initialize the SDK
const smarthome = createWIASmartHome({
  api_key: 'your-api-key',
  home_id: 'your-home-id',
  user_profile_id: 'your-profile-id',
  enable_local_discovery: true,
});

// Discover devices
const devices = await smarthome.discoverDevices();
console.log(`Found ${devices.length} devices`);

// Control a device
await smarthome.turnOn('device-id', { announce: true });

// Listen to events
smarthome.on('device_state_changed', (event) => {
  console.log('Device state changed:', event);
});
```

## Core Concepts

### Device Control

Control any smart home device with built-in accessibility features:

```typescript
// Turn device on/off
await smarthome.turnOn('light-1');
await smarthome.turnOff('light-1');

// Set brightness
await smarthome.setBrightness('light-1', 75);

// Set color
await smarthome.setColor('light-1', 120, 100); // hue, saturation

// Set temperature
await smarthome.setTemperature('thermostat-1', 22);

// Lock/unlock
await smarthome.setLockState('door-lock-1', true);

// Generic control with options
await smarthome.controlDevice('device-id', 'command', {
  parameter: 'value'
}, {
  confirm: true,      // Request confirmation
  announce: true,     // Announce via TTS
  timeout_ms: 5000    // Custom timeout
});
```

### Scene Management

Create and activate scenes for multi-device control:

```typescript
// Create a scene
const morningScene = await smarthome.createScene({
  name: 'Good Morning',
  description: 'Morning routine',
  home_id: 'home-1',
  actions: [
    {
      type: 'device_control',
      device_id: 'blinds-1',
      command: 'open',
    },
    {
      type: 'device_control',
      device_id: 'lights-1',
      command: 'set_brightness',
      parameters: { brightness: 50 },
      delay_ms: 2000,
    },
    {
      type: 'voice_announcement',
      parameters: { text: 'Good morning! Have a great day!' },
    },
  ],
  accessibility_settings: {
    announce_activation: true,
    announcement_text: 'Good morning scene activated',
  },
});

// Activate scene
await smarthome.activateScene(morningScene.scene_id, { announce: true });
```

### Automation Rules

Build powerful automation with triggers, conditions, and actions:

```typescript
// Create automation
const automation = await smarthome.createAutomation({
  name: 'Evening Lights',
  description: 'Turn on lights at sunset',
  home_id: 'home-1',
  enabled: true,
  trigger: {
    type: 'sunset',
    config: { offset_minutes: -30 }, // 30 min before sunset
  },
  conditions: [
    {
      type: 'day_of_week',
      config: { days: ['mon', 'tue', 'wed', 'thu', 'fri'] },
      operator: 'and',
    },
  ],
  actions: [
    {
      type: 'device_control',
      device_id: 'living-room-lights',
      command: 'turn_on',
      parameters: { brightness: 60 },
    },
  ],
  accessibility_settings: {
    announce_activation: true,
    confirmation_required: false,
    safe_mode_behavior: 'confirm',
  },
  schedule: {
    respect_quiet_hours: true,
  },
});

// Trigger automation manually
await smarthome.triggerAutomation(automation.automation_id);

// Enable/disable automation
await smarthome.setAutomationEnabled(automation.automation_id, false);
```

### Security Monitoring

Monitor and control your home security system:

```typescript
// Get security system status
const security = await smarthome.getSecuritySystem('home-1');
console.log('Current mode:', security?.mode);

// Set security mode
await smarthome.setSecurityMode('home-1', 'armed_away', 'access-code');

// Trigger emergency
await smarthome.triggerEmergency('home-1', 'medical', {
  location: 'bedroom',
  severity: 'high',
});
```

### Energy Monitoring

Track energy consumption and optimize usage:

```typescript
// Get device energy data
const energy = await smarthome.getEnergyMonitoring('device-1');
console.log('Daily usage:', energy?.daily_usage_kwh, 'kWh');
console.log('Estimated cost:', energy?.estimated_daily_cost);

// Get home total energy
const totalUsage = await smarthome.getHomeEnergyUsage('home-1', 'month');
console.log('Monthly usage:', totalUsage, 'kWh');
```

### Voice Control

Integrate with voice assistants:

```typescript
// Configure voice assistant
await smarthome.configureVoiceAssistant({
  type: 'alexa',
  enabled: true,
  wake_word: 'Alexa',
  language: 'en-US',
});

// Process voice command
const result = await smarthome.processVoiceCommand(
  'Turn on the living room lights',
  'en-US'
);

console.log('Intent:', result.intent);
console.log('Confidence:', result.confidence);
```

### Event Handling

Listen to real-time events:

```typescript
// Listen to specific event types
smarthome.on('device_state_changed', (event) => {
  console.log('Device changed:', event.target?.id);
});

smarthome.on('voice_command_executed', (event) => {
  console.log('Voice command executed:', event.data);
});

smarthome.on('emergency_triggered', (event) => {
  console.log('EMERGENCY:', event.data);
  // Take immediate action
});

// Listen to all events
smarthome.on('all', (event) => {
  console.log('Event:', event.event_type, event.data);
});

// Unregister callback
const callback = (event) => console.log(event);
smarthome.on('scene_activated', callback);
smarthome.off('scene_activated', callback);
```

### User Profiles

Manage accessibility profiles:

```typescript
// Get user profile
const profile = await smarthome.getUserProfile('profile-1');

// Update accessibility preferences
await smarthome.updateUserProfile('profile-1', {
  accessibility_requirements: {
    primary_disabilities: ['visual_blind'],
    wcag_level: 'AAA',
    specific_needs: {
      visual: {
        screen_reader_required: true,
        audio_descriptions_required: true,
      },
    },
  },
  interaction_preferences: {
    preferred_input_modalities: ['voice', 'switch'],
    preferred_output_modalities: ['audio_tts', 'haptic'],
    voice_settings: {
      wake_word: 'Hey WIA',
      speech_rate: 1.2,
    },
  },
});
```

## Protocol Support

### Matter

```typescript
const device: Device = {
  device_id: 'matter-light-1',
  device_type: 'light_color',
  name: 'Living Room Light',
  matter_node_id: '0x1234',
  vendor_id: '0xFFF1',
  product_id: '0x8000',
  // ... other properties
};
```

### Thread, Zigbee, Z-Wave

The SDK automatically handles protocol-specific communication through the unified API.

## Accessibility Features

### Voice Commands

Every device supports custom voice commands:

```typescript
const device: Device = {
  // ...
  accessibility_features: {
    voice_commands: [
      {
        command: 'Turn on the lights',
        aliases: ['Lights on', 'Illuminate'],
        action: 'turn_on',
        confirmation_phrase: 'Lights are now on',
      },
    ],
    audio_feedback: {
      enabled: true,
      volume: 70,
    },
  },
};
```

### Multi-Modal Input/Output

Support for diverse input and output modalities:

- **Input**: voice, touch, switch, gaze, gesture, BCI, sip-puff, keyboard, remote
- **Output**: visual (screen/LED), audio (TTS/tone), haptic, braille

### Timing Adjustments

Configurable timing for accessibility needs:

```typescript
const device: Device = {
  // ...
  accessibility_features: {
    timing: {
      response_timeout_ms: 10000,      // Extended timeout
      confirmation_required: true,      // Require confirmation
      dwell_time_ms: 2000,             // Longer dwell time
    },
  },
};
```

## API Reference

### WIASmartHome Class

#### Device Management
- `discoverDevices(options?)` - Discover devices on network
- `getDevice(deviceId)` - Get device by ID
- `getDevices(homeId?, zoneId?)` - Get all devices
- `controlDevice(deviceId, command, parameters?, options?)` - Control device
- `turnOn(deviceId, options?)` - Turn device on
- `turnOff(deviceId, options?)` - Turn device off
- `setBrightness(deviceId, brightness, options?)` - Set brightness
- `setColor(deviceId, hue, saturation, options?)` - Set color
- `setTemperature(deviceId, temperature, options?)` - Set temperature
- `setLockState(deviceId, locked, options?)` - Lock/unlock

#### Scene Management
- `createScene(scene)` - Create new scene
- `activateScene(sceneId, options?)` - Activate scene
- `getScenes(homeId?)` - Get all scenes

#### Automation
- `createAutomation(automation)` - Create automation
- `updateAutomation(automationId, updates)` - Update automation
- `setAutomationEnabled(automationId, enabled)` - Enable/disable automation
- `triggerAutomation(automationId)` - Manually trigger automation
- `getAutomations(homeId?)` - Get all automations

#### Security
- `getSecuritySystem(homeId)` - Get security system status
- `setSecurityMode(homeId, mode, accessCode?)` - Set security mode
- `triggerEmergency(homeId, type, details?)` - Trigger emergency

#### Energy
- `getEnergyMonitoring(deviceId)` - Get device energy data
- `getHomeEnergyUsage(homeId, period)` - Get home energy usage

#### Voice
- `configureVoiceAssistant(config)` - Configure voice assistant
- `processVoiceCommand(transcript, language?)` - Process voice command

#### Notifications
- `sendNotification(notification)` - Send notification
- `dismissNotification(notificationId)` - Dismiss notification

#### Events
- `on(eventType, callback)` - Register event listener
- `off(eventType, callback)` - Unregister event listener

#### Home/Zone
- `createHome(home)` - Create new home
- `createZone(zone)` - Create new zone

#### User
- `getUserProfile(profileId)` - Get user profile
- `updateUserProfile(profileId, updates)` - Update user profile

## License

Apache-2.0

## Philosophy

**弘益人間 (弘益人間)** - Benefit All Humanity

This SDK is built with accessibility at its core, ensuring that smart home technology is usable by everyone, regardless of ability.

---

© 2025 SmileStory Inc. / WIA
