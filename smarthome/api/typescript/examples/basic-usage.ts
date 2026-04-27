/**
 * WIA Smart Home SDK - Basic Usage Example
 * 弘益人間 - Benefit All Humanity
 */

import { createWIASmartHome, Device, Scene, Automation } from '../src';

async function main() {
  // ============================================================================
  // 1. Initialize the SDK
  // ============================================================================

  const smarthome = createWIASmartHome({
    api_key: 'your-api-key-here',
    api_endpoint: 'https://api.wia.live/smarthome/v1',
    home_id: 'home-123',
    user_profile_id: 'user-456',
    enable_local_discovery: true,
    enable_cloud_sync: true,
    default_protocol: 'matter',
    timeout_ms: 30000,
  });

  console.log('WIA Smart Home SDK initialized');

  // ============================================================================
  // 2. Device Discovery and Control
  // ============================================================================

  // Discover Matter devices
  console.log('\n--- Discovering Devices ---');
  const devices = await smarthome.discoverDevices({
    protocol: 'matter',
    timeout_ms: 10000,
    filter_device_types: ['light', 'thermostat', 'lock'],
  });

  console.log(`Found ${devices.length} devices`);
  devices.forEach((device: Device) => {
    console.log(`- ${device.name} (${device.device_type})`);
  });

  // Get all devices
  const allDevices = await smarthome.getDevices('home-123');
  console.log(`Total devices in home: ${allDevices.length}`);

  // Control devices
  if (allDevices.length > 0) {
    const lightId = allDevices[0].device_id;

    console.log('\n--- Controlling Devices ---');

    // Turn on with announcement
    await smarthome.turnOn(lightId, { announce: true });
    console.log('Light turned on');

    // Set brightness
    await smarthome.setBrightness(lightId, 75);
    console.log('Brightness set to 75%');

    // Set color (hue: 120 = green, saturation: 100)
    await smarthome.setColor(lightId, 120, 100);
    console.log('Color set to green');

    // Turn off
    await smarthome.turnOff(lightId);
    console.log('Light turned off');
  }

  // ============================================================================
  // 3. Scene Management
  // ============================================================================

  console.log('\n--- Creating Scene ---');

  const morningScene = await smarthome.createScene({
    name: 'Good Morning',
    description: 'Morning routine with gradual lighting',
    home_id: 'home-123',
    icon: '🌅',
    actions: [
      {
        type: 'device_control',
        device_id: 'blinds-bedroom',
        command: 'open',
      },
      {
        type: 'delay',
        delay_ms: 2000,
      },
      {
        type: 'device_control',
        device_id: 'lights-bedroom',
        command: 'turn_on',
        parameters: { brightness: 30 },
      },
      {
        type: 'delay',
        delay_ms: 5000,
      },
      {
        type: 'device_control',
        device_id: 'lights-bedroom',
        command: 'set_brightness',
        parameters: { brightness: 70 },
      },
      {
        type: 'device_control',
        device_id: 'thermostat-living',
        command: 'set_temperature',
        parameters: { temperature: 21 },
      },
      {
        type: 'voice_announcement',
        parameters: {
          text: 'Good morning! Your home is ready for the day.',
        },
      },
    ],
    accessibility_settings: {
      announce_activation: true,
      announcement_text: 'Good morning scene activated',
      confirmation_required: false,
    },
  });

  console.log(`Created scene: ${morningScene.name}`);

  // Activate the scene
  console.log('Activating morning scene...');
  await smarthome.activateScene(morningScene.scene_id, { announce: true });
  console.log('Scene activated successfully');

  // Get all scenes
  const scenes = await smarthome.getScenes('home-123');
  console.log(`Total scenes: ${scenes.length}`);

  // ============================================================================
  // 4. Automation Rules
  // ============================================================================

  console.log('\n--- Creating Automation ---');

  const eveningAutomation = await smarthome.createAutomation({
    name: 'Evening Lights',
    description: 'Turn on lights at sunset',
    home_id: 'home-123',
    enabled: true,
    trigger: {
      type: 'sunset',
      config: {
        offset_minutes: -30, // 30 minutes before sunset
      },
    },
    conditions: [
      {
        type: 'day_of_week',
        config: {
          days: ['mon', 'tue', 'wed', 'thu', 'fri'],
        },
        operator: 'and',
      },
      {
        type: 'zone_occupancy',
        config: {
          zone_id: 'living-room',
          occupied: true,
        },
        operator: 'and',
      },
    ],
    actions: [
      {
        type: 'device_control',
        device_id: 'lights-living-room',
        command: 'turn_on',
        parameters: { brightness: 60, color_temperature: 2700 },
      },
      {
        type: 'device_control',
        device_id: 'blinds-living-room',
        command: 'close',
      },
    ],
    accessibility_settings: {
      announce_activation: true,
      announcement_text: 'Evening mode activated',
      confirmation_required: false,
      override_priority: 'medium',
      safe_mode_behavior: 'confirm',
    },
    schedule: {
      active_days: ['mon', 'tue', 'wed', 'thu', 'fri'],
      respect_quiet_hours: true,
    },
  });

  console.log(`Created automation: ${eveningAutomation.name}`);

  // Get all automations
  const automations = await smarthome.getAutomations('home-123');
  console.log(`Total automations: ${automations.length}`);

  // Manually trigger automation
  console.log('Manually triggering automation...');
  await smarthome.triggerAutomation(eveningAutomation.automation_id);
  console.log('Automation triggered');

  // ============================================================================
  // 5. Security System
  // ============================================================================

  console.log('\n--- Security System ---');

  const securitySystem = await smarthome.getSecuritySystem('home-123');
  if (securitySystem) {
    console.log(`Current security mode: ${securitySystem.mode}`);

    // Arm the system
    await smarthome.setSecurityMode('home-123', 'armed_away', '1234');
    console.log('Security system armed (away mode)');
  }

  // ============================================================================
  // 6. Energy Monitoring
  // ============================================================================

  console.log('\n--- Energy Monitoring ---');

  if (allDevices.length > 0) {
    const deviceId = allDevices[0].device_id;
    const energyData = await smarthome.getEnergyMonitoring(deviceId);

    if (energyData) {
      console.log(`Device: ${deviceId}`);
      console.log(`Daily usage: ${energyData.daily_usage_kwh} kWh`);
      console.log(`Monthly usage: ${energyData.monthly_usage_kwh} kWh`);
      console.log(`Estimated daily cost: $${energyData.estimated_daily_cost}`);
    }
  }

  const homeEnergy = await smarthome.getHomeEnergyUsage('home-123', 'month');
  console.log(`Total home energy (month): ${homeEnergy} kWh`);

  // ============================================================================
  // 7. Voice Control
  // ============================================================================

  console.log('\n--- Voice Control ---');

  // Configure voice assistant
  await smarthome.configureVoiceAssistant({
    type: 'wia',
    enabled: true,
    wake_word: 'Hey WIA',
    language: 'en-US',
  });

  console.log('Voice assistant configured');

  // Process voice command
  const voiceResult = await smarthome.processVoiceCommand(
    'Turn on the living room lights',
    'en-US'
  );

  console.log(`Voice intent: ${voiceResult.intent}`);
  console.log(`Confidence: ${voiceResult.confidence}`);
  console.log(`Action: ${voiceResult.action}`);

  // ============================================================================
  // 8. Event Handling
  // ============================================================================

  console.log('\n--- Event Handling ---');

  // Listen to device state changes
  smarthome.on('device_state_changed', (event) => {
    console.log(`Device ${event.target?.id} state changed`);
    console.log('Data:', event.data);
  });

  // Listen to voice commands
  smarthome.on('voice_command_executed', (event) => {
    console.log('Voice command executed:', event.data);
  });

  // Listen to emergency events
  smarthome.on('emergency_triggered', (event) => {
    console.error('⚠️ EMERGENCY:', event.data);
    // Send alerts, call emergency contacts, etc.
  });

  // Listen to all events
  smarthome.on('all', (event) => {
    console.log(`[${event.event_type}] ${event.timestamp}`);
  });

  // ============================================================================
  // 9. User Profile Management
  // ============================================================================

  console.log('\n--- User Profile ---');

  const profile = await smarthome.getUserProfile('user-456');
  if (profile) {
    console.log(`User: ${profile.personal_info?.name}`);
    console.log(`Language: ${profile.personal_info?.preferred_language}`);
    console.log(`WCAG Level: ${profile.accessibility_requirements.wcag_level}`);
  }

  // Update accessibility preferences
  await smarthome.updateUserProfile('user-456', {
    interaction_preferences: {
      preferred_input_modalities: ['voice', 'touch'],
      preferred_output_modalities: ['audio_tts', 'visual_screen'],
      voice_settings: {
        wake_word: 'Hey WIA',
        speech_rate: 1.2,
        pitch: 1.0,
      },
      timing_settings: {
        response_timeout_ms: 10000,
        dwell_time_ms: 1500,
        confirmation_required: true,
      },
    },
  });

  console.log('User profile updated');

  // ============================================================================
  // 10. Notifications
  // ============================================================================

  console.log('\n--- Notifications ---');

  const notificationId = await smarthome.sendNotification({
    type: 'alert',
    priority: 'high',
    message: {
      'en-US': 'Front door unlocked',
      'ko-KR': '현관문이 열렸습니다',
    },
    source: {
      device_id: 'door-lock-front',
      device_name: 'Front Door Lock',
    },
    delivery: {
      modalities: ['audio_tts', 'visual_screen', 'haptic'],
      audio: {
        tts_text: 'Alert: Front door has been unlocked',
        volume: 80,
        repeat_count: 2,
      },
      visual: {
        title: 'Security Alert',
        body: 'Front door unlocked',
        icon: '🔓',
        flash_pattern: 'pulse',
      },
      haptic: {
        pattern: 'double_tap',
        intensity: 70,
        repeat: 3,
      },
    },
    actions: [
      {
        id: 'lock',
        label: 'Lock Now',
        command: 'lock',
        icon: '🔒',
      },
      {
        id: 'view',
        label: 'View Camera',
        command: 'view_camera',
        icon: '📹',
      },
    ],
    targeting: {
      all_users: true,
    },
  });

  console.log(`Notification sent: ${notificationId}`);

  console.log('\n--- Example Complete ---');
}

// Run the example
main().catch((error) => {
  console.error('Error:', error);
  process.exit(1);
});
