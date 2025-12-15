# WIA Haptic Standard - Integration Guide

This guide covers integrating the WIA Haptic Standard with various platforms and applications.

## Table of Contents

1. [Navigation Integration](#navigation-integration)
2. [Smart Home Integration](#smart-home-integration)
3. [WIA Ecosystem Integration](#wia-ecosystem-integration)
4. [Accessibility Platform Integration](#accessibility-platform-integration)
5. [Best Practices](#best-practices)

---

## Navigation Integration

### Overview

The navigation integration provides haptic feedback for turn-by-turn navigation and obstacle detection, designed for users who are blind or have low vision.

### Google Maps Plugin

```typescript
import { GoogleMapsHapticPlugin } from '@wia-standards/haptic/integrations/navigation';

// Initialize with haptic device
const plugin = new GoogleMapsHapticPlugin(hapticDevice, {
  turnWarningDistances: {
    far: 200,       // First warning at 200m
    near: 50,       // Second warning at 50m
    immediate: 10,  // Turn now at 10m
  },
  obstacleSettings: {
    enabled: true,
    criticalDistance: 1, // meters
  },
});

// Connect to Google Maps events
googleMapsNavigator.addListener('navigation_event', (event) => {
  plugin.onNavigationEvent(event);
});

plugin.start();
```

### Apple Maps Plugin

```typescript
import { AppleMapsHapticPlugin } from '@wia-standards/haptic/integrations/navigation';

const plugin = new AppleMapsHapticPlugin(hapticDevice);

// Set route from MapKit
plugin.setRoute(mkRoute);

// Update state from CoreLocation
plugin.updateNavigationState({
  currentStepIndex: 0,
  distanceToNextStep: 150,
  distanceToDestination: 2000,
  isNavigating: true,
});

// Integrate with ARKit for obstacles
plugin.onARKitObstacles(arKitObstacles);
```

### Navigation Patterns

| Pattern | Usage | Feel |
|---------|-------|------|
| `TURN_APPROACHING_FAR` | 200m before turn | Single gentle pulse |
| `TURN_APPROACHING_NEAR` | 50m before turn | Double pulse |
| `TURN_LEFT` | Turn now | Strong left-side pulse |
| `TURN_RIGHT` | Turn now | Strong right-side pulse |
| `U_TURN` | U-turn required | Sweeping pattern |
| `OBSTACLE_DETECTED` | Obstacle in path | Alert double-pulse |
| `OBSTACLE_CRITICAL` | Close obstacle (<1m) | Rapid urgent pulses |
| `DESTINATION_REACHED` | Arrival | Rising triumphant pattern |

---

## Smart Home Integration

### Overview

Control smart home devices using spatial awareness and haptic feedback, supporting Matter/Thread protocols.

### Matter Bridge Setup

```typescript
import { MatterHapticBridge } from '@wia-standards/haptic/integrations/smarthome';

const bridge = new MatterHapticBridge(hapticDevice, {
  spatialSelection: {
    enabled: true,
    angleThreshold: 30,      // degrees
    confirmationDelay: 500,  // ms
  },
});

// Discover devices
const devices = await bridge.discoverDevices();

// Spatial selection (point and select)
const device = bridge.pointToDevice(direction); // direction in degrees
bridge.confirmSelection();

// Control with feedback
await bridge.controlDevice(deviceId, { type: 'toggle' });
```

### Temperature Encoding

Temperature is encoded as haptic frequency:
- Cold (< 18°C): Low frequency, slow
- Comfortable (18-24°C): Medium frequency
- Warm (> 24°C): High frequency, fast

```typescript
import { encodeTemperature } from '@wia-standards/haptic/integrations/smarthome';

const pattern = encodeTemperature(22, { min: 10, max: 35 });
// Plays medium frequency pattern
```

### Smart Home Patterns

| Pattern | Usage |
|---------|-------|
| `LIGHT_ON` | Rising brightness pattern |
| `LIGHT_OFF` | Falling pattern |
| `DOOR_LOCKED` | Secure double-click |
| `DOOR_UNLOCKED` | Single confirmation |
| `THERMOSTAT_SET` | Temperature adjusted |
| `SMOKE_ALERT` | Critical rapid pattern |

---

## WIA Ecosystem Integration

### Eye Gaze Integration

Provides haptic feedback for eye tracking interfaces:

```typescript
import { EyeGazeHaptic } from '@wia-standards/haptic/integrations/wia';

const gazeHaptic = new EyeGazeHaptic(hapticDevice, {
  dwellProgressEnabled: true,
  dwellFeedbackInterval: 100,
});

eyeTracker.on('gaze', (event) => {
  gazeHaptic.onGazeEvent({
    type: event.type,
    target: event.target,
    position: { x: event.x, y: event.y },
    dwellProgress: event.progress,
    timestamp: Date.now(),
  });
});
```

#### Eye Gaze Patterns

| Pattern | Usage |
|---------|-------|
| `TARGET_ENTER` | Gaze entered interactive element |
| `DWELL_TICK` | Progress tick during dwell |
| `DWELL_COMPLETE` | Selection successful |
| `SCREEN_EDGE` | Gaze reached screen edge |

### AAC Integration

Haptic feedback for Augmentative and Alternative Communication:

```typescript
import { AACHaptic, CATEGORY_PATTERNS } from '@wia-standards/haptic/integrations/wia';

const aacHaptic = new AACHaptic(hapticDevice, {
  scanningEnabled: true,
  categoryHapticsEnabled: true,
});

aacApp.on('event', (event) => {
  aacHaptic.onAACEvent({
    type: event.type,
    symbol: event.symbol,
    scanPosition: { row: event.row, column: event.col },
    timestamp: Date.now(),
  });
});
```

#### AAC Category Haptics

Each symbol category has a unique haptic signature:
- **Core words**: Medium frequency, single pulse
- **Actions**: Double quick pulses
- **Feelings**: Gentle rising pattern
- **Questions**: Rising tone pattern

---

## Accessibility Platform Integration

### iOS VoiceOver (Swift)

```swift
import WIAHaptic

let haptic = VoiceOverHaptic()
haptic.isEnabled = true
haptic.intensityMultiplier = 1.0

// Automatic VoiceOver integration
// Haptics play when VoiceOver focus changes

// Manual triggers
haptic.playActivation()  // Double-tap
haptic.playRotorChange() // Rotor gesture
haptic.playScreenEdge()  // Edge reached
```

### Android TalkBack (Kotlin)

```kotlin
class MyAccessibilityService : TalkBackHapticService() {
    override fun onCreate() {
        super.onCreate()
        focusHapticsEnabled = true
        actionHapticsEnabled = true
    }
}
```

AndroidManifest.xml:
```xml
<service
    android:name=".MyAccessibilityService"
    android:permission="android.permission.BIND_ACCESSIBILITY_SERVICE">
    <intent-filter>
        <action android:name="android.accessibilityservice.AccessibilityService" />
    </intent-filter>
    <meta-data
        android:name="android.accessibilityservice"
        android:resource="@xml/accessibility_config" />
</service>
```

### Web Vibration API

```typescript
import { WebAccessibilityHaptic } from '@wia-standards/haptic/integrations/accessibility';

const webHaptic = new WebAccessibilityHaptic();
webHaptic.initialize(); // Auto-binds to focus, click events

// Manual triggers
webHaptic.playEvent('click');
webHaptic.playEvent('error');
webHaptic.playEvent('notification');
```

---

## Best Practices

### 1. Intensity Guidelines

- **Informational**: 0.2-0.4 intensity
- **Interactive**: 0.4-0.6 intensity
- **Confirmation**: 0.6-0.8 intensity
- **Alert/Error**: 0.8-1.0 intensity

### 2. Timing Guidelines

- **Tick/Hover**: 20-50ms
- **Confirmation**: 50-100ms
- **Pattern sequence**: 100-300ms total
- **Alert pattern**: Can be longer, but discrete pulses

### 3. Frequency Guidelines

- **Low frequency** (80-120Hz): Warnings, errors, cold
- **Medium frequency** (140-180Hz): Normal feedback
- **High frequency** (180-250Hz): Success, warmth, urgency

### 4. User Preferences

Always allow users to:
- Enable/disable haptic feedback
- Adjust intensity multiplier
- Configure which events trigger haptics
- Choose between pattern styles

```typescript
const config = {
  enabled: userSettings.hapticsEnabled,
  intensityMultiplier: userSettings.hapticIntensity,
  categories: {
    navigation: userSettings.navHaptics,
    notifications: userSettings.notifyHaptics,
  },
};
```

### 5. Battery Considerations

- Use shorter patterns when possible
- Implement haptic coalescing for rapid events
- Provide power-saving mode with reduced haptics

### 6. Testing with Users

- Test with actual users who have visual impairments
- Gather feedback on pattern distinguishability
- Iterate on intensity and timing based on feedback

---

## Related Documentation

- [API Reference](./API-REFERENCE.md)
- [Haptic Primitives Specification](../spec/HAPTIC-PRIMITIVES.md)
- [Body Location Map](../spec/BODY-LOCATION-MAP.md)
- [Spatial Encoding](../spec/SPATIAL-SCENE.md)

---

*"촉각으로 세상을 느끼는 분들이 어디서든 안전하게 이동하고 생활할 수 있도록."*
