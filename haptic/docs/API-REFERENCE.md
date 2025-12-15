# WIA Haptic Standard - API Reference

Complete API reference for the WIA Haptic Standard integrations.

## Table of Contents

1. [Navigation API](#navigation-api)
2. [Smart Home API](#smart-home-api)
3. [WIA Ecosystem API](#wia-ecosystem-api)
4. [Accessibility API](#accessibility-api)
5. [Pattern Reference](#pattern-reference)

---

## Navigation API

### GoogleMapsHapticPlugin

```typescript
class GoogleMapsHapticPlugin implements NavigationHapticIntegration
```

#### Constructor

```typescript
constructor(device: IHapticDevice, config?: Partial<NavigationHapticConfig>)
```

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `config` | `NavigationHapticConfig` | Current configuration |
| `isActive` | `boolean` | Whether plugin is active |

#### Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `start()` | - | `void` | Start haptic feedback |
| `stop()` | - | `void` | Stop and cleanup |
| `pause()` | - | `void` | Pause feedback |
| `resume()` | - | `void` | Resume feedback |
| `onNavigationEvent()` | `GoogleMapsNavigationEvent` | `void` | Handle Maps event |

#### TurnByTurn Interface

```typescript
turnByTurn: {
  onApproachingTurn(direction: TurnDirection, distance: number): void;
  onTurn(direction: TurnDirection): void;
  onLaneChange(direction: 'left' | 'right'): void;
  onDestinationNear(distance: number): void;
  onDestinationReached(): void;
  onOffRoute(): void;
  onRerouting(): void;
}
```

#### ObstacleDetection Interface

```typescript
obstacleDetection: {
  onObstacleDetected(obstacle: Obstacle): void;
  onObstacleUpdated(obstacle: Obstacle): void;
  onObstacleCleared(obstacleId: string): void;
  onPathClear(): void;
  onMultipleObstacles(obstacles: Obstacle[]): void;
}
```

### NavigationHapticConfig

```typescript
interface NavigationHapticConfig {
  turnWarningDistances: {
    far: number;       // meters (default: 200)
    near: number;      // meters (default: 50)
    immediate: number; // meters (default: 10)
  };
  obstacleSettings: {
    enabled: boolean;
    minDistance: number;      // meters (default: 10)
    criticalDistance: number; // meters (default: 1)
    updateInterval: number;   // ms (default: 100)
  };
  intensityLevels: {
    info: number;     // 0-1 (default: 0.3)
    warning: number;  // 0-1 (default: 0.6)
    critical: number; // 0-1 (default: 1.0)
  };
}
```

### Obstacle Type

```typescript
interface Obstacle {
  id: string;
  type: ObstacleType;
  direction: number;   // 0-360 degrees
  distance: number;    // meters
  velocity?: number;   // m/s
  size?: 'small' | 'medium' | 'large';
  confidence: number;  // 0-1
}

type ObstacleType =
  | 'wall' | 'person' | 'vehicle' | 'bicycle'
  | 'animal' | 'pole' | 'furniture' | 'stairs_up'
  | 'stairs_down' | 'curb' | 'drop' | 'unknown';
```

---

## Smart Home API

### MatterHapticBridge

```typescript
class MatterHapticBridge implements SmartHomeHapticIntegration
```

#### Constructor

```typescript
constructor(
  hapticDevice: IHapticDevice,
  config?: Partial<SmartHomeHapticConfig>
)
```

#### Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `discoverDevices()` | - | `Promise<SmartDevice[]>` | Find Matter devices |
| `getDevice()` | `id: string` | `SmartDevice \| undefined` | Get device by ID |
| `getDevicesByRoom()` | `room: string` | `SmartDevice[]` | Filter by room |
| `getDevicesByType()` | `type: SmartDeviceType` | `SmartDevice[]` | Filter by type |
| `pointToDevice()` | `direction: number` | `SmartDevice \| null` | Spatial selection |
| `selectDevice()` | `device: SmartDevice` | `void` | Select device |
| `confirmSelection()` | - | `void` | Confirm and act |
| `cancelSelection()` | - | `void` | Cancel selection |
| `controlDevice()` | `deviceId, action` | `Promise<void>` | Control device |
| `onDeviceEvent()` | `event: SmartDeviceEvent` | `void` | Handle events |

### SmartDevice Type

```typescript
interface SmartDevice {
  id: string;
  name: string;
  type: SmartDeviceType;
  room?: string;
  state: SmartDeviceState;
  direction?: number;  // degrees from user
  distance?: number;   // meters from user
  reachable: boolean;
}

type SmartDeviceType =
  | 'light' | 'switch' | 'dimmer' | 'thermostat'
  | 'door_lock' | 'door_sensor' | 'window_sensor'
  | 'motion_sensor' | 'temperature_sensor' | 'humidity_sensor'
  | 'speaker' | 'fan' | 'blind' | 'garage_door' | 'outlet';
```

### DeviceAction Type

```typescript
type DeviceAction =
  | { type: 'toggle' }
  | { type: 'on' }
  | { type: 'off' }
  | { type: 'setBrightness'; value: number }
  | { type: 'setTemperature'; value: number }
  | { type: 'setPosition'; value: number }
  | { type: 'lock' }
  | { type: 'unlock' }
  | { type: 'open' }
  | { type: 'close' };
```

### Encoding Functions

```typescript
// Temperature to haptic pattern
function encodeTemperature(
  celsius: number,
  config?: { min: number; max: number }
): HapticPattern;

// Brightness to haptic pattern
function encodeBrightness(level: number): HapticPattern;

// Position to haptic pattern
function encodePosition(position: number): HapticPattern;
```

---

## WIA Ecosystem API

### EyeGazeHaptic

```typescript
class EyeGazeHaptic
```

#### Constructor

```typescript
constructor(
  device: IHapticDevice,
  config?: Partial<EyeGazeHapticConfig>
)
```

#### Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `start()` | - | `void` | Start feedback |
| `stop()` | - | `void` | Stop feedback |
| `setScreenDimensions()` | `width, height` | `void` | Set screen size |
| `onGazeEvent()` | `GazeEvent` | `Promise<void>` | Handle gaze event |
| `createDirectionPattern()` | `direction, screenWidth` | `HapticPattern` | Create direction pattern |

### GazeEvent Type

```typescript
interface GazeEvent {
  type: GazeEventType;
  target?: GazeTarget;
  position: { x: number; y: number };
  timestamp: number;
  dwellProgress?: number;  // 0-1
}

type GazeEventType =
  | 'enter' | 'leave' | 'dwell_start' | 'dwell_progress'
  | 'dwell_complete' | 'dwell_cancel' | 'fixation' | 'saccade' | 'blink';
```

### AACHaptic

```typescript
class AACHaptic
```

#### Constructor

```typescript
constructor(
  device: IHapticDevice,
  config?: Partial<AACHapticConfig>
)
```

#### Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `start()` | - | `void` | Start feedback |
| `stop()` | - | `void` | Stop feedback |
| `setScanMode()` | `mode: ScanMode` | `void` | Set scanning mode |
| `onAACEvent()` | `AACEvent` | `Promise<void>` | Handle AAC event |
| `createScanProgressPattern()` | `row, totalRows` | `HapticPattern` | Create scan pattern |

### AACEvent Type

```typescript
interface AACEvent {
  type: AACEventType;
  symbol?: AACSymbol;
  message?: string;
  scanPosition?: { row: number; column: number };
  timestamp: number;
}

type AACEventType =
  | 'symbol_hover' | 'symbol_select' | 'symbol_delete'
  | 'message_speak' | 'message_clear' | 'row_scan' | 'column_scan'
  | 'block_scan' | 'scan_start' | 'scan_pause' | 'switch_press' | 'page_change';
```

---

## Accessibility API

### iOS VoiceOverHaptic (Swift)

```swift
class VoiceOverHaptic: NSObject
```

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `isEnabled` | `Bool` | Enable/disable haptics |
| `intensityMultiplier` | `Float` | Scale intensity (0-1) |
| `screenCurtainHapticsEnabled` | `Bool` | Enable when screen off |

#### Methods

| Method | Description |
|--------|-------------|
| `startEngine()` | Start Core Haptics engine |
| `stopEngine()` | Stop engine |
| `playPattern(_:)` | Play WIA pattern |
| `playActivation()` | Double-tap haptic |
| `playEscape()` | Two-finger scrub haptic |
| `playMagicTap()` | Two-finger double-tap |
| `playRotorChange()` | Rotor gesture haptic |
| `playScreenEdge()` | Edge reached haptic |
| `playError()` | Error haptic |

### Android TalkBackHapticService (Kotlin)

```kotlin
class TalkBackHapticService : AccessibilityService()
```

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `focusHapticsEnabled` | `Boolean` | Enable focus haptics |
| `actionHapticsEnabled` | `Boolean` | Enable action haptics |
| `scrollHapticsEnabled` | `Boolean` | Enable scroll haptics |
| `windowHapticsEnabled` | `Boolean` | Enable window haptics |

#### Methods

| Method | Description |
|--------|-------------|
| `playReadingStart()` | Continuous reading started |
| `playReadingStop()` | Reading stopped |
| `playError()` | Error haptic |
| `playEndOfContent()` | Reached end of content |

### WebVibrationHaptic

```typescript
class WebVibrationHaptic
```

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `isSupported` | `boolean` | Vibration API available |

#### Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `setEnabled()` | `enabled: boolean` | `void` | Enable/disable |
| `playPattern()` | `HapticPattern` | `boolean` | Play pattern |
| `stop()` | - | `void` | Stop vibration |
| `vibrate()` | `duration: number` | `boolean` | Simple vibration |
| `vibratePattern()` | `number[]` | `boolean` | Raw pattern |

---

## Pattern Reference

### Navigation Patterns

| ID | Name | Duration | Intensity |
|----|------|----------|-----------|
| `nav.turn.approaching.far` | Turn Approaching (Far) | 100ms | 0.3 |
| `nav.turn.approaching.near` | Turn Approaching (Near) | 250ms | 0.5 |
| `nav.turn.left` | Turn Left | 300ms | 0.8 |
| `nav.turn.right` | Turn Right | 300ms | 0.8 |
| `nav.turn.u_turn` | U-Turn | 650ms | 0.7-0.8 |
| `nav.obstacle.detected` | Obstacle Detected | 250ms | 0.6 |
| `nav.obstacle.critical` | Obstacle Critical | 290ms | 1.0 |
| `nav.destination.reached` | Destination Reached | 900ms | 0.8-1.0 |

### Smart Home Patterns

| ID | Name | Duration | Intensity |
|----|------|----------|-----------|
| `home.light.on` | Light On | 400ms | 0.3-0.7 |
| `home.light.off` | Light Off | 400ms | 0.2-0.6 |
| `home.door.locked` | Door Locked | 300ms | 0.6-0.8 |
| `home.door.unlocked` | Door Unlocked | 150ms | 0.6 |
| `home.temp.cold` | Temperature Cold | 300ms | 0.5 (80Hz) |
| `home.temp.warm` | Temperature Warm | 300ms | 0.6 (220Hz) |
| `home.alert.smoke` | Smoke Alert | 450ms | 1.0 |

### Eye Gaze Patterns

| ID | Name | Duration | Intensity |
|----|------|----------|-----------|
| `gaze.target.enter` | Target Enter | 30ms | 0.3 |
| `gaze.dwell.tick` | Dwell Tick | 20ms | 0.4 |
| `gaze.dwell.complete` | Dwell Complete | 230ms | 0.8-0.9 |
| `gaze.screen.edge` | Screen Edge | 50ms | 0.5 |

### AAC Patterns

| ID | Name | Duration | Intensity |
|----|------|----------|-----------|
| `aac.scan.row` | Row Scan | 50ms | 0.3 |
| `aac.scan.column` | Column Scan | 50ms | 0.4 |
| `aac.symbol.select` | Symbol Selected | 190ms | 0.5-0.7 |
| `aac.message.speak` | Message Spoken | 450ms | 0.7-0.9 |
| `aac.switch.press` | Switch Press | 50ms | 0.6 |

### Accessibility Patterns

| ID | Platform | Name | Duration |
|----|----------|------|----------|
| `vo.focus.change` | iOS | Focus Change | 30ms |
| `vo.action.activate` | iOS | Activate | 140ms |
| `tb.focus.button` | Android | Button Focus | 40ms |
| `tb.action.click` | Android | Click | 140ms |
| `web.focus.interactive` | Web | Interactive Focus | 30ms |
| `web.form.submit` | Web | Form Submit | 230ms |

---

## Error Codes

| Code | Description |
|------|-------------|
| `HAPTIC_NOT_INITIALIZED` | Device not initialized |
| `HAPTIC_NOT_CONNECTED` | Device disconnected |
| `HAPTIC_INVALID_PATTERN` | Invalid pattern format |
| `HAPTIC_DEVICE_BUSY` | Device playing other pattern |
| `HAPTIC_UNSUPPORTED` | Feature not supported |
| `HAPTIC_PERMISSION_DENIED` | Permission not granted |

---

*For more information, see the [Integration Guide](./INTEGRATION-GUIDE.md).*
