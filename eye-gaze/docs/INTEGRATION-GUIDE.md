# WIA Eye Gaze Standard - Integration Guide

**Phase 4: Ecosystem Integration**
**弘益人間** - 널리 인간을 이롭게

---

## Table of Contents

1. [Overview](#1-overview)
2. [WIA Ecosystem Integration](#2-wia-ecosystem-integration)
3. [OS Accessibility APIs](#3-os-accessibility-apis)
4. [Browser Extensions](#4-browser-extensions)
5. [Gaming Integration](#5-gaming-integration)
6. [Smart Home Integration](#6-smart-home-integration)
7. [Best Practices](#7-best-practices)

---

## 1. Overview

The WIA Eye Gaze Standard provides integration modules for connecting eye tracking to various platforms and applications. This guide covers the setup and usage of each integration.

### 1.1 Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Eye Tracker Device                      │
└─────────────────────┬───────────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────────┐
│               WIA Eye Gaze Core SDK                         │
│  (TypeScript / Python / Rust)                               │
└─────────────────────┬───────────────────────────────────────┘
                      │
    ┌─────────────────┼─────────────────┬─────────────────┐
    │                 │                 │                 │
    ▼                 ▼                 ▼                 ▼
┌────────┐     ┌──────────┐     ┌───────────┐     ┌───────────┐
│  WIA   │     │    OS    │     │  Browser  │     │  Gaming   │
│  AAC   │     │Accessib. │     │ Extension │     │   /IoT    │
│  BCI   │     │ APIs     │     │           │     │           │
│  CI    │     │          │     │           │     │           │
└────────┘     └──────────┘     └───────────┘     └───────────┘
```

### 1.2 Prerequisites

- WIA Eye Gaze SDK (Phase 1-3)
- Compatible eye tracker
- Platform-specific dependencies

---

## 2. WIA Ecosystem Integration

### 2.1 AAC Integration

Enable gaze-based symbol selection for AAC (Augmentative and Alternative Communication) applications.

```typescript
import { GazeToAAC, AACGrid, DwellController } from '@anthropics/wia-eye-gaze/integrations';

// Create AAC integration
const aac = new GazeToAAC({
  dwellTime: 800,        // Dwell time in ms
  blinkSelectionEnabled: false,
  wordPrediction: true,
});

// Load grid
const grid: AACGrid = {
  id: 'quick-phrases',
  name: 'Quick Phrases',
  rows: 4,
  columns: 4,
  cells: [
    { id: 'yes', label: 'Yes', action: { type: 'speak', text: 'Yes' }, ... },
    { id: 'no', label: 'No', action: { type: 'speak', text: 'No' }, ... },
    // ... more cells
  ]
};

aac.loadGrid(grid);

// Connect dwell controller
const dwellController = new DwellController();
aac.setDwellController(dwellController);

// Handle selections
aac.onSelection((result) => {
  console.log(`Selected: ${result.cell.label}`);
});

// Handle message updates
aac.onMessage((buffer) => {
  console.log(`Message: ${buffer.text}`);
});
```

### 2.2 BCI Integration

Combine gaze with EEG data for multimodal intent detection.

```typescript
import { GazeToBCI } from '@anthropics/wia-eye-gaze/integrations';

const bci = new GazeToBCI({
  useP300: true,
  useSsvep: false,
  gazeWeight: 0.6,
  confidenceThreshold: 0.7,
});

// Process combined data
tracker.subscribe((gazePoint) => {
  const eegData = bciDevice.getLatestData();
  const intent = bci.combineWithEEG(gazePoint, eegData);

  if (intent.type === 'select' && intent.confidence > 0.8) {
    // High-confidence selection detected
    handleSelection(intent.target);
  }
});

// Use gaze for BCI calibration
bci.startCalibration();
tracker.subscribe((point) => {
  bci.setBCICalibrationTarget(point);
});
```

### 2.3 CI Integration

Enhance audio sources based on gaze direction (cocktail party effect).

```typescript
import { GazeToCI, AudioSource } from '@anthropics/wia-eye-gaze/integrations';

const ci = new GazeToCI({
  dwellThreshold: 500,
  maxBoost: 12,
  suppressOthers: true,
});

// Register audio sources in the environment
ci.registerAudioSource({
  id: 'speaker-1',
  type: 'person',
  position: { x: 2, y: 1.5, z: 3 },
  loudness: 60,
  isSpeech: true,
  label: 'Speaker 1',
});

// Process 3D gaze direction
tracker.subscribe((gazePoint) => {
  if (gazePoint.gazeDirection) {
    const enhancement = ci.enhanceGazedAudioSource(gazePoint.gazeDirection);
    if (enhancement) {
      // Apply enhancement to CI device
      applyToDevice(enhancement);
    }
  }
});
```

---

## 3. OS Accessibility APIs

### 3.1 Windows UI Automation

```typescript
import { WindowsIntegration } from '@anthropics/wia-eye-gaze/integrations/os';

const windows = new WindowsIntegration();

// Register with UIA
windows.registerAsUIAutomationProvider();

// Enable Windows Eye Control compatibility
windows.enableWindowsEyeControlCompat();

// Configure
windows.setSettings({
  dwellTime: 600,
  showCursor: true,
  cursorSize: 'medium',
});

// Process gaze to UIA events
tracker.subscribe((point) => {
  const uiaEvent = windows.gazeToUIAEvent(point);
  if (uiaEvent) {
    // Event emitted to Windows accessibility system
  }
});
```

### 3.2 macOS Accessibility

```typescript
import { MacOSIntegration } from '@anthropics/wia-eye-gaze/integrations/os';

const macos = new MacOSIntegration();

// Request accessibility access
if (!macos.checkTrustedAccess()) {
  macos.requestAccessibilityAccess();
}

// Register
macos.registerWithAccessibilityAPI();

// Enable dwell-to-click
macos.setDwellConfig({
  dwellTime: 800,
  clickType: 'left',
  showFeedback: true,
});

// Process gaze
tracker.subscribe((point) => {
  macos.processGaze(point);
});

// Listen for system clicks
macos.onSystemClick((event) => {
  console.log(`Click: ${event.type} at (${event.position.x}, ${event.position.y})`);
});
```

### 3.3 Linux AT-SPI

```typescript
import { LinuxIntegration } from '@anthropics/wia-eye-gaze/integrations/os';

const linux = new LinuxIntegration();

// Register with AT-SPI2
linux.registerWithATSPI();

// Enable Orca screen reader integration
linux.orcaIntegration = true;
linux.configureOrca({
  announceFocus: true,
  announceDwell: true,
});

// Process gaze
tracker.subscribe((point) => {
  linux.processGaze(point);
});

// Listen for AT-SPI events
linux.addEventListener('object:state-changed:focused', (event) => {
  console.log(`Focused: ${event.source.name}`);
});
```

---

## 4. Browser Extensions

### 4.1 Chrome Extension Setup

1. **Load Extension**
   ```bash
   # Navigate to chrome://extensions
   # Enable "Developer mode"
   # Click "Load unpacked"
   # Select: eye-gaze/integrations/browser/chrome-extension/
   ```

2. **Configure**
   - Click extension icon
   - Enter server URL (default: `ws://localhost:8765/wia-eye-gaze/v1/stream`)
   - Adjust dwell time and cursor settings

3. **Usage**
   - Click "Connect" to connect to eye tracker
   - Click "Start" to begin tracking
   - Gaze at elements to highlight them
   - Dwell to select/click

### 4.2 WebGaze API

The content script exposes a `WiaGaze` API to web pages:

```javascript
// Detect gaze targets on page
const targets = window.WiaGaze.detectGazeTargets();

// Show/hide gaze cursor
window.WiaGaze.showGazeCursor({ color: '#00ff00', size: 50 });
window.WiaGaze.hideGazeCursor();

// Enable dwell selection
window.WiaGaze.enableDwellSelection({
  dwellTime: 800,
  showProgress: true,
});

// Get ARIA targets
const ariaTargets = window.WiaGaze.getARIATargets();
```

### 4.3 Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| Alt+G | Toggle gaze cursor |
| Alt+D | Toggle dwell selection |

---

## 5. Gaming Integration

### 5.1 Mouse Emulation

```typescript
import { GamingIntegration } from '@anthropics/wia-eye-gaze/integrations/gaming';

const gaming = new GamingIntegration(1920, 1080);

// Process gaze for mouse control
tracker.subscribe((point) => {
  gaming.processGaze(point);
});

// Listen for emulated mouse events
gaming.onMouseEvent((event) => {
  // Send to OS or game
  if (event.type === 'move') {
    moveSystemCursor(event.x, event.y);
  }
});

// Trigger click with dwell
dwell.onDwellComplete(() => {
  gaming.triggerClick('left');
});
```

### 5.2 Gamepad Emulation

```typescript
// Convert gaze to analog stick
tracker.subscribe((point) => {
  const stick = gaming.gazeToAnalogStick(point);
  // stick.x, stick.y: -1.0 to 1.0
  // stick.magnitude: 0.0 to 1.0
});

// Emit gamepad state
setInterval(() => {
  gaming.emitGamepadState();
}, 16); // 60 Hz

gaming.onGamepadState((state) => {
  // Send to virtual gamepad driver
  updateVirtualGamepad(state);
});
```

### 5.3 Aim Assist

```typescript
// Enable aim assist
gaming.configureAimAssist({
  enabled: true,
  strength: 0.5,
  slowdownRadius: 0.1,
  snapStrength: 0.3,
});

// Register game targets
game.enemies.forEach(enemy => {
  gaming.registerAimTarget({
    id: enemy.id,
    position: enemy.screenPosition,
    size: enemy.hitboxSize,
    priority: enemy.priority,
  });
});
```

### 5.4 VR/XR Integration

```typescript
// Initialize OpenXR gaze
gaming.initializeXR('openXR');

// Use 3D gaze direction for aiming
tracker.subscribe((point) => {
  if (point.gazeDirection) {
    gaming.aimAssist(point.gazeDirection);
  }
});
```

---

## 6. Smart Home Integration

### 6.1 Basic Setup

```typescript
import { SmartHomeIntegration, DeviceType } from '@anthropics/wia-eye-gaze/integrations/smarthome';

const smarthome = new SmartHomeIntegration({
  alignmentTolerance: 15,  // degrees
  dwellTime: 1000,          // ms
  userPosition: { x: 0, y: 1.5, z: 0 },
});

// Register devices
smarthome.registerDevice({
  id: 'light-1',
  name: 'Living Room Light',
  type: DeviceType.Light,
  position: { x: 2, y: 2.5, z: 3 },
  state: { on: true, brightness: 80 },
  actions: [
    { name: 'Toggle', type: 'toggle' },
    { name: 'Dim', type: 'set_brightness' },
  ],
});
```

### 6.2 Gaze-Based Control

```typescript
// Use 3D gaze for device detection
tracker.subscribe((point) => {
  if (point.gazeDirection) {
    const device = smarthome.detectGazedDevice(point.gazeDirection);
    if (device) {
      console.log(`Looking at: ${device.name}`);
    }
  }
});

// Control device
smarthome.controlDevice(device, { name: 'Toggle', type: 'toggle' });
```

### 6.3 Home Assistant Integration

```typescript
// Configure Home Assistant
smarthome.configureHomeAssistant(
  'http://homeassistant.local:8123',
  'YOUR_LONG_LIVED_TOKEN'
);

// Register HA entities
smarthome.registerDevice({
  id: 'light-kitchen',
  hassEntityId: 'light.kitchen_main',
  // ... other properties
});

// Send gaze events to HA
tracker.on('DWELL_COMPLETE', (event) => {
  smarthome.homeAssistantWebhook(event);
});
```

### 6.4 Matter Protocol

```typescript
// Enable Matter support
smarthome.enableMatter();

// Register Matter device
smarthome.registerDevice({
  id: 'matter-light-1',
  matterNodeId: '0x1234567890',
  // ... other properties
});

// Matter commands will be sent automatically
```

---

## 7. Best Practices

### 7.1 Performance

- **Throttle gaze processing** for non-critical integrations
- **Use binary protocol** for high-frequency data
- **Batch API calls** when controlling multiple devices

### 7.2 Accessibility

- Always provide **alternative input methods**
- Test with **actual AT users**
- Follow **WCAG guidelines** for web integrations
- Provide **audio/haptic feedback**

### 7.3 Security

- **Validate all inputs** from gaze data
- Use **HTTPS/WSS** for remote connections
- Store tokens securely (not in code)
- Implement **rate limiting** for actions

### 7.4 User Experience

- Allow **customizable dwell times**
- Provide clear **visual feedback**
- Support **precision mode** for small targets
- Implement **fatigue detection** for long sessions

---

## Quick Start Examples

### Simple AAC App

```typescript
import { WiaEyeTracker, DwellController, GazeToAAC } from '@anthropics/wia-eye-gaze';

async function main() {
  // Setup tracker
  const tracker = new WiaEyeTracker(new MockAdapter(60));
  await tracker.connect();

  // Setup dwell
  const dwell = new DwellController(tracker);
  dwell.setDwellTime(800);

  // Setup AAC
  const aac = new GazeToAAC();
  aac.setDwellController(dwell);
  aac.loadGrid(quickPhrasesGrid);

  // Handle selections
  aac.onSelection((result) => {
    aac.speakBuffer();
  });

  // Start tracking
  tracker.startTracking();
  dwell.start();
}
```

---

**WIA Eye Gaze Standard - Phase 4 Complete**

**弘益人間** - 널리 인간을 이롭게
