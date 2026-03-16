# WIA Braille Display Standard
## Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Author:** WIA Technical Committee

---

## Table of Contents

1. [Introduction](#introduction)
2. [API Overview](#api-overview)
3. [WiaBrailleDisplay Class](#wiabrailledisplay-class)
4. [Device Discovery](#device-discovery)
5. [Cell Management](#cell-management)
6. [Cursor Operations](#cursor-operations)
7. [Key Event Handling](#key-event-handling)
8. [Event System](#event-system)
9. [Error Handling](#error-handling)
10. [TypeScript SDK Reference](#typescript-sdk-reference)
11. [REST API](#rest-api)
12. [WebSocket API](#websocket-api)

---

## 1. Introduction

The WIA Braille Display API provides a unified interface for interacting with Braille displays across different platforms and protocols. This specification defines the standard methods, events, and behaviors for Braille display integration.

### 1.1 Design Goals

- **Platform Independence**: Work seamlessly across Windows, macOS, Linux, iOS, and Android
- **Protocol Agnostic**: Support USB, Bluetooth, Serial, and network connections
- **Event-Driven**: Asynchronous architecture with comprehensive event handling
- **Type-Safe**: Full TypeScript support with detailed type definitions
- **Backwards Compatible**: Support legacy devices while embracing modern capabilities

### 1.2 Supported Languages

- TypeScript/JavaScript (Primary)
- Python
- Java/Kotlin
- Swift/Objective-C
- C/C++
- C#/.NET
- Rust

### 1.3 API Layers

```
┌─────────────────────────────────────┐
│   Application Layer                 │
│   (Screen Readers, Apps)            │
└─────────────────────────────────────┘
           ↕
┌─────────────────────────────────────┐
│   WIA Braille Display API           │
│   (High-Level Interface)            │
└─────────────────────────────────────┘
           ↕
┌─────────────────────────────────────┐
│   Protocol Layer                    │
│   (USB HID, Bluetooth, Serial)      │
└─────────────────────────────────────┘
           ↕
┌─────────────────────────────────────┐
│   Hardware Layer                    │
│   (Physical Braille Display)        │
└─────────────────────────────────────┘
```

---

## 2. API Overview

### 2.1 Core Concepts

#### Device
A physical or virtual Braille display that can be connected and controlled.

#### Cell
A single Braille cell that can display 6 or 8 dots.

#### Display Buffer
The current state of all cells on the display.

#### Cursor Routing
Buttons or touch sensors above each cell for navigation.

#### Input Keys
Additional buttons for commands (scroll, navigation, etc.).

### 2.2 API Architecture

```typescript
// Core namespace
namespace WiaBraille {
  // Device management
  class DeviceManager {
    static discover(): Promise<Device[]>
    static connect(device: Device): Promise<BrailleDisplay>
    static disconnect(display: BrailleDisplay): Promise<void>
  }

  // Main display interface
  class BrailleDisplay extends EventEmitter {
    // Properties
    readonly config: DisplayConfiguration
    readonly connected: boolean

    // Cell operations
    writeCells(cells: BrailleCell[], offset?: number): Promise<void>
    writeText(text: string, offset?: number): Promise<void>
    clear(): Promise<void>
    scroll(direction: ScrollDirection, amount?: number): Promise<void>

    // Cursor operations
    getCursor(): CursorPosition
    setCursor(position: CursorPosition): Promise<void>
    moveCursor(delta: { line?: number; cell?: number }): Promise<void>

    // Events
    on(event: 'key', handler: (key: KeyEvent) => void): this
    on(event: 'route', handler: (cell: number) => void): this
    on(event: 'disconnect', handler: () => void): this
  }
}
```

### 2.3 Quick Start Example

```typescript
import { DeviceManager, BrailleDisplay } from 'wia-braille';

async function main() {
  // Discover devices
  const devices = await DeviceManager.discover();
  console.log(`Found ${devices.length} devices`);

  // Connect to first device
  const display = await DeviceManager.connect(devices[0]);
  console.log(`Connected to ${display.config.name}`);

  // Write text
  await display.writeText('Hello World!');

  // Listen for key events
  display.on('key', (key) => {
    console.log(`Key pressed: ${key.name}`);
  });

  // Listen for cursor routing
  display.on('route', (cell) => {
    console.log(`Cell ${cell} pressed`);
  });
}

main().catch(console.error);
```

---

## 3. WiaBrailleDisplay Class

### 3.1 Class Definition

```typescript
class BrailleDisplay extends EventEmitter {
  // Properties
  readonly id: string;
  readonly config: DisplayConfiguration;
  readonly connected: boolean;
  readonly lastUpdate: Date;

  // Constructor
  constructor(device: Device, options?: DisplayOptions);

  // Connection management
  connect(): Promise<void>;
  disconnect(): Promise<void>;
  reconnect(): Promise<void>;

  // Cell operations
  writeCells(cells: BrailleCell[], offset?: number): Promise<void>;
  writeText(text: string, offset?: number, options?: TextOptions): Promise<void>;
  writeLine(line: number, cells: BrailleCell[]): Promise<void>;
  clear(line?: number): Promise<void>;
  clearCell(position: CellPosition): Promise<void>;

  // Buffer operations
  getBuffer(): DisplayState;
  setBuffer(state: DisplayState): Promise<void>;
  updateBuffer(updates: CellUpdate[]): Promise<void>;

  // Cursor operations
  getCursor(): CursorPosition;
  setCursor(position: CursorPosition): Promise<void>;
  moveCursor(delta: CursorDelta): Promise<void>;
  showCursor(visible: boolean): Promise<void>;
  setCursorShape(shape: CursorShape): Promise<void>;

  // Scroll operations
  scroll(direction: ScrollDirection, amount?: number): Promise<void>;
  scrollTo(offset: number): Promise<void>;
  getScrollPosition(): number;

  // Status operations
  setStatus(cells: BrailleCell[]): Promise<void>;
  clearStatus(): Promise<void>;

  // Device information
  getInfo(): DeviceInfo;
  getCapabilities(): DeviceCapabilities;
  getFirmwareVersion(): string;

  // Event handling
  on(event: string, handler: (...args: any[]) => void): this;
  off(event: string, handler: (...args: any[]) => void): this;
  once(event: string, handler: (...args: any[]) => void): this;
}
```

### 3.2 Configuration Interface

```typescript
interface DisplayConfiguration {
  // Device identification
  id: string;
  name: string;
  manufacturer: string;
  model: string;

  // Display dimensions
  dimensions: {
    width: number;
    height: number;
  };

  // Cell format
  cellFormat: 6 | 8;
  totalCells: number;

  // Capabilities
  cursorRouting: boolean;
  statusCells: number;
  inputKeys: number;

  // Connection
  connection: {
    type: 'usb' | 'bluetooth' | 'serial' | 'network';
    address: string;
    protocol: string;
  };
}
```

### 3.3 Device Options

```typescript
interface DisplayOptions {
  // Auto-reconnect on disconnect
  autoReconnect?: boolean;

  // Reconnect delay (ms)
  reconnectDelay?: number;

  // Maximum reconnect attempts
  maxReconnectAttempts?: number;

  // Update throttle (ms)
  updateThrottle?: number;

  // Enable buffering
  buffered?: boolean;

  // Translation table
  translationTable?: string;

  // Default locale
  locale?: string;

  // Debug mode
  debug?: boolean;
}
```

### 3.4 Constructor Example

```typescript
import { BrailleDisplay } from 'wia-braille';

const display = new BrailleDisplay(device, {
  autoReconnect: true,
  reconnectDelay: 5000,
  maxReconnectAttempts: 3,
  updateThrottle: 50,
  buffered: true,
  translationTable: 'en-us-g2.ctb',
  locale: 'en-US',
  debug: false,
});

await display.connect();
```

---

## 4. Device Discovery

### 4.1 DeviceManager Class

```typescript
class DeviceManager {
  // Discover all available devices
  static discover(options?: DiscoveryOptions): Promise<Device[]>;

  // Discover by connection type
  static discoverUSB(): Promise<Device[]>;
  static discoverBluetooth(): Promise<Device[]>;
  static discoverSerial(): Promise<Device[]>;
  static discoverNetwork(): Promise<Device[]>;

  // Connect to device
  static connect(device: Device, options?: DisplayOptions): Promise<BrailleDisplay>;

  // Disconnect device
  static disconnect(display: BrailleDisplay): Promise<void>;

  // List connected displays
  static getConnected(): BrailleDisplay[];

  // Watch for device changes
  static watch(callback: (event: DeviceEvent) => void): () => void;
}
```

### 4.2 Discovery Options

```typescript
interface DiscoveryOptions {
  // Connection types to search
  types?: ('usb' | 'bluetooth' | 'serial' | 'network')[];

  // Discovery timeout (ms)
  timeout?: number;

  // Include virtual devices
  includeVirtual?: boolean;

  // Filter by manufacturer
  manufacturer?: string;

  // Filter by model
  model?: string;

  // Maximum devices to find
  maxDevices?: number;
}
```

### 4.3 Device Interface

```typescript
interface Device {
  // Identification
  id: string;
  name: string;
  manufacturer: string;
  model: string;
  serialNumber?: string;

  // Connection
  connection: {
    type: 'usb' | 'bluetooth' | 'serial' | 'network';
    address: string;
    protocol: string;
  };

  // Capabilities
  capabilities: DeviceCapabilities;

  // Status
  connected: boolean;
  paired?: boolean;
  battery?: BatteryInfo;
}
```

### 4.4 Device Capabilities

```typescript
interface DeviceCapabilities {
  // Display configuration
  width: number;
  height: number;
  cellFormat: 6 | 8;
  totalCells: number;

  // Input capabilities
  cursorRouting: boolean;
  inputKeys: string[];
  touchPad?: boolean;
  joystick?: boolean;

  // Display features
  statusCells: number;
  vibration?: boolean;
  audioFeedback?: boolean;

  // Advanced features
  bluetooth?: {
    version: string;
    profiles: string[];
  };
  usb?: {
    version: string;
    vendorId: number;
    productId: number;
  };
}
```

### 4.5 Discovery Examples

#### Basic Discovery

```typescript
import { DeviceManager } from 'wia-braille';

// Find all devices
const devices = await DeviceManager.discover();
console.log(`Found ${devices.length} devices`);

devices.forEach(device => {
  console.log(`- ${device.name} (${device.manufacturer})`);
  console.log(`  Connection: ${device.connection.type}`);
  console.log(`  Size: ${device.capabilities.width}x${device.capabilities.height}`);
});
```

#### Filtered Discovery

```typescript
// Find only Bluetooth devices from specific manufacturer
const devices = await DeviceManager.discover({
  types: ['bluetooth'],
  manufacturer: 'Freedom Scientific',
  timeout: 10000,
  maxDevices: 5,
});
```

#### USB-Only Discovery

```typescript
// Find USB devices only
const usbDevices = await DeviceManager.discoverUSB();

for (const device of usbDevices) {
  console.log(`USB Device: ${device.name}`);
  console.log(`  Vendor ID: 0x${device.capabilities.usb?.vendorId.toString(16)}`);
  console.log(`  Product ID: 0x${device.capabilities.usb?.productId.toString(16)}`);
}
```

#### Watch for Device Changes

```typescript
// Watch for device connect/disconnect
const unwatch = DeviceManager.watch((event) => {
  if (event.type === 'connected') {
    console.log(`Device connected: ${event.device.name}`);
  } else if (event.type === 'disconnected') {
    console.log(`Device disconnected: ${event.device.name}`);
  }
});

// Stop watching later
// unwatch();
```

---

## 5. Cell Management

### 5.1 Writing Cells

#### writeCells Method

```typescript
writeCells(cells: BrailleCell[], offset?: number): Promise<void>
```

Write an array of Braille cells to the display.

**Parameters:**
- `cells`: Array of BrailleCell objects
- `offset`: Starting cell position (default: 0)

**Example:**

```typescript
const cells: BrailleCell[] = [
  { dots: 33, format: 6, unicode: '⠡', char: 'H' },
  { dots: 23, format: 6, unicode: '⠗', char: 'e' },
  { dots: 15, format: 6, unicode: '⠇', char: 'l' },
  { dots: 15, format: 6, unicode: '⠇', char: 'l' },
  { dots: 135, format: 6, unicode: '⠕', char: 'o' },
];

await display.writeCells(cells);
```

#### writeText Method

```typescript
writeText(text: string, offset?: number, options?: TextOptions): Promise<void>
```

Write text to the display (automatically translated to Braille).

**Parameters:**
- `text`: Text to display
- `offset`: Starting cell position (default: 0)
- `options`: Text rendering options

**Example:**

```typescript
await display.writeText('Hello World!');

// With options
await display.writeText('Hello World!', 0, {
  grade: 2,
  locale: 'en-US',
  wrap: true,
});
```

#### writeLine Method

```typescript
writeLine(line: number, cells: BrailleCell[]): Promise<void>
```

Write cells to a specific line (for multi-line displays).

**Example:**

```typescript
// Write to line 0
await display.writeLine(0, headerCells);

// Write to line 1
await display.writeLine(1, contentCells);
```

### 5.2 Text Options

```typescript
interface TextOptions {
  // Translation grade (1=uncontracted, 2=contracted)
  grade?: 1 | 2;

  // Locale for translation
  locale?: string;

  // Translation table
  translationTable?: string;

  // Text wrapping
  wrap?: boolean;

  // Text alignment
  align?: 'left' | 'center' | 'right';

  // Truncate if too long
  truncate?: boolean;

  // Truncation indicator
  truncateIndicator?: string;
}
```

### 5.3 Clear Operations

```typescript
// Clear entire display
await display.clear();

// Clear specific line
await display.clear(0);

// Clear single cell
await display.clearCell({ line: 0, cell: 5 });
```

### 5.4 Buffer Operations

#### Get Buffer

```typescript
const state = display.getBuffer();
console.log('Current display state:', state);
```

#### Set Buffer

```typescript
const newState: DisplayState = {
  version: '1.0.0',
  timestamp: new Date().toISOString(),
  display: {
    config: display.config,
    lines: [
      {
        lineNumber: 0,
        cells: [/* ... */],
        length: 40,
      },
    ],
  },
};

await display.setBuffer(newState);
```

#### Update Buffer (Partial)

```typescript
const updates: CellUpdate[] = [
  {
    position: { line: 0, cell: 5 },
    cell: { dots: 2, format: 6, unicode: '⠂', char: ',' },
  },
  {
    position: { line: 0, cell: 6 },
    cell: { dots: 0, format: 6, unicode: '⠀', char: ' ' },
  },
];

await display.updateBuffer(updates);
```

### 5.5 Advanced Cell Operations

#### Blinking Cells

```typescript
const blinkingCells: BrailleCell[] = [
  {
    dots: 135,
    format: 6,
    unicode: '⠕',
    char: 'O',
    attributes: { blink: true },
  },
];

await display.writeCells(blinkingCells, 10);
```

#### Emphasized Cells

```typescript
const emphasizedCells: BrailleCell[] = [
  {
    dots: 1,
    format: 8,
    unicode: '⠁',
    char: 'A',
    attributes: { emphasis: 3 }, // Maximum emphasis
  },
];

await display.writeCells(emphasizedCells);
```

#### Selected Region

```typescript
// Highlight cells 10-20
const cells = display.getBuffer().display.lines[0].cells;

for (let i = 10; i <= 20; i++) {
  cells[i].attributes = { selected: true };
}

await display.writeCells(cells);
```

---

## 6. Cursor Operations

### 6.1 Get Cursor Position

```typescript
const cursor = display.getCursor();
console.log(`Cursor at line ${cursor.line}, cell ${cursor.cell}`);
console.log(`Visible: ${cursor.visible}`);
console.log(`Shape: ${cursor.shape}`);
```

### 6.2 Set Cursor Position

```typescript
await display.setCursor({
  line: 0,
  cell: 10,
  visible: true,
  shape: 'block',
});
```

### 6.3 Move Cursor

```typescript
// Move right 5 cells
await display.moveCursor({ cell: 5 });

// Move down 1 line
await display.moveCursor({ line: 1 });

// Move diagonally
await display.moveCursor({ line: 1, cell: -3 });
```

### 6.4 Cursor Visibility

```typescript
// Show cursor
await display.showCursor(true);

// Hide cursor
await display.showCursor(false);
```

### 6.5 Cursor Shape

```typescript
// Block cursor (fills entire cell)
await display.setCursorShape('block');

// Underline cursor (dots 7-8 only)
await display.setCursorShape('underline');

// Vertical cursor (dots 4-6 only)
await display.setCursorShape('vertical');
```

### 6.6 Cursor Types

```typescript
type CursorShape = 'block' | 'underline' | 'vertical';

interface CursorPosition {
  line: number;
  cell: number;
  visible: boolean;
  shape?: CursorShape;
}

interface CursorDelta {
  line?: number;
  cell?: number;
}
```

---

## 7. Key Event Handling

### 7.1 Key Event Interface

```typescript
interface KeyEvent {
  // Key identification
  name: string;
  code: number;

  // Event type
  type: 'press' | 'release' | 'repeat';

  // Timestamp
  timestamp: Date;

  // Modifiers
  modifiers: {
    shift?: boolean;
    control?: boolean;
    alt?: boolean;
    meta?: boolean;
  };

  // Original event
  raw?: any;
}
```

### 7.2 Standard Key Names

```typescript
// Navigation keys
'up', 'down', 'left', 'right'
'pageUp', 'pageDown'
'home', 'end'

// Action keys
'enter', 'escape', 'space'
'backspace', 'delete'

// Function keys
'f1', 'f2', 'f3', ... 'f12'

// Modifier keys
'shift', 'control', 'alt', 'meta'

// Braille-specific keys
'dot1', 'dot2', 'dot3', 'dot4', 'dot5', 'dot6', 'dot7', 'dot8'
'space'

// Device-specific keys
'scroll-left', 'scroll-right'
'advance', 'back'
'panning-left', 'panning-right'
```

### 7.3 Listening for Key Events

```typescript
display.on('key', (event: KeyEvent) => {
  console.log(`Key: ${event.name} (${event.type})`);

  if (event.modifiers.shift) {
    console.log('Shift modifier active');
  }

  // Handle specific keys
  switch (event.name) {
    case 'up':
      // Handle up arrow
      break;
    case 'down':
      // Handle down arrow
      break;
    case 'enter':
      // Handle enter
      break;
  }
});
```

### 7.4 Cursor Routing Events

```typescript
display.on('route', (cell: number) => {
  console.log(`Cursor routing button ${cell} pressed`);

  // Move cursor to that cell
  display.setCursor({
    line: 0,
    cell: cell,
    visible: true,
  });
});
```

### 7.5 Chord Input (Multiple Keys)

```typescript
interface ChordEvent {
  dots: number[];
  char?: string;
  timestamp: Date;
}

display.on('chord', (event: ChordEvent) => {
  console.log(`Chord input: dots ${event.dots.join(',')}`);
  if (event.char) {
    console.log(`Translates to: ${event.char}`);
  }
});
```

### 7.6 Key Filtering

```typescript
// Only handle specific keys
display.on('key', (event: KeyEvent) => {
  const navigationKeys = ['up', 'down', 'left', 'right'];

  if (!navigationKeys.includes(event.name)) {
    return;
  }

  // Handle navigation
});
```

---

## 8. Event System

### 8.1 Event Types

```typescript
interface DisplayEvents {
  // Connection events
  'connected': () => void;
  'disconnected': () => void;
  'reconnecting': () => void;
  'reconnected': () => void;
  'error': (error: Error) => void;

  // Input events
  'key': (event: KeyEvent) => void;
  'route': (cell: number) => void;
  'chord': (event: ChordEvent) => void;

  // Display events
  'update': (state: DisplayState) => void;
  'clear': () => void;
  'scroll': (position: number) => void;

  // Cursor events
  'cursor-move': (position: CursorPosition) => void;
  'cursor-show': (visible: boolean) => void;

  // Battery events
  'battery': (info: BatteryInfo) => void;
  'battery-low': (percent: number) => void;
}
```

### 8.2 Event Subscription

```typescript
// Subscribe to event
const handler = (event: KeyEvent) => {
  console.log('Key pressed:', event.name);
};
display.on('key', handler);

// Unsubscribe
display.off('key', handler);

// One-time subscription
display.once('connected', () => {
  console.log('Connected!');
});
```

### 8.3 Event Emitter Methods

```typescript
class BrailleDisplay extends EventEmitter {
  on(event: string, handler: Function): this;
  off(event: string, handler: Function): this;
  once(event: string, handler: Function): this;
  emit(event: string, ...args: any[]): boolean;
  removeAllListeners(event?: string): this;
  listenerCount(event: string): number;
}
```

### 8.4 Advanced Event Handling

```typescript
// Error handling
display.on('error', (error: Error) => {
  console.error('Display error:', error.message);
  // Attempt recovery
});

// Auto-reconnect
display.on('disconnected', async () => {
  console.log('Display disconnected, attempting to reconnect...');
  await display.reconnect();
});

// Battery monitoring
display.on('battery-low', (percent: number) => {
  console.warn(`Battery low: ${percent}%`);
  // Show warning to user
});

// Update tracking
display.on('update', (state: DisplayState) => {
  console.log('Display updated at:', state.timestamp);
});
```

---

## 9. Error Handling

### 9.1 Error Types

```typescript
class BrailleError extends Error {
  code: string;
  details?: any;
}

class ConnectionError extends BrailleError {
  code: 'CONNECTION_FAILED' | 'CONNECTION_LOST' | 'CONNECTION_TIMEOUT';
}

class DeviceError extends BrailleError {
  code: 'DEVICE_NOT_FOUND' | 'DEVICE_BUSY' | 'DEVICE_UNSUPPORTED';
}

class ProtocolError extends BrailleError {
  code: 'INVALID_COMMAND' | 'INVALID_DATA' | 'PROTOCOL_MISMATCH';
}

class ValidationError extends BrailleError {
  code: 'INVALID_CELL' | 'INVALID_POSITION' | 'INVALID_STATE';
}
```

### 9.2 Error Codes

```typescript
enum BrailleErrorCode {
  // Connection errors
  CONNECTION_FAILED = 'CONNECTION_FAILED',
  CONNECTION_LOST = 'CONNECTION_LOST',
  CONNECTION_TIMEOUT = 'CONNECTION_TIMEOUT',

  // Device errors
  DEVICE_NOT_FOUND = 'DEVICE_NOT_FOUND',
  DEVICE_BUSY = 'DEVICE_BUSY',
  DEVICE_UNSUPPORTED = 'DEVICE_UNSUPPORTED',
  DEVICE_NOT_READY = 'DEVICE_NOT_READY',

  // Protocol errors
  INVALID_COMMAND = 'INVALID_COMMAND',
  INVALID_DATA = 'INVALID_DATA',
  PROTOCOL_MISMATCH = 'PROTOCOL_MISMATCH',
  COMMAND_FAILED = 'COMMAND_FAILED',

  // Validation errors
  INVALID_CELL = 'INVALID_CELL',
  INVALID_POSITION = 'INVALID_POSITION',
  INVALID_STATE = 'INVALID_STATE',
  BUFFER_OVERFLOW = 'BUFFER_OVERFLOW',

  // Permission errors
  PERMISSION_DENIED = 'PERMISSION_DENIED',
  ACCESS_DENIED = 'ACCESS_DENIED',

  // Generic errors
  UNKNOWN_ERROR = 'UNKNOWN_ERROR',
  TIMEOUT = 'TIMEOUT',
}
```

### 9.3 Error Handling Examples

#### Try-Catch Pattern

```typescript
try {
  await display.writeCells(cells);
} catch (error) {
  if (error instanceof ConnectionError) {
    console.error('Connection lost:', error.message);
    await display.reconnect();
  } else if (error instanceof ValidationError) {
    console.error('Invalid cell data:', error.details);
  } else {
    console.error('Unexpected error:', error);
  }
}
```

#### Error Event Handling

```typescript
display.on('error', (error: BrailleError) => {
  console.error(`Error [${error.code}]: ${error.message}`);

  switch (error.code) {
    case BrailleErrorCode.CONNECTION_LOST:
      // Attempt reconnection
      display.reconnect();
      break;

    case BrailleErrorCode.DEVICE_NOT_READY:
      // Wait and retry
      setTimeout(() => {
        display.writeCells(cells);
      }, 1000);
      break;

    case BrailleErrorCode.PERMISSION_DENIED:
      // Request permissions
      requestPermissions();
      break;

    default:
      // Log and report
      reportError(error);
  }
});
```

#### Graceful Degradation

```typescript
async function safeWrite(display: BrailleDisplay, text: string) {
  try {
    await display.writeText(text);
  } catch (error) {
    console.warn('Failed to write to display:', error);

    // Fallback to audio output
    speakText(text);
  }
}
```

### 9.4 Validation Helpers

```typescript
function validateCellPosition(
  position: { line: number; cell: number },
  config: DisplayConfiguration
): void {
  if (position.line < 0 || position.line >= config.dimensions.height) {
    throw new ValidationError(
      `Line ${position.line} out of range (0-${config.dimensions.height - 1})`
    );
  }

  if (position.cell < 0 || position.cell >= config.dimensions.width) {
    throw new ValidationError(
      `Cell ${position.cell} out of range (0-${config.dimensions.width - 1})`
    );
  }
}

function validateCells(cells: BrailleCell[], format: 6 | 8): void {
  const maxDots = format === 6 ? 63 : 255;

  for (const cell of cells) {
    if (cell.dots < 0 || cell.dots > maxDots) {
      throw new ValidationError(
        `Invalid dots value ${cell.dots} for ${format}-dot format`
      );
    }

    if (cell.format !== format) {
      throw new ValidationError(
        `Cell format ${cell.format} doesn't match display format ${format}`
      );
    }
  }
}
```

---

## 10. TypeScript SDK Reference

### 10.1 Installation

```bash
npm install wia-braille
```

```bash
yarn add wia-braille
```

### 10.2 Importing

```typescript
// ES6 modules
import {
  DeviceManager,
  BrailleDisplay,
  BrailleCell,
  DisplayConfiguration,
} from 'wia-braille';

// CommonJS
const {
  DeviceManager,
  BrailleDisplay,
} = require('wia-braille');
```

### 10.3 Complete Example

```typescript
import { DeviceManager, BrailleDisplay, BrailleCell } from 'wia-braille';

class BrailleApp {
  private display: BrailleDisplay | null = null;

  async initialize() {
    // Discover devices
    const devices = await DeviceManager.discover({
      types: ['usb', 'bluetooth'],
      timeout: 5000,
    });

    if (devices.length === 0) {
      throw new Error('No Braille displays found');
    }

    // Connect to first device
    this.display = await DeviceManager.connect(devices[0], {
      autoReconnect: true,
      reconnectDelay: 3000,
      locale: 'en-US',
    });

    // Setup event handlers
    this.setupEventHandlers();

    console.log(`Connected to ${this.display.config.name}`);
  }

  private setupEventHandlers() {
    if (!this.display) return;

    // Key events
    this.display.on('key', (event) => {
      this.handleKey(event);
    });

    // Cursor routing
    this.display.on('route', (cell) => {
      this.handleRoute(cell);
    });

    // Connection events
    this.display.on('disconnected', () => {
      console.warn('Display disconnected');
    });

    this.display.on('reconnected', () => {
      console.log('Display reconnected');
    });

    // Error handling
    this.display.on('error', (error) => {
      console.error('Display error:', error);
    });
  }

  async showMessage(text: string) {
    if (!this.display) return;

    await this.display.clear();
    await this.display.writeText(text, 0, {
      grade: 2,
      wrap: true,
      align: 'center',
    });
  }

  async showCells(cells: BrailleCell[]) {
    if (!this.display) return;

    await this.display.writeCells(cells);
  }

  private handleKey(event: KeyEvent) {
    switch (event.name) {
      case 'up':
        this.scrollUp();
        break;
      case 'down':
        this.scrollDown();
        break;
      case 'enter':
        this.activateCurrent();
        break;
    }
  }

  private handleRoute(cell: number) {
    console.log(`User pressed cell ${cell}`);
    // Handle cursor routing
  }

  private async scrollUp() {
    if (!this.display) return;
    await this.display.scroll('up', 1);
  }

  private async scrollDown() {
    if (!this.display) return;
    await this.display.scroll('down', 1);
  }

  private activateCurrent() {
    // Activate current item
  }

  async shutdown() {
    if (this.display) {
      await this.display.clear();
      await DeviceManager.disconnect(this.display);
      this.display = null;
    }
  }
}

// Usage
const app = new BrailleApp();

await app.initialize();
await app.showMessage('Hello, Braille World!');

// Cleanup on exit
process.on('SIGINT', async () => {
  await app.shutdown();
  process.exit(0);
});
```

### 10.4 React Integration

```typescript
import React, { useEffect, useState } from 'react';
import { DeviceManager, BrailleDisplay } from 'wia-braille';

function useBrailleDisplay() {
  const [display, setDisplay] = useState<BrailleDisplay | null>(null);
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState<Error | null>(null);

  useEffect(() => {
    let currentDisplay: BrailleDisplay | null = null;

    async function connect() {
      try {
        const devices = await DeviceManager.discover();

        if (devices.length === 0) {
          setError(new Error('No devices found'));
          return;
        }

        currentDisplay = await DeviceManager.connect(devices[0]);
        setDisplay(currentDisplay);
        setConnected(true);

        currentDisplay.on('disconnected', () => {
          setConnected(false);
        });

        currentDisplay.on('error', (err) => {
          setError(err);
        });
      } catch (err) {
        setError(err as Error);
      }
    }

    connect();

    return () => {
      if (currentDisplay) {
        DeviceManager.disconnect(currentDisplay);
      }
    };
  }, []);

  return { display, connected, error };
}

function BrailleComponent() {
  const { display, connected, error } = useBrailleDisplay();
  const [message, setMessage] = useState('');

  const sendMessage = async () => {
    if (display && connected) {
      await display.writeText(message);
    }
  };

  if (error) {
    return <div>Error: {error.message}</div>;
  }

  if (!connected) {
    return <div>Connecting to Braille display...</div>;
  }

  return (
    <div>
      <h1>Braille Display</h1>
      <input
        type="text"
        value={message}
        onChange={(e) => setMessage(e.target.value)}
        placeholder="Enter text"
      />
      <button onClick={sendMessage}>Send to Display</button>
    </div>
  );
}
```

---

## 11. REST API

### 11.1 Base URL

```
https://api.wia-braille.org/v1
```

### 11.2 Authentication

```http
Authorization: Bearer <api_key>
```

### 11.3 Endpoints

#### List Devices

```http
GET /devices
```

**Response:**

```json
{
  "devices": [
    {
      "id": "device-123",
      "name": "Focus 40 Blue",
      "manufacturer": "Freedom Scientific",
      "connection": {
        "type": "bluetooth",
        "address": "00:11:22:33:44:55"
      }
    }
  ]
}
```

#### Connect to Device

```http
POST /devices/{deviceId}/connect
```

**Response:**

```json
{
  "sessionId": "session-456",
  "displayId": "display-789",
  "config": {
    "width": 40,
    "height": 1,
    "cellFormat": 8
  }
}
```

#### Write Cells

```http
POST /displays/{displayId}/cells
```

**Request:**

```json
{
  "cells": [
    {"dots": 33, "format": 6, "char": "H"},
    {"dots": 23, "format": 6, "char": "e"}
  ],
  "offset": 0
}
```

#### Write Text

```http
POST /displays/{displayId}/text
```

**Request:**

```json
{
  "text": "Hello World",
  "offset": 0,
  "options": {
    "grade": 2,
    "locale": "en-US"
  }
}
```

#### Get Display State

```http
GET /displays/{displayId}/state
```

**Response:**

```json
{
  "version": "1.0.0",
  "timestamp": "2025-12-25T10:00:00Z",
  "display": {
    "config": {...},
    "lines": [...]
  }
}
```

---

## 12. WebSocket API

### 12.1 Connection

```javascript
const ws = new WebSocket('wss://api.wia-braille.org/v1/ws');

ws.onopen = () => {
  console.log('Connected');

  // Authenticate
  ws.send(JSON.stringify({
    type: 'auth',
    apiKey: 'your-api-key',
  }));
};
```

### 12.2 Message Format

```typescript
interface WebSocketMessage {
  type: string;
  payload: any;
  timestamp?: string;
  id?: string;
}
```

### 12.3 Commands

#### Write Cells

```json
{
  "type": "writeCells",
  "payload": {
    "displayId": "display-789",
    "cells": [...],
    "offset": 0
  }
}
```

#### Subscribe to Events

```json
{
  "type": "subscribe",
  "payload": {
    "displayId": "display-789",
    "events": ["key", "route", "disconnect"]
  }
}
```

### 12.4 Events

```json
{
  "type": "key",
  "payload": {
    "displayId": "display-789",
    "event": {
      "name": "up",
      "type": "press",
      "timestamp": "2025-12-25T10:00:00Z"
    }
  }
}
```

---

**Document End**

© 2025 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity

For questions or contributions, please visit:
https://github.com/WIA-Official/wia-standards
