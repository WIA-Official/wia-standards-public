# WIA Braille Display Standard
## Phase 3: Communication Protocol Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Author:** WIA Technical Committee

---

## Table of Contents

1. [Introduction](#introduction)
2. [Protocol Overview](#protocol-overview)
3. [USB HID Protocol](#usb-hid-protocol)
4. [Bluetooth LE GATT Protocol](#bluetooth-le-gatt-protocol)
5. [Serial Communication Protocol](#serial-communication-protocol)
6. [BRLTTY Driver Protocol](#brltty-driver-protocol)
7. [Network Protocol](#network-protocol)
8. [Message Formats](#message-formats)
9. [Security Requirements](#security-requirements)
10. [Protocol Negotiation](#protocol-negotiation)
11. [Error Handling and Recovery](#error-handling-and-recovery)
12. [Performance Optimization](#performance-optimization)

---

## 1. Introduction

This specification defines the low-level communication protocols for WIA-compliant Braille displays. These protocols ensure reliable, efficient, and secure communication between software applications and hardware devices across multiple connection types.

### 1.1 Supported Protocols

| Protocol | Connection Type | Typical Use Case | Latency | Bandwidth |
|----------|----------------|------------------|---------|-----------|
| USB HID | Wired (USB) | Desktop/Laptop | <10ms | High |
| Bluetooth LE GATT | Wireless (BLE) | Mobile/Desktop | <50ms | Medium |
| Serial (RS-232) | Wired (Serial) | Legacy devices | <20ms | Low |
| BRLTTY | Software | Linux integration | <5ms | High |
| Network (TCP/IP) | Network | Remote access | Variable | Variable |

### 1.2 Design Principles

1. **Backwards Compatibility**: Support legacy devices while enabling modern features
2. **Efficiency**: Minimize bandwidth and power consumption
3. **Reliability**: Ensure data integrity and error recovery
4. **Security**: Protect against unauthorized access and tampering
5. **Extensibility**: Allow for future protocol enhancements

### 1.3 Protocol Layers

```
┌─────────────────────────────────────┐
│   Application Layer                 │
│   (WIA Braille API)                 │
└─────────────────────────────────────┘
           ↕
┌─────────────────────────────────────┐
│   Message Layer                     │
│   (Command/Response Format)         │
└─────────────────────────────────────┘
           ↕
┌─────────────────────────────────────┐
│   Transport Layer                   │
│   (USB/BLE/Serial/Network)          │
└─────────────────────────────────────┐
           ↕
┌─────────────────────────────────────┐
│   Physical Layer                    │
│   (Hardware Interface)              │
└─────────────────────────────────────┘
```

---

## 2. Protocol Overview

### 2.1 Common Message Structure

All protocols use a common message structure:

```
┌───────┬────────┬─────────┬────────┬─────────┬──────────┐
│ MAGIC │ LENGTH │ COMMAND │ SEQ_ID │ PAYLOAD │ CHECKSUM │
├───────┼────────┼─────────┼────────┼─────────┼──────────┤
│ 2B    │ 2B     │ 1B      │ 2B     │ 0-255B  │ 2B       │
└───────┴────────┴─────────┴────────┴─────────┴──────────┘
```

**Fields:**
- **MAGIC**: Protocol identifier (0x5749 = "WI")
- **LENGTH**: Total message length including header (big-endian)
- **COMMAND**: Command/response code (see Command Codes section)
- **SEQ_ID**: Sequence identifier for request/response matching
- **PAYLOAD**: Command-specific data
- **CHECKSUM**: CRC-16 of entire message excluding checksum field

### 2.2 Command Codes

```typescript
enum CommandCode {
  // Device information
  GET_INFO = 0x01,
  GET_CAPABILITIES = 0x02,
  GET_STATUS = 0x03,

  // Display operations
  WRITE_CELLS = 0x10,
  WRITE_LINE = 0x11,
  WRITE_REGION = 0x12,
  CLEAR_DISPLAY = 0x13,
  CLEAR_LINE = 0x14,

  // Cursor operations
  SET_CURSOR = 0x20,
  GET_CURSOR = 0x21,
  SHOW_CURSOR = 0x22,
  HIDE_CURSOR = 0x23,

  // Input operations
  GET_KEYS = 0x30,
  KEY_EVENT = 0x31,
  ROUTE_EVENT = 0x32,

  // Configuration
  SET_CONFIG = 0x40,
  GET_CONFIG = 0x41,

  // Status and control
  PING = 0x50,
  RESET = 0x51,
  FIRMWARE_UPDATE = 0x52,

  // Responses
  ACK = 0x80,
  NACK = 0x81,
  ERROR = 0x82,

  // Events (device-initiated)
  EVENT_KEY = 0x90,
  EVENT_ROUTE = 0x91,
  EVENT_STATUS = 0x92,
  EVENT_BATTERY = 0x93,
}
```

### 2.3 Response Codes

```typescript
enum ResponseCode {
  SUCCESS = 0x00,
  INVALID_COMMAND = 0x01,
  INVALID_PAYLOAD = 0x02,
  DEVICE_BUSY = 0x03,
  NOT_SUPPORTED = 0x04,
  CHECKSUM_ERROR = 0x05,
  TIMEOUT = 0x06,
  UNKNOWN_ERROR = 0xFF,
}
```

---

## 3. USB HID Protocol

### 3.1 USB Device Descriptors

#### Device Descriptor

```c
struct usb_device_descriptor {
  uint8_t  bLength;            // 18
  uint8_t  bDescriptorType;    // 0x01 (Device)
  uint16_t bcdUSB;             // 0x0200 (USB 2.0)
  uint8_t  bDeviceClass;       // 0x00 (see interface)
  uint8_t  bDeviceSubClass;    // 0x00
  uint8_t  bDeviceProtocol;    // 0x00
  uint8_t  bMaxPacketSize0;    // 64
  uint16_t idVendor;           // Manufacturer-specific
  uint16_t idProduct;          // Product-specific
  uint16_t bcdDevice;          // Device version
  uint8_t  iManufacturer;      // String index
  uint8_t  iProduct;           // String index
  uint8_t  iSerialNumber;      // String index
  uint8_t  bNumConfigurations; // 1
};
```

#### HID Descriptor

```c
struct hid_descriptor {
  uint8_t  bLength;            // 9
  uint8_t  bDescriptorType;    // 0x21 (HID)
  uint16_t bcdHID;             // 0x0111 (HID 1.11)
  uint8_t  bCountryCode;       // 0x00
  uint8_t  bNumDescriptors;    // 1
  uint8_t  bDescriptorType2;   // 0x22 (Report)
  uint16_t wDescriptorLength;  // Report descriptor length
};
```

### 3.2 HID Report Descriptor

```c
// Braille Display Report Descriptor
const uint8_t REPORT_DESCRIPTOR[] = {
  // Input Report (Keys and routing)
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x00,        // Usage (Undefined)
  0xA1, 0x01,        // Collection (Application)

  // Braille Keys
  0x05, 0x41,        //   Usage Page (Braille Display)
  0x09, 0x01,        //   Usage (Braille Display)
  0xA1, 0x02,        //   Collection (Logical)
  0x09, 0x02,        //     Usage (Braille Keys)
  0x15, 0x00,        //     Logical Minimum (0)
  0x26, 0xFF, 0x00,  //     Logical Maximum (255)
  0x75, 0x08,        //     Report Size (8 bits)
  0x95, 0x08,        //     Report Count (8 keys)
  0x81, 0x02,        //     Input (Data, Variable, Absolute)
  0xC0,              //   End Collection

  // Cursor Routing
  0x09, 0x03,        //   Usage (Cursor Routing)
  0xA1, 0x02,        //   Collection (Logical)
  0x09, 0x04,        //     Usage (Routing Key)
  0x15, 0x00,        //     Logical Minimum (0)
  0x26, 0xFF, 0x00,  //     Logical Maximum (255)
  0x75, 0x08,        //     Report Size (8 bits)
  0x95, 0x01,        //     Report Count (1)
  0x81, 0x02,        //     Input (Data, Variable, Absolute)
  0xC0,              //   End Collection

  // Output Report (Cell data)
  0x09, 0x05,        //   Usage (Braille Cells)
  0xA1, 0x02,        //   Collection (Logical)
  0x15, 0x00,        //     Logical Minimum (0)
  0x26, 0xFF, 0x00,  //     Logical Maximum (255)
  0x75, 0x08,        //     Report Size (8 bits)
  0x95, 0xA0,        //     Report Count (160 cells max)
  0x91, 0x02,        //     Output (Data, Variable, Absolute)
  0xC0,              //   End Collection

  0xC0               // End Collection
};
```

### 3.3 HID Reports

#### Input Report (Keys)

```c
struct hid_input_report {
  uint8_t report_id;      // 0x01
  uint8_t braille_keys;   // Braille dot keys (bits 0-7)
  uint8_t nav_keys;       // Navigation keys
  uint8_t function_keys;  // Function keys
  uint8_t routing_cell;   // Cursor routing cell (0xFF = none)
  uint8_t modifiers;      // Modifier keys
  uint8_t reserved[3];    // Reserved
} __attribute__((packed));
```

#### Output Report (Cells)

```c
struct hid_output_report {
  uint8_t report_id;      // 0x02
  uint8_t start_cell;     // Starting cell index
  uint8_t cell_count;     // Number of cells to write
  uint8_t cells[160];     // Cell data (dot patterns)
} __attribute__((packed));
```

#### Feature Report (Configuration)

```c
struct hid_feature_report {
  uint8_t report_id;      // 0x03
  uint8_t width;          // Display width
  uint8_t height;         // Display height
  uint8_t cell_format;    // 6 or 8 dots
  uint8_t firmware_major; // Firmware version major
  uint8_t firmware_minor; // Firmware version minor
  uint8_t firmware_patch; // Firmware version patch
  uint8_t features;       // Feature flags
} __attribute__((packed));
```

### 3.4 USB Communication Flow

```
Host                                Device
  │                                   │
  ├─── GET_DESCRIPTOR(Device) ───────>│
  │<──────── Device Descriptor ───────┤
  │                                   │
  ├─── GET_DESCRIPTOR(HID) ──────────>│
  │<──────── HID Descriptor ──────────┤
  │                                   │
  ├─── GET_DESCRIPTOR(Report) ───────>│
  │<────── Report Descriptor ─────────┤
  │                                   │
  ├─── SET_FEATURE(Config) ──────────>│
  │<────────── ACK ───────────────────┤
  │                                   │
  ├─── SET_OUTPUT(Cells) ────────────>│
  │<────────── ACK ───────────────────┤
  │                                   │
  │<────── INPUT(Keys) ───────────────┤
  │                                   │
```

### 3.5 USB HID Example (libusb)

```c
#include <libusb-1.0/libusb.h>

#define VENDOR_ID  0x1234
#define PRODUCT_ID 0x5678

int write_cells(libusb_device_handle *handle, uint8_t *cells, int count) {
  uint8_t report[163]; // Report ID + start + count + 160 cells

  report[0] = 0x02;    // Output report ID
  report[1] = 0;       // Start cell
  report[2] = count;   // Cell count
  memcpy(&report[3], cells, count);

  int result = libusb_control_transfer(
    handle,
    LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
    0x09, // SET_REPORT
    0x0200, // Output report
    0,      // Interface 0
    report,
    count + 3,
    1000    // Timeout (ms)
  );

  return result;
}

int read_keys(libusb_device_handle *handle, uint8_t *keys) {
  int transferred;

  int result = libusb_interrupt_transfer(
    handle,
    0x81,   // Endpoint IN
    keys,
    8,      // Input report size
    &transferred,
    1000    // Timeout (ms)
  );

  return result;
}
```

---

## 4. Bluetooth LE GATT Protocol

### 4.1 GATT Service Definition

#### Primary Service: Braille Display Service

**UUID:** `0000WIA0-0000-1000-8000-00805F9B34FB`

This service provides access to Braille display functionality.

### 4.2 GATT Characteristics

#### 4.2.1 Display Information (Read)

**UUID:** `0000WIA1-0000-1000-8000-00805F9B34FB`
**Properties:** Read
**Format:** 16 bytes

```c
struct display_info {
  uint8_t width;          // Display width
  uint8_t height;         // Display height
  uint8_t cell_format;    // 6 or 8 dots
  uint8_t total_cells;    // Total number of cells
  uint8_t status_cells;   // Number of status cells
  uint8_t features;       // Feature flags
  uint16_t vendor_id;     // Vendor ID
  uint16_t product_id;    // Product ID
  uint8_t fw_major;       // Firmware version major
  uint8_t fw_minor;       // Firmware version minor
  uint8_t fw_patch;       // Firmware version patch
  uint8_t reserved[3];    // Reserved
} __attribute__((packed));
```

#### 4.2.2 Cell Data (Write, Write Without Response)

**UUID:** `0000WIA2-0000-1000-8000-00805F9B34FB`
**Properties:** Write, Write Without Response
**Format:** Variable (3-163 bytes)

```c
struct cell_data {
  uint8_t start_cell;     // Starting cell index
  uint8_t cell_count;     // Number of cells (1-160)
  uint8_t cells[];        // Cell data (dot patterns)
} __attribute__((packed));
```

#### 4.2.3 Key Events (Notify, Read)

**UUID:** `0000WIA3-0000-1000-8000-00805F9B34FB`
**Properties:** Notify, Read
**Format:** 8 bytes

```c
struct key_event {
  uint8_t event_type;     // 0x01 = key press, 0x02 = key release
  uint8_t key_code;       // Key code
  uint8_t modifiers;      // Modifier keys
  uint8_t reserved;       // Reserved
  uint32_t timestamp;     // Timestamp (ms)
} __attribute__((packed));
```

#### 4.2.4 Cursor Routing (Notify, Read)

**UUID:** `0000WIA4-0000-1000-8000-00805F9B34FB`
**Properties:** Notify, Read
**Format:** 4 bytes

```c
struct routing_event {
  uint8_t cell_index;     // Cell index (0-159)
  uint8_t reserved[3];    // Reserved
} __attribute__((packed));
```

#### 4.2.5 Cursor Position (Read, Write)

**UUID:** `0000WIA5-0000-1000-8000-00805F9B34FB`
**Properties:** Read, Write
**Format:** 4 bytes

```c
struct cursor_position {
  uint8_t line;           // Line number
  uint8_t cell;           // Cell number
  uint8_t visible;        // 0=hidden, 1=visible
  uint8_t shape;          // 0=block, 1=underline, 2=vertical
} __attribute__((packed));
```

#### 4.2.6 Device Status (Read, Notify)

**UUID:** `0000WIA6-0000-1000-8000-00805F9B34FB`
**Properties:** Read, Notify
**Format:** 8 bytes

```c
struct device_status {
  uint8_t battery_level;  // Battery level (0-100%)
  uint8_t charging;       // 0=not charging, 1=charging
  uint8_t connection;     // Connection quality (0-100)
  uint8_t error_code;     // Last error code
  uint32_t uptime;        // Uptime (seconds)
} __attribute__((packed));
```

#### 4.2.7 Control Point (Write, Indicate)

**UUID:** `0000WIA7-0000-1000-8000-00805F9B34FB`
**Properties:** Write, Indicate
**Format:** Variable (2-16 bytes)

```c
struct control_command {
  uint8_t command;        // Command code
  uint8_t params[];       // Command parameters
} __attribute__((packed));
```

### 4.3 GATT Service Hierarchy

```
Braille Display Service (0000WIA0...)
├── Display Information (0000WIA1...)
│   └── [Read]
├── Cell Data (0000WIA2...)
│   └── [Write, Write Without Response]
├── Key Events (0000WIA3...)
│   └── [Notify, Read]
├── Cursor Routing (0000WIA4...)
│   └── [Notify, Read]
├── Cursor Position (0000WIA5...)
│   └── [Read, Write]
├── Device Status (0000WIA6...)
│   └── [Read, Notify]
└── Control Point (0000WIA7...)
    └── [Write, Indicate]
```

### 4.4 BLE Connection Flow

```
Central (Host)                      Peripheral (Display)
  │                                        │
  ├───── Scan for devices ────────────────>│
  │<────── Advertisement ──────────────────┤
  │                                        │
  ├───── Connect ─────────────────────────>│
  │<────── Connection Response ────────────┤
  │                                        │
  ├───── Discover Services ───────────────>│
  │<────── Service List ───────────────────┤
  │                                        │
  ├───── Discover Characteristics ────────>│
  │<────── Characteristic List ────────────┤
  │                                        │
  ├───── Enable Notifications ────────────>│
  │      (Key Events, Routing)             │
  │<────── ACK ────────────────────────────┤
  │                                        │
  ├───── Write Cell Data ─────────────────>│
  │<────── ACK (if requested) ─────────────┤
  │                                        │
  │<────── Key Event Notification ─────────┤
  │                                        │
```

### 4.5 BLE Example (Web Bluetooth)

```javascript
async function connectBrailleDisplay() {
  // Request device
  const device = await navigator.bluetooth.requestDevice({
    filters: [{
      services: ['0000WIA0-0000-1000-8000-00805F9B34FB']
    }]
  });

  // Connect to GATT server
  const server = await device.gatt.connect();

  // Get service
  const service = await server.getPrimaryService(
    '0000WIA0-0000-1000-8000-00805F9B34FB'
  );

  // Get characteristics
  const cellDataChar = await service.getCharacteristic(
    '0000WIA2-0000-1000-8000-00805F9B34FB'
  );

  const keyEventChar = await service.getCharacteristic(
    '0000WIA3-0000-1000-8000-00805F9B34FB'
  );

  const routingChar = await service.getCharacteristic(
    '0000WIA4-0000-1000-8000-00805F9B34FB'
  );

  // Enable notifications
  await keyEventChar.startNotifications();
  keyEventChar.addEventListener('characteristicvaluechanged', (event) => {
    const value = event.target.value;
    const eventType = value.getUint8(0);
    const keyCode = value.getUint8(1);
    console.log(`Key event: ${eventType}, code: ${keyCode}`);
  });

  await routingChar.startNotifications();
  routingChar.addEventListener('characteristicvaluechanged', (event) => {
    const value = event.target.value;
    const cellIndex = value.getUint8(0);
    console.log(`Routing: cell ${cellIndex}`);
  });

  return {
    device,
    service,
    cellDataChar,
    keyEventChar,
    routingChar,
  };
}

async function writeCells(cellDataChar, cells) {
  const data = new Uint8Array(cells.length + 2);
  data[0] = 0; // Start cell
  data[1] = cells.length; // Cell count
  data.set(cells, 2);

  await cellDataChar.writeValueWithoutResponse(data);
}
```

---

## 5. Serial Communication Protocol

### 5.1 Serial Port Configuration

```c
struct serial_config {
  uint32_t baud_rate;    // 9600, 19200, 38400, 57600, 115200
  uint8_t  data_bits;    // 7 or 8
  uint8_t  parity;       // 0=none, 1=odd, 2=even
  uint8_t  stop_bits;    // 1 or 2
  uint8_t  flow_control; // 0=none, 1=hardware, 2=software
};

// Standard configuration
#define SERIAL_DEFAULT_CONFIG { \
  .baud_rate = 115200, \
  .data_bits = 8, \
  .parity = 0, \
  .stop_bits = 1, \
  .flow_control = 0 \
}
```

### 5.2 Serial Frame Format

```
┌──────┬────────┬─────────┬────────┬─────────┬──────────┬──────┐
│ STX  │ LENGTH │ COMMAND │ SEQ_ID │ PAYLOAD │ CHECKSUM │ ETX  │
├──────┼────────┼─────────┼────────┼─────────┼──────────┼──────┤
│ 0x02 │ 2B     │ 1B      │ 2B     │ 0-255B  │ 2B       │ 0x03 │
└──────┴────────┴─────────┴────────┴─────────┴──────────┴──────┘
```

**Fields:**
- **STX**: Start of text (0x02)
- **LENGTH**: Payload length (big-endian)
- **COMMAND**: Command code
- **SEQ_ID**: Sequence identifier
- **PAYLOAD**: Command data
- **CHECKSUM**: CRC-16 of LENGTH through PAYLOAD
- **ETX**: End of text (0x03)

### 5.3 Serial Commands

#### Write Cells

```
Command: 0x10 (WRITE_CELLS)
Payload:
  - start_cell (1 byte)
  - cell_count (1 byte)
  - cells (cell_count bytes)
```

#### Read Keys

```
Command: 0x30 (GET_KEYS)
Payload: (none)

Response:
  - key_state (8 bytes) - Bit array of key states
```

#### Get Info

```
Command: 0x01 (GET_INFO)
Payload: (none)

Response:
  - width (1 byte)
  - height (1 byte)
  - cell_format (1 byte)
  - firmware_version (3 bytes)
```

### 5.4 Serial Communication Flow

```
Host                                Device
  │                                   │
  ├── STX + GET_INFO + ETX ──────────>│
  │                                   │
  │<─── STX + INFO_DATA + ETX ────────┤
  │                                   │
  ├── STX + WRITE_CELLS + ETX ───────>│
  │                                   │
  │<─── STX + ACK + ETX ───────────────┤
  │                                   │
  │<─── STX + KEY_EVENT + ETX ─────────┤
  │                                   │
  ├── STX + ACK + ETX ───────────────>│
  │                                   │
```

### 5.5 Serial Example (Python)

```python
import serial
import struct

class BrailleSerialProtocol:
    STX = 0x02
    ETX = 0x03

    def __init__(self, port, baudrate=115200):
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )
        self.seq_id = 0

    def send_command(self, command, payload=b''):
        self.seq_id = (self.seq_id + 1) % 65536

        # Build frame
        length = len(payload)
        frame_data = struct.pack('>HBH', length, command, self.seq_id) + payload

        # Calculate checksum
        checksum = self.crc16(frame_data)

        # Build complete frame
        frame = bytes([self.STX]) + frame_data + struct.pack('>H', checksum) + bytes([self.ETX])

        # Send
        self.serial.write(frame)

    def receive_response(self):
        # Wait for STX
        while True:
            byte = self.serial.read(1)
            if byte == bytes([self.STX]):
                break

        # Read length
        length_bytes = self.serial.read(2)
        length = struct.unpack('>H', length_bytes)[0]

        # Read command and seq_id
        cmd_seq = self.serial.read(3)
        command = cmd_seq[0]
        seq_id = struct.unpack('>H', cmd_seq[1:3])[0]

        # Read payload
        payload = self.serial.read(length)

        # Read checksum
        checksum_bytes = self.serial.read(2)
        checksum = struct.unpack('>H', checksum_bytes)[0]

        # Verify checksum
        frame_data = length_bytes + cmd_seq + payload
        if self.crc16(frame_data) != checksum:
            raise ValueError("Checksum mismatch")

        # Read ETX
        etx = self.serial.read(1)
        if etx != bytes([self.ETX]):
            raise ValueError("Missing ETX")

        return command, seq_id, payload

    def crc16(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def write_cells(self, cells, start=0):
        payload = struct.pack('BB', start, len(cells)) + bytes(cells)
        self.send_command(0x10, payload)
        cmd, seq, resp = self.receive_response()
        return cmd == 0x80  # ACK

# Usage
display = BrailleSerialProtocol('/dev/ttyUSB0')
cells = [0x21, 0x17, 0x0F, 0x0F, 0x55]  # "Hello"
display.write_cells(cells)
```

---

## 6. BRLTTY Driver Protocol

### 6.1 BRLTTY Overview

BRLTTY (Braille TTY) is a Linux daemon that provides access to Braille displays for console applications.

### 6.2 Driver Interface

```c
#include <brltty/api.h>

// Driver structure
typedef struct {
  const char *name;              // Driver name
  const char *description;       // Driver description
  int (*probe)(BrailleDisplay *); // Probe for device
  int (*open)(BrailleDisplay *);  // Open connection
  void (*close)(BrailleDisplay *);// Close connection
  int (*writeWindow)(BrailleDisplay *, const wchar_t *); // Write text
  int (*readCommand)(BrailleDisplay *, KeyTableCommandContext); // Read input
} BrailleDriver;

// Display structure
typedef struct {
  int textColumns;               // Number of columns
  int textRows;                  // Number of rows
  unsigned char cellSize;        // Cell format (6 or 8)
  void *data;                    // Driver-specific data
} BrailleDisplay;
```

### 6.3 WIA BRLTTY Driver Implementation

```c
#include <brltty/api.h>

// Driver data
typedef struct {
  int fd;                        // File descriptor
  unsigned char buffer[160];     // Cell buffer
} WiaDriverData;

static int wia_probe(BrailleDisplay *brl) {
  // Probe for WIA-compatible device
  // Check USB vendor/product ID or serial port
  return 1; // Found
}

static int wia_open(BrailleDisplay *brl) {
  WiaDriverData *data = malloc(sizeof(WiaDriverData));

  // Open USB/Serial connection
  data->fd = open("/dev/braille0", O_RDWR);
  if (data->fd < 0) {
    free(data);
    return 0;
  }

  // Get device info
  unsigned char info[16];
  if (wia_get_info(data->fd, info) < 0) {
    close(data->fd);
    free(data);
    return 0;
  }

  brl->textColumns = info[0];
  brl->textRows = info[1];
  brl->cellSize = info[2];
  brl->data = data;

  return 1;
}

static void wia_close(BrailleDisplay *brl) {
  WiaDriverData *data = brl->data;
  if (data) {
    close(data->fd);
    free(data);
  }
}

static int wia_writeWindow(BrailleDisplay *brl, const wchar_t *text) {
  WiaDriverData *data = brl->data;

  // Convert text to Braille
  unsigned char cells[160];
  int length = translateTextToBraille(text, cells, brl->textColumns);

  // Write cells to device
  return wia_write_cells(data->fd, cells, length);
}

static int wia_readCommand(BrailleDisplay *brl, KeyTableCommandContext context) {
  WiaDriverData *data = brl->data;

  // Read key events
  unsigned char key_data[8];
  if (read(data->fd, key_data, sizeof(key_data)) < 0) {
    return EOF;
  }

  // Translate to BRLTTY command
  return translateKeyToCommand(key_data, context);
}

// Driver registration
const BrailleDriver wiaDriver = {
  .name = "wia",
  .description = "WIA Braille Display",
  .probe = wia_probe,
  .open = wia_open,
  .close = wia_close,
  .writeWindow = wia_writeWindow,
  .readCommand = wia_readCommand,
};
```

### 6.4 BRLTTY Configuration

```ini
# /etc/brltty.conf

# Braille driver
braille-driver wia

# Braille device
braille-device usb:1234:5678  # USB Vendor:Product
# braille-device serial:/dev/ttyUSB0  # Serial port
# braille-device bluetooth:00:11:22:33:44:55  # Bluetooth

# Text table
text-table en-us-g2.ctb

# Attributes table
attributes-table left_right.atb

# Keyboard table
keyboard-table laptop.ktb
```

---

## 7. Network Protocol

### 7.1 TCP/IP Protocol

The WIA Braille Display network protocol allows remote access to Braille displays over TCP/IP.

**Default Port:** 8142 (BRAILLE on phone keypad)

### 7.2 Connection Handshake

```
Client                              Server
  │                                   │
  ├── CONNECT ───────────────────────>│
  │                                   │
  │<──── HELLO ────────────────────────┤
  │     (Protocol version, capabilities)
  │                                   │
  ├── AUTH ──────────────────────────>│
  │     (API key or credentials)      │
  │                                   │
  │<──── AUTH_OK ──────────────────────┤
  │                                   │
  ├── SUBSCRIBE ─────────────────────>│
  │     (Event types)                 │
  │                                   │
  │<──── SUBSCRIBE_OK ─────────────────┤
  │                                   │
```

### 7.3 Network Message Format

```
┌──────┬────────┬─────────┬────────┬─────────┬──────────┐
│ SYNC │ LENGTH │ MESSAGE │ SEQ_ID │ PAYLOAD │ CHECKSUM │
├──────┼────────┼─────────┼────────┼─────────┼──────────┤
│ 4B   │ 4B     │ 2B      │ 4B     │ 0-64KB  │ 4B       │
└──────┴────────┴─────────┴────────┴─────────┴──────────┘
```

**Fields:**
- **SYNC**: Synchronization marker (0x57494142 = "WIAB")
- **LENGTH**: Total message length (big-endian)
- **MESSAGE**: Message type code
- **SEQ_ID**: Sequence identifier
- **PAYLOAD**: Message data (JSON or binary)
- **CHECKSUM**: CRC-32 of entire message excluding checksum

### 7.4 Message Types

```typescript
enum NetworkMessage {
  // Connection
  CONNECT = 0x0001,
  HELLO = 0x0002,
  AUTH = 0x0003,
  AUTH_OK = 0x0004,
  AUTH_FAIL = 0x0005,
  DISCONNECT = 0x0006,

  // Subscription
  SUBSCRIBE = 0x0010,
  UNSUBSCRIBE = 0x0011,
  SUBSCRIBE_OK = 0x0012,

  // Display operations
  WRITE_CELLS = 0x0020,
  WRITE_TEXT = 0x0021,
  CLEAR = 0x0022,
  SET_CURSOR = 0x0023,

  // Events
  EVENT_KEY = 0x0030,
  EVENT_ROUTE = 0x0031,
  EVENT_STATUS = 0x0032,

  // Responses
  ACK = 0x0080,
  ERROR = 0x0081,

  // Heartbeat
  PING = 0x0090,
  PONG = 0x0091,
}
```

### 7.5 Network Protocol Example (Node.js)

```javascript
const net = require('net');
const crypto = require('crypto');

class BrailleNetworkClient {
  constructor(host, port = 8142) {
    this.host = host;
    this.port = port;
    this.socket = null;
    this.seqId = 0;
  }

  connect(apiKey) {
    return new Promise((resolve, reject) => {
      this.socket = net.createConnection({
        host: this.host,
        port: this.port,
      });

      this.socket.on('connect', async () => {
        // Wait for HELLO
        const hello = await this.receiveMessage();
        console.log('Server version:', hello.version);

        // Send AUTH
        await this.sendMessage(0x0003, { apiKey });

        // Wait for AUTH_OK
        const authResp = await this.receiveMessage();
        if (authResp.type === 0x0004) {
          resolve();
        } else {
          reject(new Error('Authentication failed'));
        }
      });

      this.socket.on('error', reject);
    });
  }

  async sendMessage(type, payload) {
    this.seqId++;

    const payloadJson = JSON.stringify(payload);
    const payloadBuffer = Buffer.from(payloadJson, 'utf8');

    const header = Buffer.alloc(14);
    header.writeUInt32BE(0x57494142, 0); // SYNC
    header.writeUInt32BE(payloadBuffer.length + 14, 4); // LENGTH
    header.writeUInt16BE(type, 8); // MESSAGE
    header.writeUInt32BE(this.seqId, 10); // SEQ_ID

    const message = Buffer.concat([header, payloadBuffer]);

    // Calculate checksum
    const checksum = this.crc32(message);
    const checksumBuffer = Buffer.alloc(4);
    checksumBuffer.writeUInt32BE(checksum, 0);

    const frame = Buffer.concat([message, checksumBuffer]);

    this.socket.write(frame);
  }

  async receiveMessage() {
    return new Promise((resolve) => {
      const onData = (data) => {
        // Parse message
        const sync = data.readUInt32BE(0);
        if (sync !== 0x57494142) {
          throw new Error('Invalid sync marker');
        }

        const length = data.readUInt32BE(4);
        const type = data.readUInt16BE(8);
        const seqId = data.readUInt32BE(10);

        const payloadLength = length - 14;
        const payload = data.slice(14, 14 + payloadLength);

        const checksum = data.readUInt32BE(14 + payloadLength);

        // Verify checksum
        const calculatedChecksum = this.crc32(data.slice(0, 14 + payloadLength));
        if (checksum !== calculatedChecksum) {
          throw new Error('Checksum mismatch');
        }

        const payloadObj = JSON.parse(payload.toString('utf8'));

        this.socket.off('data', onData);
        resolve({ type, seqId, payload: payloadObj });
      };

      this.socket.on('data', onData);
    });
  }

  async writeCells(cells, offset = 0) {
    await this.sendMessage(0x0020, {
      cells: Array.from(cells),
      offset,
    });

    const resp = await this.receiveMessage();
    return resp.type === 0x0080; // ACK
  }

  crc32(buffer) {
    return crypto.createHash('crc32').update(buffer).digest().readUInt32BE(0);
  }
}

// Usage
const client = new BrailleNetworkClient('localhost');
await client.connect('my-api-key');
await client.writeCells([0x21, 0x17, 0x0F, 0x0F, 0x55]);
```

---

## 8. Message Formats

### 8.1 Write Cells Payload

```json
{
  "start_cell": 0,
  "cell_count": 5,
  "cells": [33, 23, 15, 15, 135],
  "attributes": [
    {"cursor": true},
    {},
    {},
    {},
    {"blink": true}
  ]
}
```

### 8.2 Key Event Payload

```json
{
  "event_type": "press",
  "key_code": 38,
  "key_name": "up",
  "modifiers": {
    "shift": false,
    "control": false,
    "alt": false
  },
  "timestamp": 1735123456789
}
```

### 8.3 Device Info Payload

```json
{
  "width": 40,
  "height": 1,
  "cell_format": 8,
  "total_cells": 40,
  "firmware": {
    "major": 1,
    "minor": 2,
    "patch": 3
  },
  "capabilities": {
    "cursor_routing": true,
    "status_cells": 5,
    "vibration": false
  }
}
```

---

## 9. Security Requirements

### 9.1 Encryption

**TLS/SSL**: All network connections MUST use TLS 1.2 or higher.

```javascript
const tls = require('tls');

const options = {
  host: 'braille.example.com',
  port: 8143,
  ca: fs.readFileSync('ca.pem'),
  cert: fs.readFileSync('client-cert.pem'),
  key: fs.readFileSync('client-key.pem'),
};

const socket = tls.connect(options, () => {
  console.log('Secure connection established');
});
```

### 9.2 Authentication

**API Key**: Include in AUTH message or HTTP header.

```
Authorization: Bearer <api_key>
```

**OAuth 2.0**: For web applications.

### 9.3 Data Integrity

All messages include CRC checksums to detect corruption.

### 9.4 Access Control

Devices SHOULD implement pairing/authorization for Bluetooth connections.

---

## 10. Protocol Negotiation

### 10.1 Version Negotiation

```
Client                              Server
  │                                   │
  ├── HELLO (version 1.0) ───────────>│
  │                                   │
  │<──── HELLO (version 1.2) ──────────┤
  │                                   │
  ├── USE_VERSION (1.0) ─────────────>│
  │     (Use highest common version)  │
  │                                   │
  │<──── VERSION_OK ───────────────────┤
  │                                   │
```

### 10.2 Capability Negotiation

```json
{
  "client_capabilities": [
    "cursor_routing",
    "8_dot_cells",
    "multi_line"
  ],
  "server_capabilities": [
    "cursor_routing",
    "6_dot_cells",
    "8_dot_cells",
    "multi_line",
    "vibration"
  ],
  "negotiated": [
    "cursor_routing",
    "8_dot_cells",
    "multi_line"
  ]
}
```

---

## 11. Error Handling and Recovery

### 11.1 Retry Logic

```javascript
async function sendWithRetry(message, maxRetries = 3) {
  for (let i = 0; i < maxRetries; i++) {
    try {
      await send(message);
      return;
    } catch (error) {
      if (i === maxRetries - 1) throw error;
      await sleep(1000 * Math.pow(2, i)); // Exponential backoff
    }
  }
}
```

### 11.2 Connection Recovery

```javascript
class AutoReconnect {
  constructor(connect, options = {}) {
    this.connect = connect;
    this.maxAttempts = options.maxAttempts || 10;
    this.delay = options.delay || 5000;
  }

  async start() {
    let attempts = 0;

    while (attempts < this.maxAttempts) {
      try {
        await this.connect();
        console.log('Connected');
        return;
      } catch (error) {
        attempts++;
        console.log(`Connection attempt ${attempts} failed`);

        if (attempts < this.maxAttempts) {
          await sleep(this.delay);
        }
      }
    }

    throw new Error('Max connection attempts exceeded');
  }
}
```

---

## 12. Performance Optimization

### 12.1 Batching

```javascript
class CellBatcher {
  constructor(display, interval = 50) {
    this.display = display;
    this.interval = interval;
    this.pending = [];
    this.timer = null;
  }

  queueWrite(cells, offset) {
    this.pending.push({ cells, offset });

    if (!this.timer) {
      this.timer = setTimeout(() => this.flush(), this.interval);
    }
  }

  async flush() {
    const updates = this.pending;
    this.pending = [];
    this.timer = null;

    // Merge adjacent updates
    const merged = this.mergeUpdates(updates);

    for (const update of merged) {
      await this.display.writeCells(update.cells, update.offset);
    }
  }

  mergeUpdates(updates) {
    // Implementation of update merging logic
    return updates;
  }
}
```

### 12.2 Compression

For network protocols, use gzip compression:

```javascript
const zlib = require('zlib');

function compressPayload(payload) {
  return zlib.gzipSync(Buffer.from(JSON.stringify(payload)));
}

function decompressPayload(compressed) {
  return JSON.parse(zlib.gunzipSync(compressed).toString());
}
```

---

**Document End**

© 2025 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity

For questions or contributions, please visit:
https://github.com/WIA-Official/wia-standards
